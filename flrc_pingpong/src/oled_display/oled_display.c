/*!
 * \file    oled_display.c
 *
 * \brief   Handle the display on the OLED
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2026. All rights reserved.
 *
 * Modified for flrc_pingpong: image path removed, big-text layout added.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

#include <lvgl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "oled_display.h"

LOG_MODULE_REGISTER( display, LOG_LEVEL_INF );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define DISPLAY_THREAD_STACKSIZE 4096
#define DISPLAY_THREAD_PRIORITY 3
#define DISPLAY_TEXT_MAX_LEN 30

/* Big layout: y positions of the rows (title, big, big, footer) */
#define DISPLAY_BIG_ROW2_Y 12
#define DISPLAY_BIG_ROW3_Y 32
#define DISPLAY_BIG_ROW4_Y 53

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/* Latest text per row: writers overwrite, the display thread picks up dirty rows (no queue to overflow) */
typedef struct
{
    char text[DISPLAY_TEXT_MAX_LEN];
    bool dirty;
} display_row_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

LV_FONT_DECLARE( font_RobotoCondensed_Regular_10 );
LV_FONT_DECLARE( font_RobotoCondensed_Regular_12 );

static struct k_thread display_thread;
K_THREAD_STACK_DEFINE( display_stack, DISPLAY_THREAD_STACKSIZE );

static lv_obj_t*        text_labels[DISPLAY_MAX_ROW];
static lv_obj_t*        screen;
static display_layout_t display_layout = DISPLAY_LAYOUT_TEXT;

static display_row_t     display_rows[DISPLAY_MAX_ROW];
static struct k_spinlock display_rows_lock;
static volatile uint32_t display_refresh_count;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void display_thread_entry( void* p1, void* p2, void* p3 );
static void process_display_text( void );
static void create_text_labels( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void display_thread_init( display_layout_t layout )
{
    display_layout = layout;
    k_thread_create( &display_thread, display_stack, K_THREAD_STACK_SIZEOF( display_stack ), display_thread_entry, NULL,
                     NULL, NULL, DISPLAY_THREAD_PRIORITY, K_INHERIT_PERMS, K_FOREVER );
    k_thread_name_set( &display_thread, "display" );
    k_thread_start( &display_thread );
}

void display_text( uint8_t row, const char* fmt, ... )
{
    if( row >= DISPLAY_MAX_ROW )
    {
        LOG_ERR( "Row number is out of range" );
        return;
    }

    char text[DISPLAY_TEXT_MAX_LEN];

    va_list args;
    va_start( args, fmt );
    vsnprintf( text, sizeof( text ), fmt, args );
    va_end( args );

    k_spinlock_key_t key = k_spin_lock( &display_rows_lock );
    memcpy( display_rows[row].text, text, sizeof( text ) );
    display_rows[row].dirty = true;
    k_spin_unlock( &display_rows_lock, key );
}

uint32_t display_get_refresh_count( void )
{
    return display_refresh_count;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void display_thread_entry( void* p1, void* p2, void* p3 )
{
    ARG_UNUSED( p1 );
    ARG_UNUSED( p2 );
    ARG_UNUSED( p3 );

    screen = lv_scr_act( );
    if( screen == NULL )
    {
        LOG_ERR( "Failed to get screen object" );
        return;
    }

    create_text_labels( );
    LOG_INF( "Display thread started (%s layout)", ( display_layout == DISPLAY_LAYOUT_BIG ) ? "big" : "text" );

    while( 1 )
    {
        process_display_text( );
        lv_timer_handler( );
        display_refresh_count++;
        if( display_refresh_count == 1 )
        {
            LOG_INF( "Display first refresh done" );
        }
        k_msleep( 50 );
    }
}

static void create_text_labels( void )
{
    for( uint8_t i = 0; i < DISPLAY_MAX_ROW; i++ )
    {
        if( text_labels[i] != NULL )
        {
            continue;
        }

        text_labels[i] = lv_label_create( screen );

        if( i == 0 )
        {
            // Title row: same in both layouts
            lv_obj_set_style_text_font( text_labels[i], &font_RobotoCondensed_Regular_12,
                                        LV_PART_MAIN | LV_STATE_DEFAULT );
            lv_obj_align( text_labels[i], LV_ALIGN_TOP_MID, 0, 0 );
        }
        else if( display_layout == DISPLAY_LAYOUT_BIG )
        {
            if( ( i == DISPLAY_ROW2 ) || ( i == DISPLAY_ROW3 ) )
            {
                lv_obj_set_style_text_font( text_labels[i], &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT );
                lv_obj_align( text_labels[i], LV_ALIGN_TOP_MID, 0,
                              ( i == DISPLAY_ROW2 ) ? DISPLAY_BIG_ROW2_Y : DISPLAY_BIG_ROW3_Y );
            }
            else
            {
                lv_obj_set_style_text_font( text_labels[i], &font_RobotoCondensed_Regular_10,
                                            LV_PART_MAIN | LV_STATE_DEFAULT );
                lv_obj_align( text_labels[i], LV_ALIGN_TOP_LEFT, 0, DISPLAY_BIG_ROW4_Y );
                if( i > DISPLAY_ROW4 )
                {
                    // Rows 5 and 6 do not fit in the big layout
                    lv_obj_add_flag( text_labels[i], LV_OBJ_FLAG_HIDDEN );
                }
            }
        }
        else
        {
            lv_obj_set_style_text_font( text_labels[i], &font_RobotoCondensed_Regular_10,
                                        LV_PART_MAIN | LV_STATE_DEFAULT );
            lv_obj_align( text_labels[i], LV_ALIGN_TOP_LEFT, 0, i * 10 + 4 );
        }

        // Drawn black: with "inversion-on" in DT + CONFIG_LV_Z_COLOR_MONO_HW_INVERSION it shows lit on dark
        lv_obj_set_style_text_color( text_labels[i], lv_color_hex( 0x000000 ), LV_PART_MAIN );
        lv_label_set_text( text_labels[i], " " );
    }
}

static void process_display_text( void )
{
    for( uint8_t row = 0; row < DISPLAY_MAX_ROW; row++ )
    {
        char text[DISPLAY_TEXT_MAX_LEN];
        bool dirty;

        k_spinlock_key_t key = k_spin_lock( &display_rows_lock );
        dirty = display_rows[row].dirty;
        if( dirty )
        {
            memcpy( text, display_rows[row].text, sizeof( text ) );
            display_rows[row].dirty = false;
        }
        k_spin_unlock( &display_rows_lock, key );

        if( dirty && ( text_labels[row] != NULL ) )
        {
            lv_label_set_text( text_labels[row], text );
        }
    }
}

/* --- EOF ------------------------------------------------------------------ */
