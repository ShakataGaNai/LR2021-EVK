/*!
 * \file    oled_display.h
 *
 * \brief   Handle the display on the OLED
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2026. All rights reserved.
 *
 * Modified for flrc_throughput: image path removed, big-text layout added.
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
#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

enum display_row_e
{
    DISPLAY_ROW1 = 0,
    DISPLAY_ROW2,
    DISPLAY_ROW3,
    DISPLAY_ROW4,
    DISPLAY_ROW5,
    DISPLAY_ROW6,
    DISPLAY_MAX_ROW,
};

/*!
 * @brief Screen layouts
 *
 * DISPLAY_LAYOUT_TEXT: row 1 = 12 px title, rows 2..6 = 10 px text (original img_xfer layout).
 * DISPLAY_LAYOUT_BIG:  row 1 = 12 px title, rows 2..3 = large font, row 4 = 10 px footer.
 *                      Rows 5..6 are not shown.
 */
typedef enum display_layout_e
{
    DISPLAY_LAYOUT_TEXT = 0,
    DISPLAY_LAYOUT_BIG,
} display_layout_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief  Create a thread to display text on the OLED.
 *
 * @param [in] layout Screen layout to use
 */
void display_thread_init( display_layout_t layout );

/*!
 * @brief  Put a message in the display queue to display text on the OLED.
 *
 * Safe to call from any thread (including radio callbacks): the text is stored as the latest
 * value for that row (older unshown text is overwritten), LVGL is only touched by the display thread.
 *
 * @param [in] row The row number
 * @param [in] fmt The format string
 * @param [in] ... Variable arguments
 */
void display_text( uint8_t row, const char* fmt, ... );

/*!
 * @brief  Number of LVGL refresh cycles run by the display thread (liveness check).
 */
uint32_t display_get_refresh_count( void );

#ifdef __cplusplus
}
#endif

#endif  // OLED_DISPLAY_H
