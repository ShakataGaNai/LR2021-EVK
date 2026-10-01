/**
 * @file      main.c
 *
 * @brief     Speed and distance demo: alternates FLRC throughput (SPEED) and LoRa ranging (DISTANCE) phases
 *
 * Copied from flrc_throughput/src/main.c (display thread, stats, OLED check); the phase controller is new.
 * TX board: phase master (timer), FLRC transmitter, ranging subordinate.
 * RX board: FLRC receiver, ranging manager (computes and shows the distance); follows the TX through the switch
 * flag in the FLRC payload, or on its own after CONFIG_APP_RX_FALLBACK_MS without FLRC packets.
 *
 * Structure follows swdm031_img_xfer main.c (display thread + USP main thread) and
 * usp_zephyr samples/usp/sdk/packet_error_rate_flrc main.c (RAC init).
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2026. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include <smtc_zephyr_usp_api.h>
#include <smtc_sw_platform_helper.h>

#include "app_events.h"
#include "app_flrc.h"
#include "app_ranging.h"
#include "oled_display.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

LOG_MODULE_REGISTER( main, LOG_LEVEL_INF );

#define STATS_PERIOD_MS ( 1000 )

/* Between phase switches main only formats stats: run it preemptible, below the USP (radio) thread, so console
 * output (polled UART, ~10 ms per line) never delays the radio IRQ handling. */
#define MAIN_RUN_PRIORITY ( 5 )

/* Phase switches touch the radio and the RAC from this thread: run them cooperative (same priority as at boot) so
 * the USP thread cannot preempt half-way through. It still runs whenever this thread sleeps. */
#define MAIN_CTRL_PRIORITY ( -2 )

#define SPEED_MS ( CONFIG_APP_SPEED_PHASE_MS )
#define DISTANCE_MS ( CONFIG_APP_DISTANCE_PHASE_MS )
#define FALLBACK_MS ( CONFIG_APP_RX_FALLBACK_MS )

/* RX: leave SPEED this long before the announced TX switch, and DISTANCE this long before the TX does, so the RX
 * is always listening first. The manager sends its first config this long after the TX switch. */
#define RX_SPEED_EARLY_MS ( 2 )
#define RX_DISTANCE_EARLY_MS ( 30 )
#define RX_FIRST_CFG_DELAY_MS ( 20 )

#define DISPLAY_NODE DT_CHOSEN( zephyr_display )

#define LED0_NODE DT_ALIAS( led0 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum
{
    PHASE_SPEED = 0,
    PHASE_DISTANCE,
} phase_t;

typedef struct
{
    phase_t phase;
    int64_t phase_start_ms;
    int64_t phase_end_ms;  //!< TX: both phases. RX: DISTANCE only.
    uint8_t cycle;         //!< TX: counts SPEED phases. RX: copied from the switch flag packet.
    int64_t stats_last_ms;
    int64_t next_stats_ms;
    uint32_t switches;     //!< Phase switches since boot
} controller_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#if DT_NODE_HAS_STATUS( LED0_NODE, okay )
static const struct gpio_dt_spec usr_led = GPIO_DT_SPEC_GET( LED0_NODE, gpios );
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

K_SEM_DEFINE( app_event_sem, 0, 1 );

static controller_t ctl;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void enter_speed( int64_t now );
static void enter_distance( int64_t now, int64_t end_ms, uint32_t first_cfg_delay_ms, const char* reason );
static void report( int64_t now );
static void report_tx_speed( const app_flrc_stats_t* s, uint32_t elapsed_ms, int64_t now );
static void report_rx_speed( const app_flrc_stats_t* s, uint32_t elapsed_ms );
static void report_distance( int64_t now );
static void show_run( const app_ranging_run_t* run );
static void check_oled( void );
static void format_dm( char* buf, size_t len, int32_t dm );
static void format_cm( char* buf, size_t len, int32_t cm );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void app_event_signal( void )
{
    k_sem_give( &app_event_sem );
}

int main( void )
{
    LOG_INF( "===== Speed and distance demo (%s: FLRC %s, ranging %s) =====", APP_FLRC_IS_TX ? "TX" : "RX",
             APP_FLRC_IS_TX ? "TX" : "RX", APP_RANGING_IS_MANAGER ? "manager" : "subordinate" );
    LOG_INF( "SPEED %u ms, DISTANCE %u ms, switch flag %u ms ahead, RX fallback %u ms", SPEED_MS, DISTANCE_MS,
             CONFIG_APP_SWITCH_WARN_MS, FALLBACK_MS );

    check_oled( );
    display_thread_init( APP_FLRC_IS_TX ? DISPLAY_LAYOUT_TEXT : DISPLAY_LAYOUT_BIG );
    display_text( DISPLAY_ROW1, "Speed & distance %s", APP_FLRC_IS_TX ? "TX" : "RX" );
    display_text( DISPLAY_ROW2, "Starting..." );

#if DT_NODE_HAS_STATUS( LED0_NODE, okay )
    if( gpio_is_ready_dt( &usr_led ) )
    {
        gpio_pin_configure_dt( &usr_led, GPIO_OUTPUT_INACTIVE );
    }
#endif

    SMTC_SW_PLATFORM_INIT( );
    SMTC_SW_PLATFORM_VOID( smtc_rac_init( ) );

    // Open the ranging radio access before the first immediate access
    app_ranging_init( );
    app_flrc_init( );

    ctl.phase = PHASE_DISTANCE;  // So enter_speed() does not try to stop a ranging that never ran
    enter_speed( k_uptime_get( ) );

    while( true )
    {
        int64_t deadline = ctl.next_stats_ms;
        int64_t now      = k_uptime_get( );

        if( APP_RANGING_IS_MANAGER && ( ctl.phase == PHASE_DISTANCE ) )
        {
            app_ranging_run_t run;
            if( app_ranging_get_run( &run ) )
            {
                show_run( &run );
            }
        }

        // RX in SPEED: switch notice from the TX, or fallback after FALLBACK_MS without packets
        uint32_t notice_at_32 = 0;
        uint8_t  notice_cycle = 0;
        bool     notice       = false;
        int64_t  notice_at    = 0;
        int64_t  fallback_at  = 0;

        if( ( ctl.phase == PHASE_SPEED ) && !APP_FLRC_IS_TX )
        {
            notice = app_flrc_get_switch_notice( &notice_at_32, &notice_cycle );
            if( notice )
            {
                notice_at = now + ( int32_t ) ( notice_at_32 - ( uint32_t ) now );
                deadline  = MIN( deadline, notice_at - RX_SPEED_EARLY_MS );
            }
            else
            {
                fallback_at = now + ( int32_t ) ( app_flrc_get_last_rx_ms( ) - ( uint32_t ) now ) + FALLBACK_MS;
                deadline    = MIN( deadline, fallback_at );
            }
        }
        else
        {
            deadline = MIN( deadline, ctl.phase_end_ms );
        }

        if( deadline > now )
        {
            // Absolute wake-up (no drift); radio callbacks wake it early through app_event_signal()
            ( void ) k_sem_take( &app_event_sem, K_TIMEOUT_ABS_MS( deadline ) );
            continue;  // Re-evaluate with fresh state
        }

        // --- deadline reached ---
        if( now >= ctl.next_stats_ms )
        {
            report( now );
        }

        if( APP_FLRC_IS_TX )
        {
            if( now >= ctl.phase_end_ms )
            {
                if( ctl.phase == PHASE_SPEED )
                {
                    enter_distance( now, now + DISTANCE_MS, 0, "timer" );
                }
                else
                {
                    enter_speed( now );
                }
            }
        }
        else if( ctl.phase == PHASE_SPEED )
        {
            if( notice && ( now >= notice_at - RX_SPEED_EARLY_MS ) )
            {
                ctl.cycle = notice_cycle;
                const int32_t to_switch = MAX( 0, ( int32_t ) ( notice_at - now ) );
                enter_distance( now, notice_at + DISTANCE_MS - RX_DISTANCE_EARLY_MS,
                                ( uint32_t ) to_switch + RX_FIRST_CFG_DELAY_MS, "flag" );
            }
            else if( !notice && ( now >= fallback_at ) )
            {
                // TX silent for FALLBACK_MS: it is in DISTANCE (or off). It has at most DISTANCE_MS - FALLBACK_MS
                // left there, so do not stay longer than that.
                enter_distance( now, now + DISTANCE_MS - FALLBACK_MS, 0, "no FLRC" );
            }
        }
        else if( now >= ctl.phase_end_ms )
        {
            enter_speed( now );
        }
    }

    return 0;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void enter_speed( int64_t now )
{
    if( APP_FLRC_IS_TX )
    {
        ctl.cycle += 1;
    }

    k_thread_priority_set( k_current_get( ), MAIN_CTRL_PRIORITY );
    if( ctl.phase == PHASE_DISTANCE )
    {
        app_ranging_stop( );
    }
    now = k_uptime_get( );
    ctl.phase_start_ms = now;
    ctl.phase_end_ms   = APP_FLRC_IS_TX ? now + SPEED_MS : INT64_MAX;
    app_flrc_start( ( uint32_t ) ctl.phase_end_ms, ctl.cycle );
    ctl.phase = PHASE_SPEED;
    k_thread_priority_set( k_current_get( ), MAIN_RUN_PRIORITY );

    // Fresh 1 s stats windows aligned on the phase start
    app_flrc_stats_t discard;
    app_flrc_get_and_reset_stats( &discard );
    ctl.stats_last_ms = now;
    ctl.next_stats_ms = now + STATS_PERIOD_MS;
    ctl.switches += 1;

    printk( "=== SPEED (FLRC %s) cycle %u, t=%u ms, switch #%u\n", APP_FLRC_IS_TX ? "TX" : "RX", ctl.cycle, ( uint32_t ) now,
            ctl.switches );

    if( APP_FLRC_IS_TX )
    {
        display_text( DISPLAY_ROW1, "SPEED  FLRC TX  #%u", ctl.cycle );
        display_text( DISPLAY_ROW2, "%u MHz %+d dBm %uB", CONFIG_APP_RF_FREQ_MHZ, CONFIG_APP_TX_POWER_DBM,
                      APP_FLRC_PAYLOAD_SIZE );
        display_text( DISPLAY_ROW3, "Pkts/s: -" );
        display_text( DISPLAY_ROW4, "TX: -.-- Mbps" );
        display_text( DISPLAY_ROW5, " " );
        display_text( DISPLAY_ROW6, "-> DIST in %u s", SPEED_MS / 1000 );
    }
    else
    {
        display_text( DISPLAY_ROW1, "SPEED  FLRC RX %u", CONFIG_APP_RF_FREQ_MHZ );
        display_text( DISPLAY_ROW2, "-.-- Mbps" );
        display_text( DISPLAY_ROW3, "waiting" );
        display_text( DISPLAY_ROW4, "2.6 Mbps raw CR1/1 %uB", APP_FLRC_PAYLOAD_SIZE );
    }
}

static void enter_distance( int64_t now, int64_t end_ms, uint32_t first_cfg_delay_ms, const char* reason )
{
    // Last (partial) SPEED stats window
    if( now - ctl.stats_last_ms >= 50 )
    {
        report( now );
    }

    k_thread_priority_set( k_current_get( ), MAIN_CTRL_PRIORITY );
    app_flrc_stop( );
    now = k_uptime_get( );
    ctl.phase_start_ms = now;
    ctl.phase_end_ms   = end_ms;
    app_ranging_start( ( uint32_t ) MAX( 0, end_ms - now ), first_cfg_delay_ms );
    ctl.phase = PHASE_DISTANCE;
    k_thread_priority_set( k_current_get( ), MAIN_RUN_PRIORITY );

    ctl.stats_last_ms = now;
    ctl.next_stats_ms = now + STATS_PERIOD_MS;
    ctl.switches += 1;

    printk( "=== DISTANCE (ranging %s) cycle %u, t=%u ms, %d ms window, reason: %s, switch #%u\n",
            APP_RANGING_IS_MANAGER ? "manager" : "subordinate", ctl.cycle, ( uint32_t ) now, ( int ) ( end_ms - now ), reason, ctl.switches );

    if( APP_FLRC_IS_TX )
    {
        display_text( DISPLAY_ROW1, "DIST  ranging SUB  #%u", ctl.cycle );
        display_text( DISPLAY_ROW2, "LoRa SF%d BW812 2G4 hop", CONFIG_APP_RANGING_SF );
        display_text( DISPLAY_ROW3, "waiting for manager" );
        display_text( DISPLAY_ROW4, " " );
        display_text( DISPLAY_ROW5, " " );
        display_text( DISPLAY_ROW6, "-> SPEED in %u s", DISTANCE_MS / 1000 );
    }
    else
    {
        display_text( DISPLAY_ROW1, "DIST  ranging MGR  #%u", ctl.cycle );
        display_text( DISPLAY_ROW2, "-.- m" );
        display_text( DISPLAY_ROW3, "ranging..." );
        display_text( DISPLAY_ROW4, "SF%d BW812 2G4 hop (%s)", CONFIG_APP_RANGING_SF, reason );
    }
}

static void report( int64_t now )
{
    const uint32_t elapsed_ms = ( uint32_t ) ( now - ctl.stats_last_ms );

    ctl.stats_last_ms = now;
    ctl.next_stats_ms = now + STATS_PERIOD_MS;

    if( ctl.phase == PHASE_SPEED )
    {
        app_flrc_stats_t stats;
        app_flrc_get_and_reset_stats( &stats );
        if( APP_FLRC_IS_TX )
        {
            report_tx_speed( &stats, elapsed_ms, now );
        }
        else
        {
            report_rx_speed( &stats, elapsed_ms );
        }
    }
    else
    {
        report_distance( now );
    }

#if DT_NODE_HAS_STATUS( LED0_NODE, okay )
    gpio_pin_toggle_dt( &usr_led );
#endif
}

/*!
 * @brief Integer kbps from bytes over a period (bits per ms == kbit/s)
 */
static uint32_t to_kbps( uint32_t bytes, uint32_t elapsed_ms )
{
    return ( elapsed_ms == 0 ) ? 0 : ( uint32_t ) ( ( ( uint64_t ) bytes * 8U ) / elapsed_ms );
}

static void report_tx_speed( const app_flrc_stats_t* s, uint32_t elapsed_ms, int64_t now )
{
    const uint32_t kbps    = to_kbps( s->tx_done * APP_FLRC_PAYLOAD_SIZE, elapsed_ms );
    const uint32_t pkt_s   = ( elapsed_ms == 0 ) ? 0 : ( s->tx_done * 1000U ) / elapsed_ms;
    const uint32_t mbps    = kbps / 1000U;
    const uint32_t mbps_cd = ( kbps % 1000U ) / 10U;
    const int64_t  left_ms = MAX( 0, ctl.phase_end_ms - now );

    display_text( DISPLAY_ROW3, "Pkts/s: %u", pkt_s );
    display_text( DISPLAY_ROW4, "TX: %u.%02u Mbps", mbps, mbps_cd );
    display_text( DISPLAY_ROW5, "Seq: %u  err: %u", s->last_seq, s->tx_errors );
    display_text( DISPLAY_ROW6, "-> DIST in %u s", ( uint32_t ) ( ( left_ms + 999 ) / 1000 ) );

    printk( "TX SPEED %u ms: %u pkt (%u pkt/s), %u.%02u Mbps payload, seq %u, err %u, disp %u\n", elapsed_ms,
            s->tx_done, pkt_s, mbps, mbps_cd, s->last_seq, s->tx_errors, display_get_refresh_count( ) );
}

static void report_rx_speed( const app_flrc_stats_t* s, uint32_t elapsed_ms )
{
    const uint32_t kbps    = to_kbps( s->rx_bytes, elapsed_ms );
    const uint32_t mbps    = kbps / 1000U;
    const uint32_t mbps_cd = ( kbps % 1000U ) / 10U;
    uint32_t       ok_x10  = 0;

    display_text( DISPLAY_ROW2, "%u.%02u Mbps", mbps, mbps_cd );

    // Success in 0.1 %: good packets over the sequence span the TX covered in this window
    if( s->rx_expected > 0 )
    {
        ok_x10 = ( uint32_t ) ( ( ( uint64_t ) s->rx_ok * 1000U ) / s->rx_expected );
        display_text( DISPLAY_ROW3, "%u.%u%% pkts", ok_x10 / 10U, ok_x10 % 10U );
    }
    else
    {
        display_text( DISPLAY_ROW3, "no packets" );
    }

    if( s->rssi_valid )
    {
        display_text( DISPLAY_ROW4, "RSSI %d  CRC %u  lost %u", s->last_rssi, s->crc_errors,
                      s->rx_expected - s->rx_ok );
    }

    printk( "RX SPEED %u ms: ok %u / exp %u (%u.%u%%), %u.%02u Mbps goodput, crc %u, tmo %u, resync %u, fifo %u, "
            "seq %u, rssi %d, disp %u\n",
            elapsed_ms, s->rx_ok, s->rx_expected, ok_x10 / 10U, ok_x10 % 10U, mbps, mbps_cd, s->crc_errors,
            s->rx_timeouts, s->resyncs, s->fifo_resets, s->last_seq, s->last_rssi, display_get_refresh_count( ) );
}

static void report_distance( int64_t now )
{
    app_ranging_stats_t st;
    app_ranging_get_stats( &st );
    const int64_t left_ms = MAX( 0, ctl.phase_end_ms - now );

    if( APP_RANGING_IS_MANAGER )
    {
        printk( "RX DIST: cfg %u, ack %u, runs %u, exch ok %u fail %u, %d ms left\n", st.cfg_tx, st.cfg_ok, st.runs,
                st.exch_ok, st.exch_fail, ( int ) left_ms );
    }
    else
    {
        display_text( DISPLAY_ROW3, "cfg ok: %u  runs: %u", st.cfg_ok, st.runs );
        display_text( DISPLAY_ROW4, "answered: %u", st.exch_ok );
        display_text( DISPLAY_ROW5, "missed: %u  cut: %u", st.exch_fail, st.runs_cut );
        display_text( DISPLAY_ROW6, "-> SPEED in %u s", ( uint32_t ) ( ( left_ms + 999 ) / 1000 ) );
        printk( "TX DIST: cfg rx windows %u, cfg ok %u, runs %u (cut %u), answered %u, missed %u, %d ms left\n",
                st.cfg_tx, st.cfg_ok, st.runs, st.runs_cut, st.exch_ok, st.exch_fail, ( int ) left_ms );
    }
}

/* Manager: smoothed distance = median of the last CONFIG_APP_RANGING_SMOOTH_RUNS run results (kept across phases,
 * cleared after SMOOTH_RESET_MS without a result); spread = median absolute deviation of the same values. */
#define SMOOTH_N ( CONFIG_APP_RANGING_SMOOTH_RUNS )
#define SMOOTH_RESET_MS ( 30000 )

static int32_t smooth_cm[SMOOTH_N];
static uint8_t smooth_count;
static uint8_t smooth_next;
static int64_t smooth_last_ms;

static int32_t median_of( int32_t* v, uint8_t n )
{
    for( uint8_t i = 1; i < n; i++ )
    {
        const int32_t x = v[i];
        int8_t        j = ( int8_t ) i - 1;
        while( ( j >= 0 ) && ( v[j] > x ) )
        {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = x;
    }
    return ( ( n % 2 ) == 1 ) ? v[n / 2] : ( v[n / 2 - 1] + v[n / 2] ) / 2;
}

static void smooth_add( int32_t cm, int32_t* med_cm, int32_t* mad_cm, uint8_t* n )
{
    const int64_t now = k_uptime_get( );

    if( ( smooth_count > 0 ) && ( now - smooth_last_ms > SMOOTH_RESET_MS ) )
    {
        smooth_count = 0;
        smooth_next  = 0;
    }
    smooth_last_ms          = now;
    smooth_cm[smooth_next]  = cm;
    smooth_next             = ( smooth_next + 1 ) % SMOOTH_N;
    smooth_count            = MIN( smooth_count + 1, SMOOTH_N );

    int32_t tmp[SMOOTH_N];
    memcpy( tmp, smooth_cm, smooth_count * sizeof( int32_t ) );
    *med_cm = median_of( tmp, smooth_count );
    for( uint8_t i = 0; i < smooth_count; i++ )
    {
        tmp[i] = abs( smooth_cm[i] - *med_cm );
    }
    *mad_cm = median_of( tmp, smooth_count );
    *n      = smooth_count;
}

static void format_dm( char* buf, size_t len, int32_t dm )
{
    const uint32_t mag = ( uint32_t ) abs( dm );
    snprintf( buf, len, "%s%u.%u", ( dm < 0 ) ? "-" : "", mag / 10U, mag % 10U );
}

/* cm -> "x.y" with sign, rounded to dm */
static void format_cm( char* buf, size_t len, int32_t cm )
{
    const int32_t dm = ( cm >= 0 ) ? ( cm + 5 ) / 10 : ( cm - 5 ) / 10;
    format_dm( buf, len, dm );
}

/* Manager: one completed run -> console (per-hop detail) + OLED (smoothed) */
static void show_run( const app_ranging_run_t* run )
{
    char run_m[24];
    char avg_m[24];
    char mad_m[24];

    // Per hop: raw result, "x" = no valid exchange, "*" suffix = dropped by the RSSI filter
    printk( "HOPS run %u raw/rssi:", run->run_id );
    for( uint8_t i = 0; i < run->total; i++ )
    {
        if( !run->hop_valid[i] )
        {
            printk( " x" );
        }
        else
        {
            printk( " %d/%d%s", run->hop_raw[i], run->hop_rssi[i], run->hop_used[i] ? "" : "*" );
        }
    }
    printk( "\n" );

    if( run->used == 0 )
    {
        printk( "RANGE run %u: no valid exchange (%u valid / %u)\n", run->run_id, run->valid, run->total );
        display_text( DISPLAY_ROW4, "run %u: 0/%u valid", run->run_id, run->total );
        return;
    }

    int32_t med_cm;
    int32_t mad_cm;
    uint8_t n;
    smooth_add( run->distance_cm, &med_cm, &mad_cm, &n );

    format_cm( run_m, sizeof( run_m ), run->distance_cm );
    format_cm( avg_m, sizeof( avg_m ), med_cm );
    format_cm( mad_m, sizeof( mad_m ), mad_cm );

    printk( "RANGE run %u: %s m (raw median %d, %d cm uncal), used %u / valid %u / %u, rssi %d | shown %s m +/-%s "
            "(median of %u)\n",
            run->run_id, run_m, run->median_raw, app_ranging_raw_to_cm( run->median_raw ), run->used, run->valid,
            run->total, run->rssi, avg_m, mad_m, n );

    display_text( DISPLAY_ROW2, "%s m", avg_m );
    display_text( DISPLAY_ROW3, "+/-%s  n=%u", mad_m, n );
    display_text( DISPLAY_ROW4, "run %s m %u/%u RSSI %d", run_m, run->used, run->total, run->rssi );
}

/*!
 * @brief Log whether the OLED answers on I2C (diagnostic for a dark screen)
 */
static void check_oled( void )
{
    const struct device* display_dev = DEVICE_DT_GET( DISPLAY_NODE );
    const struct device* i2c_dev     = DEVICE_DT_GET( DT_BUS( DISPLAY_NODE ) );

    if( device_is_ready( display_dev ) )
    {
        LOG_INF( "OLED ready" );
        return;
    }

    LOG_ERR( "OLED driver not ready: probing I2C bus %s", i2c_dev->name );
    if( !device_is_ready( i2c_dev ) )
    {
        LOG_ERR( "I2C bus not ready" );
        return;
    }

    // SSD1306 control byte 0x00 (command stream) with no command: harmless if a device answers
    const uint8_t ctrl = 0x00;
    for( uint16_t addr = 0x3C; addr <= 0x3D; addr++ )
    {
        const int ret = i2c_write( i2c_dev, &ctrl, 1, addr );
        LOG_ERR( "  0x%02X: %s (%d)", addr, ( ret == 0 ) ? "ACK" : "NACK", ret );
    }
}

/* --- EOF ------------------------------------------------------------------ */
