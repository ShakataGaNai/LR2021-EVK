/**
 * @file      main.c
 *
 * @brief     FLRC 2.4 GHz throughput demo: application main
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

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include <smtc_zephyr_usp_api.h>
#include <smtc_sw_platform_helper.h>

#include "app_flrc.h"
#include "oled_display.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

LOG_MODULE_REGISTER( main, LOG_LEVEL_INF );

#define STATS_PERIOD_MS ( 1000 )

/* After init, main only formats stats: run it preemptible, below the USP (radio) thread, so console
 * output (polled UART, ~10 ms per line) never delays the radio IRQ handling. */
#define MAIN_RUN_PRIORITY ( 5 )

#define DISPLAY_NODE DT_CHOSEN( zephyr_display )

#define LED0_NODE DT_ALIAS( led0 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#if DT_NODE_HAS_STATUS( LED0_NODE, okay )
static const struct gpio_dt_spec usr_led = GPIO_DT_SPEC_GET( LED0_NODE, gpios );
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void show_static_rows( void );
static void check_oled( void );
static void report_tx( const app_flrc_stats_t* s, uint32_t elapsed_ms );
static void report_rx( const app_flrc_stats_t* s, uint32_t elapsed_ms );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

int main( void )
{
    LOG_INF( "===== FLRC throughput demo (%s) =====", APP_FLRC_IS_TX ? "TX" : "RX" );

    check_oled( );
    display_thread_init( APP_FLRC_IS_TX ? DISPLAY_LAYOUT_TEXT : DISPLAY_LAYOUT_BIG );
    show_static_rows( );

#if DT_NODE_HAS_STATUS( LED0_NODE, okay )
    if( gpio_is_ready_dt( &usr_led ) )
    {
        gpio_pin_configure_dt( &usr_led, GPIO_OUTPUT_INACTIVE );
    }
#endif

    SMTC_SW_PLATFORM_INIT( );
    SMTC_SW_PLATFORM_VOID( smtc_rac_init( ) );

    app_flrc_init( );

    k_thread_priority_set( k_current_get( ), MAIN_RUN_PRIORITY );

    int64_t last_ms = k_uptime_get( );
    int64_t next_ms = last_ms + STATS_PERIOD_MS;

    while( true )
    {
        // Absolute wake-up: no drift from the time spent formatting
        k_sleep( K_TIMEOUT_ABS_MS( next_ms ) );
        next_ms += STATS_PERIOD_MS;

        app_flrc_stats_t stats;
        app_flrc_get_and_reset_stats( &stats );

        const int64_t  now_ms     = k_uptime_get( );
        const uint32_t elapsed_ms = ( uint32_t ) ( now_ms - last_ms );
        last_ms                   = now_ms;

        if( APP_FLRC_IS_TX )
        {
            report_tx( &stats, elapsed_ms );
        }
        else
        {
            report_rx( &stats, elapsed_ms );
        }

#if DT_NODE_HAS_STATUS( LED0_NODE, okay )
        gpio_pin_toggle_dt( &usr_led );
#endif
    }

    return 0;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/*!
 * @brief Integer kbps from bytes over a period (bits per ms == kbit/s)
 */
static uint32_t to_kbps( uint32_t bytes, uint32_t elapsed_ms )
{
    return ( elapsed_ms == 0 ) ? 0 : ( uint32_t ) ( ( ( uint64_t ) bytes * 8U ) / elapsed_ms );
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

static void show_static_rows( void )
{
    if( APP_FLRC_IS_TX )
    {
        display_text( DISPLAY_ROW1, "FLRC TX  2.4 GHz" );
        display_text( DISPLAY_ROW2, "%u MHz  %+d dBm", CONFIG_APP_RF_FREQ_MHZ, APP_FLRC_TX_POWER_DBM );
        display_text( DISPLAY_ROW3, "2.6 Mbps raw CR1/1 %uB", APP_FLRC_PAYLOAD_SIZE );
        display_text( DISPLAY_ROW4, "Starting..." );
    }
    else
    {
        display_text( DISPLAY_ROW1, "FLRC RX  %u MHz", CONFIG_APP_RF_FREQ_MHZ );
        display_text( DISPLAY_ROW2, "-.-- Mbps" );
        display_text( DISPLAY_ROW3, "waiting" );
        display_text( DISPLAY_ROW4, "2.6 Mbps raw CR1/1 %uB", APP_FLRC_PAYLOAD_SIZE );
    }
}

static void report_tx( const app_flrc_stats_t* s, uint32_t elapsed_ms )
{
    const uint32_t kbps    = to_kbps( s->tx_done * APP_FLRC_PAYLOAD_SIZE, elapsed_ms );
    const uint32_t pkt_s   = ( elapsed_ms == 0 ) ? 0 : ( s->tx_done * 1000U ) / elapsed_ms;
    const uint32_t mbps    = kbps / 1000U;
    const uint32_t mbps_cd = ( kbps % 1000U ) / 10U;

    display_text( DISPLAY_ROW4, "Pkts/s: %u", pkt_s );
    display_text( DISPLAY_ROW5, "TX: %u.%02u Mbps", mbps, mbps_cd );
    display_text( DISPLAY_ROW6, "Seq: %u  err: %u", s->last_seq, s->tx_errors );

    printk( "TX %u ms: %u pkt (%u pkt/s), %u.%02u Mbps payload, seq %u, err %u, disp %u\n", elapsed_ms, s->tx_done,
            pkt_s, mbps, mbps_cd, s->last_seq, s->tx_errors, display_get_refresh_count( ) );
}

static void report_rx( const app_flrc_stats_t* s, uint32_t elapsed_ms )
{
    const uint32_t kbps    = to_kbps( s->rx_bytes, elapsed_ms );
    const uint32_t mbps    = kbps / 1000U;
    const uint32_t mbps_cd = ( kbps % 1000U ) / 10U;

    display_text( DISPLAY_ROW2, "%u.%02u Mbps", mbps, mbps_cd );

    // Success in 0.1 %: good packets over the sequence span the TX covered in this window
    if( s->rx_expected > 0 )
    {
        const uint32_t ok_x10 = ( uint32_t ) ( ( ( uint64_t ) s->rx_ok * 1000U ) / s->rx_expected );
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

    printk( "RX %u ms: ok %u / exp %u, %u.%02u Mbps goodput, crc %u, tmo %u, resync %u, fifo %u, seq %u, rssi %d, disp %u\n",
            elapsed_ms, s->rx_ok, s->rx_expected, mbps, mbps_cd, s->crc_errors, s->rx_timeouts, s->resyncs,
            s->fifo_resets, s->last_seq, s->last_rssi, display_get_refresh_count( ) );
}

/* --- EOF ------------------------------------------------------------------ */
