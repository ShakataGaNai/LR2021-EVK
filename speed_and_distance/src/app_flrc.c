/**
 * @file      app_flrc.c
 *
 * @brief     One-way FLRC throughput (SPEED phase): TX streams, RX measures goodput / success rate
 *
 * Copied from flrc_throughput/src/app_flrc.c, with start/stop added so the radio can be handed back to the
 * RAC scheduler for ranging between SPEED phases, and a mode-switch flag in the payload header.
 *
 * Derived from usp examples/main_examples/packet_error_rate_flrc_example (app_per_flrc.c):
 * same FLRC parameters, but the RAC scheduler is bypassed for the data path: the app takes the
 * radio with smtc_rac_immediate_radio_access() (as in the immediate_radio_access example) and
 * drives the LR20xx directly. The radio is configured once; then
 *   - TX: SetTx on each TX_DONE, with the next payload already pre-loaded in the 1024-byte TX FIFO;
 *   - RX: continuous RX (radio stays in RX), FIFO drained on each RX_DONE.
 * The per-packet RAC path (setup + ms scheduler) cost ~3 ms/packet and capped goodput at ~0.8 Mbps.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2024. All rights reserved.
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

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "app_flrc.h"
#include "app_events.h"
#include "app_radio_limits.h"

#include "smtc_rac_api.h"
#include "smtc_modem_hal.h"

#include "ral_lr20xx.h"
#include "lr20xx_system.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/* FLRC radio parameters. Enum names from usp smtc_rac_lib/smtc_ral/src/ral_defs.h */
#define FLRC_RAW_BIT_RATE RAL_FLRC_RAW_BIT_RATE_2_600_MBPS  // Highest FLRC rate (LR20XX_RADIO_FLRC_BR_2_600_BW_2_666)
#define FLRC_CR RAL_FLRC_CR_1_1                              // No FEC (LR20XX_RADIO_FLRC_CR_NONE)
#define FLRC_PULSE_SHAPE RAL_FLRC_PULSE_SHAPE_BT_05
#define FLRC_PREAMBLE_BITS RAL_FLRC_PREAMBLE_LENGTH_32_BITS
#define FLRC_SYNCWORD_LEN RAL_FLRC_SYNCWORD_LENGTH_4_BYTES
#define FLRC_TX_SYNCWORD RAL_FLRC_TX_SYNCWORD_1
#define FLRC_MATCH_SYNCWORD RAL_FLRC_RX_MATCH_SYNCWORD_1
#define FLRC_PLD_IS_FIX false  // Variable length (header carries the length)
#define FLRC_CRC RAL_FLRC_CRC_2_BYTES

#define SEQ_SIZE ( 4 )
#define SEQ_OFFSET ( 0 )
#define FLAGS_OFFSET ( 4 )
#define SWITCH_MS_OFFSET ( 5 )
#define CYCLE_OFFSET ( 7 )

APP_ASSERT_FREQ_IN_ISM_2G4( APP_FLRC_RF_FREQ_IN_HZ );
BUILD_ASSERT( APP_FLRC_PAYLOAD_SIZE >= APP_FLRC_HDR_SIZE, "payload too small for the header" );

/* LR20xx FIFOs are switched to 1024 bytes by ral_lr20xx_init() (LR20XX_FIFO_SIZE_INCREASE_WORKAROUND_ENABLE) */
#define RADIO_FIFO_SIZE ( 1024 )

/* RAL and LR20xx driver statuses both use 0 for OK */
#define PANIC_IF_ERR( x ) SMTC_MODEM_HAL_PANIC_ON_FAILURE( ( int ) ( x ) == 0 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct flrc_throughput_s
{
    uint8_t          tx_payload[APP_FLRC_PAYLOAD_SIZE];
    uint8_t          rx_payload[RADIO_FIFO_SIZE];
    void*            radio;       //!< LR20xx driver context
    bool             tx_pipeline; //!< Next TX payload is pre-loaded in the FIFO
    uint32_t         tx_seq;      //!< Sequence number of the packet on air
    uint32_t         rx_seq;      //!< Last sequence number received
    bool             rx_synced;   //!< rx_seq is valid
    volatile bool    running;     //!< FLRC owns the radio (immediate access held)
    uint32_t         switch_at_ms;  //!< TX: end of this SPEED phase (k_uptime_get_32)
    uint8_t          cycle;         //!< TX: cycle counter for the header
    volatile bool     notice_valid;   //!< RX: switch flag seen in this phase
    volatile uint32_t notice_at_ms;   //!< RX: TX switch time derived from the flag packet
    volatile uint8_t  notice_cycle;   //!< RX: cycle counter of the flag packet
    volatile uint32_t last_rx_ms;     //!< RX: time of the last good packet
    app_flrc_stats_t stats;
} flrc_throughput_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static flrc_throughput_t flrc = { 0 };

static const uint8_t default_syncword_1[4] = { 0x90, 0x56, 0x34, 0x12 };
static const uint8_t default_syncword_2[4] = { 0x00, 0x00, 0x00, 0x00 };
static const uint8_t default_syncword_3[4] = { 0x00, 0x00, 0x00, 0x00 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void radio_configure( void );
static void irq_callback( void );
static void tx_load_payload( uint32_t seq );
static void rx_drain_packet( bool crc_ok );
static void rx_account_packet( uint32_t seq, uint16_t size, int16_t rssi );
static void rx_check_switch_flag( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void app_flrc_init( void )
{
    // Filler: incremental byte pattern after the header (as app_per_flrc.c)
    for( uint16_t i = APP_FLRC_HDR_SIZE; i < APP_FLRC_PAYLOAD_SIZE; i++ )
    {
        flrc.tx_payload[i] = ( uint8_t ) i;
    }

    // Pre-loading needs room for two packets in the TX FIFO
    flrc.tx_pipeline = IS_ENABLED( CONFIG_APP_TX_PIPELINE ) && ( 2 * APP_FLRC_PAYLOAD_SIZE <= RADIO_FIFO_SIZE );

    printk( "FLRC %s: %u Hz, %d dBm, %s, %s, %u B payload, CRC 2 B, direct radio access%s\n",
            APP_FLRC_IS_TX ? "TX" : "RX", ( unsigned int ) APP_FLRC_RF_FREQ_IN_HZ, APP_FLRC_TX_POWER_DBM,
            ral_flrc_raw_bit_rate_to_str( FLRC_RAW_BIT_RATE ), ral_flrc_cr_to_str( FLRC_CR ), APP_FLRC_PAYLOAD_SIZE,
            ( APP_FLRC_IS_TX && flrc.tx_pipeline ) ? ", TX FIFO pre-load" : "" );
}

void app_flrc_start( uint32_t switch_at_ms, uint8_t cycle )
{
    flrc.switch_at_ms = switch_at_ms;
    flrc.cycle        = cycle;
    flrc.notice_valid = false;
    flrc.rx_synced    = false;  // Packets missed during the DISTANCE phase are not losses
    flrc.last_rx_ms   = k_uptime_get_32( );

    // Take the radio away from the RAC scheduler. irq_callback() then runs in the USP thread on each radio IRQ.
    // If a RAC task is still running, the planner aborts it and puts the radio in standby first.
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( smtc_rac_immediate_radio_access( RAC_VERY_HIGH_PRIORITY, irq_callback ) ==
                                     SMTC_RAC_SUCCESS );
    flrc.radio = smtc_rac_get_radio_driver_context( );

    radio_configure( );
    flrc.running = true;

    if( APP_FLRC_IS_TX )
    {
        PANIC_IF_ERR( lr20xx_radio_fifo_clear_tx( flrc.radio ) );
        tx_load_payload( flrc.tx_seq );
        PANIC_IF_ERR( lr20xx_radio_common_set_tx( flrc.radio, 0 ) );
        if( flrc.tx_pipeline )
        {
            // Queue packet #1 behind packet #0 while #0 is on air
            tx_load_payload( flrc.tx_seq + 1 );
        }
    }
    else
    {
        // Continuous RX: the radio stays in RX after each packet (ral_lr20xx_set_rx() clears the RX FIFO)
        PANIC_IF_ERR( ral_lr20xx_set_rx( flrc.radio, RAL_RX_TIMEOUT_CONTINUOUS_MODE ) );
    }
}

void app_flrc_stop( void )
{
    // irq_callback() checks this first: no new SetTx / FIFO access from now on
    flrc.running = false;

    PANIC_IF_ERR( ral_lr20xx_set_standby( flrc.radio, RAL_STANDBY_CFG_XOSC ) );
    PANIC_IF_ERR( ral_lr20xx_clear_irq_status( flrc.radio, RAL_IRQ_ALL ) );
    PANIC_IF_ERR( lr20xx_radio_fifo_clear_tx( flrc.radio ) );
    PANIC_IF_ERR( lr20xx_radio_fifo_clear_rx( flrc.radio ) );
    // RAC never sets the fallback mode: put back the chip default we changed for FLRC TX
    PANIC_IF_ERR( ral_lr20xx_set_rx_tx_fallback_mode( flrc.radio, RAL_FALLBACK_STDBY_RC ) );

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( smtc_rac_release_immediate_radio_access( ) == SMTC_RAC_SUCCESS );
}

bool app_flrc_get_switch_notice( uint32_t* switch_at_ms, uint8_t* cycle )
{
    unsigned int key = irq_lock( );
    const bool   ok  = flrc.notice_valid;
    *switch_at_ms    = flrc.notice_at_ms;
    *cycle           = flrc.notice_cycle;
    irq_unlock( key );
    return ok;
}

uint32_t app_flrc_get_last_rx_ms( void )
{
    return flrc.last_rx_ms;
}

void app_flrc_get_and_reset_stats( app_flrc_stats_t* stats )
{
    // Counters are written by the radio IRQ callback: copy + reset under lock
    unsigned int key = irq_lock( );

    *stats = flrc.stats;

    flrc.stats.tx_done     = 0;
    flrc.stats.tx_errors   = 0;
    flrc.stats.rx_ok       = 0;
    flrc.stats.rx_bytes    = 0;
    flrc.stats.rx_expected = 0;
    flrc.stats.crc_errors  = 0;
    flrc.stats.rx_timeouts = 0;
    flrc.stats.resyncs     = 0;
    flrc.stats.fifo_resets = 0;

    irq_unlock( key );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* FLRC setup at the start of each SPEED phase: same sequence as ralf_lr20xx_setup_flrc(), called on the driver context */
static void radio_configure( void )
{
    const ral_flrc_mod_params_t mod_params = {
        .raw_bit_rate = FLRC_RAW_BIT_RATE,
        .cr           = FLRC_CR,
        .pulse_shape  = FLRC_PULSE_SHAPE,
    };
    const ral_flrc_pkt_params_t pkt_params = {
        .preamble_len     = FLRC_PREAMBLE_BITS,
        .sync_word_len    = FLRC_SYNCWORD_LEN,
        .tx_syncword      = FLRC_TX_SYNCWORD,
        .match_sync_word  = FLRC_MATCH_SYNCWORD,
        .pld_is_fix       = FLRC_PLD_IS_FIX,
        .pld_len_in_bytes = APP_FLRC_PAYLOAD_SIZE,  // TX length / RX max length
        .crc_type         = FLRC_CRC,
    };

    PANIC_IF_ERR( ral_lr20xx_set_standby( flrc.radio, RAL_STANDBY_CFG_XOSC ) );
    PANIC_IF_ERR( ral_lr20xx_set_pkt_type( flrc.radio, RAL_PKT_TYPE_FLRC ) );
    // >= 1.5 GHz: the LR20xx BSP selects the HF RX path (and below, the HF PA). Never the sub-GHz path here.
    app_radio_check_freq( APP_FLRC_RF_FREQ_IN_HZ );
    PANIC_IF_ERR( ral_lr20xx_set_rf_freq( flrc.radio, APP_FLRC_RF_FREQ_IN_HZ ) );
    PANIC_IF_ERR( ral_lr20xx_set_flrc_mod_params( flrc.radio, &mod_params ) );
    PANIC_IF_ERR( ral_lr20xx_set_flrc_pkt_params( flrc.radio, &pkt_params ) );
    PANIC_IF_ERR( ral_lr20xx_set_flrc_crc_params( flrc.radio, 0, 0 ) );  // Keep chip default CRC seed/polynomial

    if( APP_FLRC_IS_TX )
    {
        PANIC_IF_ERR( ral_lr20xx_set_tx_cfg( flrc.radio, APP_FLRC_TX_POWER_DBM, APP_FLRC_RF_FREQ_IN_HZ ) );
        PANIC_IF_ERR( ral_lr20xx_set_flrc_sync_word( flrc.radio, FLRC_TX_SYNCWORD, default_syncword_1,
                                                     FLRC_SYNCWORD_LEN ) );
        // Stay in FS between packets: shortest turnaround to the next SetTx
        PANIC_IF_ERR( ral_lr20xx_set_rx_tx_fallback_mode( flrc.radio, RAL_FALLBACK_FS ) );
        PANIC_IF_ERR( ral_lr20xx_set_dio_irq_params( flrc.radio, RAL_IRQ_TX_DONE ) );
    }
    else
    {
        PANIC_IF_ERR( ral_lr20xx_set_flrc_sync_word( flrc.radio, 1, default_syncword_1, FLRC_SYNCWORD_LEN ) );
        PANIC_IF_ERR( ral_lr20xx_set_flrc_sync_word( flrc.radio, 2, default_syncword_2, FLRC_SYNCWORD_LEN ) );
        PANIC_IF_ERR( ral_lr20xx_set_flrc_sync_word( flrc.radio, 3, default_syncword_3, FLRC_SYNCWORD_LEN ) );
        PANIC_IF_ERR( ral_lr20xx_set_dio_irq_params(
            flrc.radio, RAL_IRQ_RX_DONE | RAL_IRQ_RX_CRC_ERROR | RAL_IRQ_RX_LEN_ERROR | RAL_IRQ_RX_TIMEOUT ) );
    }

    PANIC_IF_ERR( ral_lr20xx_clear_irq_status( flrc.radio, RAL_IRQ_ALL ) );
}

/* Runs in the USP thread on each radio IRQ: keep it short, never touch LVGL / I2C here */
static void irq_callback( void )
{
    lr20xx_system_irq_mask_t irq = LR20XX_SYSTEM_IRQ_NONE;

    if( !flrc.running )
    {
        return;
    }

    lr20xx_system_get_and_clear_irq_status( flrc.radio, &irq );

    if( APP_FLRC_IS_TX )
    {
        if( ( irq & LR20XX_SYSTEM_IRQ_TX_DONE ) != 0 )
        {
            flrc.stats.tx_done += 1;
            flrc.stats.last_seq = flrc.tx_seq;
            flrc.tx_seq += 1;

            if( flrc.tx_pipeline )
            {
                // Next payload is already in the FIFO: start it first, then queue the one after it
                lr20xx_radio_common_set_tx( flrc.radio, 0 );
                tx_load_payload( flrc.tx_seq + 1 );
            }
            else
            {
                tx_load_payload( flrc.tx_seq );
                lr20xx_radio_common_set_tx( flrc.radio, 0 );
            }
        }
        else if( irq != LR20XX_SYSTEM_IRQ_NONE )
        {
            flrc.stats.tx_errors += 1;
        }
        return;
    }

    // RX: the radio is still in continuous RX, only the FIFO needs draining
    if( ( irq & ( LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR ) ) != 0 )
    {
        flrc.stats.crc_errors += 1;
        rx_drain_packet( false );
    }
    else if( ( irq & LR20XX_SYSTEM_IRQ_RX_DONE ) != 0 )
    {
        rx_drain_packet( true );
    }

    if( ( irq & LR20XX_SYSTEM_IRQ_TIMEOUT ) != 0 )
    {
        // Should not happen in continuous mode: re-arm anyway
        flrc.stats.rx_timeouts += 1;
        ral_lr20xx_set_rx( flrc.radio, RAL_RX_TIMEOUT_CONTINUOUS_MODE );
    }
}

static void tx_load_payload( uint32_t seq )
{
    // Header: time left in this SPEED phase, flagged during the last CONFIG_APP_SWITCH_WARN_MS
    const int32_t left_ms  = ( int32_t ) ( flrc.switch_at_ms - k_uptime_get_32( ) );
    const bool    flag     = ( left_ms <= CONFIG_APP_SWITCH_WARN_MS );
    const uint16_t left_16 = ( uint16_t ) CLAMP( left_ms, 0, UINT16_MAX );

    ( void ) memcpy( flrc.tx_payload + SEQ_OFFSET, &seq, SEQ_SIZE );
    flrc.tx_payload[FLAGS_OFFSET] = flag ? APP_FLRC_FLAG_SWITCH : 0;
    ( void ) memcpy( flrc.tx_payload + SWITCH_MS_OFFSET, &left_16, sizeof( left_16 ) );
    flrc.tx_payload[CYCLE_OFFSET] = flrc.cycle;
    lr20xx_radio_fifo_write_tx( flrc.radio, flrc.tx_payload, APP_FLRC_PAYLOAD_SIZE );
}

static void rx_drain_packet( bool crc_ok )
{
    uint16_t pkt_len = 0;
    uint16_t level   = 0;

    lr20xx_radio_common_get_rx_packet_length( flrc.radio, &pkt_len );
    lr20xx_radio_fifo_get_rx_level( flrc.radio, &level );

    if( ( pkt_len == 0 ) || ( pkt_len > level ) || ( pkt_len > sizeof( flrc.rx_payload ) ) )
    {
        // FIFO out of step (bad length / overflow): drop everything and resync on the next packet.
        // Clearing may cut a packet being received right now; it then shows up as one lost packet.
        lr20xx_radio_fifo_clear_rx( flrc.radio );
        flrc.stats.fifo_resets += 1;
        return;
    }

    lr20xx_radio_fifo_read_rx( flrc.radio, flrc.rx_payload, pkt_len );

    if( crc_ok && ( pkt_len >= SEQ_SIZE ) )
    {
        ral_flrc_rx_pkt_status_t pkt_status = { 0 };
        ral_lr20xx_get_flrc_rx_pkt_status( flrc.radio, &pkt_status );

        uint32_t seq = 0;
        ( void ) memcpy( &seq, flrc.rx_payload + SEQ_OFFSET, SEQ_SIZE );
        rx_account_packet( seq, pkt_len, pkt_status.rssi_sync_in_dbm );
        if( pkt_len >= APP_FLRC_HDR_SIZE )
        {
            rx_check_switch_flag( );
        }
    }
}

static void rx_account_packet( uint32_t seq, uint16_t size, int16_t rssi )
{
    if( flrc.rx_synced == false )
    {
        flrc.stats.rx_expected += 1;
    }
    else
    {
        // Signed difference handles 32-bit wrap
        const int32_t gap = ( int32_t ) ( seq - flrc.rx_seq );

        if( ( gap > 0 ) && ( gap <= CONFIG_APP_SEQ_RESYNC_GAP ) )
        {
            // gap - 1 packets were lost (bad CRC or missed)
            flrc.stats.rx_expected += ( uint32_t ) gap;
        }
        else
        {
            // Backward jump or huge jump: TX restarted. Re-sync without counting losses.
            flrc.stats.rx_expected += 1;
            flrc.stats.resyncs += 1;
        }
    }

    flrc.rx_synced = true;
    flrc.rx_seq    = seq;

    flrc.stats.rx_ok += 1;
    flrc.stats.rx_bytes += size;
    flrc.stats.last_seq   = seq;
    flrc.stats.last_rssi  = rssi;
    flrc.stats.rssi_valid = true;
    flrc.last_rx_ms       = k_uptime_get_32( );
}

/* RX: TX announced the end of its SPEED phase. Keep the latest estimate; wake the main thread once. */
static void rx_check_switch_flag( void )
{
    if( ( flrc.rx_payload[FLAGS_OFFSET] & APP_FLRC_FLAG_SWITCH ) == 0 )
    {
        return;
    }

    uint16_t left_ms = 0;
    ( void ) memcpy( &left_ms, flrc.rx_payload + SWITCH_MS_OFFSET, sizeof( left_ms ) );

    flrc.notice_at_ms = k_uptime_get_32( ) + left_ms;
    flrc.notice_cycle = flrc.rx_payload[CYCLE_OFFSET];
    if( !flrc.notice_valid )
    {
        flrc.notice_valid = true;
        app_event_signal( );
    }
}

/* --- EOF ------------------------------------------------------------------ */
