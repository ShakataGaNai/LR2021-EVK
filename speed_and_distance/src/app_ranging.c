/**
 * @file      app_ranging.c
 *
 * @brief     LoRa RTToF ranging with 2.4 GHz frequency hopping (DISTANCE phase)
 *
 * Derived from usp examples/main_examples/ranging_demo/app_ranging_hopping.c: same config / ack handshake,
 * slot timing, stock "2G4" hop table and calibration (delay indicator) values. Changes: 2.4 GHz only (every
 * frequency checked at build time and before each submit), SF5 / 812 kHz, +5 dBm, short ack timeout and fast
 * config retries, runs bounded by the DISTANCE window, subordinate gives up a run after repeated misses, distance
 * computed from the raw result (0.045 m/step) instead of integer metres, results handed to the main thread
 * (no OLED / printk from the RAC callback), LED and multi-data-rate code removed.
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "app_ranging.h"
#include "app_events.h"
#include "app_radio_limits.h"

#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/* Ranging address shared by both boards (RANGING_ADDR_1 in the stock demo) */
#define RANGING_ADDR ( 0x32101222UL )

/* LoRa settings for the config / ack packets and the RTToF exchanges (stock apps_configuration.h values, except
 * frequency, power, SF and BW). Preamble 12 and standard IQ are what the calibration tables assume. */
#define RANGING_CFG_FREQ_HZ ( ( uint32_t ) CONFIG_APP_RANGING_CFG_FREQ_MHZ * 1000000UL )
#define RANGING_TX_POWER_DBM ( CONFIG_APP_TX_POWER_DBM )
#define RANGING_SF ( ( ral_lora_sf_t ) CONFIG_APP_RANGING_SF )
#define RANGING_BW RAL_LORA_BW_800_KHZ  // LR20XX_RADIO_LORA_BW_812
#define RANGING_BW_HZ ( 812000LL )      // lr20xx_radio_lora_get_bw_in_hz( LR20XX_RADIO_LORA_BW_812 )
#define RANGING_BW_KHZ_FOR_SLOT ( 800U )  // The stock slot formula uses 800 for this BW
#define RANGING_CR RAL_LORA_CR_4_5
#define RANGING_PREAMBLE_SYMB ( 12 )
#define RANGING_RESPONSE_SYMBOLS ( 15 )

#define CFG_PAYLOAD_LEN ( 7 )  // 4 B address, first channel, channel count, spare (ack: RSSI)

/* Slot = 64.25 symbols + 20 ms processing (stock RANGING_ALL_SYMBOL / RANGING_DONE_PROCESSING_TIME) */
#define SLOT_SYMBOLS_X100 ( 6425U )
#define SLOT_PROCESSING_MS ( 20U )

#define ACK_TIMEOUT_MS ( 50U )            // Manager: wait for the ack (stock: 3000 ms)
#define CFG_RETRY_DELAY_MS ( 10U )        // Manager: config retry after no ack (stock: 5000 ms)
#define SUB_CFG_RX_TIMEOUT_MS ( 1000U )   // Subordinate: config RX window, re-opened until stopped (stock: 64 s)
#define SUB_MAX_MISSES ( 6U )             // Subordinate: give up a run after this many misses in a row
#define RUN_MARGIN_MS ( 30U )
#define STOP_WAIT_MS ( 100 )

/* Stock "2G4" hop table (app_ranging_hopping.c, RF_FREQ_IN_HZ >= 2 GHz), 2402..2480 MHz. Each entry is checked at
 * compile time to lie inside the 2.4 GHz ISM band. */
#define HOP_TABLE_2G4( X )                                                                                       \
    X( 2450000000 ) X( 2402000000 ) X( 2476000000 ) X( 2436000000 ) X( 2430000000 ) X( 2468000000 )              \
    X( 2458000000 ) X( 2416000000 ) X( 2422000000 ) X( 2442000000 ) X( 2460000000 ) X( 2474000000 )              \
    X( 2414000000 ) X( 2464000000 ) X( 2454000000 ) X( 2444000000 ) X( 2404000000 ) X( 2434000000 )              \
    X( 2410000000 ) X( 2408000000 ) X( 2440000000 ) X( 2452000000 ) X( 2480000000 ) X( 2426000000 )              \
    X( 2428000000 ) X( 2466000000 ) X( 2418000000 ) X( 2412000000 ) X( 2406000000 )

#define HOP_ENTRY( f ) ( uint32_t )( f##UL ),
#define HOP_CHECK( f ) APP_ASSERT_FREQ_IN_ISM_2G4( f##ULL );

static const uint32_t hop_table[] = { HOP_TABLE_2G4( HOP_ENTRY ) };

HOP_TABLE_2G4( HOP_CHECK )
APP_ASSERT_FREQ_IN_ISM_2G4( RANGING_CFG_FREQ_HZ );
BUILD_ASSERT( ARRAY_SIZE( hop_table ) == APP_RANGING_CHANNELS, "hop table size" );
BUILD_ASSERT( RANGING_TX_POWER_DBM <= APP_TX_POWER_MAX_DBM, "ranging TX power above +12 dBm" );

/* Delay indicator for BW812 above 2 GHz, SF5..SF10 (rttof_delay_indicator_table_above_2ghz in the stock demo;
 * its SF11/SF12 columns look like placeholders and are not used). */
static const uint32_t rttof_delay_indicator_bw812_above_2ghz[6] = { 18562, 18632, 18716, 18792, 18971, 19344 };

BUILD_ASSERT( ( CONFIG_APP_RANGING_SF >= 5 ) && ( CONFIG_APP_RANGING_SF <= 10 ), "no calibration for this SF" );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum
{
    ST_IDLE = 0,
    ST_CONFIG,  //!< Manager: config TX / ack RX. Subordinate: config RX.
    ST_ACK,     //!< Subordinate: ack TX
    ST_RUN,     //!< RTToF exchanges over the hop table
} rng_state_t;

typedef struct
{
    uint8_t             id;   //!< RAC radio access id
    smtc_rac_context_t* rac;  //!< RAC context of that id
    uint8_t             payload[CFG_PAYLOAD_LEN];

    volatile bool  active;        //!< DISTANCE phase running: callbacks may submit
    volatile bool  task_pending;  //!< A submitted task has not called back yet
    rng_state_t    state;
    uint32_t       end_ms;        //!< Manager: no run may start unless it ends before this
    uint8_t        channel;
    uint8_t        misses;        //!< Subordinate: consecutive missed exchanges
    uint32_t       slot_ms;

    bool    ex_valid[APP_RANGING_CHANNELS];
    int32_t ex_raw[APP_RANGING_CHANNELS];
    int8_t  ex_rssi[APP_RANGING_CHANNELS];

    app_ranging_stats_t stats;
    app_ranging_run_t   latest;
    bool                latest_new;
    uint32_t            run_count;
    uint32_t            submit_errors;
} ranging_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static ranging_t rng = { 0 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void ranging_callback( rp_status_t status );
static void manager_callback( rp_status_t status );
static void subordinate_callback( rp_status_t status );
static void manager_send_config( uint32_t delay_ms );
static void subordinate_listen( void );
static void set_freq( uint32_t freq_hz );
static void submit( void );
static void manager_finish_run( void );
static uint32_t run_duration_ms( void );
static bool cfg_address_ok( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void app_ranging_init( void )
{
    rng.id  = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_HIGH_PRIORITY ) );
    rng.rac = smtc_rac_get_context( rng.id );

    rng.slot_ms = app_ranging_slot_ms( );

    smtc_rac_context_t* rac = rng.rac;

    rac->scheduler_config.callback_post_radio_transaction = ranging_callback;
    rac->scheduler_config.callback_pre_radio_transaction  = NULL;

    rac->modulation_type                        = SMTC_RAC_MODULATION_LORA;
    rac->radio_params.lora.frequency_in_hz      = RANGING_CFG_FREQ_HZ;
    rac->radio_params.lora.tx_power_in_dbm      = RANGING_TX_POWER_DBM;
    rac->radio_params.lora.preamble_len_in_symb = RANGING_PREAMBLE_SYMB;
    rac->radio_params.lora.header_type          = RAL_LORA_PKT_EXPLICIT;
    rac->radio_params.lora.invert_iq_is_on      = false;
    rac->radio_params.lora.crc_is_on            = true;
    rac->radio_params.lora.sync_word            = LORA_PRIVATE_NETWORK_SYNCWORD;
    rac->radio_params.lora.sf                   = RANGING_SF;
    rac->radio_params.lora.bw                   = RANGING_BW;
    rac->radio_params.lora.cr                   = RANGING_CR;

    rac->radio_params.lora.is_ranging_exchange          = false;
    rac->radio_params.lora.rttof.request_address        = RANGING_ADDR;
    rac->radio_params.lora.rttof.response_symbols_count = RANGING_RESPONSE_SYMBOLS;
    rac->radio_params.lora.rttof.bw_ranging             = RANGING_BW;
    rac->radio_params.lora.rttof.delay_indicator = rttof_delay_indicator_bw812_above_2ghz[CONFIG_APP_RANGING_SF - 5];

    rac->smtc_rac_data_buffer_setup.tx_payload_buffer         = rng.payload;
    rac->smtc_rac_data_buffer_setup.rx_payload_buffer         = rng.payload;
    rac->smtc_rac_data_buffer_setup.size_of_tx_payload_buffer = sizeof( rng.payload );
    rac->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = sizeof( rng.payload );
    rac->radio_params.lora.max_rx_size                        = sizeof( rng.payload );
    rac->radio_params.lora.tx_size                            = sizeof( rng.payload );

    printk( "Ranging %s: LoRa SF%d BW812 CR4/5, cfg %u Hz, 2G4 hop table %u ch (%u..%u MHz), %d dBm, slot %u ms, "
            "delay indicator %u\n",
            APP_RANGING_IS_MANAGER ? "manager" : "subordinate", CONFIG_APP_RANGING_SF,
            ( unsigned int ) RANGING_CFG_FREQ_HZ, APP_RANGING_CHANNELS, 2402U, 2480U, RANGING_TX_POWER_DBM,
            rng.slot_ms, ( unsigned int ) rac->radio_params.lora.rttof.delay_indicator );
}

uint32_t app_ranging_slot_ms( void )
{
    // Same truncation as the stock ( uint16_t )( symbol_ms * 64.25f ) + 20
    const uint32_t symbols_ms = ( ( 1U << CONFIG_APP_RANGING_SF ) * SLOT_SYMBOLS_X100 ) / ( RANGING_BW_KHZ_FOR_SLOT * 100U );
    return symbols_ms + SLOT_PROCESSING_MS + CONFIG_APP_RANGING_EXTRA_DELAY_MS;
}

void app_ranging_start( uint32_t window_ms, uint32_t first_delay_ms )
{
    memset( &rng.stats, 0, sizeof( rng.stats ) );
    rng.task_pending = false;
    rng.end_ms       = smtc_modem_hal_get_time_in_ms( ) + window_ms;
    rng.active       = true;

    if( APP_RANGING_IS_MANAGER )
    {
        manager_send_config( first_delay_ms );
    }
    else
    {
        subordinate_listen( );
    }
}

void app_ranging_stop( void )
{
    rng.active = false;

    if( rng.task_pending )
    {
        // Queued task: aborted and called back at once. Running task: fake IRQ, called back by the USP thread.
        ( void ) smtc_rac_abort_radio_submit( rng.id );
        for( int i = 0; ( i < STOP_WAIT_MS ) && rng.task_pending; i++ )
        {
            k_msleep( 1 );
        }
        if( rng.task_pending )
        {
            printk( "Ranging stop: no callback after %d ms\n", STOP_WAIT_MS );
            rng.task_pending = false;
        }
    }
    rng.state = ST_IDLE;
}

bool app_ranging_get_run( app_ranging_run_t* run )
{
    unsigned int key = irq_lock( );
    const bool   is_new = rng.latest_new;
    *run                = rng.latest;
    rng.latest_new      = false;
    irq_unlock( key );
    return is_new;
}

void app_ranging_get_stats( app_ranging_stats_t* stats )
{
    unsigned int key = irq_lock( );
    *stats           = rng.stats;
    irq_unlock( key );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* RAC post-transaction callback: runs in the USP thread. No LVGL, I2C or printk here. */
static void ranging_callback( rp_status_t status )
{
    rng.task_pending = false;

    if( !rng.active )
    {
        rng.state = ST_IDLE;
        return;
    }

    if( APP_RANGING_IS_MANAGER )
    {
        manager_callback( status );
    }
    else
    {
        subordinate_callback( status );
    }
}

static void manager_callback( rp_status_t status )
{
    smtc_rac_context_t* rac        = rng.rac;
    const uint32_t      radio_end  = rac->smtc_rac_data_result.radio_end_timestamp_ms;

    switch( rng.state )
    {
    case ST_CONFIG:
        if( status == RP_STATUS_TX_DONE )
        {
            // Config sent: wait for the subordinate's ack
            rac->radio_params.lora.is_tx               = false;
            rac->radio_params.lora.is_ranging_exchange = false;
            rac->radio_params.lora.rx_timeout_ms       = ACK_TIMEOUT_MS;
            rac->scheduler_config.scheduling           = SMTC_RAC_ASAP_TRANSACTION;
            rac->scheduler_config.start_time_ms        = smtc_modem_hal_get_time_in_ms( );
            submit( );
        }
        else if( ( status == RP_STATUS_RX_PACKET ) && cfg_address_ok( ) )
        {
            // Ack received: first exchange one slot after it (stock timing)
            rng.stats.cfg_ok += 1;
            rng.state   = ST_RUN;
            rng.channel = 0;
            memset( rng.ex_valid, 0, sizeof( rng.ex_valid ) );

            rac->radio_params.lora.is_tx               = true;
            rac->radio_params.lora.is_ranging_exchange = true;
            rac->radio_params.lora.rx_timeout_ms       = rng.slot_ms >> 1;
            rac->scheduler_config.scheduling           = SMTC_RAC_SCHEDULED_TRANSACTION;
            rac->scheduler_config.start_time_ms        = radio_end + rng.slot_ms;
            set_freq( hop_table[0] );
            submit( );
        }
        else
        {
            // No ack (subordinate not listening yet, or still in FLRC): retry soon
            manager_send_config( CFG_RETRY_DELAY_MS );
        }
        break;

    case ST_RUN:
        if( rng.channel < APP_RANGING_CHANNELS )
        {
            if( status == RP_STATUS_RTTOF_EXCH_VALID )
            {
                // Read the result before the next submit clears smtc_rac_data_result
                int32_t raw = ( int32_t ) rac->smtc_rac_data_result.ranging_result.raw_distance;
                if( raw >= ( int32_t ) ( 1 << 23 ) )
                {
                    raw -= ( int32_t ) ( 1 << 24 );  // Signed 24-bit result
                }
                rng.ex_valid[rng.channel] = true;
                rng.ex_raw[rng.channel]   = raw;
                rng.ex_rssi[rng.channel]  = rac->smtc_rac_data_result.ranging_result.rssi;
                rng.stats.exch_ok += 1;
            }
            else
            {
                rng.stats.exch_fail += 1;
            }
        }

        rng.channel += 1;
        rac->scheduler_config.start_time_ms += rng.slot_ms;
        while( ( int32_t ) ( rac->scheduler_config.start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) < 0 )
        {
            // Late (a slot was missed): skip the slots that are already in the past, as the stock demo does
            rac->scheduler_config.start_time_ms += rng.slot_ms;
            rng.channel += 1;
        }

        if( rng.channel >= APP_RANGING_CHANNELS )
        {
            manager_finish_run( );
            manager_send_config( 0 );
            break;
        }

        rac->scheduler_config.scheduling = SMTC_RAC_SCHEDULED_TRANSACTION;
        set_freq( hop_table[rng.channel] );
        submit( );
        break;

    default:
        manager_send_config( CFG_RETRY_DELAY_MS );
        break;
    }
}

static void subordinate_callback( rp_status_t status )
{
    smtc_rac_context_t* rac       = rng.rac;
    const uint32_t      radio_end = rac->smtc_rac_data_result.radio_end_timestamp_ms;

    switch( rng.state )
    {
    case ST_CONFIG:
        if( ( status == RP_STATUS_RX_PACKET ) && cfg_address_ok( ) )
        {
            // Ack: echo the config with our RSSI in the spare byte
            rng.payload[6] = ( uint8_t ) rac->smtc_rac_data_result.rssi_result;
            rng.state      = ST_ACK;

            rac->radio_params.lora.is_tx               = true;
            rac->radio_params.lora.is_ranging_exchange = false;
            rac->scheduler_config.scheduling           = SMTC_RAC_ASAP_TRANSACTION;
            rac->scheduler_config.start_time_ms        = smtc_modem_hal_get_time_in_ms( );
            submit( );
        }
        else
        {
            subordinate_listen( );
        }
        break;

    case ST_ACK:
        if( status == RP_STATUS_TX_DONE )
        {
            // First exchange: half a slot after the ack, RX window one slot long (stock timing)
            rng.stats.cfg_ok += 1;
            rng.state   = ST_RUN;
            rng.channel = 0;
            rng.misses  = 0;

            rac->radio_params.lora.is_tx               = false;
            rac->radio_params.lora.is_ranging_exchange = true;
            rac->radio_params.lora.rx_timeout_ms       = rng.slot_ms;
            rac->scheduler_config.scheduling           = SMTC_RAC_SCHEDULED_TRANSACTION;
            rac->scheduler_config.start_time_ms        = radio_end + ( rng.slot_ms >> 1 );
            set_freq( hop_table[0] );
            submit( );
        }
        else
        {
            subordinate_listen( );
        }
        break;

    case ST_RUN:
    {
        const bool answered = ( status == RP_STATUS_RTTOF_RESP_DONE );

        if( answered )
        {
            rng.stats.exch_ok += 1;
            rng.misses = 0;
        }
        else
        {
            rng.stats.exch_fail += 1;
            rng.misses += 1;
        }

        rng.channel += 1;
        rac->scheduler_config.start_time_ms += rng.slot_ms;
        if( !answered )
        {
            while( ( int32_t ) ( rac->scheduler_config.start_time_ms - smtc_modem_hal_get_time_in_ms( ) ) < 0 )
            {
                rac->scheduler_config.start_time_ms += rng.slot_ms;
                rng.channel += 1;
            }
        }

        if( rng.channel >= APP_RANGING_CHANNELS )
        {
            rng.stats.runs += 1;
            subordinate_listen( );
            break;
        }
        if( rng.misses >= SUB_MAX_MISSES )
        {
            // Manager gone (phase ended, or it never got our ack): back to config RX instead of idling out the run
            rng.stats.runs_cut += 1;
            subordinate_listen( );
            break;
        }

        rac->scheduler_config.scheduling = SMTC_RAC_ASAP_TRANSACTION;  // As the stock subordinate
        set_freq( hop_table[rng.channel] );
        submit( );
        break;
    }

    default:
        subordinate_listen( );
        break;
    }
}

/* Manager: send a config packet, unless a full run no longer fits in the DISTANCE window */
static void manager_send_config( uint32_t delay_ms )
{
    smtc_rac_context_t* rac   = rng.rac;
    const uint32_t      start = smtc_modem_hal_get_time_in_ms( ) + delay_ms;

    if( ( int32_t ) ( rng.end_ms - start ) < ( int32_t ) run_duration_ms( ) )
    {
        rng.state = ST_IDLE;
        return;
    }

    rng.payload[0] = ( uint8_t ) ( RANGING_ADDR >> 24 );
    rng.payload[1] = ( uint8_t ) ( RANGING_ADDR >> 16 );
    rng.payload[2] = ( uint8_t ) ( RANGING_ADDR >> 8 );
    rng.payload[3] = ( uint8_t ) ( RANGING_ADDR );
    rng.payload[4] = 0;                     // First channel
    rng.payload[5] = APP_RANGING_CHANNELS;  // Number of hops
    rng.payload[6] = 0;

    rng.state = ST_CONFIG;
    rng.stats.cfg_tx += 1;

    rac->radio_params.lora.is_tx               = true;
    rac->radio_params.lora.is_ranging_exchange = false;
    rac->scheduler_config.scheduling           = SMTC_RAC_ASAP_TRANSACTION;
    rac->scheduler_config.start_time_ms        = start;
    set_freq( RANGING_CFG_FREQ_HZ );
    submit( );
}

/* Subordinate: open a config RX window */
static void subordinate_listen( void )
{
    smtc_rac_context_t* rac = rng.rac;

    memset( rng.payload, 0, sizeof( rng.payload ) );
    rng.state = ST_CONFIG;
    rng.stats.cfg_tx += 1;

    rac->radio_params.lora.is_tx               = false;
    rac->radio_params.lora.is_ranging_exchange = false;
    rac->radio_params.lora.rx_timeout_ms       = SUB_CFG_RX_TIMEOUT_MS;
    rac->scheduler_config.scheduling           = SMTC_RAC_ASAP_TRANSACTION;
    rac->scheduler_config.start_time_ms        = smtc_modem_hal_get_time_in_ms( );
    set_freq( RANGING_CFG_FREQ_HZ );
    submit( );
}

static bool cfg_address_ok( void )
{
    return ( rng.rac->smtc_rac_data_result.rx_size == CFG_PAYLOAD_LEN ) &&
           ( rng.payload[0] == ( uint8_t ) ( RANGING_ADDR >> 24 ) ) &&
           ( rng.payload[1] == ( uint8_t ) ( RANGING_ADDR >> 16 ) ) &&
           ( rng.payload[2] == ( uint8_t ) ( RANGING_ADDR >> 8 ) ) &&
           ( rng.payload[3] == ( uint8_t ) ( RANGING_ADDR ) );
}

/* Every frequency handed to the RAC goes through here: 2.4 GHz ISM only */
static void set_freq( uint32_t freq_hz )
{
    app_radio_check_freq( freq_hz );
    rng.rac->radio_params.lora.frequency_in_hz = freq_hz;
}

static void submit( void )
{
    rng.task_pending = true;
    if( SMTC_SW_PLATFORM( smtc_rac_submit_radio_transaction( rng.id ) ) != SMTC_RAC_SUCCESS )
    {
        rng.task_pending = false;
        rng.submit_errors += 1;
        rng.state = ST_IDLE;
    }
}

static uint32_t run_duration_ms( void )
{
    return ACK_TIMEOUT_MS + ( ( APP_RANGING_CHANNELS + 1 ) * rng.slot_ms ) + RUN_MARGIN_MS;
}

int32_t app_ranging_raw_to_cm( int32_t raw )
{
    // metres = raw * 150 / (BW_Hz * 4096 / 1e6) (lr20xx_rttof_distance_raw_to_meter); in cm, rounded
    const int64_t num = ( int64_t ) raw * 15000LL * 1000000LL;
    const int64_t den = RANGING_BW_HZ * 4096LL;
    return ( int32_t ) ( ( num + ( ( num >= 0 ) ? den / 2 : -den / 2 ) ) / den );
}

/* Manager: median of the kept raw results -> cm; hand the run (with per-hop data) to the main thread.
 * Kept = RTTOF_EXCH_VALID and RSSI within CONFIG_APP_RANGING_RSSI_WINDOW_DB of the strongest hop of the run: a hop
 * in a deep fade is the one most likely measuring a reflected path. */
static void manager_finish_run( void )
{
    app_ranging_run_t run = { 0 };
    int32_t           sorted[APP_RANGING_CHANNELS];
    uint8_t           n        = 0;
    int32_t           rssi_sum = 0;
    int8_t            best     = INT8_MIN;

    run.run_id = ++rng.run_count;
    run.total  = APP_RANGING_CHANNELS;

    for( uint8_t i = 0; i < APP_RANGING_CHANNELS; i++ )
    {
        run.hop_valid[i] = rng.ex_valid[i];
        run.hop_raw[i]   = rng.ex_valid[i] ? rng.ex_raw[i] : 0;
        run.hop_rssi[i]  = rng.ex_valid[i] ? rng.ex_rssi[i] : 0;
        if( rng.ex_valid[i] )
        {
            run.valid += 1;
            best = MAX( best, rng.ex_rssi[i] );
        }
    }

    for( uint8_t i = 0; i < APP_RANGING_CHANNELS; i++ )
    {
        if( !rng.ex_valid[i] || ( rng.ex_rssi[i] < best - CONFIG_APP_RANGING_RSSI_WINDOW_DB ) )
        {
            continue;
        }
        run.hop_used[i] = true;
        // Insertion sort
        uint8_t j = n;
        while( ( j > 0 ) && ( sorted[j - 1] > rng.ex_raw[i] ) )
        {
            sorted[j] = sorted[j - 1];
            j--;
        }
        sorted[j] = rng.ex_raw[i];
        rssi_sum += rng.ex_rssi[i];
        n++;
    }

    run.used = n;
    if( n > 0 )
    {
        run.median_raw  = ( ( n % 2 ) == 1 ) ? sorted[n / 2] : ( sorted[n / 2 - 1] + sorted[n / 2] ) / 2;
        run.rssi        = ( int16_t ) ( rssi_sum / n );
        run.distance_cm = app_ranging_raw_to_cm( run.median_raw ) - CONFIG_APP_RANGING_OFFSET_CM;
    }

    rng.stats.runs += 1;
    rng.latest     = run;
    rng.latest_new = true;
    app_event_signal( );
}

/* --- EOF ------------------------------------------------------------------ */
