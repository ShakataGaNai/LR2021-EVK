/**
 * @file      app_ble.c
 *
 * @brief     BLE legacy advertising (beacon) and channel-37 scanning with the LR2021 Bluetooth LE packet engine.
 *
 * Throwaway prototype. Takes the radio with smtc_rac_immediate_radio_access() like flrc_throughput, then uses
 * the LR20xx driver's Bluetooth_LE commands (lr20xx_radio_bluetooth_le.h): the chip adds preamble, access address,
 * CRC24 and channel whitening; the FIFO carries the PDU (2-byte header + payload).
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/random/random.h>

#include "app_ble.h"

#include "smtc_rac_api.h"
#include "smtc_modem_hal.h"

#include "ral_lr20xx.h"
#include "lr20xx_system.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_radio_bluetooth_le.h"

#define PANIC_IF_ERR( x ) SMTC_MODEM_HAL_PANIC_ON_FAILURE( ( int ) ( x ) == 0 )

#define BLE_ADV_ACCESS_ADDRESS ( 0x8E89BED6UL )
#define BLE_ADV_CRC_INIT ( 0x555555UL )

#define PDU_TYPE_ADV_IND ( 0x0 )
#define PDU_TYPE_ADV_NONCONN_IND ( 0x2 )
#define PDU_TYPE_SCAN_RSP ( 0x4 )
#define PDU_TYPE_ADV_SCAN_IND ( 0x6 )
#define PDU_HDR_TXADD_RANDOM ( 1u << 6 )

#define ADV_PDU_MAX ( 2 + 37 )  // header + AdvA (6) + AdvData (31)
#define RX_BUF_SIZE ( 256 )

#define DEVICE_TTL_MS ( 10000 )

typedef struct
{
    uint8_t  chan_idx;
    uint32_t freq_hz;
} adv_chan_t;

static const adv_chan_t adv_chans[3] = {
    { 37, 2402000000UL },
    { 38, 2426000000UL },
    { 39, 2480000000UL },
};

typedef struct
{
    uint8_t addr[6];
    int16_t rssi;
    int64_t last_seen_ms;
} ble_device_t;

static void*   radio;
static uint8_t adv_addr[6];
static uint8_t adv_pdu[ADV_PDU_MAX];
static uint8_t adv_pdu_len;
static uint8_t rx_buf[RX_BUF_SIZE];

static K_SEM_DEFINE( tx_done_sem, 0, 1 );
static volatile lr20xx_system_irq_mask_t last_tx_irq;

static app_ble_stats_t stats;
static ble_device_t    devices[APP_BLE_MAX_DEVICES];

#define BEACON_STACK_SIZE ( 2048 )
#define BEACON_PRIORITY ( 4 )
K_THREAD_STACK_DEFINE( beacon_stack, BEACON_STACK_SIZE );
static struct k_thread beacon_thread;

/*
 * BLE whitening LFSR: position 0 = 1, positions 1..6 = channel index (MSB first). The LR20xx register holds
 * position 0 in bit 0, so the value is reverse6(chan) << 1 | 1 (37 -> 0x53, matching the driver comment).
 */
static uint8_t ble_whitening_init( uint8_t chan_idx )
{
    uint8_t rev = 0;
    for( int i = 0; i < 6; i++ )
    {
        if( chan_idx & ( 1u << i ) )
        {
            rev |= ( uint8_t ) ( 1u << ( 5 - i ) );
        }
    }
    return ( uint8_t ) ( ( rev << 1 ) | 1u );
}

static void set_adv_channel( const adv_chan_t* ch )
{
    const lr20xx_radio_bluetooth_le_pkt_params_t pkt = {
        .phy_channel_pdu = LR20XX_RADIO_BLUETOOTH_LE_PHY_CHANNEL_PDU_ADVERTISING,
        .whitening_init  = ble_whitening_init( ch->chan_idx ),
        .is_crc_in_fifo  = false,
        .crc_init        = BLE_ADV_CRC_INIT,
        .sync_word       = BLE_ADV_ACCESS_ADDRESS,
    };

    PANIC_IF_ERR( ral_lr20xx_set_rf_freq( radio, ch->freq_hz ) );
    PANIC_IF_ERR( lr20xx_radio_bluetooth_le_set_pkt_params( radio, &pkt ) );
}

static void make_address( void )
{
    uint8_t id[8] = { 0 };
    if( hwinfo_get_device_id( id, sizeof( id ) ) < 6 )
    {
        sys_rand_get( id, sizeof( id ) );
    }
    memcpy( adv_addr, id, 6 );
    adv_addr[5] |= 0xC0;  // Random static address: two MSBs set (addr[5] is the MSB; sent last, LSB first)
}

/* ADV_NONCONN_IND: header, AdvA, AdvData { Flags, Complete Local Name, Manufacturer data (0xFFFF test ID + counter) } */
static void build_adv_pdu( uint32_t counter )
{
    uint8_t* p = adv_pdu + 2;

    memcpy( p, adv_addr, 6 );
    p += 6;

    *p++ = 2;     // Flags
    *p++ = 0x01;
    *p++ = 0x06;  // LE General Discoverable, BR/EDR not supported

    const char* name     = CONFIG_APP_ADV_NAME;
    size_t      name_len = strlen( name );
    if( name_len > 14 )
    {
        name_len = 14;
    }
    *p++ = ( uint8_t ) ( name_len + 1 );
    *p++ = 0x09;  // Complete Local Name
    memcpy( p, name, name_len );
    p += name_len;

    *p++ = 7;     // Manufacturer Specific Data
    *p++ = 0xFF;
    *p++ = 0xFF;  // Company ID 0xFFFF (reserved for testing), little endian
    *p++ = 0xFF;
    memcpy( p, &counter, 4 );
    p += 4;

    adv_pdu_len = ( uint8_t ) ( p - adv_pdu );
    adv_pdu[0]  = PDU_TYPE_ADV_NONCONN_IND | PDU_HDR_TXADD_RANDOM;
    adv_pdu[1]  = ( uint8_t ) ( adv_pdu_len - 2 );
}

static void radio_configure_common( void )
{
    PANIC_IF_ERR( ral_lr20xx_set_standby( radio, RAL_STANDBY_CFG_XOSC ) );
    PANIC_IF_ERR( lr20xx_radio_common_set_pkt_type( radio, LR20XX_RADIO_COMMON_PKT_TYPE_BLUETOOTH_LE ) );
    PANIC_IF_ERR( lr20xx_radio_bluetooth_le_set_modulation_params( radio, LR20XX_RADIO_BLUETOOTH_LE_PHY_LE_1M ) );
    set_adv_channel( &adv_chans[0] );
}

/* Runs in the USP thread on each radio IRQ */
static void irq_callback( void )
{
    lr20xx_system_irq_mask_t irq = LR20XX_SYSTEM_IRQ_NONE;
    lr20xx_system_get_and_clear_irq_status( radio, &irq );

    if( APP_BLE_IS_BEACON )
    {
        last_tx_irq = irq;
        k_sem_give( &tx_done_sem );
        return;
    }

    if( ( irq & ( LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR ) ) != 0 )
    {
        stats.crc_errors += 1;
        lr20xx_radio_fifo_clear_rx( radio );
    }
    else if( ( irq & LR20XX_SYSTEM_IRQ_RX_DONE ) != 0 )
    {
        uint16_t level = 0;
        lr20xx_radio_fifo_get_rx_level( radio, &level );
        if( level > sizeof( rx_buf ) )
        {
            lr20xx_radio_fifo_clear_rx( radio );
            return;
        }
        lr20xx_radio_fifo_read_rx( radio, rx_buf, level );

        lr20xx_radio_bluetooth_le_pkt_status_t st = { 0 };
        lr20xx_radio_bluetooth_le_get_pkt_status( radio, &st );

        // Advertising PDU: header(2) + AdvA(6) at least. PDU types with AdvA first: ADV_IND, ADV_NONCONN_IND,
        // SCAN_RSP, ADV_SCAN_IND, ADV_DIRECT_IND (1).
        if( ( level < 8 ) || ( rx_buf[1] + 2 > level ) )
        {
            return;
        }
        stats.rx_ok += 1;

        const uint8_t* addr = rx_buf + 2;
        const int64_t  now  = k_uptime_get( );
        int            slot = -1;
        int            lru  = 0;
        for( int i = 0; i < APP_BLE_MAX_DEVICES; i++ )
        {
            if( ( devices[i].last_seen_ms != 0 ) && ( memcmp( devices[i].addr, addr, 6 ) == 0 ) )
            {
                slot = i;
                break;
            }
            if( devices[i].last_seen_ms < devices[lru].last_seen_ms )
            {
                lru = i;
            }
        }
        if( slot < 0 )
        {
            slot = lru;
            memcpy( devices[slot].addr, addr, 6 );
        }
        devices[slot].rssi         = st.rssi_avg_in_dbm;
        devices[slot].last_seen_ms = now;
    }
}

static void beacon_loop( void* a, void* b, void* c )
{
    ARG_UNUSED( a );
    ARG_UNUSED( b );
    ARG_UNUSED( c );

    uint32_t counter = 0;

    while( true )
    {
        const int64_t start = k_uptime_get( );
        build_adv_pdu( counter );

        for( int i = 0; i < 3; i++ )
        {
            set_adv_channel( &adv_chans[i] );
            lr20xx_radio_fifo_clear_tx( radio );
            lr20xx_radio_fifo_write_tx( radio, adv_pdu, adv_pdu_len );
            k_sem_reset( &tx_done_sem );
            lr20xx_radio_bluetooth_le_set_tx( radio, adv_pdu_len );
            // 39-byte PDU at 1 Mbps is ~0.4 ms on air
            if( ( k_sem_take( &tx_done_sem, K_MSEC( 10 ) ) != 0 ) || ( ( last_tx_irq & LR20XX_SYSTEM_IRQ_TX_DONE ) == 0 ) )
            {
                stats.tx_errors += 1;
                ral_lr20xx_set_standby( radio, RAL_STANDBY_CFG_XOSC );
            }
        }

        counter += 1;
        stats.adv_events = counter;

        // advInterval + advDelay (0..10 ms pseudo-random), per the Core spec
        k_sleep( K_TIMEOUT_ABS_MS( start + CONFIG_APP_ADV_INTERVAL_MS + ( sys_rand32_get( ) % 11 ) ) );
    }
}

void app_ble_init( void )
{
    make_address( );

    SMTC_MODEM_HAL_PANIC_ON_FAILURE( smtc_rac_immediate_radio_access( RAC_VERY_HIGH_PRIORITY, irq_callback ) ==
                                     SMTC_RAC_SUCCESS );
    radio = smtc_rac_get_radio_driver_context( );

    radio_configure_common( );

    if( APP_BLE_IS_BEACON )
    {
        PANIC_IF_ERR( ral_lr20xx_set_tx_cfg( radio, CONFIG_APP_TX_POWER_DBM, adv_chans[1].freq_hz ) );
        PANIC_IF_ERR( ral_lr20xx_set_rx_tx_fallback_mode( radio, RAL_FALLBACK_STDBY_XOSC ) );
        PANIC_IF_ERR( ral_lr20xx_set_dio_irq_params( radio, RAL_IRQ_TX_DONE | RAL_IRQ_RX_TIMEOUT ) );
        PANIC_IF_ERR( ral_lr20xx_clear_irq_status( radio, RAL_IRQ_ALL ) );

        k_thread_create( &beacon_thread, beacon_stack, K_THREAD_STACK_SIZEOF( beacon_stack ), beacon_loop, NULL,
                         NULL, NULL, BEACON_PRIORITY, 0, K_NO_WAIT );
        k_thread_name_set( &beacon_thread, "ble_beacon" );
    }
    else
    {
        PANIC_IF_ERR( lr20xx_radio_bluetooth_le_set_pdu_length( radio, 255 ) );  // RX max length (assumed)
        PANIC_IF_ERR( ral_lr20xx_set_dio_irq_params(
            radio, RAL_IRQ_RX_DONE | RAL_IRQ_RX_CRC_ERROR | RAL_IRQ_RX_LEN_ERROR | RAL_IRQ_RX_TIMEOUT ) );
        PANIC_IF_ERR( ral_lr20xx_clear_irq_status( radio, RAL_IRQ_ALL ) );
        PANIC_IF_ERR( ral_lr20xx_set_rx( radio, RAL_RX_TIMEOUT_CONTINUOUS_MODE ) );
    }
}

void app_ble_get_adv_addr( uint8_t addr[6] )
{
    memcpy( addr, adv_addr, 6 );
}

void app_ble_get_stats( app_ble_stats_t* out )
{
    const int64_t now = k_uptime_get( );
    unsigned int  key = irq_lock( );

    *out             = stats;
    out->devices     = 0;
    out->best_rssi   = -200;
    stats.rx_ok      = 0;
    stats.crc_errors = 0;

    for( int i = 0; i < APP_BLE_MAX_DEVICES; i++ )
    {
        if( ( devices[i].last_seen_ms != 0 ) && ( now - devices[i].last_seen_ms < DEVICE_TTL_MS ) )
        {
            out->devices += 1;
            if( devices[i].rssi > out->best_rssi )
            {
                out->best_rssi = devices[i].rssi;
                memcpy( out->best_addr, devices[i].addr, 6 );
            }
        }
    }

    irq_unlock( key );
}
