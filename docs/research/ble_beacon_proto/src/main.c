/* LR2021 BLE beacon / scanner demo: main (throwaway prototype, derived from flrc_throughput/src/main.c) */
#include <stdint.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include <smtc_zephyr_usp_api.h>
#include <smtc_sw_platform_helper.h>

#include "app_ble.h"
#include "oled_display.h"

LOG_MODULE_REGISTER( main, LOG_LEVEL_INF );

#define STATS_PERIOD_MS ( 1000 )
#define MAIN_RUN_PRIORITY ( 5 )
#define DISPLAY_NODE DT_CHOSEN( zephyr_display )

static void show_static_rows( void );
static void report( const app_ble_stats_t* s );

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

int main( void )
{
    LOG_INF( "===== LR2021 BLE demo (%s) =====", APP_BLE_IS_BEACON ? "beacon" : "scanner" );

    check_oled( );
    display_thread_init( DISPLAY_LAYOUT_TEXT );

    SMTC_SW_PLATFORM_INIT( );
    SMTC_SW_PLATFORM_VOID( smtc_rac_init( ) );

    app_ble_init( );
    show_static_rows( );

    k_thread_priority_set( k_current_get( ), MAIN_RUN_PRIORITY );

    int64_t next_ms = k_uptime_get( ) + STATS_PERIOD_MS;
    while( true )
    {
        k_sleep( K_TIMEOUT_ABS_MS( next_ms ) );
        next_ms += STATS_PERIOD_MS;

        app_ble_stats_t stats;
        app_ble_get_stats( &stats );
        report( &stats );
    }
    return 0;
}

static void show_static_rows( void )
{
    uint8_t a[6];

    if( APP_BLE_IS_BEACON )
    {
        app_ble_get_adv_addr( a );
        display_text( DISPLAY_ROW1, "BLE beacon (LR2021)" );
        display_text( DISPLAY_ROW2, "Name: %s", CONFIG_APP_ADV_NAME );
        display_text( DISPLAY_ROW3, "ch 37/38/39  %+d dBm", CONFIG_APP_TX_POWER_DBM );
        display_text( DISPLAY_ROW4, "%02X:%02X:%02X:%02X:%02X:%02X", a[5], a[4], a[3], a[2], a[1], a[0] );
    }
    else
    {
        display_text( DISPLAY_ROW1, "BLE scan ch37 (LR2021)" );
        display_text( DISPLAY_ROW2, "Devices: -" );
    }
}

static void report( const app_ble_stats_t* s )
{
    if( APP_BLE_IS_BEACON )
    {
        display_text( DISPLAY_ROW5, "Adverts: %u", s->adv_events );
        display_text( DISPLAY_ROW6, "TX err: %u", s->tx_errors );
        printk( "BEACON adv %u, err %u\n", s->adv_events, s->tx_errors );
        return;
    }

    const uint8_t* a = s->best_addr;
    display_text( DISPLAY_ROW2, "Devices: %u", s->devices );
    display_text( DISPLAY_ROW3, "Adv/s: %u  CRC: %u", s->rx_ok, s->crc_errors );
    if( s->devices > 0 )
    {
        display_text( DISPLAY_ROW4, "Best %d dBm", s->best_rssi );
        display_text( DISPLAY_ROW5, "%02X:%02X:%02X:%02X:%02X:%02X", a[5], a[4], a[3], a[2], a[1], a[0] );
    }
    printk( "SCAN devices %u, adv %u, crc %u, best %d dBm\n", s->devices, s->rx_ok, s->crc_errors, s->best_rssi );
}
