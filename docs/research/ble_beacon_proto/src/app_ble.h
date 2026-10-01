#ifndef APP_BLE_H
#define APP_BLE_H

#include <stdbool.h>
#include <stdint.h>

#if defined( CONFIG_APP_ROLE_SCAN )
#define APP_BLE_IS_BEACON false
#else
#define APP_BLE_IS_BEACON true
#endif

#define APP_BLE_MAX_DEVICES ( 32 )

typedef struct app_ble_stats_s
{
    uint32_t adv_events;  //!< Advertising events sent (3 packets each), cumulative
    uint32_t tx_errors;   //!< TX timeouts / unexpected IRQs, cumulative
    uint32_t rx_ok;       //!< Adverts received with good CRC since the last read
    uint32_t crc_errors;  //!< Adverts with bad CRC since the last read
    uint32_t devices;     //!< Distinct advertisers seen in the last APP_BLE_DEVICE_TTL_MS
    int16_t  best_rssi;   //!< Strongest RSSI among those devices
    uint8_t  best_addr[6];
} app_ble_stats_t;

void app_ble_init( void );
void app_ble_get_stats( app_ble_stats_t* stats );
void app_ble_get_adv_addr( uint8_t addr[6] );

#endif
