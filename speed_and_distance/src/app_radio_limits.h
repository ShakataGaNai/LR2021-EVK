/*
 * SPDX-License-Identifier: MIT
 *
 * Compile-time and run-time guards for the bench rules: transmit only in the 2.4 GHz ISM band
 * (2400-2483.5 MHz) and at most +12 dBm (LR2021 HF PA limit). The sub-GHz antenna on these
 * boards is a CN490 part, so a sub-GHz frequency must never reach the radio.
 */

#ifndef APP_RADIO_LIMITS_H
#define APP_RADIO_LIMITS_H

#include <stdint.h>

#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#include "smtc_modem_hal.h"

#define APP_ISM_2G4_MIN_HZ ( 2400000000ULL )
#define APP_ISM_2G4_MAX_HZ ( 2483500000ULL )

/* Widest signal used here: LoRa 812 kHz (+/- 406 kHz). FLRC 2.6 Mbps occupies about 2.7 MHz (+/- 1.35 MHz). */
#define APP_ISM_EDGE_MARGIN_HZ ( 1400000ULL )

/*!
 * @brief True when a channel centred on f_hz stays inside the 2.4 GHz ISM band
 */
#define APP_FREQ_IN_ISM_2G4( f_hz )                                      \
    ( ( ( uint64_t ) ( f_hz ) >= APP_ISM_2G4_MIN_HZ + APP_ISM_EDGE_MARGIN_HZ ) && \
      ( ( uint64_t ) ( f_hz ) <= APP_ISM_2G4_MAX_HZ - APP_ISM_EDGE_MARGIN_HZ ) )

/*!
 * @brief Compile-time check of one frequency (use at file scope)
 */
#define APP_ASSERT_FREQ_IN_ISM_2G4( f_hz ) \
    BUILD_ASSERT( APP_FREQ_IN_ISM_2G4( f_hz ), "frequency outside the 2.4 GHz ISM band (2400-2483.5 MHz)" )

#define APP_TX_POWER_MAX_DBM ( 12 )

BUILD_ASSERT( CONFIG_APP_TX_POWER_DBM <= APP_TX_POWER_MAX_DBM, "TX power above the +12 dBm HF PA limit" );

/*!
 * @brief Run-time guard, called right before a frequency is handed to the radio
 */
static inline void app_radio_check_freq( uint32_t f_hz )
{
    SMTC_MODEM_HAL_PANIC_ON_FAILURE( APP_FREQ_IN_ISM_2G4( f_hz ) );
}

#endif  // APP_RADIO_LIMITS_H
