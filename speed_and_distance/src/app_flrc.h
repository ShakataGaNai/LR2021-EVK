/**
 * @file      app_flrc.h
 *
 * @brief     One-way FLRC throughput (SPEED phase): TX streams, RX measures goodput / success rate
 *
 * Copied from flrc_throughput/src/app_flrc.h (derived from usp examples/main_examples/
 * packet_error_rate_flrc_example, app_per_flrc.c). Adds start/stop so the radio can be handed to the
 * RAC ranging transactions between SPEED phases, and a mode-switch flag in the payload header.
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

#ifndef APP_FLRC_H
#define APP_FLRC_H

#include <stdbool.h>
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define APP_FLRC_RF_FREQ_IN_HZ ( ( uint32_t ) CONFIG_APP_RF_FREQ_MHZ * 1000000UL )
#define APP_FLRC_TX_POWER_DBM ( CONFIG_APP_TX_POWER_DBM )
#define APP_FLRC_PAYLOAD_SIZE ( CONFIG_APP_PAYLOAD_SIZE )

#if defined( CONFIG_APP_ROLE_RX )
#define APP_FLRC_IS_TX false
#else
#define APP_FLRC_IS_TX true
#endif

/*!
 * @brief Payload header: [0..3] sequence number (LE), [4] flags, [5..6] ms until the phase switch (LE),
 *        [7] cycle counter. Filler bytes follow.
 */
#define APP_FLRC_HDR_SIZE ( 8 )
#define APP_FLRC_FLAG_SWITCH ( 0x01 )  //!< TX leaves the SPEED phase in "ms until switch" ms

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Counters accumulated by the radio callback since the last read
 */
typedef struct app_flrc_stats_s
{
    // TX side
    uint32_t tx_done;      //!< Packets transmitted
    uint32_t tx_errors;    //!< Aborted / unexpected statuses on TX
    // RX side
    uint32_t rx_ok;        //!< Packets received with good CRC
    uint32_t rx_bytes;     //!< Payload bytes received with good CRC
    uint32_t rx_expected;  //!< Packets the TX sent in the same span (from sequence numbers)
    uint32_t crc_errors;   //!< Packets received with bad CRC
    uint32_t rx_timeouts;  //!< RX windows that expired without a packet
    uint32_t resyncs;      //!< Sequence restarts detected (TX reboot, big jump)
    uint32_t fifo_resets;  //!< RX FIFO cleared because it was out of step
    // Snapshot values (not reset)
    uint32_t last_seq;     //!< Last sequence number sent / received
    int16_t  last_rssi;    //!< RSSI of the last good packet (dBm)
    bool     rssi_valid;   //!< At least one packet received
} app_flrc_stats_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief One-time init (payload filler). No radio access. Call after smtc_rac_init().
 */
void app_flrc_init( void );

/*!
 * @brief Take the radio with smtc_rac_immediate_radio_access(), configure FLRC and start the TX stream or
 *        continuous RX. Call from the main thread at a cooperative priority, with no RAC task pending.
 *
 * @param [in] switch_at_ms TX: k_uptime_get_32() time at which this SPEED phase ends (the payload carries the
 *                          switch flag during the last CONFIG_APP_SWITCH_WARN_MS). Ignored on RX.
 * @param [in] cycle        TX: cycle counter written in the payload header. Ignored on RX.
 */
void app_flrc_start( uint32_t switch_at_ms, uint8_t cycle );

/*!
 * @brief Stop FLRC and hand the radio back to the RAC scheduler: standby, clear IRQs and FIFOs, fallback mode
 *        back to STDBY_RC, smtc_rac_release_immediate_radio_access(). Same calling rules as app_flrc_start().
 */
void app_flrc_stop( void );

/*!
 * @brief Copy the counters accumulated since the last call, then reset them.
 *
 * @param [out] stats Destination
 */
void app_flrc_get_and_reset_stats( app_flrc_stats_t* stats );

/*!
 * @brief RX: switch notice received in this SPEED phase
 *
 * @param [out] switch_at_ms k_uptime_get_32() time at which the TX leaves the SPEED phase
 * @param [out] cycle        TX cycle counter
 *
 * @returns true if a packet with APP_FLRC_FLAG_SWITCH arrived since app_flrc_start()
 */
bool app_flrc_get_switch_notice( uint32_t* switch_at_ms, uint8_t* cycle );

/*!
 * @brief RX: k_uptime_get_32() time of the last good packet (or of app_flrc_start() if none yet)
 */
uint32_t app_flrc_get_last_rx_ms( void );

#endif  // APP_FLRC_H
