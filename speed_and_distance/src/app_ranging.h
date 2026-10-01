/**
 * @file      app_ranging.h
 *
 * @brief     LoRa RTToF ranging with 2.4 GHz frequency hopping (DISTANCE phase)
 *
 * Derived from usp examples/main_examples/ranging_demo (app_ranging_hopping.h).
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

#ifndef APP_RANGING_H
#define APP_RANGING_H

#include <stdbool.h>
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * @brief Number of RTToF exchanges (one per hop channel) in one ranging run
 */
#define APP_RANGING_CHANNELS ( 29 )

#if defined( CONFIG_APP_ROLE_RX )
#define APP_RANGING_IS_MANAGER true
#else
#define APP_RANGING_IS_MANAGER false
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Result of one complete ranging run (manager only)
 */
typedef struct app_ranging_run_s
{
    uint32_t run_id;       //!< Increments on each completed run (since boot)
    uint8_t  valid;        //!< Exchanges with a valid result (RTTOF_EXCH_VALID)
    uint8_t  used;         //!< Valid exchanges kept by the RSSI filter (used for the median)
    uint8_t  total;        //!< Exchanges attempted (APP_RANGING_CHANNELS)
    int32_t  median_raw;   //!< Median of the kept sign-extended 24-bit raw results
    int32_t  distance_cm;  //!< median_raw in cm, minus CONFIG_APP_RANGING_OFFSET_CM
    int16_t  rssi;         //!< Mean RSSI of the kept exchanges (dBm)
    // Per hop (index = hop table position), for the console log
    bool     hop_valid[APP_RANGING_CHANNELS];
    bool     hop_used[APP_RANGING_CHANNELS];
    int32_t  hop_raw[APP_RANGING_CHANNELS];
    int8_t   hop_rssi[APP_RANGING_CHANNELS];
} app_ranging_run_t;

/*!
 * @brief Raw RTToF result to centimetres, without offset (0.0451 m per step at 812 kHz)
 */
int32_t app_ranging_raw_to_cm( int32_t raw );

/*!
 * @brief Counters since the last app_ranging_start()
 */
typedef struct app_ranging_stats_s
{
    uint32_t cfg_tx;     //!< Manager: config packets sent. Subordinate: config RX windows opened.
    uint32_t cfg_ok;     //!< Manager: acks received. Subordinate: configs accepted (ack sent).
    uint32_t runs;       //!< Runs completed (all hop channels done)
    uint32_t runs_cut;   //!< Subordinate: runs abandoned after consecutive misses
    uint32_t exch_ok;    //!< Manager: RTTOF_EXCH_VALID. Subordinate: RTTOF_RESP_DONE.
    uint32_t exch_fail;  //!< Timeouts, discarded requests, aborted exchanges
} app_ranging_stats_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Open the RAC radio access used for ranging. Call once after smtc_rac_init(), before the first
 *        immediate radio access.
 */
void app_ranging_init( void );

/*!
 * @brief Start the DISTANCE phase. The radio must already be released by app_flrc_stop().
 *        Manager: sends a config packet after first_delay_ms, then runs back to back while a full run still
 *        fits in window_ms. Subordinate: listens for config packets until app_ranging_stop().
 *        Call from the main thread at a cooperative priority.
 */
void app_ranging_start( uint32_t window_ms, uint32_t first_delay_ms );

/*!
 * @brief Stop ranging: no new submits, abort the pending RAC task and wait for its callback.
 *        Same calling rules as app_ranging_start().
 */
void app_ranging_stop( void );

/*!
 * @brief Latest completed run (manager)
 *
 * @returns true if a run completed since the previous call
 */
bool app_ranging_get_run( app_ranging_run_t* run );

/*!
 * @brief Counters since app_ranging_start()
 */
void app_ranging_get_stats( app_ranging_stats_t* stats );

/*!
 * @brief Time of one RTToF exchange slot in ms (64.25 symbols + 20 ms processing + extra)
 */
uint32_t app_ranging_slot_ms( void );

#endif  // APP_RANGING_H

/* --- EOF ------------------------------------------------------------------ */
