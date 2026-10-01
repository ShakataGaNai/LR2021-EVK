/*
 * SPDX-License-Identifier: MIT
 *
 * Wake-up path from the radio callbacks (USP thread) to the main phase controller.
 */

#ifndef APP_EVENTS_H
#define APP_EVENTS_H

/*!
 * @brief Wake the main thread (k_sem_give). Safe from the USP thread; never blocks.
 */
void app_event_signal( void );

#endif  // APP_EVENTS_H
