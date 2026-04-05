/**
 * @file    hal_timer.h
 * @brief   Hardware timer abstraction for fixed-period control loop tick.
 *
 *  This module configures a hardware timer to assert a periodic interrupt
 *  at exactly SAMPLE_PERIOD_MS intervals. The ISR sets a volatile flag
 *  that the main loop polls — keeping all real work out of the ISR itself.
 *
 *  WHY keep the ISR minimal?
 *  Floating-point operations (PID math) inside an ISR block all other
 *  interrupts for their duration, degrading UART, ADC, and other peripherals.
 *  The flag pattern lets the ISR finish in ~5 CPU cycles, then the main
 *  loop does the heavy lifting at normal execution priority.
 */

#ifndef HAL_TIMER_H
#define HAL_TIMER_H

#include <stdint.h>

/** Control loop sample period in milliseconds. Must match PID dt parameter. */
#define SAMPLE_PERIOD_MS (100U) /* 100 ms = 10 Hz control rate   */

/** Equivalent sample period in seconds — passed to PID_Init() as dt.        */
#define SAMPLE_PERIOD_S (0.1f)

/**
 * @brief  Configure the hardware timer to generate an interrupt every
 *         SAMPLE_PERIOD_MS milliseconds.
 *         Enables global interrupts on AVR (sei()).
 *         [PLATFORM-SPECIFIC: timer peripheral selection]
 */
void Timer_Init(void);

/**
 * @brief  Returns 1 if the timer tick flag has fired since last call.
 *         Atomically clears the flag on read — safe to call from main loop.
 *
 * @return 1 if a new sample period has elapsed, 0 otherwise.
 */
uint8_t Timer_TickOccurred(void);

#endif /* HAL_TIMER_H */