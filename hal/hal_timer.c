/**
 * @file    hal_timer.c
 * @brief   100 ms periodic tick using Timer2 CTC mode on ATmega328P.
 *          [PLATFORM-SPECIFIC: replace for your MCU]
 *
 *  Timer selection rationale:
 *    Timer0 — used by Arduino millis()/delay() — avoid conflicts.
 *    Timer1 — already claimed by PWM (Milestone 2).
 *    Timer2 — free 8-bit timer, perfect for a slow 10 Hz tick.
 *
 *  CTC (Clear Timer on Compare) mode:
 *    Counter increments until it matches OCR2A, then resets to 0
 *    and fires a compare-match interrupt. Period is exact — no drift.
 *
 *  Timing calculation:
 *    Target: 100 ms tick
 *    Prescaler /1024: tick period = 1024 / 16,000,000 = 64 µs per count
 *    Counts for 10 ms: 10,000 µs / 64 µs = 156.25 → use 156 (9.984 ms)
 *
 *    WHY 10 ms interrupts counted to 10, not one 100 ms interrupt?
 *    Timer2 is 8-bit: max count = 255 → max single-shot period = 16.32 ms.
 *    We cannot reach 100 ms in one compare. Instead, fire every 10 ms
 *    and use a software counter to assert the flag every 10th interrupt.
 *    10 × 9.984 ms = 99.84 ms — error of 0.16 ms per cycle, negligible
 *    for a thermal system with a time constant of tens of seconds.
 */

#include "hal_timer.h"
#include <stdint.h>

#ifdef __AVR__
#include <avr/io.h>
#include <avr/interrupt.h>
#endif

/* ── Internal State ────────────────────────────────────────────────────────── */

/**
 * volatile: tells the compiler this variable is modified in an ISR.
 * Without volatile, the optimizer may cache it in a register and the
 * main loop will never see the ISR's update — a classic embedded bug.
 */
static volatile uint8_t s_tick_flag = 0U;   /* set by ISR, cleared by main */
static volatile uint8_t s_isr_counter = 0U; /* counts 10 ms ticks to 10    */

/** Number of 10 ms ISR firings that equal one SAMPLE_PERIOD_MS tick. */
#define ISR_TICKS_PER_SAMPLE (SAMPLE_PERIOD_MS / 10U) /* = 10            */

/** Timer2 CTC compare value for 10 ms at 16 MHz, prescaler /1024.
 *  OCR2A = (F_CPU / prescaler / target_freq) - 1
 *        = (16,000,000 / 1024 / 100) - 1
 *        = 155.25 - 1 → round to 155
 *  Actual period: (155+1) × 64 µs = 9.984 ms                                */
#define TIMER2_OCR_10MS (155U)

/* ─────────────────────────────────────────────────────────────────────────── */

void Timer_Init(void)
{
#ifdef __AVR__
    /* CTC mode: WGM22:0 = 010 */
    TCCR2A = (1 << WGM21); /* [PLATFORM-SPECIFIC] */
    TCCR2B = 0U;           /* timer stopped while we configure             */

    /* Set compare value for 10 ms period */
    OCR2A = TIMER2_OCR_10MS; /* [PLATFORM-SPECIFIC] */

    /* Enable Timer2 Compare Match A interrupt */
    TIMSK2 = (1 << OCIE2A); /* [PLATFORM-SPECIFIC] */

    /* Start timer: prescaler /1024 → CS22=1, CS21=0, CS20=1 */
    TCCR2B = (1 << CS22) | (1 << CS20); /* [PLATFORM-SPECIFIC] */

    /* Enable global interrupts — required for ISR to fire */
    sei(); /* [PLATFORM-SPECIFIC] */
#endif
    /* [PLATFORM-SPECIFIC] STM32: configure TIM6/TIM7 as basic timer,
       period = 100ms, enable TIM_IT_UPDATE interrupt in NVIC.               */
}

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Timer2 Compare Match A ISR — fires every 10 ms.
 *         Increments counter; asserts flag every SAMPLE_PERIOD_MS.
 *         [PLATFORM-SPECIFIC: ISR vector name varies by MCU]
 */
#ifdef __AVR__
ISR(TIMER2_COMPA_vect) /* [PLATFORM-SPECIFIC] */
{
    s_isr_counter++;

    if (s_isr_counter >= ISR_TICKS_PER_SAMPLE)
    {
        s_isr_counter = 0U;
        s_tick_flag = 1U; /* signal main loop: time to run PID           */
    }
}
#endif

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Atomically read and clear the tick flag.
 *
 * WHY disable interrupts during read-clear?
 * Without the atomic guard, this race can occur:
 *   1. Main reads s_tick_flag = 1
 *   2. ISR fires, sets s_tick_flag = 1 again
 *   3. Main clears s_tick_flag = 0   ← ISR tick silently lost!
 * Disabling interrupts for 2 instructions eliminates the race.
 *
 * @return 1 if tick occurred, 0 if not.
 */
uint8_t Timer_TickOccurred(void)
{
    uint8_t tick = 0U;

#ifdef __AVR__
    cli(); /* disable interrupts — enter critical section */
    if (s_tick_flag)
    {
        s_tick_flag = 0U;
        tick = 1U;
    }
    sei(); /* re-enable interrupts */
#else
    /* Host stub: always return 1 so simulation loop runs continuously        */
    tick = 1U;
#endif

    return tick;
}