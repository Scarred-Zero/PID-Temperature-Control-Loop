
#ifndef HAL_PWM_H
#define HAL_PWM_H

#include <stdint.h>

/* ── PWM Configuration Constants ────────────────────────────────────────────
 *
 *  Timer1, Fast PWM, 8-bit mode on ATmega328P.
 *
 *  PWM frequency calculation:
 *    f_PWM = F_CPU / (prescaler × 256)
 *    f_PWM = 16,000,000 / (128 × 256) ≈ 488 Hz
 *
 *  WHY 8-bit (256 steps) and not 16-bit (65536 steps)?
 *    Temperature control does NOT need fine PWM resolution.
 *    1 step = 100%/256 ≈ 0.39% power — far finer than any thermal
 *    system can respond to. 8-bit keeps the math simple and the
 *    frequency in a comfortable range.
 *
 * ─────────────────────────────────────────────────────────────────────────── */

/** PWM output duty cycle: fully off. */
#define PWM_DUTY_MIN_PERCENT (0U)

/** PWM output duty cycle: fully on. */
#define PWM_DUTY_MAX_PERCENT (100U)

/** Internal timer top value for 8-bit resolution. */
#define PWM_TIMER_TOP (255U)

/**
 * @brief  Initialise the PWM timer peripheral and output pin.
 *         Must be called once before set_pwm_duty().
// ...existing code...
 *         Output starts at 0% duty (heater OFF — safe default).
 */
void PWM_Init(void);

/**
 * @brief  Set the heater PWM duty cycle.
 *
 * @param  duty_percent  Desired duty cycle, 0–100 (inclusive).
 *                       Values > 100 are clamped to 100.
 *                       0  = heater fully OFF.
 *                       100 = heater fully ON.
 */
void set_pwm_duty(uint8_t duty_percent);

#endif /* HAL_PWM_H */