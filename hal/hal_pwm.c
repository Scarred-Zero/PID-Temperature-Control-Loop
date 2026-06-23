/**
 * @file    hal_pwm.c
 * @brief   PWM driver using Timer1 (OC1A, Arduino Pin 9) on ATmega328P.
 *          [PLATFORM-SPECIFIC: replace Timer/register config for your MCU]
 *
 *  Wiring:
 *    MCU Pin OC1A (Pin 9) → MOSFET Gate
 *    MOSFET Drain → Heater (-) terminal
 *    Heater (+) → Power supply (+)
 *    MOSFET Source → GND
 *    Gate resistor: 100Ω between MCU pin and MOSFET gate (limits ringing)
 */

#include "hal_pwm.h"
#include <stdint.h>

/* [PLATFORM-SPECIFIC] AVR register header */
#ifdef __AVR__
  #include <avr/io.h>
#endif

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Configure Timer1 for Fast PWM, 8-bit, non-inverting on OC1A.
 *
 * Register breakdown (ATmega328P datasheet §15):
 *
 *  TCCR1A:
 *    COM1A1=1, COM1A0=0  → Non-inverting: OC1A HIGH at BOTTOM, LOW on compare
 *    WGM11=0,  WGM10=1   → (with WGM13:12 below) → Mode 5: Fast PWM 8-bit
 *
 *  TCCR1B:
 *    WGM13=0,  WGM12=1   → completes Mode 5 selection
 *    CS12=1,   CS11=0, CS10=1 → prescaler = /128 → f_PWM ≈ 488 Hz
 *
 *  OCR1A:
 *    Compare register — controls duty cycle.
 *    duty = OCR1A / 255  (0 = always LOW, 255 = always HIGH)
 *
 *  WHY non-inverting mode?
 *    HIGH gate voltage turns N-channel MOSFET ON → heater ON.
 *    duty_percent=100 should mean full power, so HIGH=ON is intuitive.
 */
void PWM_Init(void)
{
#ifdef __AVR__
    /* Set OC1A (PB1, Arduino Pin 9) as output */
    DDRB |= (1 << PB1);   /* [PLATFORM-SPECIFIC] */

    /* Configure Timer1: Fast PWM 8-bit, non-inverting, prescaler /128 */
    TCCR1A = (1 << COM1A1)   /* non-inverting compare output on OC1A       */
           | (0 << COM1A0)
           | (0 << WGM11)    /* WGM bits [1:0] = 01 → Fast PWM 8-bit       */
           | (1 << WGM10);

    TCCR1B = (0 << WGM13)    /* WGM bits [3:2] = 01 (completes Mode 5)     */
           | (1 << WGM12)
           | (1 << CS12)     /* prescaler bits CS[2:0] = 101 → /128        */
           | (0 << CS11)
           | (1 << CS10);

    /* Start with heater OFF — always the safe power-on default */
    OCR1A = 0;             /* [PLATFORM-SPECIFIC] */
#endif
    /* [PLATFORM-SPECIFIC] STM32: call HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1)
       after MX_TIM1_Init(). Set duty via __HAL_TIM_SET_COMPARE().           */
}

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Update the PWM duty cycle on the heater output pin.
 *
 * Conversion:
 *   OCR1A = round(duty_percent / 100.0 × PWM_TIMER_TOP)
 *
 * WHY integer multiply-then-divide instead of float?
 *   Avoids pulling in the floating-point library just for a scale factor.
 *   (uint16_t)(duty × 255) / 100  stays in range [0, 255] — no overflow
 *   risk with uint16_t intermediate as long as duty ≤ 100.
 *
 * @param  duty_percent  0–100. Clamped internally; no UB on bad input.
 */
void set_pwm_duty(uint8_t duty_percent)
{
    /* Clamp input — defensive: PID output clamping is handled in pid.c too */
    if (duty_percent > PWM_DUTY_MAX_PERCENT)
    {
        duty_percent = PWM_DUTY_MAX_PERCENT;
    }

    /* Scale 0–100 → 0–255 using integer arithmetic (no float needed here)  */
    uint16_t ocr_value = ((uint16_t)duty_percent * PWM_TIMER_TOP)
                         / PWM_DUTY_MAX_PERCENT;

    /* Write compare register — takes effect at next timer BOTTOM crossing   */
#ifdef __AVR__
    OCR1A = (uint8_t)ocr_value;   /* [PLATFORM-SPECIFIC] */
#else
    (void)ocr_value;  /* host stub — nothing to write */
#endif
}