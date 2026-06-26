/**
 * @file    main.c
 * @brief   Milestone 4: Full closed-loop PID temperature control.
 *
 *  Control loop pipeline (executes every SAMPLE_PERIOD_MS = 100 ms):
 *
 *    ┌─────────────┐     ┌─────────────┐     ┌──────────────┐
 *    │read_temp()  │────▶│ PID_Compute │────▶│set_pwm_duty()│
 *    │(thermistor) │     │             │     │  (heater)    │
 *    └─────────────┘     └─────────────┘     └──────────────┘
 *           ▲                                       │
 *           └───────────── thermal plant ───────────┘
 *                     (chamber heats up)
 *
 *  Timing:
 *    Timer2 ISR fires every 10 ms, counts to 10 → sets tick flag.
 *    Main loop polls flag — executes pipeline exactly every 100 ms.
 *    All heavy computation (float PID math) runs in main, not ISR.
 */

#include "hal/hal_adc.h"
#include "hal/hal_pwm.h"
#include "hal/hal_timer.h"
#include "sensors/thermistor.h"
#include "controllers/pid.h"
#include "controllers/control_loop.h"
#include <stdint.h>
#include <stdio.h>     /* printf — retarget to UART on MCU */

/* ── UART stub (replace with real UART driver on your MCU) ──────────────── */
/* [PLATFORM-SPECIFIC] Implement uart_printf() or retarget stdout to UART    */
#define log_printf(...)   printf(__VA_ARGS__)

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Apply safety cutoff: if temperature is dangerously high or
 *         sensor is faulty, immediately cut heater power and halt.
 *
 *  WHY a hard cutoff in firmware and not just relying on the PID?
 *  The PID could malfunction due to a software bug, corrupted gains,
 *  or a floating-point exception. A simple threshold check outside the
 *  PID is an independent, testable safety layer. In production systems
 *  this would also trip a hardware comparator or watchdog.
 *
 * @param  temp_c  Current temperature reading.
 * @return 1 if safe to continue, 0 if cutoff triggered.
 */
static uint8_t safety_check(float temp_c)
{
    if (temp_c <= TEMP_SENSOR_FAULT + 1.0f)   /* fault sentinel              */
    {
        set_pwm_duty(0U);
        log_printf("FAULT: sensor error — heater OFF\r\n");
        return 0U;
    }

    if (temp_c >= TEMP_SAFETY_CUTOFF_C)
    {
        set_pwm_duty(0U);
        log_printf("SAFETY: over-temp cutoff at %.1f C — heater OFF\r\n",
                   temp_c);
        return 0U;
    }

    return 1U;
}

/* ─────────────────────────────────────────────────────────────────────────── */

int main(void)
{
    /* ── 1. Peripheral Initialisation ─────────────────────────────────── */
    ADC_Init();     /* thermistor ADC                                         */
    PWM_Init();     /* heater PWM output — starts at 0% (safe)               */
    Timer_Init();   /* 100 ms control loop tick                               */

    /* [PLATFORM-SPECIFIC] UART_Init() here for serial logging               */

    /* ── 2. PID Initialisation ────────────────────────────────────────── */
    PID_Controller pid;
    PID_Init(&pid,
             PID_KP_INITIAL,
             PID_KI_INITIAL,
             PID_KD_INITIAL,
             PID_TAU_INITIAL,
             SAMPLE_PERIOD_S,    /* dt MUST equal actual loop period          */
             PID_OUTPUT_MIN,
             PID_OUTPUT_MAX);

    log_printf("PID Temperature Controller started\r\n");
    log_printf("Setpoint: %.1f C | Kp=%.2f Ki=%.3f Kd=%.2f\r\n",
               (double)SETPOINT_CELSIUS,
               (double)PID_KP_INITIAL,
               (double)PID_KI_INITIAL,
               (double)PID_KD_INITIAL);
    log_printf("----------------------------------------\r\n");

    /* ── 3. Main Control Loop ─────────────────────────────────────────── */
    uint32_t loop_count = 0U;   /* elapsed sample count for logging          */

    while (1)
    {
        /*
         * Busy-wait for the timer tick flag.
         * The CPU is idle here — on a real MCU with sleep modes, you
         * could call sleep_cpu() and wake on the ISR for lower power.
         * For clarity in this milestone, we poll.
         */
        if (!Timer_TickOccurred())
        {
            continue;   /* not time yet — keep waiting                        */
        }

        /* ── TICK: execute full pipeline ─────────────────────────────── */
        loop_count++;

        /* Step A: sense */
        float temp_c = read_temperature();

        /* Step B: safety check — always before PID, always before actuator */
        if (!safety_check(temp_c))
        {
            /* Latch: heater is OFF. Keep checking temperature but don't
             * re-enable heater automatically — require a system reset.
             * This prevents the safety cutoff from bouncing on/off.         */
            continue;
        }

        /* Step C: compute PID output */
        float pid_output = PID_Compute(&pid, SETPOINT_CELSIUS, temp_c);

        /* Step D: actuate — cast float 0.0–100.0 to uint8_t 0–100          */
        uint8_t duty = (uint8_t)pid_output;   /* truncation: 99.9 → 99, fine */
        set_pwm_duty(duty);

        /* Step E: log — every sample, tab-separated for easy parsing        */
        log_printf("t=%lus | SP=%.1f | T=%.2f | err=%.2f | PWM=%u%%\r\n",
                   (unsigned long)loop_count,
                   (double)SETPOINT_CELSIUS,
                   (double)temp_c,
                   (double)(SETPOINT_CELSIUS - temp_c),
                   duty);
    }

    return 0;  /* unreachable */
}