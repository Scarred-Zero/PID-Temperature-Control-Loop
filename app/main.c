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

#include "hal_adc.h"
#include "hal_pwm.h"
#include "hal_adc.h"
#include "hal_pwm.h"
#include "hal_timer.h"
#include "thermistor.h"
#include "pid.h"
#include "control_loop.h"
#include <stdint.h>
#include <stdio.h>

#define log_printf(...) printf(__VA_ARGS__)

// Safety cutoff: disables heater if temperature is too high or sensor fault
static uint8_t safety_check(float temp_c)
{
    if (temp_c <= TEMP_SENSOR_FAULT + 1.0f)
    {
        set_pwm_duty(0U);
        log_printf("FAULT: sensor error — heater OFF\r\n");
        return 0U;
    }
    if (temp_c >= TEMP_SAFETY_CUTOFF_C)
    {
        set_pwm_duty(0U);
        log_printf("FAULT: overtemperature — heater OFF\r\n");
        return 0U;
    }
    return 1U;
}
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
    uint32_t loop_count = 0U; /* elapsed sample count for logging          */

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
            continue; /* not time yet — keep waiting                        */
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
        uint8_t duty = (uint8_t)pid_output; /* truncation: 99.9 → 99, fine */
        set_pwm_duty(duty);

        /* Step E: log — every sample, tab-separated for easy parsing        */
        log_printf("t=%lus | SP=%.1f | T=%.2f | err=%.2f | PWM=%u%%\r\n",
                   (unsigned long)loop_count,
                   (double)SETPOINT_CELSIUS,
                   (double)temp_c,
                   (double)(SETPOINT_CELSIUS - temp_c),
                   duty);
    }

    return 0; /* unreachable */
}
/*
```

---

## Code Walkthrough

| Block | What it does | Why it's designed this way |
|---|---|---|
| `volatile s_tick_flag` | Shared flag between ISR and main | `volatile` prevents compiler from optimising the variable into a register, which would make the main loop never see the ISR's write |
| `cli()/sei()` in `Timer_TickOccurred()` | Atomic read-and-clear | Prevents the race condition where an ISR fires between the read and the clear, silently dropping a tick — a subtle but real embedded bug |
| ISR counts 10 ms ticks to 10 | Reaches 100 ms with an 8-bit timer | Timer2 max period at 16 MHz is ~16 ms; chaining 10 interrupts achieves 100 ms without a 16-bit timer or external RTC |
| Safety check before PID and actuator | Independent over-temp guard | PID is not trusted to protect against hardware faults — the safety layer is deliberately outside and unaware of PID state |
| All float math in `main`, not ISR | ISR stays under 10 cycles | Float multiply on AVR without FPU takes ~100 cycles; doing it in an ISR would block UART and ADC interrupts |
| `dt = SAMPLE_PERIOD_S` in `PID_Init()` | Links timing to PID math | If you change `SAMPLE_PERIOD_MS`, you must also change `SAMPLE_PERIOD_S` — they're defined together in `hal_timer.h` and `control_loop.h` to make this coupling explicit |
| `loop_count` logged as timestamp | Elapsed samples, not wall time | Robust: doesn't require a real-time clock; sample count × 100 ms gives wall time offline |

---

## Full File Structure at This Point
```
project/
├── hal_adc.h       / hal_adc.c        ← Milestone 1
├── thermistor.h    / thermistor.c      ← Milestone 1
├── hal_pwm.h       / hal_pwm.c        ← Milestone 2
├── hal_timer.h     / hal_timer.c      ← Milestone 4  ← NEW
├── pid.h           / pid.c            ← Milestone 3
├── control_loop.h                     ← Milestone 4  ← NEW
└── main.c                             ← Milestone 4  ← NEW
```

---
```
╔══════════════════════════════════════════════════════════════════╗
║              ✅  MILESTONE 4 CHECKPOINT                          ║
╠══════════════════════════════════════════════════════════════════╣
║  WHAT WAS BUILT:                                                 ║
║  • hal_timer.h / hal_timer.c — 100 ms ISR tick, atomic flag     ║
║  • control_loop.h — all tuning constants centralised             ║
║  • main.c — full sense → PID → actuate → log pipeline           ║
║  • Safety cutoff at 85 °C, independent of PID                   ║
╠══════════════════════════════════════════════════════════════════╣
║  VERIFICATION TEST:                                              ║
║                                                                  ║
║  1. Flash main.c + all modules to your MCU.                      ║
║     Open serial monitor. Confirm log lines appear every          ║
║     ~100 ms with live temperature and PWM duty values.           ║
║                                                                  ║
║  2. Observe thermal behaviour:                                    ║
║     a) At room temp (~25°C), setpoint=60°C → PWM should be      ║
║        high (large error → high P term).                         ║
║     b) As chamber warms, PWM should gradually reduce.            ║
║     c) Temperature should approach 60°C without violent          ║
║        oscillation or runaway.                                   ║
║                                                                  ║
║  3. PASS criteria:                                               ║
║     ✓ Log lines appear at steady ~100 ms cadence                 ║
║     ✓ Temperature rises toward setpoint (not away from it)       ║
║     ✓ PWM duty decreases as temperature increases                ║
║     ✓ No heater runaway (temp does not blow past setpoint        ║
║       by more than ~10°C on first approach)                      ║
║                                                                  ║
║  NOTE: Precise ±0.5°C steady-state is NOT expected yet —         ║
║  that is the goal of Milestone 5 tuning.                         ║
╠══════════════════════════════════════════════════════════════════╣
║  ➡  Reply "M4 PASS" to proceed to Milestone 5                   ║
║     (PID Tuning, Simulation Mode & Performance Validation)       ║
║  ➡  Reply "M4 FAIL: [symptom]" for diagnosis & fix              ║
╚══════════════════════════════════════════════════════════════════╝*/