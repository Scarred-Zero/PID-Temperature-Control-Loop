/**
 * @file    logger.c
 * @brief   CSV logger — wraps printf (retarget to UART on MCU).
 *          [PLATFORM-SPECIFIC: replace printf with UART transmit if needed]
 */

#include "logger.h"
#include <stdio.h>

void Logger_PrintHeader(void)
{
  printf("timestamp_ms,setpoint_c,measured_c,error_c,pid_output_pct\r\n");
}

void Logger_LogSample(uint32_t timestamp_ms,
                      float setpoint_c,
                      float measured_c,
                      float pid_output_pct)
{
  float error_c = setpoint_c - measured_c;

  printf("%lu,%.2f,%.2f,%.3f,%.2f\r\n",
         (unsigned long)timestamp_ms,
         (double)setpoint_c,
         (double)measured_c,
         (double)error_c,
         (double)pid_output_pct);
}
/*
```

---

### Tuning Guide — Heuristic Step-Response Method

Before the final simulation, here is the tuning process encoded as a repeatable procedure:
```
STEP 1 — Proportional only (Ki=0, Kd=0)
────────────────────────────────────────
Start: Kp=1.0, Ki=0.0, Kd=0.0
Run sim. Observe:
  • Too slow to reach setpoint? → Double Kp. Repeat.
  • Oscillating and not settling? → Halve Kp.
Target: Reaches within ~5°C of setpoint in reasonable time,
        with small overshoot or none. Note this Kp.

STEP 2 — Add Integral to kill steady-state offset
──────────────────────────────────────────────────
Keep Kp from Step 1. Start: Ki=0.01
Run sim. Observe:
  • Steady-state error remains? → Double Ki.
  • Oscillation introduced? → Halve Ki.
Target: Settles to within 1°C of setpoint without sustained oscillation.

STEP 3 — Add Derivative to damp overshoot
──────────────────────────────────────────
Keep Kp, Ki from Steps 1–2. Start: Kd=0.1
Run sim. Observe:
  • Overshoot on approach? → Increase Kd.
  • Response becomes sluggish or noisy? → Reduce Kd or increase tau.
Target: Clean approach to setpoint, <2°C overshoot.

STEP 4 — Fine-trim for ±0.5°C steady-state
────────────────────────────────────────────
Small Ki increases tighten the steady-state band.
Increase tau if D-term noise appears in the CSV data.
*/