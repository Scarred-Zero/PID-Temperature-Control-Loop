
#include "hal_pwm.h"
#include <stdint.h>
#include <stdio.h>

// Simple busy-delay (replace with hardware timer on real MCU)
static void delay_ms(uint32_t ms)
{
#ifdef __AVR__
#include <util/delay.h>
    while (ms--) _delay_ms(1);
#else
    (void)ms;
#endif
}

static const uint8_t DUTY_STEPS[] = {0U, 25U, 50U, 75U, 100U};
#define SWEEP_HOLD_PERIOD_MS (2000U)

int main(void)
{
    PWM_Init();
    // Add UART_Init() here for serial output if needed
    uint8_t step_count = (uint8_t)(sizeof(DUTY_STEPS) / sizeof(DUTY_STEPS[0]));
    while (1)
    {
        for (uint8_t i = 0U; i < step_count; i++)
        {
            uint8_t duty = DUTY_STEPS[i];
            set_pwm_duty(duty);
            printf("PWM duty set: %u%%\r\n", duty);
            delay_ms(SWEEP_HOLD_PERIOD_MS);
        }
    }
    return 0;
}
| `TCCR1A / TCCR1B` config | Sets Fast PWM 8-bit Mode 5, non-inverting, prescaler /128 | Mode 5 uses the hardware's internal 0xFF top — no extra register needed; prescaler /128 gives ~488 Hz |
| `OCR1A = 0` in Init | Heater starts OFF | Safe default — a heater that powers on at 100% on reset could cause thermal runaway before the PID takes control |
| Clamp in `set_pwm_duty()` | Caps input at 100 | Defense in depth: the PID output clamp (Milestone 3) is the first line; this is the second — hardware must never receive out-of-range values |
| Integer scale `(duty × 255) / 100` | Converts % to OCR count | Keeps the PWM driver float-free; uint16_t intermediate prevents overflow (max = 100 × 255 = 25500, well within uint16_t range of 65535) |
| Sweep in `main_m2.c` | Steps through 5 duty levels | Lets you verify linearity with a scope — each step should show a proportionally wider pulse |

---

## MOSFET Wiring Note
```
    +12V (heater supply)
        │
    [HEATER]
        │
    MOSFET Drain
    MOSFET Gate ──[100Ω]── OC1A (MCU Pin 9)
    MOSFET Source ──────── GND
        │
        GND (shared with MCU GND — critical)
```

The 100Ω gate resistor damps switching oscillations. The shared GND between MCU and MOSFET supply is **mandatory** — without it, the gate-source voltage reference is undefined.

---
```
╔══════════════════════════════════════════════════════════════════╗
║              ✅  MILESTONE 2 CHECKPOINT                          ║
╠══════════════════════════════════════════════════════════════════╣
║  WHAT WAS BUILT:                                                 ║
║  • hal_pwm.h / hal_pwm.c — portable PWM abstraction             ║
║  • Timer1 Fast PWM @ ~488 Hz on OC1A (Arduino Pin 9)            ║
║  • set_pwm_duty(uint8_t) — clean 0–100% API                     ║
║  • main_m2.c — automated duty-cycle sweep for verification       ║
╠══════════════════════════════════════════════════════════════════╣
║  VERIFICATION TEST:                                              ║
║                                                                  ║
║  Option A — Oscilloscope:                                        ║
║    Probe OC1A pin. Confirm pulse widths at each step:            ║
║      0%  → constant LOW                                          ║
║     25%  → HIGH ≈ 0.51 ms  (of ~2.05 ms period)                 ║
║     50%  → HIGH ≈ 1.02 ms                                        ║
║     75%  → HIGH ≈ 1.54 ms                                        ║
║    100%  → constant HIGH                                         ║
║                                                                  ║
║  Option B — LED + 220Ω resistor (no oscilloscope):              ║
║    Wire LED between OC1A and GND (via 220Ω).                    ║
║    Confirm LED steps through 5 visible brightness levels.        ║
║    PASS: clear brightness difference at each 2-second hold.      ║
╠══════════════════════════════════════════════════════════════════╣

*/