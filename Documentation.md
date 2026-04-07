# PID Temperature Controller Documentation

## Overview

This project is a modular PID temperature controller for embedded systems, designed for easy porting and extension. The codebase is split by function:

- **HAL (`hal/`)**: Abstracts hardware access (ADC, PWM, etc.)
- **Sensors (`sensors/`)**: Handles sensor-specific logic (e.g., thermistor conversion)
- **Control (`control/`)**: Implements control algorithms (PID)
- **App (`app/`)**: Contains main application files for each milestone
- **Tests (`tests/`)**: Unit tests and verification harnesses

## File Roles

- `hal_adc.c/h`: ADC initialization and reading
- `hal_pwm.h`: PWM output abstraction
- `thermistor.c/h`: Converts ADC readings to temperature
- `pid.c/h`: PID controller implementation
- `main_m1.c`: Reads and prints temperature
- `main_m2.c`: Steps PWM duty cycle
- `test_pid.c`: Verifies PID output

## Folder Structure

PID-temperature-controller/
│
├── hal/              # Hardware abstraction layer (ADC, PWM, timer, etc.)
│   ├── hal_adc.c
│   ├── hal_adc.h
│   ├── hal_pwm.c
│   ├── hal_pwm.h
│   ├── hal_timer.c
│   └── hal_timer.h
│
├── sensors/            # Sensor-specific code (thermistor, etc.)
│   ├── thermistor.c
│   └── thermistor.h
│
├── control/            # Control algorithms (PID, etc.)
│   ├── pid.c
│   └── pid.h
│
├── app/               # Application logic (main files for each milestone)
│   ├── main_m1.c
│   ├── main_m2.c
│   ├── main.c         # Real hardware closed-loop entry point
│   └── main_sim.c     # Host simulation + validation harness
│
├── sim/                # Simulation and logging modules
│   ├── thermal_sim.c
│   ├── thermal_sim.h
│   ├── logger.c
│   └── logger.h
│
├── include/            # (Optional) Shared headers, if needed
│
├── tests/              # Unit tests and verification harnesses
│   └── test_pid.c
│
├── README.md
├── Documentation.md
├── .gitignore
└── .gitkeep            # (optional, for empty folders)

## Adding New Features

- Add new hardware drivers in `hal/`
- Add new sensors in `sensors/`
- Add new control algorithms in `control/`
- Add new application logic in `app/`
- Add tests in `tests/`

## Build Instructions

- For host testing, use GCC:
  ```sh
  gcc -Wall -Wextra -o test_pid tests/test_pid.c control/pid.c -lm && ./test_pid
  ```
- For MCU, port HAL files and use your platform's toolchain.

## Control Flow
POWER ON
    │
    ▼
ADC_Init()    ← configure ADC clock and reference voltage
PWM_Init()    ← configure Timer1, set heater to 0% (safe)
Timer_Init()  ← configure Timer2, start 10ms ISR, enable interrupts
PID_Init()    ← store Kp/Ki/Kd, zero integral and state
    │
    ▼
┌─────────────────────────────────────────────────────┐
│  MAIN LOOP (infinite)                               │
│                                                     │
│  Wait... (CPU idle, Timer2 ISR fires every 10ms)   │
│  ISR counts to 10 → sets tick flag                  │
│                                                     │
│  Timer_TickOccurred()? NO  → keep waiting           │
│  Timer_TickOccurred()? YES → proceed                │
│          │                                          │
│          ▼                                          │
│  read_temperature()                                 │
│    ADC_Read(ch0) → raw count (0–1023)               │
│    adc_to_resistance() → ohms                       │
│    resistance_to_kelvin() → Kelvin                  │
│    - 273.15 → °C                                    │
│          │                                          │
│          ▼                                          │
│  safety_check(temp_c)                               │
│    fault?    → heater OFF, skip PID                 │
│    over-temp?→ heater OFF, skip PID                 │
│    OK?       → continue                             │
│          │                                          │
│          ▼                                          │
│  PID_Compute(setpoint=60.0, measured=temp_c)        │
│    error = 60.0 - temp_c                            │
│    P = Kp × error                                   │
│    I += Ki × dt × error  (clamped)                  │
│    D = filtered(Kd × Δerror / dt)                   │
│    output = P + I + D    (clamped 0–100)            │
│          │                                          │
│          ▼                                          │
│  set_pwm_duty(output)                               │
│    scale 0–100 → 0–255                              │
│    write OCR1A register                             │
│    Timer1 hardware generates PWM signal             │
│    MOSFET switches heater ON/OFF at 488Hz           │
│    Heater delivers average power = duty × P_max     │
│          │                                          │
│          ▼                                          │
│  log_printf(timestamp, setpoint, temp, duty)        │
│          │                                          │
│          └──────────────────────────────────────────┘
│                     back to top, wait for next tick
│
▼
(never reaches here — embedded systems run forever)

## Best Practices

- Keep hardware-specific code isolated in HAL
- Use modular, reusable code for sensors and control
- Document new files and functions
- Add tests for new features

---
For quick start, see `README.md`.
