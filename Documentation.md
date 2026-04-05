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

## Best Practices

- Keep hardware-specific code isolated in HAL
- Use modular, reusable code for sensors and control
- Document new files and functions
- Add tests for new features

---
For quick start, see `README.md`.
