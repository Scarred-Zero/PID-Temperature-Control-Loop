# PID Temperature Controller

This project implements a modular PID temperature controller for embedded systems. It is organized for clarity, portability, and scalability.

## Folder Structure

- `/hal` — Hardware abstraction layers (ADC, PWM, etc.)
- `/sensors` — Sensor-specific code (thermistor, etc.)
- `/control` — Control algorithms (PID)
- `/app` — Application logic (main files for each milestone)
- `/tests` — Unit tests and verification harnesses

## Build & Run

- Use a suitable toolchain for your MCU or host.
- Example (host test):
  ```sh
  gcc -Wall -Wextra -o test_pid tests/test_pid.c control/pid.c -lm && ./test_pid
  ```
- For MCU, port HAL files as needed and flash the appropriate main file.

## Milestones

1. **Temperature readout** — Serial print temperature from thermistor
2. **PWM output** — Step through PWM duty cycles
3. **PID control** — Closed-loop temperature regulation

## Contribution

- Keep hardware access in `/hal`
- Add new sensors to `/sensors`
- Add new control algorithms to `/control`
- Application logic goes in `/app`
- Add tests to `/tests`

---
For detailed documentation, see `Documentation.md`.
