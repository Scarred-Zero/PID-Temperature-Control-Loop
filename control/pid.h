
#ifndef PID_H
#define PID_H

#include <stdint.h>

/* ── PID Controller State Structure ─────────────────────────────────────────
 *
 *  All fields are maintained internally by PID_Compute().
 *  Initialise with PID_Init() — do not write fields directly after init.
 * ─────────────────────────────────────────────────────────────────────────── */
typedef struct
{
    /* ── Tuning Parameters (set via PID_Init) ─────────────────────────── */

    float kp; /**< Proportional gain.
               *   Analogy: how hard you yank the steering wheel
               *   based on how far off the road you are.           */

    float ki; /**< Integral gain.
               *   Analogy: how much you correct for consistently
               *   drifting to one side over a long drive.          */

    float kd; /**< Derivative gain.
// ...existing code...
               *   Analogy: easing off the wheel as you approach
               *   the target lane — anticipating overshoot.        */

    /* ── Output Limits ────────────────────────────────────────────────── */

    float out_min; /**< Minimum controller output (e.g. 0.0 = 0% PWM).  */
    float out_max; /**< Maximum controller output (e.g. 100.0 = 100%).   */

    /* ── Derivative Filter ─────────────────────────────────────────────── */

    float tau; /**< Low-pass filter time constant for derivative (s).
                *   Higher tau = smoother D term, slower response.
                *   Typical starting value: tau = 0.1 × sample_period */

    /* ── Timing ────────────────────────────────────────────────────────── */

    float dt; /**< Sample period in seconds (e.g. 0.1 for 100 ms).
               *   MUST match the real loop execution period exactly.
               *   Mismatch here corrupts integral and derivative.   */

    /* ── Internal State (do not modify externally) ─────────────────────── */

    float integral;    /**< Running integral accumulator (clamped).          */
    float prev_error;  /**< Previous error — used for derivative calculation. */
    float prev_d_term; /**< Filtered derivative from last compute cycle.      */

} PID_Controller;

/* ── Public API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise a PID controller instance with tuning and limit values.
 *         Resets all internal state (integral, previous error, filter state).
 *         Safe to call at runtime to re-initialise after a setpoint jump.
 *
 * @param  pid       Pointer to a PID_Controller instance.
 * @param  kp        Proportional gain (>= 0.0).
 * @param  ki        Integral gain     (>= 0.0).
 * @param  kd        Derivative gain   (>= 0.0).
 * @param  tau       Derivative low-pass filter time constant (seconds, >= 0.0).
 * @param  dt        Sample period in seconds (> 0.0). Must match loop timing.
 * @param  out_min   Output lower clamp (e.g. 0.0 for PWM).
 * @param  out_max   Output upper clamp (e.g. 100.0 for PWM).
 */
void PID_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float tau, float dt,
              float out_min, float out_max);

/**
 * @brief  Compute one PID output sample.
 *         Call exactly once per sample period (dt).
 *
 * @param  pid        Pointer to an initialised PID_Controller.
 * @param  setpoint   Desired target value (e.g. 60.0 °C).
 * @param  measured   Current measured value from sensor (e.g. 57.3 °C).
 * @return Controller output, clamped to [out_min, out_max].
 *         Pass this directly to set_pwm_duty() (after casting to uint8_t).
 */
float PID_Compute(PID_Controller *pid, float setpoint, float measured);

/**
 * @brief  Reset integral and derivative state without changing tuning params.
 *         Use when the system is restarted or setpoint changes drastically,
 *         to prevent integral windup from a previous operating point.
 *
 * @param  pid  Pointer to an initialised PID_Controller.
 */
void PID_Reset(PID_Controller *pid);

#endif /* PID_H */