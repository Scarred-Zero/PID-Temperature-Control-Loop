
#include "pid.h"
#include <stddef.h> /* NULL */

/* ── Internal helper: clamp a float to [low, high] ─────────────────────── */
static float clamp(float value, float low, float high)
{
    if (value > high)
        return high;
    if (value < low)
        return low;
    return value;
}

/* ─────────────────────────────────────────────────────────────────────────── */
// ...existing code...

/**
 * @brief  Initialise PID instance. See pid.h for parameter documentation.
 */
void PID_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float tau, float dt,
              float out_min, float out_max)
{
    if (pid == NULL)
    {
        return;
    } /* defensive null-check */

    /* Store tuning parameters */
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->tau = tau;
    pid->dt = dt;
    pid->out_min = out_min;
    pid->out_max = out_max;

    /* Zero all runtime state — clean slate on every init */
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_d_term = 0.0f;
}

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Compute one PID control step.
 *
 * ── ALGORITHM DETAIL ────────────────────────────────────────────────────────
 *
 *  Error:
 *    e[n] = setpoint - measured
 *
 *  Proportional term:
 *    P = Kp × e[n]
 *
 *  Integral term (trapezoidal / bilinear integration for accuracy):
 *    I_acc += Ki × dt × e[n]
 *    Anti-windup: I_acc is clamped to [out_min, out_max] BEFORE summing.
 *
 *    WHY clamp the integral, not just the output?
 *    Clamping only the final output lets the integrator keep growing
 *    unseen — "windup". When the error finally reverses, the bloated
 *    integral drives a long, slow overshoot. Clamping the accumulator
 *    directly prevents it from ever exceeding what the actuator can deliver.
 *
 *  Derivative term (filtered, acting on error — not measurement):
 *
 *    Raw derivative: d_raw = Kd × (e[n] - e[n-1]) / dt
 *
 *    Low-pass filter (discrete EWA):
 *    d[n] = (tau / (tau + dt)) × d[n-1]
 *         + (Kd / (tau + dt)) × (e[n] - e[n-1])
 *
 *    This is the bilinear-transform approximation of:
 *       D(s) = Kd × s / (tau × s + 1)
 *    — a standard first-order low-pass on the derivative.
 *
 *    WHY filter?
 *    Differentiation amplifies high-frequency content. A 12-bit ADC
 *    with 1 LSB noise produces a derivative spike of 1/dt counts per
 *    second. At dt=0.1s that's 10 counts/s of jitter straight into
 *    your actuator. The low-pass filter rolls that off above 1/(2π×tau).
 *
 *    WHY differentiate error (not measurement)?
 *    Differentiating measurement gives "derivative kick" — a sudden
 *    large D pulse whenever the setpoint changes. Acting on error
 *    shares that kick proportionally, which is better for step changes.
 *    (Note: some implementations use -d(measurement)/dt instead, which
 *    completely eliminates setpoint kick — either is valid; error-based
 *    is simpler to understand and tune.)
 *
 *  Output:
 *    output = P + I + D
 *    output = clamp(output, out_min, out_max)
 *
 * ────────────────────────────────────────────────────────────────────────────
 */
float PID_Compute(PID_Controller *pid, float setpoint, float measured)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    /* ── 1. Error ─────────────────────────────────────────────────────── */
    float error = setpoint - measured;

    /* ── 2. Proportional Term ─────────────────────────────────────────── */
    float p_term = pid->kp * error;

    /* ── 3. Integral Term with Anti-Windup ────────────────────────────── */
    pid->integral += pid->ki * pid->dt * error;

    /*
     * Anti-windup clamp: bound the accumulator to the output range.
     * This means the integral can NEVER contribute more than the full
     * output range — the actuator's physical limit.
     */
    pid->integral = clamp(pid->integral, pid->out_min, pid->out_max);

    float i_term = pid->integral;

    /* ── 4. Derivative Term with Low-Pass Filter ──────────────────────── */
    /*
     * Discrete low-pass filtered derivative.
     * Coefficients derived from bilinear transform of D(s) = Kd*s/(tau*s+1):
     *
     *   alpha = tau / (tau + dt)     — "memory" weight (how much last D matters)
     *   beta  = Kd  / (tau + dt)     — "new data" weight
     *
     * When tau=0: alpha=0, beta=Kd/dt → pure unfiltered derivative.
     * When tau>>dt: alpha→1 → heavily smoothed, slow-reacting derivative.
     */
    float alpha = pid->tau / (pid->tau + pid->dt);
    float beta = pid->kd / (pid->tau + pid->dt);

    float d_term = alpha * pid->prev_d_term + beta * (error - pid->prev_error);

    /* ── 5. Combine Terms & Clamp Output ─────────────────────────────── */
    float output = p_term + i_term + d_term;
    output = clamp(output, pid->out_min, pid->out_max);

    /* ── 6. Save State for Next Iteration ────────────────────────────── */
    pid->prev_error = error;
    pid->prev_d_term = d_term;

    return output;
}

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Reset internal integrator and derivative state.
 *         Tuning parameters (Kp, Ki, Kd, tau, dt) are preserved.
 */
void PID_Reset(PID_Controller *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_d_term = 0.0f;
}