/**
 * @file    main_sim_test.c
 * @brief   Milestone 5: software-only PID simulation with CSV logging.
 *
 *  Compile and run entirely on host PC — no MCU required:
 *    gcc -Wall -Wextra -o pid_sim main_sim_test.c pid.c thermal_sim.c logger_test.c -lm
 *    ./pid_sim > run.csv
 *    # then open run.csv in Excel / LibreOffice / Python
 *
 *  Simulation parameters:
 *    Duration:      SIM_DURATION_S seconds of simulated time
 *    Sample period: SAMPLE_PERIOD_S (matches real hardware loop)
 *    Setpoint:      SETPOINT_CELSIUS
 *
 *  Validation target:
 *    For at least STEADY_STATE_WINDOW_S consecutive seconds after
 *    the system first enters the ±STEADY_STATE_BAND_C band,
 *    ALL samples must remain within that band.
 */

#include "pid_test.h"
#include "thermal_sim_test.h"
#include "logger_test.h"
#include "control_loop_test.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h> /* fabsf */

/* ── Simulation Configuration ────────────────────────────────────────────── */

/** Total simulated run time in seconds. */
#define SIM_DURATION_S (400U)

/** Steady-state error band for validation (±°C). */
#define STEADY_STATE_BAND_C (0.5f)

/**
 * Consecutive seconds within the band required to claim steady-state.
 * At SAMPLE_PERIOD_S = 0.1s, 30 seconds = 300 consecutive samples.
 */
#define STEADY_STATE_WINDOW_S (30U)

/** Samples per second at our control rate */
#define SAMPLES_PER_SECOND ((uint32_t)(1.0f / SAMPLE_PERIOD_S))

/** Minimum consecutive in-band samples to pass validation */
#define STEADY_STATE_WINDOW_SAMPLES (STEADY_STATE_WINDOW_S * SAMPLES_PER_SECOND)

/* ── Tuned PID Parameters (from tuning guide above) ─────────────────────── */
#define PID_KP_TUNED (3.5f)
#define PID_KI_TUNED (0.08f)
#define PID_KD_TUNED (1.2f)
#define PID_TAU_TUNED (3.0f)

// Badly tuned controller — Tested for bad tuning
// #define PID_KP_TUNED (10.0f)   // too aggressive
// #define PID_KI_TUNED (0.5f)    // too much integral
// #define PID_KD_TUNED (0.0f)    // no damping
// #define PID_TAU_TUNED (3.0f)

/* ─────────────────────────────────────────────────────────────────────────── */

int main(void)
{
        /* ── Initialise subsystems ─────────────────────────────────────────── */
        PID_Controller pid;
        PID_Init(&pid,
                 PID_KP_TUNED,
                 PID_KI_TUNED,
                 PID_KD_TUNED,
                 PID_TAU_TUNED,
                 SAMPLE_PERIOD_S,
                 PID_OUTPUT_MIN,
                 PID_OUTPUT_MAX);

        ThermalSim sim;
        Sim_Init(&sim);

        Logger_PrintHeader();

        /* ── Validation state ──────────────────────────────────────────────── */
        uint32_t consecutive_in_band = 0U; /* samples currently within ±band  */
        uint32_t best_in_band_run = 0U;    /* longest streak seen so far      */
        uint32_t settle_time_ms = 0U;      /* when steady-state first achieved */
        uint8_t settled = 0U;              /* flag: ±0.5°C target met         */

        /* ── Total sample count ────────────────────────────────────────────── */
        uint32_t total_samples = (uint32_t)(SIM_DURATION_S / SAMPLE_PERIOD_S);

        /* ── Main simulation loop ──────────────────────────────────────────── */
        for (uint32_t i = 0U; i < total_samples; i++)
        {
                uint32_t timestamp_ms = i * (uint32_t)(SAMPLE_PERIOD_S * 1000.0f);

                /* Step A: read temperature from simulation model */
                float temp_c = sim.temperature_c;

                /* Step B: compute PID output */
                float pid_out = PID_Compute(&pid, SETPOINT_CELSIUS, temp_c);

                /* Step C: apply output to simulated heater */
                uint8_t duty = (uint8_t)pid_out;
                Sim_Step(&sim, duty, SAMPLE_PERIOD_S);

                /* Step D: log CSV row */
                Logger_LogSample(timestamp_ms, SETPOINT_CELSIUS, temp_c, pid_out);

                /* Step E: validation tracking ─────────────────────────────────── */
                float error = fabsf(SETPOINT_CELSIUS - temp_c);

                if (error <= STEADY_STATE_BAND_C)
                {
                        consecutive_in_band++;

                        if (consecutive_in_band > best_in_band_run)
                        {
                                best_in_band_run = consecutive_in_band;
                        }

                        /* First time we hit the required window — record settle time  */
                        if (!settled &&
                            consecutive_in_band >= STEADY_STATE_WINDOW_SAMPLES)
                        {
                                settled = 1U;
                                settle_time_ms = timestamp_ms;
                        }
                }
                else
                {
                        consecutive_in_band = 0U; /* streak broken — reset counter   */
                }
        }

        /* ── Validation Report (printed to stderr so it doesn't pollute CSV) ─ */
        fprintf(stderr, "\n");
        fprintf(stderr, "========================================\n");
        fprintf(stderr, "  PID SIMULATION VALIDATION REPORT\n");
        fprintf(stderr, "========================================\n");
        fprintf(stderr, "  Tuning:  Kp=%.2f  Ki=%.3f  Kd=%.2f  tau=%.1f\n",
                (double)PID_KP_TUNED,
                (double)PID_KI_TUNED,
                (double)PID_KD_TUNED,
                (double)PID_TAU_TUNED);
        fprintf(stderr, "  Setpoint:          %.1f °C\n",
                (double)SETPOINT_CELSIUS);
        fprintf(stderr, "  Steady-state band: ±%.1f °C\n",
                (double)STEADY_STATE_BAND_C);
        fprintf(stderr, "  Required window:   %u s (%u samples)\n",
                STEADY_STATE_WINDOW_S,
                STEADY_STATE_WINDOW_SAMPLES);
        fprintf(stderr, "  Simulation time:   %u s\n", SIM_DURATION_S);
        fprintf(stderr, "----------------------------------------\n");

        if (settled)
        {
                fprintf(stderr, "  Settled at:        %lu ms (%.1f s)\n",
                        (unsigned long)settle_time_ms,
                        (double)settle_time_ms / 1000.0);
                fprintf(stderr, "  Longest in-band run: %lu s\n",
                        (unsigned long)(best_in_band_run / SAMPLES_PER_SECOND));
                fprintf(stderr, "  RESULT: PASS ✓  — steady-state error < ±%.1f°C\n",
                        (double)STEADY_STATE_BAND_C);
                fprintf(stderr, "          maintained for >%us\n",
                        STEADY_STATE_WINDOW_S);
        }
        else
        {
                fprintf(stderr, "  Longest in-band run: %lu s\n",
                        (unsigned long)(best_in_band_run / SAMPLES_PER_SECOND));
                fprintf(stderr, "  RESULT: FAIL ✗  — did not achieve %u s of\n",
                        STEADY_STATE_WINDOW_S);
                fprintf(stderr, "          steady-state within ±%.1f°C\n",
                        (double)STEADY_STATE_BAND_C);
                fprintf(stderr, "  ACTION: Increase Ki slightly and re-run.\n");
        }

        fprintf(stderr, "========================================\n");

        return settled ? 0 : 1; /* exit code 0 = PASS, 1 = FAIL (scriptable) */
}