
#include "controllers/pid.h"
#include <stdio.h>
#include <math.h> /* fabsf */

/* ── Test Parameters ──────────────────────────────────────────────────────── */
#define TEST_KP (2.0f)
#define TEST_KI (0.5f)
#define TEST_KD (0.1f)
#define TEST_TAU (0.0f) /* filter OFF for clean hand-calc */
#define TEST_DT (1.0f)  /* 1 second per step              */
#define TEST_OUT_MIN (-100.0f)
#define TEST_OUT_MAX (100.0f)
#define TEST_TOLERANCE (0.001f) /* acceptable float rounding delta */

/* ── Fixed Error Sequence ─────────────────────────────────────────────────── */
/*
 *  We feed setpoint=10.0, measured varies to produce these errors:
 *    Step 0: error = 10.0
 *    Step 1: error =  6.0
 *    Step 2: error =  3.0
 *
 *  Hand-calculation (tau=0, so alpha=0, beta=Kd/dt=0.1/1.0=0.1):
 *
 *  STEP 0:
 *    P = 2.0 × 10.0              = 20.000
 *    I = 0.0 + 0.5×1.0×10.0     = +5.000  → integral = 5.000
 *    D = 0×0 + 0.1×(10.0 - 0.0) = +1.000  (prev_error=0 at start)
 *    output = 20.0 + 5.0 + 1.0  = 26.000
 *
 *  STEP 1:
 *    P = 2.0 × 6.0               = 12.000
 *    I = 5.0 + 0.5×1.0×6.0      =  8.000  → integral = 8.000
 *    D = 0×1.0 + 0.1×(6.0-10.0) = -0.400
 *    output = 12.0 + 8.0 - 0.4  = 19.600
 *
 *  STEP 2:
 *    P = 2.0 × 3.0               =  6.000
 *    I = 8.0 + 0.5×1.0×3.0      = 9.500   → integral = 9.500
 *    D = 0×(-0.4) + 0.1×(3.0-6.0) = -0.300
 *    output = 6.0 + 9.5 - 0.3   = 15.200
 */

typedef struct
{
    float setpoint;
    float measured;
    float expected_output;
} TestStep;

static const TestStep TEST_SEQUENCE[] = {
    {10.0f, 0.0f, 26.000f}, /* step 0 */
    {10.0f, 4.0f, 19.600f}, /* step 1 */
    {10.0f, 7.0f, 15.200f}, /* step 2 */
};

#define NUM_TEST_STEPS (sizeof(TEST_SEQUENCE) / sizeof(TEST_SEQUENCE[0]))

/* ─────────────────────────────────────────────────────────────────────────── */

int main(void)
{
    PID_Controller pid;
    PID_Init(&pid,
             TEST_KP, TEST_KI, TEST_KD,
             TEST_TAU, TEST_DT,
             TEST_OUT_MIN, TEST_OUT_MAX);

    printf("\n=== Milestone 3: PID Unit Test ===\n");
    printf("Kp=%.1f  Ki=%.1f  Kd=%.1f  tau=%.1f  dt=%.1f\n\n",
           TEST_KP, TEST_KI, TEST_KD, TEST_TAU, TEST_DT);

    printf("%-6s %-10s %-10s %-12s %-12s %-6s\n",
           "Step", "Setpoint", "Measured", "Expected", "Actual", "Result");
    printf("---------------------------------------------------------------\n");

    uint8_t all_pass = 1U;

    for (uint8_t i = 0U; i < (uint8_t)NUM_TEST_STEPS; i++)
    {
        float actual = PID_Compute(&pid,
                                   TEST_SEQUENCE[i].setpoint,
                                   TEST_SEQUENCE[i].measured);

        float delta = fabsf(actual - TEST_SEQUENCE[i].expected_output);
        uint8_t pass = (delta <= TEST_TOLERANCE) ? 1U : 0U;

        if (!pass)
        {
            all_pass = 0U;
        }

        printf("%-6u %-10.2f %-10.2f %-12.3f %-12.3f %s\n",
               i,
               TEST_SEQUENCE[i].setpoint,
               TEST_SEQUENCE[i].measured,
               TEST_SEQUENCE[i].expected_output,
               actual,
               pass ? "PASS" : "FAIL <<<");
    }

    printf("---------------------------------------------------------------\n");
    printf("Overall: %s\n\n", all_pass ? "ALL PASS" : "FAILURES DETECTED");

    /* Anti-windup test: force integral to saturate, confirm clamping */
    printf("=== Anti-Windup Clamp Test ===\n");
    PID_Reset(&pid);

    /* Feed large sustained error — integral should saturate at out_max=100 */
    for (uint8_t j = 0U; j < 50U; j++)
    {
        PID_Compute(&pid, 100.0f, 0.0f); /* error=100, every step */
    }

    printf("Integral after 50 steps of error=100: %.2f  (expect <= %.1f)\n",
           pid.integral, TEST_OUT_MAX);
    printf("Anti-windup: %s\n",
           (pid.integral <= TEST_OUT_MAX) ? "PASS" : "FAIL");
    printf("\n");

    return 0;
}
