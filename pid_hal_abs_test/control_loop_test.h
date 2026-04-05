#ifndef CONTROL_LOOP_TEST_H
#define CONTROL_LOOP_TEST_H

#define SETPOINT_CELSIUS (60.0f)
#define PID_KP_INITIAL (2.0f)
#define PID_KI_INITIAL (0.05f)
#define PID_KD_INITIAL (0.5f)
#define PID_TAU_INITIAL (5.0f)
#define PID_OUTPUT_MIN (0.0f)
#define PID_OUTPUT_MAX (100.0f)
#define TEMP_SAFETY_CUTOFF_C (85.0f)
#define TEMP_SENSOR_FAULT (-999.0f)
#define SAMPLE_PERIOD_MS (100U)      
#define SAMPLE_PERIOD_S (0.1f)      

#endif