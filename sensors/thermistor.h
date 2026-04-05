
#ifndef THERMISTOR_H
#define THERMISTOR_H

#include <stdint.h>

/* ── Thermistor & Circuit Parameters ────────────────────────────────────────
 *
 *  Circuit (voltage divider):
 *
 *      VREF ──┤ R_REF (10 kΩ) ├──┬──┤ THERMISTOR (NTC) ├── GND
 *                                 │
 *                               ADC_PIN
 *
 *  V_ADC = VREF * R_THERM / (R_REF + R_THERM)
 *  → R_THERM = R_REF * V_ADC / (VREF - V_ADC)
 *            = R_REF * ADC_count / (ADC_MAX - ADC_count)
 *
 * ─────────────────────────────────────────────────────────────────────────── */

/** ADC channel the thermistor voltage divider is wired to. */
#define THERMISTOR_ADC_CHANNEL (0U)

/** Pull-up (reference) resistor value in Ohms. */
#define THERMISTOR_R_REF_OHMS (10000U) /* 10 kΩ */

/** Thermistor nominal resistance at T_NOM, in Ohms. */
#define THERMISTOR_R_NOM_OHMS (10000U) /* 10 kΩ @ 25 °C */

/** Nominal temperature for R_NOM, in Kelvin. 25 °C = 298.15 K */
#define THERMISTOR_T_NOM_K (298.15f)

/** Beta coefficient of the NTC thermistor (from datasheet). */
#define THERMISTOR_BETA_K (3950.0f) /* Kelvin */
// ...existing code...

/** Conversion offset: 0 °C in Kelvin */
#define KELVIN_OFFSET (273.15f)

/**
 * @brief  Read the current temperature from the thermistor.
 * @return Temperature in degrees Celsius (float).
 *         Returns -999.0f on a hardware fault (ADC saturated).
 */
float read_temperature(void);

#endif /* THERMISTOR_H */