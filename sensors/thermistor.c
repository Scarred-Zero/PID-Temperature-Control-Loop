
#include "thermistor.h"
#include "hal_adc.h"
#include <math.h> /* logf()  — link with -lm on GCC */
#include <stdint.h>

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Convert a raw ADC count to thermistor resistance (Ohms).
 *
 * Derivation:
 *   V_adc   = VREF × R_therm / (R_ref + R_therm)
 *   Rearranging for R_therm:
 *   R_therm = R_ref × adc_count / (ADC_MAX - adc_count)
 *
 * WHY float? Resistance values span 1 kΩ–100 kΩ. Integer arithmetic would
 * lose precision in the division. Float is fine — this runs at 10 Hz.
 *
 * @param  adc_count  Raw ADC reading (0–ADC_MAX_COUNT).
 * @return Thermistor resistance in Ohms (float).
 */
static float adc_to_resistance(uint16_t adc_count)
{
    /* Guard: avoid divide-by-zero at ADC saturation (shorted thermistor) */
    if (adc_count >= ADC_MAX_COUNT)
    {
        return 0.0f; /* caller checks for fault */
    }

    return (float)THERMISTOR_R_REF_OHMS * (float)adc_count / ((float)ADC_MAX_COUNT - (float)adc_count);
}

/**
 * @brief  Convert thermistor resistance to temperature using the Beta equation.
 *
 * Beta Equation (rearranged for T):
 *
 *   1/T = 1/T_nom + (1/B) × ln(R / R_nom)
 *
 * Where:
 *   T     = unknown temperature (Kelvin)
 *   T_nom = nominal temperature (298.15 K = 25 °C)
 *   B     = Beta coefficient (material constant, ~3950 K for this thermistor)
 *   R     = measured resistance
 *   R_nom = resistance at T_nom (10 kΩ)
 *
 * WHY Beta and not full Steinhart-Hart?
 *   Steinhart-Hart needs 3 calibration constants (A, B, C) from a datasheet
 *   or lab measurement. The Beta equation needs only B and R_nom — simpler,
 *   and accurate to ±0.5 °C across 0–85 °C for most NTC thermistors.
 *
 * @param  resistance_ohms  Thermistor resistance in Ohms.
 * @return Temperature in Kelvin.
 */
static float resistance_to_kelvin(float resistance_ohms)
{
    float inv_T = (1.0f / THERMISTOR_T_NOM_K) + (1.0f / THERMISTOR_BETA_K) * logf(resistance_ohms / (float)THERMISTOR_R_NOM_OHMS);

    return 1.0f / inv_T;
}

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Public API: read temperature from thermistor-connected ADC channel.
 *
 * Pipeline:
 *   ADC_Read() → adc_to_resistance() → resistance_to_kelvin() → °C
 *
 * @return Temperature in °C, or -999.0f on hardware fault.
 */
float read_temperature(void)
{
    uint16_t adc_raw = ADC_Read(THERMISTOR_ADC_CHANNEL);
    float r_therm = adc_to_resistance(adc_raw);

    /* Fault detection: 0 Ω = shorted sensor, implausible result */
    if (r_therm <= 0.0f)
    {
        return -999.0f;
    }

    float temp_kelvin = resistance_to_kelvin(r_therm);
    float temp_celsius = temp_kelvin - KELVIN_OFFSET;

    return temp_celsius;
}