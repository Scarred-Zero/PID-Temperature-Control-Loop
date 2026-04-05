/**
 * @file    hal_adc.h
 * @brief   Hardware Abstraction Layer for ADC peripheral.
 *          Platform-specific implementations go in hal_adc.c.
 *          Upper layers (thermistor, PID) never touch registers directly.
 */

#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdint.h>

/* ── Configuration Constants ─────────────────────────────────────────────── */

/** ADC resolution in bits. ATmega328P = 10, STM32 = 12. */
#define ADC_RESOLUTION_BITS (10U)

/** Maximum ADC count value (2^BITS - 1). */
#define ADC_MAX_COUNT ((1U << ADC_RESOLUTION_BITS) - 1U) /* 1023 */

/** ADC reference voltage in millivolts. Adjust to your VREF. */
#define ADC_VREF_MV (5000U) /* 5.0 V */

/**
 * @brief  Initialise the ADC peripheral.
 *         Call once at startup before any read.
 */
void ADC_Init(void);

/**
 * @brief  Perform a single blocking ADC conversion on the given channel.
 * @param  channel  ADC input channel number (0–7 on ATmega328P).
 * @return 10-bit (or platform-resolution) raw ADC count.
 */
uint16_t ADC_Read(uint8_t channel);

#endif /* HAL_ADC_H */