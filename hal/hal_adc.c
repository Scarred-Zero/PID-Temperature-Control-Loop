/**
 * @file    hal_adc.c
 * @brief   ADC peripheral driver for ATmega328P.
 *          [PLATFORM-SPECIFIC: replace register names for your MCU]
 */

#include "hal_adc.h"
#include <stdint.h>

/* ── AVR register headers (remove for non-AVR platforms) ─────────────────── */
/* [PLATFORM-SPECIFIC: replace with your MCU's CMSIS / HAL headers]          */
#ifdef __AVR__
#include <avr/io.h>
#endif

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Configure the ADC: reference = AVCC, prescaler = /128, right-adjust.
 *
 * WHY prescaler /128?
 *   ADC accuracy on the ATmega328P requires a clock between 50 kHz–200 kHz.
 *   At F_CPU = 16 MHz:  16,000,000 / 128 = 125,000 Hz — squarely in range.
 */
void ADC_Init(void)
{
#ifdef __AVR__
    /* ADMUX: REFS1:0 = 01 → AVCC as reference; ADLAR = 0 → right-justify    */
    ADMUX = (1 << REFS0);

    /* ADCSRA: ADEN=1 enable; prescaler bits ADPS2:0 = 111 → divide by 128   */
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
#endif
    /* [PLATFORM-SPECIFIC] STM32 example: call MX_ADC1_Init() or HAL_ADC_Init() */
}

/* ─────────────────────────────────────────────────────────────────────────── */

/**
 * @brief  Read one ADC channel using a polled (blocking) conversion.
 *
 * WHY blocking here?
 *   Temperature changes slowly (thermal time constant >> ms). A blocking read
 *   every 100 ms is perfectly acceptable and keeps the driver simple. In a
 *   high-throughput system you would use DMA or interrupt-driven ADC instead.
 *
 * @param  channel  ADC channel (0–7).
 * @return Raw 10-bit ADC count.
 */
uint16_t ADC_Read(uint8_t channel)
{
#ifdef __AVR__
    /* Select channel; mask lower 4 bits to prevent accidental register damage */
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);

    /* Start conversion by setting ADSC bit */
    ADCSRA |= (1 << ADSC);

    /* Poll until ADSC clears — hardware clears it when conversion is done    */
    while (ADCSRA & (1 << ADSC))
    {
        /* busy-wait — acceptable for 100 ms thermal loop */
    }

    /* ADC is a 16-bit register; reading it atomically returns the 10-bit result */
    return ADC; /* [PLATFORM-SPECIFIC] */
#else
    (void)channel;
    return 512U; /* stub: mid-scale, for host-based unit tests               */
#endif
}