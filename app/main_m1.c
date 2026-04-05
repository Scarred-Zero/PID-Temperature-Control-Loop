
#include "hal_adc.h"
#include "thermistor.h"
#include <stdio.h>

// Simple busy-delay (replace with hardware timer on real MCU)
static void delay_ms(uint32_t ms)
{
#ifdef __AVR__
#include <util/delay.h>
    while (ms--) _delay_ms(1);
#else
    (void)ms;
#endif
}

int main(void)
{
    ADC_Init();
    // Add UART_Init() here for serial output if needed
    while (1)
    {
        float temp_c = read_temperature();
        if (temp_c <= -999.0f)
            printf("FAULT: sensor open or shorted\r\n");
        else
            printf("TEMP: %.2f C\r\n", temp_c);
        delay_ms(500U); // print twice per second
    }
    return 0;
}