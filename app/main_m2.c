
#include "hal/hal_pwm.h"
#include <stdint.h>
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

static const uint8_t DUTY_STEPS[] = {0U, 25U, 50U, 75U, 100U};
#define SWEEP_HOLD_PERIOD_MS (2000U)

int main(void)
{
    PWM_Init();
    // Add UART_Init() here for serial output if needed
    uint8_t step_count = (uint8_t)(sizeof(DUTY_STEPS) / sizeof(DUTY_STEPS[0]));
    while (1)
    {
        for (uint8_t i = 0U; i < step_count; i++)
        {
            uint8_t duty = DUTY_STEPS[i];
            set_pwm_duty(duty);
            printf("PWM duty set: %u%%\r\n", duty);
            delay_ms(SWEEP_HOLD_PERIOD_MS);
        }
    }
    return 0;
}
