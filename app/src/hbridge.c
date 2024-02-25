/**
 * @file encoder.c
 * @brief Encoder utils
 * @author Gonzalo G. Fernandez
 */

#include "tim.h"

/**
 * @brief H-Bridge init
 */
void hbridge_init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    TIM4->CCR1 = 0;
}

void hbridge_set_pwm(int value)
{
    TIM4->CCR1 = value;
}