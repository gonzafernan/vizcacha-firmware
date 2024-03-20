/**
 * @file encoder.c
 * @brief Encoder utils
 * @author Gonzalo G. Fernandez
 */

#include <stdbool.h>

#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include "tim.h"

/**
 * @brief H-Bridge structure
 */
// typedef struct {
//
// } hbridge_t;

/**
 * @brief H-Bridge init
 */
void hbridge_init(void) {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    TIM4->CCR1 = 0;
    HAL_GPIO_WritePin(L298N_IN1_GPIO_Port, L298N_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(L298N_IN2_GPIO_Port, L298N_IN2_Pin, GPIO_PIN_RESET);
}

void hbridge_set_dir(bool dir) {
    if (dir) {
        HAL_GPIO_WritePin(L298N_IN1_GPIO_Port, L298N_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(L298N_IN2_GPIO_Port, L298N_IN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(L298N_IN1_GPIO_Port, L298N_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(L298N_IN2_GPIO_Port, L298N_IN2_Pin, GPIO_PIN_SET);
    }
}

void hbridge_set_pwm(int value) {
    if (value < 0) {
        hbridge_set_dir(true);
        value = -value;
    } else {
        hbridge_set_dir(false);
    }
    TIM4->CCR1 = value;
}
