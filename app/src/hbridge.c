/**
 * @file hbridge.c
 * @brief H-Bridge utils
 * @author Gonzalo G. Fernandez
 */

#include <stdbool.h>

#include "hbridge.h"
#include "stm32f4xx_hal_gpio.h"

/**
 * @brief H-Bridge init
 */
void hbridge_init(hbridge_t *hbridge) {
    HAL_TIM_PWM_Start(hbridge->ena_pwm, hbridge->ena_chn);
    // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    switch (hbridge->ena_chn) {
    case TIM_CHANNEL_1:
        hbridge->ena_pwm->Instance->CCR1 = 0;
        break;
    case TIM_CHANNEL_2:
        hbridge->ena_pwm->Instance->CCR2 = 0;
    default:
        break;
    }
    // TIM4->CCR1 = 0;
    HAL_GPIO_WritePin(hbridge->in1_port, hbridge->in1_pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(L298N_IN1_GPIO_Port, L298N_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hbridge->in2_port, hbridge->in2_pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(L298N_IN2_GPIO_Port, L298N_IN2_Pin, GPIO_PIN_RESET);
}

static void hbridge_set_dir(hbridge_t *hbridge, bool dir) {
    if (dir) {
        HAL_GPIO_WritePin(hbridge->in1_port, hbridge->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(hbridge->in2_port, hbridge->in2_pin, GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(L298N_IN1_GPIO_Port, L298N_IN1_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(L298N_IN2_GPIO_Port, L298N_IN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(hbridge->in1_port, hbridge->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(hbridge->in2_port, hbridge->in2_pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(L298N_IN1_GPIO_Port, L298N_IN1_Pin, GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(L298N_IN2_GPIO_Port, L298N_IN2_Pin, GPIO_PIN_SET);
    }
}

void hbridge_set_pwm(hbridge_t *hbridge, int value) {
    if (value < 0) {
        hbridge_set_dir(hbridge, true);
        value = -value;
    } else {
        hbridge_set_dir(hbridge, false);
    }
    // TIM4->CCR1 = value;
    switch (hbridge->ena_chn) {
    case TIM_CHANNEL_1:
        hbridge->ena_pwm->Instance->CCR1 = value;
        break;
    case TIM_CHANNEL_2:
        hbridge->ena_pwm->Instance->CCR2 = value;
        break;
    default:
        break;
    }
}
