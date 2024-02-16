/**
 * @file encoder.c
 * @brief Encoder utils
 * @author Gonzalo G. Fernandez
 */

#include <stdint.h>

#include "encoder.h"
#include "tim.h"

uint32_t enc_hist;

void encoder_init(void)
{
    enc_hist = 0;
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

uint32_t encoder_get_value(void)
{
    return ((TIM2->CNT) >> 2);
}

int32_t encoder_diff_value(void)
{
    uint32_t enc_new = encoder_get_value();
    int32_t enc_diff = enc_new - enc_hist;
    enc_hist = enc_new;
    return enc_diff * 1000;
}
