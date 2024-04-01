/**
 * @file encoder.c
 * @brief Encoder utils
 * @author Gonzalo G. Fernandez
 */

#include "encoder.h"
#include "tim.h"
#include <stdint.h>

void encoder_init(encoder_t *henc, void *context) {
    henc->hist_size = 1;
    for (uint8_t i = 0; i < henc->hist_size; i++) {
        henc->hist[i] = 0;
    }
    henc->handler = context;
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)henc->handler;
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

uint16_t encoder_get_value(encoder_t *henc) {
    // return (uint16_t)((TIM2->CNT) >> 2);
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)henc->handler;
    return (uint16_t)((htim->Instance->CNT) >> 2);
}

int16_t encoder_diff_value(encoder_t *henc) {
    int16_t enc_new = (int16_t)encoder_get_value(henc);
    int16_t enc_diff = enc_new - henc->hist[0];
    henc->hist[0] = enc_new;
    return enc_diff * 1000;
}
