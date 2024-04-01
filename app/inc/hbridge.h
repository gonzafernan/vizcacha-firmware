/**
 * @file hbridge.h
 * @brief H-bridge utils
 * @author Gonzalo G. Fernandez
 */

#ifndef __HBRIDGE_H
#define __HBRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "gpio.h"
#include "tim.h"

/**
 * @brief H-Bridge structure
 */
typedef struct {
    TIM_HandleTypeDef *ena_pwm;
    uint16_t ena_chn;
    GPIO_TypeDef *in1_port;
    GPIO_TypeDef *in2_port;
    uint16_t in1_pin;
    uint16_t in2_pin;
} hbridge_t;

void hbridge_init(hbridge_t *hbridge);
void hbridge_set_pwm(hbridge_t *hbridge, int value);

#ifdef __cplusplus
}
#endif

#endif /* __HBRIDGE_H */
