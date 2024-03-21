/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "encoder.h"
#include "filter.h"
#include "gpio.h"
#include "hbridge.h"
#include "micro_ros_layer.h"
#include "pid.h"
#include "usart.h"
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159
#define PID_DT_MS 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
pid_controller_t hpid1; /*!> PID controller 1 */
int32_t encoder_diff = 0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[128];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .cb_mem = &defaultTaskControlBlock,
    .cb_size = sizeof(defaultTaskControlBlock),
    .stack_mem = &defaultTaskBuffer[0],
    .stack_size = sizeof(defaultTaskBuffer),
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void subscriber_callback_wrapper(void *sub_args, int value);

void pid_kp_update_wrapper(double new_value);
void pid_ki_update(double new_value);
void pid_kd_update(double new_value);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    hbridge_init();
    encoder_init();
    pid_controller_init(&hpid1, PID_DT_MS);
    uros_layer_init((void *)&huart3, subscriber_callback_wrapper, (void *)&hpid1);
    /* USER CODE END Init */
    /* USER CODE BEGIN Header */
    /**
     ******************************************************************************
     * File Name          : freertos.c
     * Description        : Code for freertos applications
     ******************************************************************************
     * @attention
     *
     * Copyright (c) 2024 STMicroelectronics.
     * All rights reserved.
     *
     * This software is licensed under terms that can be found in the LICENSE file
     * in the root directory of this software component.
     * If no LICENSE file comes with this software, it is provided AS-IS.
     *
     ******************************************************************************
     */
    /* USER CODE END Header */

    /**
     * @}
     */

    /**
     * @}
     */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
    /* USER CODE BEGIN StartDefaultTask */
    float pid_out = 0; // PID controller output
    float vv_wheel = 0.0;
    float vv_wheel_filtered = 0.0;

    uros_status_t uros_status;

    uros_status = uros_parameter_queue_double("pid_kp", "PID controller proportional gain",
                                              "Only positive values", 20.0, pid_kp_update_wrapper);
    uros_status = uros_parameter_queue_double("pid_ki", "PID controller integral gain",
                                              "Only positive values", 20.0, pid_ki_update);
    uros_status = uros_parameter_queue_double("pid_kd", "PID controller derivative gain",
                                              "Only positive values", 20.0, pid_kd_update);
    uros_publisher_register_float32("encoder1/vel_raw");
    uros_publisher_register_float32("encoder1/vel_filtered");

    filter_iir_t encoder_filter;
    filter_init(&encoder_filter);

    /* Infinite loop */
    for (;;) {
        osDelay(PID_DT_MS);
        encoder_diff = encoder_diff_value();
        vv_wheel = ((float)encoder_diff) * (10.0 / 34.0) / PID_DT_MS;
        vv_wheel_filtered = filter_update(&encoder_filter, vv_wheel);
        pid_out = pid_controller_update(&hpid1, vv_wheel_filtered);
        hbridge_set_pwm((int32_t)pid_out);
        uros_publisher_queue_float32_value("encoder1/vel_raw", &vv_wheel);
        uros_publisher_queue_float32_value("encoder1/vel_filtered", &vv_wheel_filtered);
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void subscriber_callback_wrapper(void *sub_args, int value) {
    pid_controller_t *hpid = (pid_controller_t *)sub_args;
    pid_setpoint_update(hpid, (float)value);
}

void pid_kp_update_wrapper(double new_value) {
    pid_kp_update(&hpid1, (float)new_value);
}
void pid_ki_update(double new_value) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}
void pid_kd_update(double new_value) {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

/* USER CODE END Application */
