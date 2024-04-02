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
#include "stm32f4xx_hal_tim.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "encoder.h"
#include "filter.h"
#include "gpio.h"
#include "hbridge.h"
#include "micro_ros_layer.h"
#include "pid.h"
#include "tim.h"
#include "usart.h"
#include <std_msgs/msg/float32.h>
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
pid_controller_t hpid2; /*!> PID controller 2 */
encoder_t henc1;        /*!> Incremental encoder 1 */
encoder_t henc2;        /*!> Incremental encoder 2 */
hbridge_t hbridge1;     /*!> H-Bridge 1 */
hbridge_t hbridge2;     /*!> H-Bridge 2 */

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
void pid1_setpoint_callback(const void *msgin);
void pid2_setpoint_callback(const void *msgin);

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

    // H-Bridge 1 initialization
    hbridge1.in1_port = L298N_IN1_GPIO_Port;
    hbridge1.in2_port = L298N_IN2_GPIO_Port;
    hbridge1.in1_pin = L298N_IN1_Pin;
    hbridge1.in2_pin = L298N_IN2_Pin;
    hbridge1.ena_pwm = &htim4;
    hbridge1.ena_chn = TIM_CHANNEL_1;
    hbridge_init(&hbridge1);

    // H-Bridge 2 initialization
    hbridge2.in1_port = L298N_IN3_GPIO_Port;
    hbridge2.in2_port = L298N_IN4_GPIO_Port;
    hbridge2.in1_pin = L298N_IN3_Pin;
    hbridge2.in2_pin = L298N_IN4_Pin;
    hbridge2.ena_pwm = &htim4;
    hbridge2.ena_chn = TIM_CHANNEL_2;
    hbridge_init(&hbridge2);

    encoder_init(&henc1, &htim2);
    encoder_init(&henc2, &htim3);

    pid_controller_init(&hpid1, PID_DT_MS);
    pid_controller_init(&hpid2, PID_DT_MS);

    uros_layer_init((void *)&huart3);
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
    float pid1_out = 0;
    float pid2_out = 0;
    int16_t enc1_diff = 0;
    int16_t enc2_diff = 0;
    float vv_wheel1 = 0.0;
    float vv_wheel2 = 0.0;
    float vv_wheel1_filtered = 0.0;
    float vv_wheel2_filtered = 0.0;

    uros_status_t uros_status;

    uros_status = uros_parameter_queue_double("pid_kp", "PID controller proportional gain",
                                              "Only positive values", 20.0, pid_kp_update_wrapper);
    uros_status = uros_parameter_queue_double("pid_ki", "PID controller integral gain",
                                              "Only positive values", 20.0, pid_ki_update);
    uros_status = uros_parameter_queue_double("pid_kd", "PID controller derivative gain",
                                              "Only positive values", 20.0, pid_kd_update);
    uros_publisher_register_float32("encoder1/vel_raw");
    uros_publisher_register_float32("encoder1/vel_filtered");
    uros_publisher_register_float32("encoder2/vel_raw");
    uros_publisher_register_float32("encoder2/vel_filtered");

    uros_subscriber_register_float32("wheel1/vel_cmd", pid1_setpoint_callback);
    uros_subscriber_register_float32("wheel2/vel_cmd", pid2_setpoint_callback);

    filter_iir_t enc1_filter, enc2_filter;
    filter_init(&enc1_filter);
    filter_init(&enc2_filter);

    /* Infinite loop */
    for (;;) {
        osDelay(PID_DT_MS);
        // read encoders
        enc1_diff = encoder_diff_value(&henc1);
        enc2_diff = encoder_diff_value(&henc2);
        // get vel
        vv_wheel1 = ((float)enc1_diff) * (10.0 / 34.0) / PID_DT_MS;
        vv_wheel2 = ((float)enc2_diff) * (10.0 / 34.0) / PID_DT_MS;
        // filter vel
        vv_wheel1_filtered = filter_update(&enc1_filter, vv_wheel1);
        vv_wheel2_filtered = filter_update(&enc2_filter, vv_wheel2);

        pid1_out = pid_controller_update(&hpid1, vv_wheel1_filtered);
        pid2_out = pid_controller_update(&hpid2, vv_wheel2_filtered);
        hbridge_set_pwm(&hbridge1, (int32_t)pid1_out);
        hbridge_set_pwm(&hbridge2, (int32_t)pid2_out);

        uros_publisher_queue_float32_value("encoder1/vel_raw", &vv_wheel1);
        uros_publisher_queue_float32_value("encoder1/vel_filtered", &vv_wheel1_filtered);
        uros_publisher_queue_float32_value("encoder2/vel_raw", &vv_wheel2);
        uros_publisher_queue_float32_value("encoder2/vel_filtered", &vv_wheel2_filtered);
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void pid1_setpoint_callback(const void *msgin) {
    std_msgs__msg__Float32 *msg = (std_msgs__msg__Float32 *)msgin;
    pid_setpoint_update(&hpid1, (float)msg->data);
}

void pid2_setpoint_callback(const void *msgin) {
    std_msgs__msg__Float32 *msg = (std_msgs__msg__Float32 *)msgin;
    pid_setpoint_update(&hpid2, (float)msg->data);
}

void pid_kp_update_wrapper(double new_value) {
    pid_kp_update(&hpid1, (float)new_value);
    pid_kp_update(&hpid2, (float)new_value);
}
void pid_ki_update(double new_value) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}
void pid_kd_update(double new_value) {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

/* USER CODE END Application */
