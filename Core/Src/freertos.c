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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
pid_controller_t hpid1; /*!> PID controller 1 */

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
    pid_controller_init(&hpid1);
    uros_layer_init((void *)&huart3, encoder_diff_value, subscriber_callback_wrapper,
                    (void *)&hpid1);
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
    int32_t pid_out = 0; // PID controller output

    /* Infinite loop */
    for (;;) {
        osDelay(50);
        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
        pid_out = pid_controller_update(&hpid1, encoder_diff_value());
        hbridge_set_pwm(pid_out);
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void subscriber_callback_wrapper(void *sub_args, int value) {
    pid_controller_t *hpid = (pid_controller_t *)sub_args;
    pid_setpoint_update(hpid, value);
}
/* USER CODE END Application */
