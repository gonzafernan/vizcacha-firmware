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
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usart.h"
#include "tim.h"

#include "micro_ros_layer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
typedef struct
{
  void *cfg;
  uint32_t enc_hist;
} encoder_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RCCHECK(fn)                                                             \
  {                                                                             \
    rcl_ret_t temp_rc = fn;                                                     \
    if ((temp_rc != RCL_RET_OK))                                                \
    {                                                                           \
      printf(                                                                   \
          "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
      return 1;                                                                 \
    }                                                                           \
  }
#define RCSOFTCHECK(fn)                                                           \
  {                                                                               \
    rcl_ret_t temp_rc = fn;                                                       \
    if ((temp_rc != RCL_RET_OK))                                                  \
    {                                                                             \
      printf(                                                                     \
          "Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
    }                                                                             \
  }
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// encoder_t henc1;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[3000];
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
void app_encoder_init(encoder_t *henc, void *encoder_cfg);
uint32_t app_encoder_get_value(encoder_t *henc);
int32_t app_encoder_diff_value(encoder_t *henc);

// void publisher_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
// {
//   rcl_ret_t rc;
//   UNUSED(last_call_time);
//   if (timer != NULL)
//   {
//     msg.data = (uint32_t)app_encoder_diff_value(&henc1);
//     rc = rcl_publish(&publisher, &msg, NULL);
//   }
// }

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  // Init encoder
  // app_encoder_init(&henc1, (void *)&htim2);

  // Init PWM channels
  // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  // TIM4->CCR1 = 65535;

  /* Infinite loop */
  for (;;)
  {
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void app_encoder_init(encoder_t *henc, void *encoder_cfg)
{
  henc->cfg = encoder_cfg;
  henc->enc_hist = 0;
  HAL_TIM_Encoder_Start((TIM_HandleTypeDef *)encoder_cfg, TIM_CHANNEL_ALL);
}

uint32_t app_encoder_get_value(encoder_t *henc)
{
  return ((TIM2->CNT) >> 2);
}

int32_t app_encoder_diff_value(encoder_t *henc)
{
  uint32_t enc_new = app_encoder_get_value(henc);
  int32_t enc_diff = enc_new - henc->enc_hist;
  henc->enc_hist = enc_new;
  return enc_diff * 1000;
}

/* USER CODE END Application */
