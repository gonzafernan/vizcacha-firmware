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
#include <stdbool.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#include "usart.h"
#include "tim.h"

#include "example_interfaces/srv/add_two_ints.h"

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
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

example_interfaces__srv__AddTwoInts_Request req;
example_interfaces__srv__AddTwoInts_Response res;

encoder_t henc1;

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
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);

void app_encoder_init(encoder_t *henc, void *encoder_cfg);
uint32_t app_encoder_get_value(encoder_t *henc);
int32_t app_encoder_diff_value(encoder_t *henc);

// Implementation example:
void service_callback(const void *request_msg, void *response_msg)
{
  // Cast messages to expected types
  example_interfaces__srv__AddTwoInts_Request *req_in =
      (example_interfaces__srv__AddTwoInts_Request *)request_msg;
  example_interfaces__srv__AddTwoInts_Response *res_in =
      (example_interfaces__srv__AddTwoInts_Response *)response_msg;

  // Handle request message and set the response message values
  printf("Client requested sum of %d and %d.\n", (int)req_in->a, (int)req_in->b);
  res_in->sum = req_in->a + req_in->b;
}

void publisher_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  UNUSED(last_call_time);
  if (timer != NULL)
  {
    msg.data = (uint32_t)app_encoder_diff_value(&henc1);
    rc = rcl_publish(&publisher, &msg, NULL);
  }
}

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
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
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
  rmw_uros_set_custom_transport(
      true,
      (void *)&huart3,
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator))
  {
    printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // micro-ROS app
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;
  rcl_ret_t rc;

  allocator = rcl_get_default_allocator();

  // create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  rclc_node_init_default(&node, "cubemx_node", "", &support);

  // create publisher
  rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "cubemx_publisher");

  rcl_timer_t my_timer;
  const unsigned int timer_timeout = 1000; // in ms

  msg.data = 0;

  // Init encoder
  app_encoder_init(&henc1, (void *)&htim2);

  // Init PWM channels
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  TIM4->CCR1 = 65535;

  // Service server object
  rcl_service_t service = rcl_get_zero_initialized_service();
  // Initialize server with default configuration
  rclc_service_init_default(
      &service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts), "/addtwoints");

  // create executor
  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 2, &allocator);

  unsigned int rcl_wait_timeout = 1000; // in ms
  rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout));
  rc = rclc_timer_init_default(&my_timer, &support, RCL_MS_TO_NS(timer_timeout), publisher_timer_callback);
  rclc_executor_add_timer(&executor, &my_timer);
  // if (rc != RCL_RET_OK)
  // {
  //   printf("Error in rcl_timer_init_default.\n");
  //   return -1;
  // }
  // else
  // {
  //   printf("Created timer with timeout %d ms.\n", timer_timeout);
  // }

  // Add server callback to the executor
  rclc_executor_add_service(&executor, &service, &req, &res, service_callback);

  // Optional prepare for avoiding allocations during spin
  rclc_executor_prepare(&executor);

  // Spin executor to receive requests
  rclc_executor_spin(&executor);

  rcl_service_fini(&service, &node);
  rcl_node_fini(&node);

  // if (rc != RCL_RET_OK)
  // {
  //   // return -1;
  // }

  /* Infinite loop */
  for (;;)
  {
    // rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    // if (ret != RCL_RET_OK)
    // {
    //   printf("Error publishing (line %d)\n", __LINE__);
    // }

    // msg.data++;
    // if (msg.data < -2500)
    // {
    //   TIM4->CCR1 = 0;
    // }
    // else
    // {
    //   TIM4->CCR1 = 65535;
    // }
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
