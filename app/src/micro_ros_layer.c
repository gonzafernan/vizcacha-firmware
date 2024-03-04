/**
 * @file micro_ros_layer.c
 * @brief Micro-ROS application layer
 * @author Gonzalo G. Fernandez
 */

#include "micro_ros_layer.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include <stdbool.h>
#include <stdint.h>

#include <rcl/error_handling.h> /** @todo To use error handling */
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h> // not sure why
#include <std_msgs/msg/int32.h>
#include <uxr/client/transport.h>

#include "example_interfaces/srv/add_two_ints.h"

typedef StaticTask_t osStaticThreadDef_t;

typedef struct {
    void *transport_obj;
    int32_t (*pub_callback)(void);
    void (*sub_callback)(void *, int);
    void *sub_args;

} uros_task_args_t;

osThreadId_t uros_task_handle;

// RTOS private variables
/*!> micro-ROS task stack 3000 words * 4 bytes = 12 kB > 10 kB requirement */
uint32_t uros_task_buffer[3000];
osStaticThreadDef_t uros_task_control_block;
const osThreadAttr_t uros_task_attr = {
    .name = "micro_ros_task",
    .cb_mem = &uros_task_control_block,
    .cb_size = sizeof(uros_task_control_block),
    .stack_mem = &uros_task_buffer[0],
    .stack_size = sizeof(uros_task_buffer),
    .priority = (osPriority_t)osPriorityNormal,
};
static uros_task_args_t uros_task_args;

// micro-ROS private variables
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 msg_sub;

example_interfaces__srv__AddTwoInts_Request req;
example_interfaces__srv__AddTwoInts_Response res;

// RTOS private function prototypes
void uros_layer_task(void *pv_parameters);

// micro-ROS private function prototypes
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len,
                              uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len,
                             int timeout, uint8_t *err);

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);

/**
 * @brief micro-ROS layer task creation
 * To be called between the kernel initialization (osKernelInitialize)
 * and the kernel start (osKernelStart).
 */
void uros_layer_init(void *transport_obj, int32_t (*pub_callback)(void),
                     void (*sub_callback)(void *, int), void *sub_args) {
    uros_task_args.transport_obj = transport_obj;
    uros_task_args.pub_callback = pub_callback;
    uros_task_args.sub_callback = sub_callback;
    uros_task_args.sub_args = sub_args;
    uros_task_handle = osThreadNew(uros_layer_task, (void *)&uros_task_args, &uros_task_attr);
}

/**
 * @brief micro-ROS publisher timer callback
 */
void publisher_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    rcl_ret_t rc;
    if (timer != NULL) {
        if (uros_task_args.pub_callback != NULL) {
            msg.data = uros_task_args.pub_callback();
            rc = rcl_publish(&publisher, &msg, NULL);
        }
    }
}

/**
 * @brief micro-ROS subscriber callback
 */
void subscription_callback(const void *msgin) {
    // Cast received message to used type
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    if (uros_task_args.sub_callback != NULL) {
        uros_task_args.sub_callback(uros_task_args.sub_args, (int)msg->data);
    }
}

/**
 * @brief micro-ROS service callback
 */
void service_callback(const void *request_msg, void *response_msg) {
    // Cast messages to expected types
    example_interfaces__srv__AddTwoInts_Request *req_in =
        (example_interfaces__srv__AddTwoInts_Request *)request_msg;
    example_interfaces__srv__AddTwoInts_Response *res_in =
        (example_interfaces__srv__AddTwoInts_Response *)response_msg;

    // Handle request message and set the response message values
    printf("Client requested sum of %d and %d.\n", (int)req_in->a, (int)req_in->b);
    res_in->sum = req_in->a + req_in->b;
}

/**
 * @brief micro-ROS layer task
 * @param transport
 */
void uros_layer_task(void *pv_parameters) {
    uros_task_args_t *task_args = (uros_task_args_t *)pv_parameters;
    rmw_uros_set_custom_transport(true, (void *)task_args->transport_obj, cubemx_transport_open,
                                  cubemx_transport_close, cubemx_transport_write,
                                  cubemx_transport_read);

    rcl_allocator_t freertos_allocator = rcutils_get_zero_initialized_allocator();
    freertos_allocator.allocate = microros_allocate;
    freertos_allocator.deallocate = microros_deallocate;
    freertos_allocator.reallocate = microros_reallocate;
    freertos_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freertos_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__);
    }

    /** @todo Add RCC check for error handling */
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;
    rcl_ret_t rc;

    msg.data = 0;

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    // node
    rclc_node_init_default(&node, "vizcc_node", "", &support);

    // publisher
    rclc_publisher_init_default(
        &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "vizcc_publisher");

    // executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    unsigned int rcl_wait_timeout = 1000; // ms
    rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout));

    // subscriber
    rclc_subscription_init_default(&subscriber, &node,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "vel_cmd");
    rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback,
                                   ON_NEW_DATA);

    // timer for publisher
    rcl_timer_t publisher_timer;
    const unsigned int timer_timeout = 50; // ms
    rc = rclc_timer_init_default(&publisher_timer, &support, RCL_MS_TO_NS(timer_timeout),
                                 publisher_timer_callback);
    rclc_executor_add_timer(&executor, &publisher_timer);

    // service
    rcl_service_t service = rcl_get_zero_initialized_service();
    rclc_service_init_default(&service, &node,
                              ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts),
                              "/addtwoints");
    rclc_executor_add_service(&executor, &service, &req, &res, service_callback);

    // launch executor
    rclc_executor_prepare(&executor);
    rclc_executor_spin(&executor);

    // close micro-ROS service and node
    rcl_service_fini(&service, &node);
    rcl_publisher_fini(&publisher, &node);
    rcl_subscription_fini(&subscriber, &node);
    rcl_node_fini(&node);
}
