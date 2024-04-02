/**
 * @file uros_layer.c
 * @brief Micro-ROS application layer
 * @author Gonzalo G. Fernandez
 */

#include "uros_layer.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include <stdbool.h>
#include <stdint.h>

#include <rcl/error_handling.h> /** @todo To use error handling */
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h> // not sure why
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <uxr/client/transport.h>

// #include "example_interfaces/srv/add_two_ints.h"
#include "rcl/types.h"
#include "rclc_parameter/rclc_parameter.h"

typedef StaticTask_t osStaticThreadDef_t;

osThreadId_t uros_task_handle;

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

// micro-ROS private variables
rcl_node_t node;
rclc_executor_t executor;

// example_interfaces__srv__AddTwoInts_Request req;
// example_interfaces__srv__AddTwoInts_Response res;

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
 * @param transport_obj transport object (e.g. USART handle)
 */
void uros_layer_init(void *transport_obj) {
    uros_task_handle = osThreadNew(uros_layer_task, transport_obj, &uros_task_attr);
}

/**
 * @brief micro-ROS publisher timer callback
 */
void uros_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer != NULL) {
        uros_parameter_register_double();
        uros_publisher_init(&node);
        uros_publisher_publish(&node);
        uros_subscriber_init(&node, &executor);
    }
}

/**
 * @brief micro-ROS service callback
 */
// void service_callback(const void *request_msg, void *response_msg) {
//     // Cast messages to expected types
//     example_interfaces__srv__AddTwoInts_Request *req_in =
//         (example_interfaces__srv__AddTwoInts_Request *)request_msg;
//     example_interfaces__srv__AddTwoInts_Response *res_in =
//         (example_interfaces__srv__AddTwoInts_Response *)response_msg;
//
//     // Handle request message and set the response message values
//     printf("Client requested sum of %d and %d.\n", (int)req_in->a, (int)req_in->b);
//     res_in->sum = req_in->a + req_in->b;
// }

/**
 * @brief micro-ROS layer task
 * @param transport
 */
void uros_layer_task(void *transport_obj) {
    rmw_uros_set_custom_transport(true, transport_obj, cubemx_transport_open,
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
    rcl_ret_t rc;

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    // node
    rclc_node_init_default(&node, "vizcc_node", "", &support);

    // executor
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 5,
                       &allocator);
    unsigned int rcl_wait_timeout = 1000; // ms
    rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout));

    // paramater server
    uros_parameter_server_init(&node, &executor);

    // timer for publisher
    rcl_timer_t publisher_timer;
    const unsigned int timer_timeout = 20; // ms
    rc = rclc_timer_init_default(&publisher_timer, &support, RCL_MS_TO_NS(timer_timeout),
                                 uros_timer_callback);
    rclc_executor_add_timer(&executor, &publisher_timer);

    // service
    // rcl_service_t service = rcl_get_zero_initialized_service();
    // rclc_service_init_default(&service, &node,
    //                           ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts),
    //                           "/addtwoints");
    // rclc_executor_add_service(&executor, &service, &req, &res, service_callback);

    // launch executor
    rclc_executor_prepare(&executor);
    rclc_executor_spin(&executor);

    // close micro-ROS service and node
    // rcl_service_fini(&service, &node);
    uros_publisher_close(&node);
    uros_subscriber_close(&node);
    uros_parameter_server_deinit(&node);
    rcl_node_fini(&node);
}
