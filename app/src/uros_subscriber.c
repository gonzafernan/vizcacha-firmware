/**
 * @file uros_subscriber.c
 * @brief Micro-ROS subscriber
 * @author Gonzalo G. Fernandez
 */

#include <stdbool.h>

#include "micro_ros_layer.h"
#include <std_msgs/msg/float32.h>

/**
 * @brief Maximum nomber of float32 subscriber in micro-ROS
 */
#define UROS_SUBSCRIBER_MAX_FLOAT32 10

/**
 * @brief micro-ROS subscriber register
 */
typedef struct {
    rcl_subscription_t subscriber;  /*!> micro-ROS subscriber */
    const char *name;               /*!> Subscriber name */
    void (*callback)(const void *); /*!> Subscriber callback */
    std_msgs__msg__Float32 msg;     /*!> Subscriber message */
    bool is_init;                   /*!> Is subscriber initialized */
} subscriber_float32_reg_t;

static subscriber_float32_reg_t _subscriber_float32_reg[UROS_SUBSCRIBER_MAX_FLOAT32];
static uint8_t _subscriber_float32_w_head = 0; /*!> Index for subscribers register */

/**
 * @brief micro-ROS subscribers layer initialization.
 * @param node: micro-ROS node
 */
void uros_subscriber_init(rcl_node_t *node, rclc_executor_t *executor) {
    for (uint8_t i = 0; i < _subscriber_float32_w_head; i++) {
        if (_subscriber_float32_reg[i].is_init) {
            continue;
        }
        rclc_subscription_init_default(&_subscriber_float32_reg[i].subscriber, node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                       _subscriber_float32_reg[i].name);
        rclc_executor_add_subscription(executor, &_subscriber_float32_reg[i].subscriber,
                                       &_subscriber_float32_reg[i].msg,
                                       _subscriber_float32_reg[i].callback, ON_NEW_DATA);
        _subscriber_float32_reg[i].is_init = true;
    }
}

/**
 * @brief Register float32 subscriber in micro-ROS layer.
 * @param subscriber_name: micro-ROS subscriber name
 */
uros_status_t uros_subscriber_register_float32(const char *subscriber_name,
                                               void (*sub_callback)(const void *)) {
    if (_subscriber_float32_w_head >= UROS_SUBSCRIBER_MAX_FLOAT32) {
        return UROS_ERROR;
    }
    _subscriber_float32_w_head++;
    _subscriber_float32_reg[_subscriber_float32_w_head - 1].name = subscriber_name;
    _subscriber_float32_reg[_subscriber_float32_w_head - 1].callback = sub_callback;
    _subscriber_float32_reg[_subscriber_float32_w_head - 1].is_init = false;
    return UROS_OK;
}

/**
 * @brief Close all micro-ROS subscribers
 * @param node: micro-ROS node
 */
uros_status_t uros_subscriber_close(rcl_node_t *node) {
    for (uint8_t i = 0; i < _subscriber_float32_w_head; i++) {
        rcl_subscription_fini(&_subscriber_float32_reg[i].subscriber, node);
    }
    // TODO: clean _subscriber_float32_reg
    return UROS_OK;
}
