/**
 * @file uros_publisher.c
 * @brief Micro-ROS publishers
 * @author Gonzalo G. Fernandez
 */

#include "main.h"
#include "uros_layer.h"

#include "FreeRTOS.h"
#include "portmacro.h"
#include "projdefs.h"
#include "queue.h"
#include "rcl/node.h"
#include <std_msgs/msg/float32.h>
#include <stdint.h>
#include <string.h>

/**
 * @brief Maximum number of float32 publishers
 */
#define UROS_PUBLISHER_MAX_FLOAT32 10

#define UROS_PUBLISHER_FLOAT32_BUFFER_SIZE 5 /*!> Float32 publisher FIFO size */

/**
 * @brief micro-ROS double publisher register
 */
typedef struct {
    rcl_publisher_t publisher; /*!> micro-ROS publisher */
    const char *name;          /*!> Publisher name */
    QueueHandle_t queue;       /*!> Publisher queue for pending values */
    bool is_init;
} publisher_float32_reg_t;

static publisher_float32_reg_t
    _publisher_float32_reg[UROS_PUBLISHER_MAX_FLOAT32]; /*!> Array of float32 publishers */
static uint8_t _publisher_float32_w_head = 0; /*!> Index for float32 publishers register */
std_msgs__msg__Float32 msg_float32;

/**
 * @brief micro-ROS publishers layer initialization.
 * @param node: micro-ROS node
 */
void uros_publisher_init(rcl_node_t *node) {
    for (uint8_t i = 0; i < _publisher_float32_w_head; i++) {
        if (_publisher_float32_reg[i].is_init) {
            continue;
        }
        rclc_publisher_init_default(&_publisher_float32_reg[i].publisher, node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                    _publisher_float32_reg[i].name);
        _publisher_float32_reg[i].is_init = true;
    }
}

/**
 * @brief Register float32 publisher in micro-ROS layer.
 * @param publisher_name: micro-ROS publisher name
 */
uros_status_t uros_publisher_register_float32(const char *publisher_name) {
    if (_publisher_float32_w_head >= UROS_PUBLISHER_MAX_FLOAT32) {
        return UROS_ERROR;
    }
    _publisher_float32_w_head++;
    _publisher_float32_reg[_publisher_float32_w_head - 1].name = publisher_name;
    _publisher_float32_reg[_publisher_float32_w_head - 1].is_init = false;
    _publisher_float32_reg[_publisher_float32_w_head - 1].queue =
        xQueueCreate(UROS_PUBLISHER_FLOAT32_BUFFER_SIZE, sizeof(float));
    if (NULL == _publisher_float32_reg[_publisher_float32_w_head - 1].queue) {
        return UROS_ERROR;
    }
    return UROS_OK;
}

/**
 * @brief Queue value in publisher
 * @param publisher_name: micro-ROS publisher name
 * @param value: float32 value to publish
 */
uros_status_t uros_publisher_queue_float32_value(const char *publisher_name, float *value) {
    // TODO: Not sure if float should be received as pointer
    for (uint8_t i = 0; i < _publisher_float32_w_head; i++) {
        if (0 == strcmp(publisher_name, _publisher_float32_reg[i].name)) {
            if (pdPASS != xQueueSend(_publisher_float32_reg[i].queue, value, portMAX_DELAY)) {
                return UROS_ERROR;
            }
            return UROS_OK;
        }
    }
    return UROS_ERROR;
}

/**
 * @brief Publish all values in queue
 * @node node: micro-ROS node
 */
uros_status_t uros_publisher_publish(rcl_node_t *node) {
    rcl_ret_t rc;
    BaseType_t status;
    float value;
    for (uint8_t i = 0; i < _publisher_float32_w_head; i++) {
        status = xQueueReceive(_publisher_float32_reg[i].queue, &value, 0);
        if (pdPASS != status) {
            continue;
        }
        msg_float32.data = value;
        rc = rcl_publish(&_publisher_float32_reg[i].publisher, &msg_float32.data, NULL);
    }
}

/**
 * @brief Close all micro-ROS publishers
 * @param node: micro-ROS node
 */
uros_status_t uros_publisher_close(rcl_node_t *node) {
    for (uint8_t i = 0; i < _publisher_float32_w_head; i++) {
        rcl_publisher_fini(&_publisher_float32_reg[i].publisher, node);
    }
    // TODO: clean _publisher_float32_reg
    return UROS_OK;
}
