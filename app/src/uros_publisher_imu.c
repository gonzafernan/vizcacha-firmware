/**
 * @file uros_publisher_imu.c
 * @brief Micro-ROS publisher for Inertial Measurement Unit (IMU)
 * Architecture: Singleton pattern
 * @author Gonzalo G. Fernandez
 */

#include "FreeRTOS.h"
#include "gpio.h"
#include "queue.h"
#include "vizcc_sensor.h"
#include <rcl/node.h>
#include <rcl/publisher.h>
#include <sensor_msgs/msg/imu.h>
#include <stdbool.h>

#define UROS_PUBLISHER_IMU_BUFFER_SIZE 5 /*!> Float32 publisher FIFO size */

struct {
    rcl_publisher_t publisher;
    sensor_msgs__msg__Imu msg;
    QueueHandle_t queue; /*!> Publisher queue for pending values */
    bool is_init;
} uros_publisher_imu = {.is_init = false};

/**
 * @brief Initialize micro-ROS IMU publisher
 * @param node micro-ROS node
 */
int uros_publisher_imu_init(rcl_node_t *node) {
    if (uros_publisher_imu.is_init) {
        return 0;
    }
    rcl_ret_t rc;
    rc = rclc_publisher_init_default(&uros_publisher_imu.publisher, node,
                                     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu");
    if (rc != RCL_RET_OK) {
        return -1;
    }

    uros_publisher_imu.queue =
        xQueueCreate(UROS_PUBLISHER_IMU_BUFFER_SIZE, sizeof(vizcc_imu_msg_t));
    if (NULL == uros_publisher_imu.queue) {
        return -1;
    }

    uros_publisher_imu.is_init = true;

    return 0;
}

int uros_publisher_imu_queue_value(vizcc_imu_msg_t *value) {
    if (!uros_publisher_imu.is_init) {
        return -1;
    }
    if (pdPASS != xQueueSend(uros_publisher_imu.queue, (void *)value, portMAX_DELAY)) {
        return -1;
    }
    return 0;
}

int uros_publisher_imu_publish(rcl_node_t *node) {
    rcl_ret_t rc;
    vizcc_imu_msg_t value;

    // receive message in queue
    BaseType_t status = xQueueReceive(uros_publisher_imu.queue, &value, 0);
    if (pdPASS != status) {
        return 0; // no message
    }

    // fill imu message
    uros_publisher_imu.msg.angular_velocity.x = value.angular_velocity[0];
    uros_publisher_imu.msg.angular_velocity.y = value.angular_velocity[1];
    uros_publisher_imu.msg.angular_velocity.z = value.angular_velocity[2];
    uros_publisher_imu.msg.linear_acceleration.x = value.linear_acceleration[0];
    uros_publisher_imu.msg.linear_acceleration.y = value.linear_acceleration[1];
    uros_publisher_imu.msg.linear_acceleration.z = value.linear_acceleration[2];

    // publish message
    rc = rcl_publish(&uros_publisher_imu.publisher, &uros_publisher_imu.msg, NULL);
    if (RCL_RET_OK != rc) {
        return -1;
    }
    return 0;
}

int uros_publisher_imu_close(rcl_node_t *node) {
    if (rcl_publisher_fini(&uros_publisher_imu.publisher, node) != RCL_RET_OK) {
        return -1;
    }
    return 0;
}