/**
 * @file micro_ros_layer.h
 * @brief Micro-ROS application layer
 * @author Gonzalo G. Fernandez
 */

#ifndef __MICRO_ROS_LAYER_H
#define __MICRO_ROS_LAYER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <stdint.h>

typedef enum { UROS_OK, UROS_ERROR } uros_status_t;

typedef void (*param_double_callback_t)(double); /*!> Double parameter callback for update */

/**
 * @brief micro-ROS layer task creation
 * To be called between the kernel initialization (osKernelInitialize)
 * and the kernel start (osKernelStart).
 * @param transport_obj transport object (e.g. USART handle)
 * @param pub_callback publisher callback
 * @param sub_callback subscriber callback
 */
void uros_layer_init(void *transport_obj, void (*sub_callback)(void *, int), void *sub_args);

// micro-ROS publisher
void uros_publisher_init(rcl_node_t *node);
uros_status_t uros_publisher_register_float32(const char *publisher_name);
uros_status_t uros_publisher_queue_float32_value(const char *publisher_name, float *value);
uros_status_t uros_publisher_publish(rcl_node_t *node);
uros_status_t uros_publisher_close(rcl_node_t *node);

// micro-ROS parameter server APIs
rclc_parameter_server_t *uros_parameter_server_get(void);
uros_status_t uros_parameter_server_init(rcl_node_t *node, rclc_executor_t *executor);
uros_status_t uros_parameter_server_deinit(rcl_node_t *node);
uros_status_t uros_parameter_queue_double(const char *param_name, const char *param_description,
                                          const char *param_limits, double initial_value,
                                          param_double_callback_t on_change_callback);
uros_status_t uros_parameter_register_double(void);

#ifdef __cplusplus
}
#endif

#endif /* __MICRO_ROS_LAYER_H */
