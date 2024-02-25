/**
 * @file micro_ros_layer.h
 * @brief Micro-ROS application layer
 * @author Gonzalo G. Fernandez
 */

#ifndef __MICRO_ROS_LAYER_H
#define __MICRO_ROS_LAYER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
    /**
     * @brief micro-ROS layer task creation
     * To be called between the kernel initialization (osKernelInitialize)
     * and the kernel start (osKernelStart).
     * @param transport_obj transport object (e.g. USART handle)
     * @param pub_callback publisher callback
     * @param sub_callback subscriber callback
     */
    void uros_layer_init(void *transport_obj, int32_t (*pub_callback)(void), void (*sub_callback)(int));

#ifdef __cplusplus
}
#endif

#endif /* __MICRO_ROS_LAYER_H */