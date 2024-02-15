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
    /**
     * @brief micro-ROS layer task creation
     * To be called between the kernel initialization (osKernelInitialize)
     * and the kernel start (osKernelStart).
     * @param transport_obj transport object (e.g. USART handle)
     */
    void uros_layer_init(void *transport_obj);

#ifdef __cplusplus
}
#endif

#endif /* __MICRO_ROS_LAYER_H */