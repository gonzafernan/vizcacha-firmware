/**
 ******************************************************************************
 * @file           : vizcc_defs.h
 * @author         : Gonzalo Gabriel Fernandez
 * @brief          : Vizcacha useful definitions
 ******************************************************************************
 */
#ifndef __VIZCC_DEFS_H
#define __VIZCC_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Vizcacha status type definition
 */
typedef enum {
    VIZCC_STATUS_ERROR = -1, /*!> Error in the process */
    VIZCC_STATUS_OK = 0,     /*!> Process ended as expected */
} vizcc_status_t;

/**
 * @brief Vizcacha current joint state structure
 */
typedef struct {
    float left_wheel_vel;  /*!> Left-wheel angular velocity */
    float right_wheel_vel; /*!> Right-wheel body velocity */
} vizcc_joint_state_t;

/**
 * @brief Vizcacha current task state structure
 */
typedef struct {
    float linear_body_vel;  /*!> Linear body velocity */
    float angular_body_vel; /*!> Angular body velocity */
} vizcc_task_state_t;

#ifdef __cplusplus
}
#endif

#endif /* __VIZCC_DEFS_H */
