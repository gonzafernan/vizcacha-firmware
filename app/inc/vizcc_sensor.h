/**
 ******************************************************************************
 * @file           : vizcc_sensor.h
 * @author         : Gonzalo Gabriel Fernandez
 * @brief          : Vizcacha sensorial system
 ******************************************************************************
 * @attention
 *
 * Vizcacha sensor acquisition and sensor fusion.
 *
 ******************************************************************************
 */
#ifndef __VIZCC_SENSOR_H
#define __VIZCC_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vizcc_defs.h"

/**
 * @brief Message definition for Inertial Measurement Unit (IMU)
 */
typedef struct {
    double angular_velocity[3];    /*!> Angular velocity in x, y, z axis */
    double linear_acceleration[3]; /*!> Linear acceleration in x, y, z axis */
} vizcc_imu_msg_t;

vizcc_status_t vizcc_sensor_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __VIZCC_SENSOR_H */
