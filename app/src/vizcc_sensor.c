/**
 ******************************************************************************
 * @file           : vizcc_sensor.c
 * @author         : Gonzalo Gabriel Fernandez
 * @brief          : Vizcacha sensorial system
 ******************************************************************************
 *
 * Vizcacha sensor acquisition and sensor fusion.
 *
 ******************************************************************************
 */

#include <stdint.h>

#include "cmsis_os.h"
#include "i2c.h"
#include "mpu6050.h"
#include "vizcc_defs.h"

// sensor task
static osThreadId_t _vizcc_sensor_task_handle;
static uint32_t _vizcc_sensor_task_buffer[128];
static StaticTask_t _vizcc_sensor_task_control_block;
static const osThreadAttr_t _vizcc_sensor_task_attributes = {
    .name = "vizcc_sensor_task",
    .cb_mem = &_vizcc_sensor_task_control_block,
    .cb_size = sizeof(_vizcc_sensor_task_control_block),
    .stack_mem = &_vizcc_sensor_task_buffer[0],
    .stack_size = sizeof(_vizcc_sensor_task_buffer),
    .priority = (osPriority_t)osPriorityNormal,
};

/**
 * @brief Vizcacha sensor system singleton.
 */
struct {
    mpu6050_t himu; /*!> IMU handle */
} vizcc_sensor;

void vizcc_sensor_task(void *argument); /*!> Vizcacha sensor system task */

/**
 * @brief Vizcacha sensor system initialization
 * @returns VIZCC_STATUS_OK on correct initialization
 */
vizcc_status_t vizcc_sensor_init(void) {
    // IMU initialization
    mpu6050_status_t imu_status;
    imu_status = mpu6050_init(&vizcc_sensor.himu, (void *)&hi2c1);
    if (MPU6050_OK != imu_status)
        return VIZCC_STATUS_ERROR;

    // rtos tasks initialization
    _vizcc_sensor_task_handle =
        osThreadNew(vizcc_sensor_task, NULL, &_vizcc_sensor_task_attributes);
}

/**
 * @brief Vizcacha sensor system task
 */
void vizcc_sensor_task(void *argument) {
    // IMU sanity check
    mpu6050_status_t imu_status;
    imu_status = mpu6050_sanity_check(&vizcc_sensor.himu);
    if (MPU6050_OK != imu_status) {
        // TODO: Handle invalid sanity check
    }

    for (;;) {
    }
}
