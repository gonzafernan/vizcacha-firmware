/**
 * @file vizcc_app.c
 * @brief Vizcacha application singleton
 * @author Gonzalo G. Fernandez
 */

#include "cmsis_os.h"
#include "encoder.h"
#include "filter.h"
#include "gpio.h"
#include "hbridge.h"
#include "pid.h"
#include "tim.h"
#include "uros_layer.h"
#include "usart.h"
#include "vizcc_model.h"
#include <std_msgs/msg/float32.h>

#define PID_DT_MS 20

osThreadId_t vizcc_task_handle;
uint32_t vizcc_task_buffer[128];
StaticTask_t vizcc_task_control_block;
const osThreadAttr_t vizcc_task_attributes = {
    .name = "vizcc_task",
    .cb_mem = &vizcc_task_control_block,
    .cb_size = sizeof(vizcc_task_control_block),
    .stack_mem = &vizcc_task_buffer[0],
    .stack_size = sizeof(vizcc_task_buffer),
    .priority = (osPriority_t)osPriorityNormal,
};

struct {
    hbridge_t hbridge1;     /*!> H-Bridge 1 */
    hbridge_t hbridge2;     /*!> H-Bridge 2 */
    encoder_t henc1;        /*!> Incremental encoder 1 */
    encoder_t henc2;        /*!> Incremental encoder 2 */
    pid_controller_t hpid1; /*!> PID controller 1 */
    pid_controller_t hpid2; /*!> PID controller 2 */
    vizcc_model_t model;    /*!> Vizcacha mechanical model */
} _vizcc_app;

void pid1_setpoint_callback(const void *msgin);
void pid2_setpoint_callback(const void *msgin);

void pid_kp_update_wrapper(double new_value);
void pid_ki_update(double new_value);
void pid_kd_update(double new_value);

void vizcc_app_logger_task(void *argument);

void vizcc_app_init(void) {
    // H-Bridge 1 initialization
    _vizcc_app.hbridge1.in1_port = L298N_IN1_GPIO_Port;
    _vizcc_app.hbridge1.in2_port = L298N_IN2_GPIO_Port;
    _vizcc_app.hbridge1.in1_pin = L298N_IN1_Pin;
    _vizcc_app.hbridge1.in2_pin = L298N_IN2_Pin;
    _vizcc_app.hbridge1.ena_pwm = &htim4;
    _vizcc_app.hbridge1.ena_chn = TIM_CHANNEL_1;
    hbridge_init(&_vizcc_app.hbridge1);

    // H-Bridge 2 initialization
    _vizcc_app.hbridge2.in1_port = L298N_IN3_GPIO_Port;
    _vizcc_app.hbridge2.in2_port = L298N_IN4_GPIO_Port;
    _vizcc_app.hbridge2.in1_pin = L298N_IN3_Pin;
    _vizcc_app.hbridge2.in2_pin = L298N_IN4_Pin;
    _vizcc_app.hbridge2.ena_pwm = &htim4;
    _vizcc_app.hbridge2.ena_chn = TIM_CHANNEL_2;
    hbridge_init(&_vizcc_app.hbridge2);

    // Incremental encoder initialization
    encoder_init(&_vizcc_app.henc1, &htim2);
    encoder_init(&_vizcc_app.henc2, &htim3);

    // PID controllers initialization
    pid_controller_init(&_vizcc_app.hpid1, PID_DT_MS);
    pid_controller_init(&_vizcc_app.hpid2, PID_DT_MS);

    // vizcacha mechanical model initialization
    vizcc_model_init(&_vizcc_app.model, 64.0, 200.0);

    // initialize micro-ROS layer
    uros_layer_init((void *)&huart3);

    // rtos tasks initialization
    vizcc_task_handle = osThreadNew(vizcc_app_logger_task, NULL, &vizcc_task_attributes);
}

void vizcc_app_logger_task(void *argument) {
    float pid1_out = 0;
    float pid2_out = 0;
    int16_t enc1_diff = 0;
    int16_t enc2_diff = 0;
    float vv_wheel1 = 0.0;
    float vv_wheel2 = 0.0;
    float vv_wheel1_filtered = 0.0;
    float vv_wheel2_filtered = 0.0;

    float vizcc_v = 0.0, vizcc_w = 0.0;

    uros_status_t uros_status;

    uros_status = uros_parameter_queue_double("pid_kp", "PID controller proportional gain",
                                              "Only positive values", 20.0, pid_kp_update_wrapper);
    uros_status = uros_parameter_queue_double("pid_ki", "PID controller integral gain",
                                              "Only positive values", 20.0, pid_ki_update);
    uros_status = uros_parameter_queue_double("pid_kd", "PID controller derivative gain",
                                              "Only positive values", 20.0, pid_kd_update);
    // uros_publisher_register_float32("encoder1/vel_raw");
    // uros_publisher_register_float32("encoder1/vel_filtered");
    // uros_publisher_register_float32("encoder2/vel_raw");
    // uros_publisher_register_float32("encoder2/vel_filtered");

    uros_publisher_register_float32("pose/cmd_vel");
    uros_publisher_register_float32("pose/cmd_rot");

    uros_subscriber_register_float32("wheel1/vel_cmd", pid1_setpoint_callback);
    uros_subscriber_register_float32("wheel2/vel_cmd", pid2_setpoint_callback);

    filter_iir_t enc1_filter, enc2_filter;
    filter_init(&enc1_filter);
    filter_init(&enc2_filter);

    /* Infinite loop */
    for (;;) {
        osDelay(PID_DT_MS);
        // read encoders
        enc1_diff = encoder_diff_value(&_vizcc_app.henc1);
        enc2_diff = encoder_diff_value(&_vizcc_app.henc2);
        // get vel
        vv_wheel1 = ((float)enc1_diff) * (10.0 / 34.0) / PID_DT_MS;
        vv_wheel2 = ((float)enc2_diff) * (10.0 / 34.0) / PID_DT_MS;
        // filter vel
        vv_wheel1_filtered = filter_update(&enc1_filter, vv_wheel1);
        vv_wheel2_filtered = filter_update(&enc2_filter, vv_wheel2);

        pid1_out = pid_controller_update(&_vizcc_app.hpid1, vv_wheel1_filtered);
        pid2_out = pid_controller_update(&_vizcc_app.hpid2, vv_wheel2_filtered);
        hbridge_set_pwm(&_vizcc_app.hbridge1, (int32_t)pid1_out);
        hbridge_set_pwm(&_vizcc_app.hbridge2, (int32_t)pid2_out);

        vizcc_model_forward_kinematics(&_vizcc_app.model, vv_wheel1_filtered, vv_wheel2_filtered,
                                       &vizcc_v, &vizcc_w);

        // uros_publisher_queue_float32_value("encoder1/vel_raw", &vv_wheel1);
        // uros_publisher_queue_float32_value("encoder1/vel_filtered", &vv_wheel1_filtered);
        // uros_publisher_queue_float32_value("encoder2/vel_raw", &vv_wheel2);
        // uros_publisher_queue_float32_value("encoder2/vel_filtered", &vv_wheel2_filtered);
        uros_publisher_queue_float32_value("pose/cmd_vel", &vizcc_v);
        uros_publisher_queue_float32_value("pose/cmd_rot", &vizcc_w);
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
}

void pid1_setpoint_callback(const void *msgin) {
    std_msgs__msg__Float32 *msg = (std_msgs__msg__Float32 *)msgin;
    pid_setpoint_update(&_vizcc_app.hpid1, (float)msg->data);
}

void pid2_setpoint_callback(const void *msgin) {
    std_msgs__msg__Float32 *msg = (std_msgs__msg__Float32 *)msgin;
    pid_setpoint_update(&_vizcc_app.hpid2, (float)msg->data);
}

void pid_kp_update_wrapper(double new_value) {
    pid_kp_update(&_vizcc_app.hpid1, (float)new_value);
    pid_kp_update(&_vizcc_app.hpid2, (float)new_value);
}
void pid_ki_update(double new_value) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}
void pid_kd_update(double new_value) {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}
