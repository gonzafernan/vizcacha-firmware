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
#include "main.h"
#include "pid.h"
#include "queue.h"
#include "tim.h"
#include "uros_layer.h"
#include "usart.h"
#include "vizcc_sensor.h"
#include "vizcc_wbc.h"
#include "vizzcc_config.h"
#include <std_msgs/msg/float32.h>

#define VIZCC_CONTROL_DT_MS 2 /*!> Actuator control time period in ms */
#define VIZCC_LOGGER_DT_MS 100

// logger task
osThreadId_t vizcc_logger_task_handle;
uint32_t vizcc_logger_task_buffer[128];
StaticTask_t vizcc_logger_task_control_block;
const osThreadAttr_t vizcc_logger_task_attributes = {
    .name = "vizcc_logger_task",
    .cb_mem = &vizcc_logger_task_control_block,
    .cb_size = sizeof(vizcc_logger_task_control_block),
    .stack_mem = &vizcc_logger_task_buffer[0],
    .stack_size = sizeof(vizcc_logger_task_buffer),
    .priority = (osPriority_t)osPriorityNormal,
};

// control task
osThreadId_t vizcc_control_task_handle;
uint32_t vizcc_control_task_buffer[128];
StaticTask_t vizcc_control_task_control_block;
const osThreadAttr_t vizcc_control_task_attributes = {
    .name = "vizcc_control_task",
    .cb_mem = &vizcc_control_task_control_block,
    .cb_size = sizeof(vizcc_control_task_control_block),
    .stack_mem = &vizcc_control_task_buffer[0],
    .stack_size = sizeof(vizcc_control_task_buffer),
    .priority = (osPriority_t)osPriorityNormal,
};

struct {
    hbridge_t hbridge1;     /*!> H-Bridge 1 */
    hbridge_t hbridge2;     /*!> H-Bridge 2 */
    encoder_t henc1;        /*!> Incremental encoder 1 */
    encoder_t henc2;        /*!> Incremental encoder 2 */
    pid_controller_t hpid1; /*!> PID controller 1 */
    pid_controller_t hpid2; /*!> PID controller 2 */
} _vizcc_app;

static vizcc_joint_state_t _vizcc_joint_state;    /*!> Vizcacha current joint state */
static vizcc_joint_state_t _vizcc_joint_setpoint; /*!> Vizcacha current joint setpoint */
static vizcc_task_state_t _vizcc_task_state;      /*!> Vizcacha current task state */
static vizcc_task_state_t _vizcc_task_setpoint;   /*!> Vizcacha current task setpoint */

static QueueHandle_t _vizcc_joint_state_mailbox; /*!> Mailbox to share joint state between tasks */
static QueueHandle_t
    _vizcc_joint_setpoint_mailbox; /*!> Mailbox to share joint setpoint between tasks */
static QueueHandle_t _vizcc_task_state_mailbox; /*!> Mailbox to share task state between tasks */
static QueueHandle_t
    _vizcc_task_setpoint_mailbox; /*!> Mailbox to share task setpoint between tasks */

float vv_wheel1 = 0.0;
float vv_wheel2 = 0.0;

void pid_setpoint_callback(const void *msgin, void *context);

void pid_kp_update_wrapper(void *context, double new_value);
void pid_ki_update_wrapper(void *context, double new_value);
void pid_kd_update_wrapper(void *context, double new_value);

void task_linear_setpoint_callback(const void *msgin, void *context);
void task_angular_setpoint_callback(const void *msgin, void *context);

void vizcc_app_logger_task(void *argument);
void vizcc_app_control_task(void *argument);

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
    pid_controller_init(&_vizcc_app.hpid1, VIZCC_CONTROL_DT_MS);
    pid_controller_init(&_vizcc_app.hpid2, VIZCC_CONTROL_DT_MS);

    // current state initialization
    _vizcc_joint_state.left_wheel_vel = 0.0;
    _vizcc_joint_state.right_wheel_vel = 0.0;
    _vizcc_joint_state_mailbox = xQueueCreate(1, sizeof(vizcc_joint_state_t));
    _vizcc_task_state.linear_body_vel = 0.0;
    _vizcc_task_state.angular_body_vel = 0.0;
    _vizcc_task_state_mailbox = xQueueCreate(1, sizeof(vizcc_task_state_t));

    // setpoint initialization
    _vizcc_joint_setpoint.left_wheel_vel = 0.0;
    _vizcc_joint_setpoint.right_wheel_vel = 0.0;
    _vizcc_joint_setpoint_mailbox = xQueueCreate(1, sizeof(vizcc_joint_state_t));
    _vizcc_task_setpoint.linear_body_vel = 0.0;
    _vizcc_task_setpoint.angular_body_vel = 0.0;
    _vizcc_task_setpoint_mailbox = xQueueCreate(1, sizeof(vizcc_task_state_t));

    // Whole-body control init
    vizcc_wbc_init((void *)_vizcc_joint_state_mailbox, (void *)_vizcc_joint_setpoint_mailbox,
                   (void *)_vizcc_task_state_mailbox, (void *)_vizcc_task_setpoint_mailbox);

    // initialize micro-ROS layer
    uros_layer_init((void *)&huart3);

    vizcc_sensor_init();

    // rtos tasks initialization
    vizcc_logger_task_handle =
        osThreadNew(vizcc_app_logger_task, NULL, &vizcc_logger_task_attributes);
    vizcc_control_task_handle =
        osThreadNew(vizcc_app_control_task, NULL, &vizcc_control_task_attributes);
}

void vizcc_app_control_task(void *argument) {
    float pid1_out = 0;
    float pid2_out = 0;
    int16_t enc1_diff = 0;
    int16_t enc2_diff = 0;

    filter_iir_t enc1_filter, enc2_filter;
    filter_init(&enc1_filter);
    filter_init(&enc2_filter);

    TickType_t last_wake_time = xTaskGetTickCount();

    /* Infinite loop */
    for (;;) {

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(VIZCC_CONTROL_DT_MS));

        // read encoders
        enc1_diff = encoder_diff_value(&_vizcc_app.henc1);
        enc2_diff = encoder_diff_value(&_vizcc_app.henc2);

        // get velocity
        vv_wheel1 = ((float)enc1_diff) * 60 / (float)ACTUATOR_ENCODER_PPR /
                    (float)ACTUATOR_GEARBOX_RATIO / (float)VIZCC_CONTROL_DT_MS;
        vv_wheel2 = ((float)enc2_diff) * 60 / (float)ACTUATOR_ENCODER_PPR /
                    (float)ACTUATOR_GEARBOX_RATIO / (float)VIZCC_CONTROL_DT_MS;

        // filter vel
        _vizcc_joint_state.left_wheel_vel = filter_update(&enc1_filter, vv_wheel1);
        _vizcc_joint_state.right_wheel_vel = filter_update(&enc2_filter, vv_wheel2);

        // state update
        xQueueOverwrite(_vizcc_joint_state_mailbox, (void *)&_vizcc_joint_state);

        // setpoint update
        xQueuePeek(_vizcc_joint_setpoint_mailbox, (void *)&_vizcc_joint_setpoint, 0);
        pid_setpoint_update(&_vizcc_app.hpid1, _vizcc_joint_setpoint.left_wheel_vel);
        pid_setpoint_update(&_vizcc_app.hpid2, _vizcc_joint_setpoint.right_wheel_vel);

        // perform actuator control
        pid1_out = pid_controller_update(&_vizcc_app.hpid1, _vizcc_joint_state.left_wheel_vel);
        pid2_out = pid_controller_update(&_vizcc_app.hpid2, _vizcc_joint_state.right_wheel_vel);
        hbridge_set_pwm(&_vizcc_app.hbridge1, (int32_t)pid1_out);
        hbridge_set_pwm(&_vizcc_app.hbridge2, (int32_t)pid2_out);
    }
}

void vizcc_app_logger_task(void *argument) {
    uros_status_t uros_status;

    uros_status = uros_parameter_queue_double("wheel1/pid_kp", "Wheel 1 PID KP", NULL, 800.0,
                                              pid_kp_update_wrapper, (void *)&_vizcc_app.hpid1);
    uros_status = uros_parameter_queue_double("wheel1/pid_ki", "Wheel 1 PID KI", NULL, 0.0,
                                              pid_ki_update_wrapper, (void *)&_vizcc_app.hpid1);
    // uros_status = uros_parameter_queue_double("wheel1/pid_kd", "Wheel 1 PID KD", NULL, 0.0,
    //                                           pid_kd_update_wrapper, (void
    //                                           *)&_vizcc_app.hpid1);
    uros_status = uros_parameter_queue_double("wheel2/pid_kp", "Wheel 2 PID KP", NULL, 800.0,
                                              pid_kp_update_wrapper, (void *)&_vizcc_app.hpid2);
    uros_status = uros_parameter_queue_double("wheel2/pid_ki", "Wheel 2 PID KI", NULL, 0.0,
                                              pid_ki_update_wrapper, (void *)&_vizcc_app.hpid2);
    // uros_status = uros_parameter_queue_double("wheel2/pid_kd", "Wheel 2 PID KD", NULL, 0.0,
    //                                           pid_kd_update_wrapper, (void
    //                                           *)&_vizcc_app.hpid2);

    uros_publisher_register_float32("encoder1/vel_raw");
    uros_publisher_register_float32("encoder1/vel_filtered");
    uros_publisher_register_float32("encoder2/vel_raw");
    uros_publisher_register_float32("encoder2/vel_filtered");

    uros_publisher_register_float32("wheel1/angular_setpoint");
    uros_publisher_register_float32("wheel2/angular_setpoint");

    uros_publisher_register_float32("pose/cmd_vel");
    uros_publisher_register_float32("pose/cmd_rot");

    // uros_subscriber_register_float32("wheel1/vel_cmd", pid_setpoint_callback,
    //                                  (void *)&_vizcc_app.hpid1);
    // uros_subscriber_register_float32("wheel2/vel_cmd", pid_setpoint_callback,
    //                                  (void *)&_vizcc_app.hpid2);

    // task setpoint
    uros_subscriber_register_float32("pose/linear_setpoint", task_linear_setpoint_callback, NULL);
    uros_subscriber_register_float32("pose/angular_setpoint", task_angular_setpoint_callback, NULL);

    TickType_t last_wake_time = xTaskGetTickCount();
    vizcc_task_state_t task_state;
    vizcc_joint_state_t joint_setpoint;

    /* Infinite loop */
    for (;;) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(VIZCC_LOGGER_DT_MS));

        // Get current task state
        xQueuePeek(_vizcc_task_state_mailbox, (void *)&task_state, 0);
        xQueuePeek(_vizcc_joint_setpoint_mailbox, (void *)&joint_setpoint, 0);

        uros_publisher_queue_float32_value("encoder1/vel_raw", &vv_wheel1);
        // TODO: Receive state through mailbox in task argument
        uros_publisher_queue_float32_value("encoder1/vel_filtered",
                                           &_vizcc_joint_state.left_wheel_vel);
        uros_publisher_queue_float32_value("encoder2/vel_raw", &vv_wheel2);
        uros_publisher_queue_float32_value("encoder2/vel_filtered",
                                           &_vizcc_joint_state.right_wheel_vel);

        uros_publisher_queue_float32_value("wheel1/angular_setpoint",
                                           (float *)&joint_setpoint.left_wheel_vel);
        uros_publisher_queue_float32_value("wheel2/angular_setpoint",
                                           (float *)&joint_setpoint.right_wheel_vel);

        // uros_publisher_queue_float32_value("wheel1/pid_output", (float
        // *)&_vizcc_app.hpid1.output); uros_publisher_queue_float32_value("wheel2/pid_output",
        // (float *)&_vizcc_app.hpid2.output);

        uros_publisher_queue_float32_value("pose/cmd_vel", &task_state.linear_body_vel);
        uros_publisher_queue_float32_value("pose/cmd_rot", &task_state.angular_body_vel);

        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    }
}

// void pid_setpoint_callback(const void *msgin, void *context) {
//     pid_controller_t *hpid = (pid_controller_t *)context;
//     std_msgs__msg__Float32 *msg = (std_msgs__msg__Float32 *)msgin;
//     pid_setpoint_update(hpid, (float)msg->data);
// }

void pid_kp_update_wrapper(void *context, double new_value) {
    pid_kp_update((pid_controller_t *)context, (float)new_value);
}

void pid_ki_update_wrapper(void *context, double new_value) {
    pid_ki_update((pid_controller_t *)&context, (float)new_value);
}

void pid_kd_update_wrapper(void *context, double new_value) {
    pid_kd_update((pid_controller_t *)&context, (float)new_value);
}

// TODO: Change message to pose type for simplified logic
void task_linear_setpoint_callback(const void *msgin, void *context) {
    std_msgs__msg__Float32 *msg = (std_msgs__msg__Float32 *)msgin;
    vizcc_wbc_set_linear_setpoint((float)msg->data);
}

void task_angular_setpoint_callback(const void *msgin, void *context) {
    std_msgs__msg__Float32 *msg = (std_msgs__msg__Float32 *)msgin;
    vizcc_wbc_set_angular_setpoint((float)msg->data);
}
