/**
 * @file vizcc_wbc.c
 * @brief Vizcacha Whole-body control
 * @author Gonzalo G. Fernandez
 */

#include "cmsis_os.h"
#include "queue.h"
#include "vizcc_defs.h"
#include "vizcc_model.h"

#define VIZCC_WBC_DT_MS 40 /*!> Whole-body control time period in ms */

// Whole-body control task
osThreadId_t vizcc_wbc_task_handle;
uint32_t vizcc_wbc_task_buffer[128];
StaticTask_t vizcc_wbc_task_control_block;
const osThreadAttr_t vizcc_wbc_task_attributes = {
    .name = "vizcc_control_task",
    .cb_mem = &vizcc_wbc_task_control_block,
    .cb_size = sizeof(vizcc_wbc_task_control_block),
    .stack_mem = &vizcc_wbc_task_buffer[0],
    .stack_size = sizeof(vizcc_wbc_task_buffer),
    .priority = (osPriority_t)osPriorityNormal,
};

/**
 * @brief Whole-body control singleton
 */
static struct {
    vizcc_model_t model; /*!> Vizcacha mechanical model */
    vizcc_joint_state_t joint_state;
    vizcc_joint_state_t joint_setpoint;
    vizcc_task_state_t task_state;
    vizcc_task_state_t task_setpoint;
    QueueHandle_t joint_state_mailbox;
    QueueHandle_t joint_setpoint_mailbox;
    QueueHandle_t task_state_mailbox;
    QueueHandle_t task_setpoint_mailbox;
} _vizcc_wbc;

static void vizcc_wbc_task(void *argument) {
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(VIZCC_WBC_DT_MS));

        // Current state computation
        xQueuePeek(_vizcc_wbc.joint_state_mailbox, (void *)&_vizcc_wbc.joint_state, 0);
        vizcc_model_forward_kinematics(&_vizcc_wbc.model, _vizcc_wbc.joint_state.left_wheel_vel,
                                       _vizcc_wbc.joint_state.right_wheel_vel,
                                       &_vizcc_wbc.task_state.linear_body_vel,
                                       &_vizcc_wbc.task_state.angular_body_vel);
        xQueueOverwrite(_vizcc_wbc.task_state_mailbox, (void *)&_vizcc_wbc.task_state);

        // Setpoint computation
        vizcc_model_inverse_kinematics(&_vizcc_wbc.model, _vizcc_wbc.task_setpoint.linear_body_vel,
                                       _vizcc_wbc.task_setpoint.angular_body_vel,
                                       &_vizcc_wbc.joint_setpoint.left_wheel_vel,
                                       &_vizcc_wbc.joint_setpoint.right_wheel_vel);
        xQueueOverwrite(_vizcc_wbc.joint_setpoint_mailbox, (void *)&_vizcc_wbc.joint_setpoint);
    }
}

void vizcc_wbc_init(void *joint_state_comm, void *joint_setpoint_comm, void *task_state_comm,
                    void *task_setpoint_comm) {
    // vizcacha mechanical model initialization
    vizcc_model_init(&_vizcc_wbc.model, 65.0, 165.0);

    _vizcc_wbc.task_setpoint.angular_body_vel = 0.0;
    _vizcc_wbc.task_setpoint.linear_body_vel = 0.0;

    // rtos related init
    _vizcc_wbc.joint_state_mailbox = (QueueHandle_t)joint_state_comm;
    _vizcc_wbc.joint_setpoint_mailbox = (QueueHandle_t)joint_setpoint_comm;
    _vizcc_wbc.task_state_mailbox = (QueueHandle_t)task_state_comm;
    _vizcc_wbc.task_setpoint_mailbox = (QueueHandle_t)task_setpoint_comm;

    // init Whole-body control task
    vizcc_wbc_task_handle = osThreadNew(vizcc_wbc_task, NULL, &vizcc_wbc_task_attributes);
}

// TODO: This should be moved to the comm task and write a mailbox
void vizcc_wbc_set_linear_setpoint(float setpoint) {
    _vizcc_wbc.task_setpoint.linear_body_vel = setpoint;
}

void vizcc_wbc_set_angular_setpoint(float setpoint) {
    _vizcc_wbc.task_setpoint.angular_body_vel = setpoint;
}