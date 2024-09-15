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
    vizcc_task_state_t task_state;
    QueueHandle_t joint_state_mailbox;
    QueueHandle_t task_state_mailbox;
} _vizcc_wbc;

static void vizcc_wbc_task(void *argument) {
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(VIZCC_WBC_DT_MS));
        xQueuePeek(_vizcc_wbc.joint_state_mailbox, (void *)&_vizcc_wbc.joint_state, portMAX_DELAY);
        vizcc_model_forward_kinematics(&_vizcc_wbc.model, _vizcc_wbc.joint_state.left_wheel_vel,
                                       _vizcc_wbc.joint_state.right_wheel_vel,
                                       &_vizcc_wbc.task_state.linear_body_vel,
                                       &_vizcc_wbc.task_state.angular_body_vel);
        xQueueOverwrite(_vizcc_wbc.task_state_mailbox, (void *)&_vizcc_wbc.task_state);
    }
}

void vizcc_wbc_init(void *joint_state_comm, void *task_state_comm) {
    // vizcacha mechanical model initialization
    vizcc_model_init(&_vizcc_wbc.model, 65.0, 165.0);

    // rtos related init
    _vizcc_wbc.joint_state_mailbox = (QueueHandle_t)joint_state_comm;
    _vizcc_wbc.task_state_mailbox = (QueueHandle_t)task_state_comm;

    // init Whole-body control task
    vizcc_wbc_task_handle = osThreadNew(vizcc_wbc_task, NULL, &vizcc_wbc_task_attributes);
}