/**
 * @file vizcc_wbc.c
 * @brief Vizcacha Whole-body control
 * @author Gonzalo G. Fernandez
 */

#ifndef APP_INC_VIZCC_WBC_H_
#define APP_INC_VIZCC_WBC_H_

void vizcc_wbc_init(void *joint_state_comm, void *joint_setpoint_comm, void *task_state_comm,
                    void *task_setpoint_comm);
void vizcc_wbc_set_linear_setpoint(float setpoint);
void vizcc_wbc_set_angular_setpoint(float setpoint);

#endif // APP_INC_VIZCC_WBC_H_