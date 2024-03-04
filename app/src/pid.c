/**
 * @file pid.c
 * @brief PID controller
 * @author Gonzalo G. Fernandez
 */

#include "pid.h"

/**
 * @brief PID controller initialization
 * - Clear controller memory values.
 */
void pid_controller_init(pid_controller_t *pid) {
    pid->prev_error = 0;
    pid->prev_input = 0;
    pid->integrate = 0;
    pid->derivate = 0;
    pid->setpoint = 0;
    pid->output = 0;

    pid->lim_max = 65500;
    pid->lim_min = -65500;

    pid->lim_max_int = 65500;
    pid->lim_min_int = -65500;

    pid->dt = 1;
    pid->tau = 1;

    pid->kp = 100;
    pid->ki = 0;
    pid->kp = 0;
}

/**
 * @brief PID controller setpoint set
 * @param pid: PID controller
 * @param new_value: New setpoint value
 */
void pid_setpoint_update(pid_controller_t *pid, int32_t new_value) {
    pid->setpoint = new_value;
}

/**
 * @brief PID controller update
 */
int32_t pid_controller_update(pid_controller_t *pid, int32_t input) {

    int32_t error = pid->setpoint - input;
    int32_t proportional = pid->kp * error;

    // pid->integrate += 0.5 * pid->ki * pid->dt * (error + pid->prev_error);
    //
    // // integral path saturation: anti-wind-up via integrator clamping
    // if (pid->integrate > pid->lim_max) {
    //     pid->integrate = pid->lim_max;
    // }
    // if (pid->integrate < pid->lim_min) {
    //     pid->integrate = pid->lim_min;
    // }
    //
    // pid->derivate = -(2.0f * pid->kd * (input - pid->prev_input) +
    //                   (2.0f * pid->tau - pid->dt) * pid->derivate) /
    //                 (2.0f * pid->tau + pid->dt);
    //
    // pid->output = proportional + pid->integrate + pid->integrate;

    pid->output = proportional;

    // clamp controller output
    if (pid->output > pid->lim_max) {
        pid->output = pid->lim_max;
    }
    if (pid->output < pid->lim_min) {
        pid->output = pid->lim_min;
    }

    pid->prev_error = error;

    return pid->setpoint;
}
