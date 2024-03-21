/**
 * @file pid.c
 * @brief PID controller
 * @author Gonzalo G. Fernandez
 */

#include "pid.h"
#include <stdint.h>

/**
 * @brief PID controller initialization
 * - Clear controller memory values.
 */
void pid_controller_init(pid_controller_t *pid, float dt) {
    pid->prev_error = 0;
    pid->prev_input = 0;
    pid->integrate = 0;
    pid->derivate = 0;
    pid->setpoint = 0;
    pid->output = 0.0;

    pid->lim_max = +65535;
    pid->lim_min = -65535;

    pid->lim_max_int = 20000.0;
    pid->lim_min_int = -20000.0;

    pid->dt = dt;
    pid->tau = 0.2;

    pid->kp = 1.0;
    pid->ki = 166.7;
    pid->kp = 0.0;
}

/**
 * @brief PID controller setpoint set
 * @param pid: PID controller
 * @param new_value: New setpoint value
 */
void pid_setpoint_update(pid_controller_t *pid, float new_value) {
    pid->setpoint = new_value;
}

void pid_kp_update(pid_controller_t *pid, float new_value) {
    pid->kp = new_value;
}

/**
 * @brief PID controller update
 */
float pid_controller_update(pid_controller_t *pid, float input) {
    //
    // on-off controller
    // if (pid->setpoint > 0){
    //     pid->output = (input < pid->setpoint) ? pid->lim_max : 0.0;
    // } else {
    //     pid->output = (input > pid->setpoint) ? pid->lim_min : 0.0;
    // }

    // float zn_t = 100;
    // float zn_l = 25;

    float error = pid->setpoint - input;

    // proportional controller
    float proportional = pid->kp * error;

    // proportional-integral controller
    // float kp = 0.9 * (zn_t / zn_l);
    // float ki = zn_l / 0.3;
    // float proportional = kp * error;
    pid->integrate += 0.5 * pid->ki * pid->dt * (error + pid->prev_error);

    // integral path saturation: anti-wind-up via integrator clamping
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
    //
    pid->output = proportional + pid->integrate;

    // clamp controller output
    if (pid->output > pid->lim_max) {
        pid->output = pid->lim_max;
    }
    if (pid->output < pid->lim_min) {
        pid->output = pid->lim_min;
    }

    pid->prev_error = error;

    return pid->output;
}
