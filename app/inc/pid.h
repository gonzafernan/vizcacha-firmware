/**
 * @file pid.h
 * @brief PID controller interface
 * @author Gonzalo G. Fernandez
 */

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief PID controller structure
 */
typedef struct {
    /* Controller gains */
    float kp; /*!> Proportional constant */
    float ki; /*!> Integral constant */
    float kd; /*!> Derivative constant */

    float tau; /*!> Derivative low-pass filter constant */

    /* Controller saturation */
    float lim_min; /*!> Controller minimum output */
    float lim_max; /*!> Controller maximum output */

    float lim_min_int; /*!> Integrator clamp minimum output */
    float lim_max_int; /*!> Integrator clamp maximum output */

    /* Controller memory */
    float prev_error; /*!> Previous error */
    float prev_input; /*!> Previous measurement (controller input) */
    float integrate;  /*!> Current integration result */
    float derivate;   /*!> Current derivation result */

    float dt; /*!> Sample time in ms */

    float setpoint; /*!> Controller setpoint */
    float output;   /*!> PID controller output */

} pid_controller_t;

void pid_controller_init(pid_controller_t *pid, float dt);
void pid_kp_update(pid_controller_t *pid, float new_value);
void pid_ki_update(pid_controller_t *pid, float new_value);
void pid_kd_update(pid_controller_t *pid, float new_value);
void pid_setpoint_update(pid_controller_t *pid, float new_value);
float pid_controller_update(pid_controller_t *pid, float input);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
