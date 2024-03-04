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
    int32_t kp; /*!> Proportional constant */
    int32_t ki; /*!> Integral constant */
    int32_t kd; /*!> Derivative constant */

    int32_t tau; /*!> Derivative low-pass filter constant */

    /* Controller saturation */
    int32_t lim_min; /*!> Controller minimum output */
    int32_t lim_max; /*!> Controller maximum output */

    int32_t lim_min_int; /*!> Integrator clamp minimum output */
    int32_t lim_max_int; /*!> Integrator clamp maximum output */

    /* Controller memory */
    int32_t prev_error; /*!> Previous error */
    int32_t prev_input; /*!> Previous measurement (controller input) */
    int32_t integrate;  /*!> Current integration result */
    int32_t derivate;   /*!> Current derivation result */

    int32_t dt; /*!> Sample time in ms */

    int32_t setpoint; /*!> Controller setpoint */
    int32_t output;   /*!> PID controller output */

} pid_controller_t;

void pid_controller_init(pid_controller_t *pid);
void pid_setpoint_update(pid_controller_t *pid, int32_t new_value);
int32_t pid_controller_update(pid_controller_t *pid, int32_t input);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
