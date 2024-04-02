/**
 * @file vizcc_model.c
 * @brief Vizcacha mechanical model
 * @author Gonzalo G. Fernandez
 */

#include "vizcc_model.h"

/**
 * @brief Vizcacha mechanical model initialization
 * @param self: main context
 * @param wheel_r: Wheel radius
 * @param body_width: Body (chasis) length wheel-to-wheel
 */
void vizcc_model_init(vizcc_model_t *self, float wheel_r, float body_width) {
    self->wheel_r = wheel_r;
    self->body_width = body_width;
    // TODO: Handle 0 value error
}

/**
 * @brief Vizcacha mechanical model deinitialization
 * @param self: main context
 */
void vizcc_model_deinit(vizcc_model_t *self) {
}

/**
 * @brief Vizchacha forward kinematics
 * @param self: main context
 */
void vizcc_model_forward_kinematics(vizcc_model_t *self, float w_left, float w_right, float *v_out,
                                    float *w_out) {
    *v_out = (self->wheel_r / 2) * (w_left + w_right);
    *w_out = (self->wheel_r / self->body_width) * (w_right - w_left);
}
