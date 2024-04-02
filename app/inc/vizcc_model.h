/**
 * @file vizcc_model.h
 * @brief Vizcacha mechanical model
 * @author Gonzalo G. Fernandez
 */

/**
 * @brief Vizcacha mechanical model context
 */
typedef struct {
    float wheel_r;    /*!> Wheel radius */
    float body_width; /*!> Body (chasis) length wheel-to-wheel */
} vizcc_model_t;

void vizcc_model_init(vizcc_model_t *self, float wheel_r, float body_width);
void vizcc_model_deinit(vizcc_model_t *self);
void vizcc_model_forward_kinematics(vizcc_model_t *self, float w_left, float w_right, float *v_out,
                                    float *w_out);
