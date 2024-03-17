/**
 * @file uros_parameter_server.c
 * @brief Micro-ROS parameter server
 * @author Gonzalo G. Fernandez
 */

#include "FreeRTOS.h"
#include "portmacro.h"
#include "queue.h"
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <wchar.h>

#include "micro_ros_layer.h"

/**
 * @brief Maximum number of double parameters in micro-ROS parameter server
 */
#define UROS_PARAM_SERVER_MAX_DOUBLES 20

/**
 * @brief micro-ROS double parameter register
 */
typedef struct {
    const char *name;                           /*!> Parameter name */
    const char *description;                    /*!> Parameter description */
    const char *limits;                         /*!> Parameter limits description */
    double value;                               /*!> Parameter value */
    param_double_callback_t on_change_callback; /*!> Callback for update on change */
} param_double_reg_t;

static rclc_parameter_server_t param_server; /*!> micro-ROS parameter server */
static param_double_reg_t
    _param_double_reg[UROS_PARAM_SERVER_MAX_DOUBLES]; /*!> Array of double parameters */
static uint8_t _param_double_w_head = 0;              /*!> Index for double parameters register */
static QueueHandle_t _param_double_reg_queue = NULL;

/**
 * @brief Get double parameter callback for update
 * @param param_name: micro-ROS parameter name
 */
param_double_callback_t get_on_change_callback(const char *param_name) {
    for (uint8_t i = 0; i < _param_double_w_head; i++) { // linear search o(n)
        if (0 == strcmp(param_name, _param_double_reg[i].name)) {
            return _param_double_reg[i].on_change_callback;
        }
    }
    return NULL;
}

/**
 * @brief micro-ROS parameter server callback on parameter change
 */
bool on_parameter_changed(const Parameter *old_param, const Parameter *new_param, void *context) {
    (void)context;

    if (old_param == NULL && new_param == NULL) {
        printf("Callback error, both parameters are NULL\n");
        return false;
    }

    if (old_param == NULL) {
        printf("Creating new parameter %s\n", new_param->name.data);
    } else if (new_param == NULL) {
        printf("Deleting parameter %s\n", old_param->name.data);
    } else {
        printf("Parameter %s modified.", old_param->name.data);
        switch (old_param->value.type) {
        case RCLC_PARAMETER_BOOL:
            printf(" Old value: %d, New value: %d (bool)", old_param->value.bool_value,
                   new_param->value.bool_value);
            break;
        case RCLC_PARAMETER_INT:
            printf(" Old value: %lld, New value: %lld (int)", old_param->value.integer_value,
                   new_param->value.integer_value);
            break;
        case RCLC_PARAMETER_DOUBLE:
            printf(" Old value: %f, New value: %f (double)", old_param->value.double_value,
                   new_param->value.double_value);
            param_double_callback_t on_change_double_callback;
            on_change_double_callback = get_on_change_callback(new_param->name.data);
            on_change_double_callback(new_param->value.double_value);
            break;
        default:
            break;
        }
        printf("\n");
    }

    return true;
}

/**
 * @brief Get micro-ROS parameter server
 */
rclc_parameter_server_t *uros_parameter_server_get(void) {
    return &param_server;
}

/**
 * @brief micro-ROS parameter server initialization
 * @param node: micro-ROS node
 * @param executor: micro-ROS executor
 */
uros_status_t uros_parameter_server_init(rcl_node_t *node, rclc_executor_t *executor) {
    rclc_parameter_server_init_default(&param_server, node);
    rclc_executor_add_parameter_server(executor, &param_server, on_parameter_changed);
}

/**
 * @brief micro-ROS parameter server deinitialization
 * @param node: micro-ROS node
 */
uros_status_t uros_parameter_server_deinit(rcl_node_t *node) {
    rclc_parameter_server_fini(&param_server, node);
}

/**
 * @brief Enqueue double parameter dor micro-ROS parameter server
 */
uros_status_t uros_parameter_enqueue_double(const char *param_name, const char *param_description,
                                            const char *param_limits, double initial_value,
                                            param_double_callback_t on_change_callback) {
    _param_double_w_head++;
    if (_param_double_w_head > UROS_PARAM_SERVER_MAX_DOUBLES) {
        return UROS_ERROR;
    }

    if (NULL == _param_double_reg_queue) {
        _param_double_reg_queue =
            xQueueCreate(UROS_PARAM_SERVER_MAX_DOUBLES, sizeof(param_double_reg_t *));
    }

    _param_double_reg[_param_double_w_head - 1].name = param_name;
    _param_double_reg[_param_double_w_head - 1].description = param_description;
    _param_double_reg[_param_double_w_head - 1].limits = param_limits;
    _param_double_reg[_param_double_w_head - 1].value = initial_value;
    _param_double_reg[_param_double_w_head - 1].on_change_callback = on_change_callback;

    if (pdPASS != xQueueSend(_param_double_reg_queue, &_param_double_reg[_param_double_w_head - 1],
                             portMAX_DELAY)) {
        return UROS_ERROR;
    }
    return UROS_OK;
}

/**
 * @brief Register double parameters in micro-ROS parameter server from queue
 */
uros_status_t uros_parameter_register_double(void) {
    param_double_reg_t param_double_reg;
    BaseType_t status = xQueueReceive(_param_double_reg_queue, &param_double_reg, 0);
    // TODO: Error handling
    if (pdPASS == status) {
        rcl_ret_t rc;
        rc = rclc_add_parameter(&param_server, param_double_reg.name, RCLC_PARAMETER_DOUBLE);
        rc =
            rclc_parameter_set_double(&param_server, param_double_reg.name, param_double_reg.value);
        rclc_add_parameter_description(&param_server, param_double_reg.name,
                                       param_double_reg.description, param_double_reg.limits);
    }
}
