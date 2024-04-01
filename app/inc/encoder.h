/**
 * @file encoder.h
 * @brief Encoder utils
 * @author Gonzalo G. Fernandez
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Incremental encoder handle
 */
typedef struct {
    void *handler;
    uint8_t hist_size;
    uint16_t hist[1];
} encoder_t;

void encoder_init(encoder_t *henc, void *context);
uint16_t encoder_get_value(encoder_t *henc);
int16_t encoder_diff_value(encoder_t *henc);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
