/**
 * @file filter.h
 * @brief Digital filter utils
 * @author Gonzalo G. Fernandez
 */

#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IIR filter component
 */
typedef struct {
    float a[2];               /*!> Input signal constants */
    float b[1];               /*!> Output signal constants */
    float signal_in_hist[1];  /*!> Input signal history */
    float signal_out_hist[1]; /*!> Output signal history */
} filter_iir_t;

void filter_init(filter_iir_t *filter);
float filter_update(filter_iir_t *filter, float signal_in);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
