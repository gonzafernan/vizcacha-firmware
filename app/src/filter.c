/**
 * @file filter.c
 * @brief Digital filter utils
 * @author Gonzalo G. Fernandez
 */

#include "filter.h"

void filter_init(filter_iir_t *filter) {
    filter->a[0] = 0.0728;
    filter->a[1] = 0.0728;
    filter->b[0] = 0.854;
    filter->signal_in_hist[0] = 0.0;
    filter->signal_out_hist[0] = 0.0;
}

float filter_update(filter_iir_t *filter, float signal_in) {
    float signal_out = filter->a[0] * signal_in + filter->a[1] * filter->signal_in_hist[0] +
                       filter->b[0] * filter->signal_out_hist[0];
    filter->signal_in_hist[0] = signal_in;
    filter->signal_out_hist[0] = signal_out;
    return signal_out;
}
