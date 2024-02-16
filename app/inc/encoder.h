/**
 * @file encoder.h
 * @brief Encoder utils
 * @author Gonzalo G. Fernandez
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C"
{
#endif

    void encoder_init(void);
    uint32_t encoder_get_value(void);
    int32_t encoder_diff_value(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */