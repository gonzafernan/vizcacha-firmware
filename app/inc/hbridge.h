/**
 * @file hbridge.h
 * @brief H-bridge utils
 * @author Gonzalo G. Fernandez
 */

#ifndef __HBRIDGE_H
#define __HBRIDGE_H

#ifdef __cplusplus
extern "C"
{
#endif

    void hbridge_init(void);
    void hbridge_set_pwm(int value);

#ifdef __cplusplus
}
#endif

#endif /* __HBRIDGE_H */