/**
 ******************************************************************************
 * @file           : vizcc_defs.h
 * @author         : Gonzalo Gabriel Fernandez
 * @brief          : Vizcacha useful definitions
 ******************************************************************************
 */
#ifndef __VIZCC_DEFS_H
#define __VIZCC_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Vizcacha status type definition
 */
typedef enum {
    VIZCC_STATUS_ERROR = -1, /*!> Error in the process */
    VIZCC_STATUS_OK = 0,     /*!> Process ended as expected */
} vizcc_status_t;

#ifdef __cplusplus
}
#endif

#endif /* __VIZCC_DEFS_H */
