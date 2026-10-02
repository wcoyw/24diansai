/**
 * @file    pid.h
 * @brief   PID controller — pure algorithm, platform-independent
 *
 * Ported directly from STM32 version.  No HAL dependency.
 * Replaces #include "main.h" with #include <stdint.h>.
 */
#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float Target;
    float Actual;
    float Out;

    float Kp;
    float Ki;
    float Kd;

    float Error0;
    float Error1;
    float ErrorInt;

    float OutMax;           /* Output upper limit    */
    float OutMin;           /* Output lower limit    */
    float IntMax;           /* Integral upper limit  */
    float IntMin;           /* Integral lower limit  */
    float ErrHighThresh;    /* Error > this → integral coefficient = 0 */
    float ErrLowThresh;     /* Error < this → integral coefficient = 1 */
} PID_t;

void PID_Update(PID_t *p);

#ifdef __cplusplus
}
#endif
#endif /* __PID_H__ */
