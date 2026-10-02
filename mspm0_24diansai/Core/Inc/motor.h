/**
 * @file    motor.h
 * @brief   Motor control API — PWM + encoder
 *
 * Replaces STM32 HAL version. Uses TIMA0 PWM and TIMG QEI.
 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- API (unchanged from STM32 version) ---- */
void motor_start(int16_t left_pwm, int16_t right_pwm);
void motor_Init(void);

#ifdef __cplusplus
}
#endif
#endif /* __MOTOR_H__ */
