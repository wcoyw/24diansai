/**
 * @file    motor.h
 * @brief   电机控制 — PWM + 方向 GPIO + 编码器 (GPIO EXTI)
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

void motor_Init(void);
void motor_start(int16_t left_pwm, int16_t right_pwm);
int16_t encoder_get_left(void);
int16_t encoder_get_right(void);
void encoder_clear_both(void);

#endif /* __MOTOR_H__ */
