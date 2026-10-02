/**
 * @file    pid.h
 * @brief   位置式 PID + 变速积分 + 输出/积分限幅 (纯数学, 无平台依赖)
 */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float Target;           /* 目标值         */
    float Actual;           /* 实际值         */
    float Kp, Ki, Kd;       /* PID 系数       */
    float Error0;           /* 本次误差       */
    float Error1;           /* 上次误差       */
    float ErrorInt;         /* 累计积分       */
    float Out;              /* PID 输出       */
    float OutMax, OutMin;   /* 输出限幅       */
    float IntMax, IntMin;   /* 积分限幅       */
    float ErrHighThresh;    /* 变速积分高阈值 */
    float ErrLowThresh;     /* 变速积分低阈值 */
} PID_t;

void PID_Update(PID_t *p);

#endif /* __PID_H__ */
