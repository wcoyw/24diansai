/**
 * @file    pid.c
 * @brief   位置式 PID + 变速积分 + 输出/积分限幅
 *
 * 纯数学模块, 无平台依赖。直接从 STM32 版本复用。
 */
#include "pid.h"
#include <math.h>

void PID_Update(PID_t *p)
{
    /* 误差计算 */
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    if (p->Ki != 0.0f)
    {
        /* 变速积分: 误差大 → 减小 Ki 防止积分饱和 */
        float ki_coeff;
        float abs_err = fabsf(p->Error0);

        if (abs_err >= p->ErrHighThresh)
            ki_coeff = 0.0f;
        else if (abs_err <= p->ErrLowThresh)
            ki_coeff = 1.0f;
        else
            ki_coeff = (p->ErrHighThresh - abs_err)
                     / (p->ErrHighThresh - p->ErrLowThresh);

        p->ErrorInt += ki_coeff * p->Error0;

        /* 积分限幅 */
        if (p->ErrorInt > p->IntMax) p->ErrorInt = p->IntMax;
        if (p->ErrorInt < p->IntMin) p->ErrorInt = p->IntMin;
    }
    else
    {
        p->ErrorInt = 0.0f;
    }

    /* 位置式 PID */
    p->Out = p->Kp * p->Error0
           + p->Ki * p->ErrorInt
           + p->Kd * (p->Error0 - p->Error1);

    /* 输出限幅 */
    if (p->Out > p->OutMax) p->Out = p->OutMax;
    if (p->Out < p->OutMin) p->Out = p->OutMin;
}
