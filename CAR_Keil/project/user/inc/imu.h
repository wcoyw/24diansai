/**
 * @file    imu.h
 * @brief   IMU 姿态解算 — 基于 ICM-42688-P 数据
 *
 * 使用互补滤波/Mahony 算法计算 Roll/Pitch/Yaw。
 * 在 25Hz 控制 ISR 中调用 IMU_Update()。
 */
#ifndef __IMU_H
#define __IMU_H

#include "main.h"

void IMU_Init(void);
void IMU_Update(float *roll, float *pitch, float *yaw);

#endif /* __IMU_H__ */
