/**
 * @file    imu.c
 * @brief   IMU 姿态解算 — 互补滤波 (基于 ICM-42688-P 原始数据)
 *
 * 在 25Hz 控制 ISR 中调用 IMU_Update() 更新 Roll/Pitch/Yaw。
 *
 * 算法:
 *   Roll/Pitch: 加速度计 + 陀螺仪互补滤波 (α = 0.98 gyro + 0.02 accel)
 *   Yaw:       纯陀螺仪积分 (无磁力计, 会有漂移)
 *
 * 校准: 启动时采集静止样本, 计算陀螺仪零偏。
 *
 * 单位换算:
 *   Gyro ±2000dps / 32768 = dps/LSB → rad/s
 *   Accel ±16g / 32768 = g/LSB → m/s² (1g = 9.80665)
 */
#include "main.h"
#include "imu.h"
#include "icm42688.h"
#include <math.h>

/* ================================================================== */
/*  校准数据                                                           */
/* ================================================================== */
#define CALIBRATION_SAMPLES     100     /* 校准采样数 */

static float s_gyro_bias_x  = 0.0f;
static float s_gyro_bias_y  = 0.0f;
static float s_gyro_bias_z  = 0.0f;

/* ================================================================== */
/*  互补滤波系数                                                       */
/* ================================================================== */
#define FILTER_ALPHA            0.98f   /* gyro 权重 */

/* ================================================================== */
/*  尺度因子                                                           */
/* ================================================================== */
/* Gyro: ±2000dps → 1 LSB = 2000/32768 ≈ 0.061035 dps
 * 转换为 rad/s: * PI / 180.0
 */
#define GYRO_SCALE_DPS          (2000.0f / 32768.0f)
#define GYRO_SCALE_RADS         (GYRO_SCALE_DPS * 3.14159265359f / 180.0f)

/* Accel: ±16g → 1 LSB = 16/32768 ≈ 0.000488 g
 * 转换为 m/s²: * 9.80665
 */
#define ACCEL_SCALE_G           (16.0f / 32768.0f)
#define ACCEL_SCALE_MS2         (ACCEL_SCALE_G * 9.80665f)

/* ================================================================== */
/*  IMU 姿态 (角度制)                                                  */
/* ================================================================== */
static float s_roll  = 0.0f;
static float s_pitch = 0.0f;
static float s_yaw   = 0.0f;

/* 上次调用时间步长 (秒) */
static float s_dt     = 0.04f;       /* 25Hz = 40ms */

/* ================================================================== */
/*  IMU 初始化                                                          */
/* ================================================================== */

void IMU_Init(void)
{
    /* 初始化 ICM-42688-P */
    if (!ICM42688_Init()) {
        /* 初始化失败 — 停在原地闪烁红灯 */
        while (1) {
            gpio_toggle_level(LED_R_PIN);
            for (volatile uint32_t d = 0; d < 200000; d++) { }
        }
    }

    /* ---- 陀螺仪零偏校准 ---- */
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    icm42688_raw_data_t raw;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        /* 等待数据就绪 (INT1 高电平) */
        while (gpio_get_level(ICM_INT1_PIN) == 0) { }

        if (ICM42688_ReadRaw(&raw)) {
            sum_gx += (float)raw.gyro_x;
            sum_gy += (float)raw.gyro_y;
            sum_gz += (float)raw.gyro_z;
        }

        /* 简单延时 (~1ms) */
        for (volatile uint32_t d = 0; d < 1000; d++) { }
    }

    s_gyro_bias_x = sum_gx / (float)CALIBRATION_SAMPLES;
    s_gyro_bias_y = sum_gy / (float)CALIBRATION_SAMPLES;
    s_gyro_bias_z = sum_gz / (float)CALIBRATION_SAMPLES;

    /* 姿态归零 */
    s_roll  = 0.0f;
    s_pitch = 0.0f;
    s_yaw   = 0.0f;
}

/* ================================================================== */
/*  IMU 姿态更新 (25Hz 控制 ISR 调用)                                   */
/* ================================================================== */

void IMU_Update(float *roll, float *pitch, float *yaw)
{
    icm42688_raw_data_t raw;

    if (!ICM42688_ReadRaw(&raw))
        return;

    /* ---- 陀螺仪数据 (rad/s, 减去零偏) ---- */
    float gx = ((float)raw.gyro_x - s_gyro_bias_x) * GYRO_SCALE_RADS;
    float gy = ((float)raw.gyro_y - s_gyro_bias_y) * GYRO_SCALE_RADS;
    float gz = ((float)raw.gyro_z - s_gyro_bias_z) * GYRO_SCALE_RADS;

    /* ---- 加速度计角度 (Roll/Pitch) ---- */
    float ax = (float)raw.accel_x * ACCEL_SCALE_G;
    float ay = (float)raw.accel_y * ACCEL_SCALE_G;
    float az = (float)raw.accel_z * ACCEL_SCALE_G;

    /* accel roll  = atan2(ay, az)
     * accel pitch = atan2(-ax, sqrt(ay² + az²))
     */
    float accel_roll  = atan2f(ay, az) * 180.0f / 3.14159265359f;
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / 3.14159265359f;

    /* ---- 陀螺仪积分 (Euler 法) ---- */
    float gyro_roll  = s_roll  + gx * s_dt * 180.0f / 3.14159265359f;
    float gyro_pitch = s_pitch + gy * s_dt * 180.0f / 3.14159265359f;

    /* ---- Yaw 纯积分 (Z 轴陀螺仪) ---- */
    s_yaw += gz * s_dt * 180.0f / 3.14159265359f;

    /* Yaw 保持在 [-180, 180] */
    while (s_yaw >  180.0f) s_yaw -= 360.0f;
    while (s_yaw < -180.0f) s_yaw += 360.0f;

    /* ---- 互补滤波: Roll/Pitch = α * gyro + (1-α) * accel ---- */
    s_roll  = FILTER_ALPHA * gyro_roll  + (1.0f - FILTER_ALPHA) * accel_roll;
    s_pitch = FILTER_ALPHA * gyro_pitch + (1.0f - FILTER_ALPHA) * accel_pitch;

    /* ---- 输出 ---- */
    *roll  = s_roll;
    *pitch = s_pitch;
    *yaw   = s_yaw;
}
