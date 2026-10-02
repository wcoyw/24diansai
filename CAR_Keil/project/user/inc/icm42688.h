/**
 * @file    icm42688.h
 * @brief   ICM-42688-P 6 轴 IMU 驱动 (软件 I2C, 地址 0x69)
 *
 * 基于逐飞库 zf_driver_soft_iic 实现。
 * 配置: ±2000dps gyro, ±16g accel, ODR=1kHz, INT1 数据就绪。
 */
#ifndef __ICM42688_H
#define __ICM42688_H

#include "main.h"

/* ICM-42688-P 寄存器地址 */
#define ICM42688_REG_DEVICE_CONFIG      0x11
#define ICM42688_REG_DRIVE_CONFIG       0x13
#define ICM42688_REG_INT_CONFIG         0x14
#define ICM42688_REG_FIFO_CONFIG        0x16
#define ICM42688_REG_GYRO_CONFIG0       0x4F
#define ICM42688_REG_ACCEL_CONFIG0      0x50
#define ICM42688_REG_GYRO_ACCEL_CONFIG0 0x52
#define ICM42688_REG_PWR_MGMT0          0x4E
#define ICM42688_REG_INT_CONFIG0        0x63
#define ICM42688_REG_INT_CONFIG1        0x64
#define ICM42688_REG_INT_SOURCE0        0x65
#define ICM42688_REG_WHO_AM_I           0x75
#define ICM42688_REG_ACCEL_DATA_X1      0x1F  /* ACCEL_X[15:8] */
#define ICM42688_REG_TEMP_DATA1         0x1D  /* TEMP[15:8] */
#define ICM42688_REG_GYRO_DATA_X1       0x25  /* GYRO_X[15:8] */

/* 传感器数据 (原始值) */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} icm42688_raw_data_t;

uint8_t ICM42688_Init(void);
uint8_t ICM42688_ReadRaw(icm42688_raw_data_t *data);
uint8_t ICM42688_WhoAmI(void);

#endif /* __ICM42688_H__ */
