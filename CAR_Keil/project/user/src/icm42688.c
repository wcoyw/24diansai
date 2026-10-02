/**
 * @file    icm42688.c
 * @brief   ICM-42688-P 6 轴 IMU 驱动 (软件 I2C, 地址 0x69)
 *
 * 基于逐飞库 zf_driver_soft_iic 实现。
 * 寄存器读写使用 ICM-42688-P 标准协议。
 *
 * 配置:
 *   Gyro:  ±2000 dps, ODR = 1kHz
 *   Accel: ±16g,     ODR = 1kHz
 *   INT1:  数据就绪中断 (PA13)
 *   FSYNC: 未使用
 *
 * ICM-42688-P 寄存器操作模式:
 *   写: START + ADDR(W) + REG + DATA[] + STOP
 *   读: START + ADDR(W) + REG + REPEAT_START + ADDR(R) + DATA[] + STOP
 */
#include "main.h"
#include "icm42688.h"

/* 软件 I2C 对象 */
static soft_iic_info_struct s_i2c;

/* ================================================================== */
/*  I2C 底层读写                                                        */
/* ================================================================== */

static uint8_t icm_read_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    /* ICM-42688-P 读: 先写寄存器地址, 再读数据 */
    soft_iic_start(&s_i2c);
    if (soft_iic_send_data(&s_i2c, (uint8_t)(ICM42688_I2C_ADDR << 1))) {
        soft_iic_stop(&s_i2c);
        return 0;
    }
    if (soft_iic_send_data(&s_i2c, reg)) {
        soft_iic_stop(&s_i2c);
        return 0;
    }

    /* 重复起始 + 读 */
    soft_iic_start(&s_i2c);
    if (soft_iic_send_data(&s_i2c, (uint8_t)((ICM42688_I2C_ADDR << 1) | 0x01))) {
        soft_iic_stop(&s_i2c);
        return 0;
    }

    for (uint8_t i = 0; i < len; i++) {
        data[i] = soft_iic_read_data(&s_i2c, (i < len - 1) ? 0 : 1);  /* 最后字节 NACK */
    }

    soft_iic_stop(&s_i2c);
    return 1;
}

static uint8_t icm_write_reg(uint8_t reg, uint8_t data)
{
    soft_iic_start(&s_i2c);
    if (soft_iic_send_data(&s_i2c, (uint8_t)(ICM42688_I2C_ADDR << 1))) {
        soft_iic_stop(&s_i2c);
        return 0;
    }
    if (soft_iic_send_data(&s_i2c, reg)) {
        soft_iic_stop(&s_i2c);
        return 0;
    }
    if (soft_iic_send_data(&s_i2c, data)) {
        soft_iic_stop(&s_i2c);
        return 0;
    }
    soft_iic_stop(&s_i2c);
    return 1;
}

/* 读/修改/写 寄存器某几位 */
static uint8_t icm_write_reg_bits(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t tmp;
    if (!icm_read_reg(reg, &tmp, 1))
        return 0;
    tmp = (tmp & ~mask) | (value & mask);
    return icm_write_reg(reg, tmp);
}

/* ================================================================== */
/*  初始化                                                              */
/* ================================================================== */

uint8_t ICM42688_WhoAmI(void)
{
    uint8_t whoami = 0;
    if (!icm_read_reg(ICM42688_REG_WHO_AM_I, &whoami, 1))
        return 0;
    /* ICM-42688-P WHO_AM_I = 0x47 */
    return whoami;
}

uint8_t ICM42688_Init(void)
{
    uint8_t whoami;
    uint32_t retry = 0;

    /* ---- 初始化软 I2C ---- */
    soft_iic_init(&s_i2c, ICM42688_I2C_ADDR, I2C_DELAY_US, I2C_SCL_PIN, I2C_SDA_PIN);

    /* ---- 等待芯片就绪 (WHO_AM_I 检查) ---- */
    do {
        whoami = ICM42688_WhoAmI();
        if (whoami == 0x47) break;
        /* 短暂延时 */
        for (volatile uint32_t d = 0; d < 10000; d++) { }
        retry++;
    } while (retry < 100);

    if (whoami != 0x47)
        return 0;  /* ICM-42688-P 未检测到 */

    /* ---- 复位设备 ---- */
    icm_write_reg(ICM42688_REG_DEVICE_CONFIG, 0x01);  /* DEVICE_CONFIG: soft reset */
    /* 等待复位完成 (约 1ms) */
    for (volatile uint32_t d = 0; d < 50000; d++) { }

    /* 确认复位后 WHO_AM_I 依然正确 */
    if (ICM42688_WhoAmI() != 0x47)
        return 0;

    /* ---- 电源管理: 关闭睡眠, 使能温度传感器 ---- */
    /* PWR_MGMT0: TEMP_DIS=0, IDLE=0, GYRO_MODE=10(LN), ACCEL_MODE=10(LN) */
    icm_write_reg(ICM42688_REG_PWR_MGMT0, 0x0F);

    /* ---- Gyro 配置: ±2000dps, ODR=1kHz ---- */
    /* GYRO_CONFIG0: FS_SEL=0(2000dps), ODR=6(1kHz) */
    icm_write_reg(ICM42688_REG_GYRO_CONFIG0, 0x06);

    /* ---- Accel 配置: ±16g, ODR=1kHz ---- */
    /* ACCEL_CONFIG0: FS_SEL=0(16g), ODR=6(1kHz) */
    icm_write_reg(ICM42688_REG_ACCEL_CONFIG0, 0x06);

    /* ---- Gyro + Accel 低通滤波: BW=ODR/4 左右 ---- */
    icm_write_reg(ICM42688_REG_GYRO_ACCEL_CONFIG0, 0x44);

    /* ---- INT 配置: INT1=数据就绪 ---- */
    /* INT_CONFIG: INT1_MODE=pulsed, INT1_DRIVE=push-pull, INT1_POL=active-high */
    icm_write_reg(ICM42688_REG_INT_CONFIG, 0x02);
    /* INT_CONFIG0: 使能 DATA_RDY_INT1 (bit 3) = 1 */
    icm_write_reg(ICM42688_REG_INT_CONFIG0, 0x08);
    /* INT_CONFIG1: 清除所有 INT 使能, 仅数据就绪 */
    icm_write_reg(ICM42688_REG_INT_CONFIG1, 0x00);

    /* ---- 配置 ICM INT1 引脚 (PA13) 作为输入 + 上拉 ---- */
    gpio_init(ICM_INT1_PIN, GPI, GPIO_LOW, GPI_PULL_UP);

    return 1;
}

/* ================================================================== */
/*  读取原始数据                                                        */
/* ================================================================== */

uint8_t ICM42688_ReadRaw(icm42688_raw_data_t *data)
{
    uint8_t buf[14];  /* ACCEL_X[15:8] ... GYRO_Z[7:0] + TEMP[15:8] + TEMP[7:0] */

    /* ICM-42688-P: 从 ACCEL_DATA_X1 (0x1F) 开始连续读 14 字节
     *   [0]  ACCEL_DATA_X1 (X[15:8])
     *   [1]  ACCEL_DATA_X0 (X[7:0])
     *   [2]  ACCEL_DATA_Y1
     *   [3]  ACCEL_DATA_Y0
     *   [4]  ACCEL_DATA_Z1
     *   [5]  ACCEL_DATA_Z0
     *   [6]  GYRO_DATA_X1
     *   [7]  GYRO_DATA_X0
     *   [8]  GYRO_DATA_Y1
     *   [9]  GYRO_DATA_Y0
     *   [10] GYRO_DATA_Z1
     *   [11] GYRO_DATA_Z0
     *   [12] TEMP_DATA1
     *   [13] TEMP_DATA0
     *
     * 注意: ICM-42688-P 的寄存器布局与 ICM-42688-V 不同!
     *  ACCEL 在前, GYRO 在后, TEMP 在最后。
     *  实际上标准布局是 TEMP → ACCEL → GYRO,
     *  从 TEMP_DATA1 (0x1D) 开始读 14 字节:
     *   [0:1]  TEMP
     *   [2:7]  ACCEL X/Y/Z
     *   [8:13] GYRO X/Y/Z
     */

    /* 从 TEMP_DATA1 (0x1D) 开始读 14 字节更符合标准布局 */
    if (!icm_read_reg(ICM42688_REG_TEMP_DATA1, buf, 14))
        return 0;

    data->temp    = (int16_t)((buf[0]  << 8) | buf[1]);
    data->accel_x = (int16_t)((buf[2]  << 8) | buf[3]);
    data->accel_y = (int16_t)((buf[4]  << 8) | buf[5]);
    data->accel_z = (int16_t)((buf[6]  << 8) | buf[7]);
    data->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    data->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    return 1;
}
