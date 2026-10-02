/**
 * @file    main.h
 * @brief   CAR 项目 — MSPM0G3507 主头文件 (基于逐飞库 + Keil MDK)
 *
 * 引脚分配严格按 CAR_引脚配置总表.md (2026-07-21)
 * 系统时钟: SYSOSC 32MHz, 无 PLL
 */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ti_msp_dl_config.h"
#include "zf_common_typedef.h"

#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "zf_driver_uart.h"
#include "zf_driver_pit.h"
#include "zf_driver_exti.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_encoder.h"
#include "zf_driver_timer.h"
#include "zf_driver_delay.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* ================================================================== */
/*  系统时钟                                                           */
/* ================================================================== */
#define SYSTEM_CLOCK_HZ         32000000UL          /* SYSOSC 32MHz, 无 PLL */

/* ================================================================== */
/*  PWM — TIMA0, 20kHz                                                */
/* ================================================================== */
#define PWM_FREQ_HZ             20000               /* 20kHz */
/* SeekFree pwm_channel_enum 映射 */
#define PWM_L_CH                PWM_TIM_A0_CH0_A21  /* PA21 = TIMA0 CH0, 左轮 */
#define PWM_R_CH                PWM_TIM_A0_CH1_A7   /* PA07 = TIMA0 CH1, 右轮 */

/* ================================================================== */
/*  电机方向 — GPIO 推挽输出                                           */
/* ================================================================== */
#define AIN1_PIN                B2                  /* PB02, 左轮方向1 */
#define AIN2_PIN                B3                  /* PB03, 左轮方向2 */
#define BIN1_PIN                B13                 /* PB13, 右轮方向1 */
#define BIN2_PIN                B15                 /* PB15, 右轮方向2 */

/* ================================================================== */
/*  编码器 — GPIO EXTI 双边沿 (非硬件 QEI)                             */
/* ================================================================== */
/* 左编码器 */
#define ENC_L_A_PIN             A28                 /* PA28, 左编码器 A */
#define ENC_L_B_PIN             A29                 /* PA29, 左编码器 B */
/* 右编码器 */
#define ENC_R_A_PIN             B14                 /* PB14, 右编码器 A */
#define ENC_R_B_PIN             B18                 /* PB18, 右编码器 B */

/* ================================================================== */
/*  UART                                                               */
/* ================================================================== */
/* 调试串口: UART0, PA10 TX / PA11 RX, 115200 */
#define DEBUG_UART_INDEX        UART_0
#define DEBUG_TX_PIN            UART0_TX_A10
#define DEBUG_RX_PIN            UART0_RX_A11

/* 视觉串口: UART1, PA08 TX / PA09 RX, 115200 */
#define VISION_UART_INDEX       UART_1
#define VISION_TX_PIN           UART1_TX_A8
#define VISION_RX_PIN           UART1_RX_A9

/* ================================================================== */
/*  软件 I2C — OLED + ICM-42688-P                                      */
/* ================================================================== */
#define I2C_SCL_PIN             A15                 /* PA15 = SCL */
#define I2C_SDA_PIN             A30                 /* PA30 = SDA */
#define I2C_DELAY_US            2                   /* 软 I2C 延时 */

/* ICM-42688-P I2C 地址: SA0=VCC → 0x69 (7-bit) */
#define ICM42688_I2C_ADDR       0x69

/* ICM INT1: PA13 */
#define ICM_INT1_PIN            A13

/* ================================================================== */
/*  循迹传感器 D1~D7 — GPIO 上拉输入                                   */
/* ================================================================== */
#define TRACK_D1_PIN            A12                 /* PA12 */
#define TRACK_D2_PIN            B19                 /* PB19 */
#define TRACK_D3_PIN            B17                 /* PB17 */
#define TRACK_D4_PIN            A16                 /* PA16 */
#define TRACK_D5_PIN            A14                 /* PA14 */
#define TRACK_D6_PIN            B20                 /* PB20 */
#define TRACK_D7_PIN            B25                 /* PB25 */

/* ================================================================== */
/*  按键 SW1~SW4 — GPIO 上拉输入                                       */
/* ================================================================== */
#define SW1_PIN                 B4                  /* PB04 */
#define SW2_PIN                 B23                 /* PB23 */
#define SW3_PIN                 B22                 /* PB22 */
#define SW4_PIN                 B5                  /* PB05 */

/* 按键返回值 */
#define BTN_NONE                0
#define BTN_1                   1
#define BTN_2                   2
#define BTN_3                   3
#define BTN_4                   4

/* ================================================================== */
/*  LED — GPIO 推挽输出                                                */
/* ================================================================== */
#define LED_R_PIN               B11                 /* PB11, 红 */
#define LED_G_PIN               B10                 /* PB10, 绿 */
#define LED_B_PIN               B0                  /* PB00, 蓝 */

/* ================================================================== */
/*  蜂鸣器 — GPIO 推挽输出                                             */
/* ================================================================== */
#define BUZZER_PIN              B16                 /* PB16 */

/* ================================================================== */
/*  控制定时器 PIT — TIMA1, 25Hz (40ms)                                */
/* ================================================================== */
#define CONTROL_PIT             PIT_TIM_A1
#define CONTROL_PERIOD_US       40000               /* 40ms = 25Hz */

/* ================================================================== */
/*  通用宏                                                             */
/* ================================================================== */
#define UNUSED(x)               ((void)(x))
#define ARRAY_SIZE(x)           (sizeof(x) / sizeof((x)[0]))

/* ================================================================== */
/*  全局变量声明                                                       */
/* ================================================================== */
extern int32_t current_encoder1;
extern int32_t current_encoder2;
extern int32_t location;

/* ================================================================== */
/*  函数声明                                                           */
/* ================================================================== */
void SystemClock_Config(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
