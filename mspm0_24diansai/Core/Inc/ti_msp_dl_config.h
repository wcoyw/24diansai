/**
 * @file    ti_msp_dl_config.h
 * @brief   MSPM0G3507 DriverLib configuration
 *
 * This header aggregates all TI DriverLib includes and declares
 * the clock/peripheral init functions.  It replaces what SysConfig
 * would auto-generate — we hand-write it so the project is
 * self-contained and editable in VSCode.
 *
 * SysConfig equivalent: Board_init(), SYSCFG_DL_init(), etc.
 */
#ifndef TI_MSP_DL_CONFIG_H
#define TI_MSP_DL_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* DriverLib core                                                     */
/* ------------------------------------------------------------------ */
#include <ti/devices/msp/msp.h>          /* CMSIS + device header    */

/* DriverLib peripheral modules (only the ones we use) */
#include <ti/driverlib/dl_sysctl.h>       /* Clock, power, reset      */
#include <ti/driverlib/dl_gpio.h>         /* GPIO                     */
#include <ti/driverlib/dl_timer.h>        /* TIMG/TIMA (PWM, QEI, etc) */
#include <ti/driverlib/dl_uart.h>         /* UART                     */
#include <ti/driverlib/dl_common.h>       /* DL_Common functions      */

/* Interrupt helpers */
#include <ti/driverlib/m0p/dl_cortex.h>

/* ------------------------------------------------------------------ */
/* Clock speeds (Hz) — after system clock init                        */
/* ------------------------------------------------------------------ */
#define SYSTEM_CLOCK_HZ         80000000UL
#define MCLK_HZ                 80000000UL

/* ------------------------------------------------------------------ */
/* Peripheral init prototypes (implemented in clock.c + motor.c, etc) */
/* ------------------------------------------------------------------ */
void SystemClock_Config(void);          /* CPU / bus clock setup      */
void Board_GPIO_Init(void);             /* GPIO init for LED/Buzzer/… */
void Motor_PWM_Init(void);              /* TIMA0 PWM init             */
void Encoder_QEI_Init(void);            /* TIMG0 + TIMG1 QEI init     */
void Control_Timer_Init(void);          /* TIMG2 25 Hz period init    */
void Debug_UART_Init(void);             /* UART0 115200 init          */
void IMU_UART_Init(void);               /* UART1 115200 init          */

#ifdef __cplusplus
}
#endif

#endif /* TI_MSP_DL_CONFIG_H */
