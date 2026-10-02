/**
 * @file    jy61p.h
 * @brief   JY61P IMU (UART1, 115200 bps) — angle packet parser
 *
 * STM32 → MSPM0:
 *   HAL_UART_Receive_IT()  → DL_UART RX interrupt, direct read in ISR
 *   HAL_UART_RxCpltCallback → replaced by UART1_IRQHandler in main.c
 */
#ifndef __JY61P_H_
#define __JY61P_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void JY61p_Init(void);
void JY61p_GetAngle(float *Roll, float *Pitch, float *Yaw);

#ifdef __cplusplus
}
#endif
#endif /* __JY61P_H_ */
