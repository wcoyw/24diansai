/**
 * @file    button.h
 * @brief   Non-blocking button scanner
 *
 * Replaces the STM32 version which used HAL_Delay() for debounce.
 * Scanning runs inside the 25 Hz control ISR — no blocking.
 */
#ifndef __BUTTON_H__
#define __BUTTON_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- Button ID returned by scan ---- */
#define BTN_NONE    0
#define BTN_1       1
#define BTN_2       2
#define BTN_3       3
#define BTN_4       4

void Button_Init(void);
uint8_t Button_Scan(void);      /* Call at 25 Hz; returns BTN_1..4 on press */

#ifdef __cplusplus
}
#endif
#endif /* __BUTTON_H__ */
