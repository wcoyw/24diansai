/**
 * @file    button.h
 * @brief   非阻塞按键扫描 (在 25Hz 控制 ISR 中调用)
 */
#ifndef __BUTTON_H
#define __BUTTON_H

#include "main.h"

void    Button_Init(void);
uint8_t Button_Scan(void);      /* 在控制 ISR 中调用, 返回 BTN_NONE/BTN_1~4 */

#endif /* __BUTTON_H__ */
