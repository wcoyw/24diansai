/**
 * @file    led.h
 * @brief   RGB LED 控制
 */
#ifndef __LED_H
#define __LED_H

#include "main.h"

void LED_Init(void);
void LED_ON(void);          /* 所有灯全亮 = 白光 */
void LED_OFF(void);         /* 所有灯全灭 */
void LED_Toggle(void);      /* 翻转红灯 */

void LED_Red(uint8_t on);
void LED_Green(uint8_t on);
void LED_Blue(uint8_t on);

#endif /* __LED_H__ */
