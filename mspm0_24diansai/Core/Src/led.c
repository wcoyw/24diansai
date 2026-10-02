/**
 * @file    led.c
 * @brief   LED indicator — active-high on PB12
 *
 * STM32 → MSPM0:
 *   HAL_GPIO_WritePin(x, y, SET)   → DL_GPIO_setPins(x, y)
 *   HAL_GPIO_WritePin(x, y, RESET) → DL_GPIO_clearPins(x, y)
 *   HAL_GPIO_TogglePin(x, y)       → DL_GPIO_togglePins(x, y)
 */
#include "main.h"
#include "led.h"

void LED_Init(void)
{
    DL_GPIO_initDigitalOutput(LED_IOMUX);
    DL_GPIO_clearPins(LED_PORT, LED_PIN);
}

void LED_ON(void)
{
    DL_GPIO_setPins(LED_PORT, LED_PIN);
}

void LED_OFF(void)
{
    DL_GPIO_clearPins(LED_PORT, LED_PIN);
}

void LED_Toggle(void)
{
    DL_GPIO_togglePins(LED_PORT, LED_PIN);
}
