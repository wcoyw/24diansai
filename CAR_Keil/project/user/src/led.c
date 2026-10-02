/**
 * @file    led.c
 * @brief   RGB LED 控制 (PB11=红, PB10=绿, PB00=蓝)
 */
#include "main.h"
#include "led.h"

void LED_Init(void)
{
    gpio_init(LED_R_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LED_G_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LED_B_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void LED_ON(void)
{
    gpio_high(LED_R_PIN);
    gpio_high(LED_G_PIN);
    gpio_high(LED_B_PIN);
}

void LED_OFF(void)
{
    gpio_low(LED_R_PIN);
    gpio_low(LED_G_PIN);
    gpio_low(LED_B_PIN);
}

void LED_Toggle(void)
{
    gpio_toggle_level(LED_R_PIN);
}

void LED_Red(uint8_t on)
{
    if (on) gpio_high(LED_R_PIN); else gpio_low(LED_R_PIN);
}

void LED_Green(uint8_t on)
{
    if (on) gpio_high(LED_G_PIN); else gpio_low(LED_G_PIN);
}

void LED_Blue(uint8_t on)
{
    if (on) gpio_high(LED_B_PIN); else gpio_low(LED_B_PIN);
}
