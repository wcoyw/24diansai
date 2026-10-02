/**
 * @file    buzzer.c
 * @brief   蜂鸣器控制 (PB16, 高电平 = ON)
 */
#include "main.h"
#include "buzzer.h"

void Buzzer_Init(void)
{
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void Buzzer_ON(void)
{
    gpio_high(BUZZER_PIN);
}

void Buzzer_OFF(void)
{
    gpio_low(BUZZER_PIN);
}
