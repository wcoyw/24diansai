/**
 * @file    buzzer.c
 * @brief   Buzzer — active-low on PB13
 *
 * Removed Buzzer_Beep() blocking delay.  Use s_alert_ticks in main.c instead.
 */
#include "main.h"
#include "buzzer.h"

void Buzzer_Init(void)
{
    DL_GPIO_initDigitalOutput(BUZZER_IOMUX);
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);   /* inactive = high */
}

void Buzzer_ON(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN);  /* active = low */
}

void Buzzer_OFF(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN);
}
