/**
 * @file    buzzer.h
 * @brief   Buzzer — DL_GPIO wrapper
 */
#ifndef __BUZZER_H__
#define __BUZZER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_ON(void);
void Buzzer_OFF(void);

#ifdef __cplusplus
}
#endif
#endif /* __BUZZER_H__ */
