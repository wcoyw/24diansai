/**
 * @file    track.h
 * @brief   7-sensor line tracking — DL_GPIO wrapper
 *
 * STM32 → MSPM0:
 *   HAL_GPIO_ReadPin(port, pin) macro → DL_GPIO_readPins(port, mask) & pin
 *
 * Optimisation: all 7 sensors on PA port ⇒ one read for all.
 */
#ifndef __TRACK_H__
#define __TRACK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void Track_Init(void);
void Track_move(void);
uint8_t Track_AnyLine(void);

#ifdef __cplusplus
}
#endif
#endif /* __TRACK_H__ */
