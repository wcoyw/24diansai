/**
 * @file    track.h
 * @brief   7 路红外循迹传感器
 */
#ifndef __TRACK_H
#define __TRACK_H

#include "main.h"

void    Track_Init(void);
uint8_t Track_AnyLine(void);
void    Track_move(void);

#endif /* __TRACK_H__ */
