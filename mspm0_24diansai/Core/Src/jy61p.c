/**
 * @file    jy61p.c
 * @brief   JY61P IMU angle-packet parser (0x55 0x53 ...)
 *
 * Key change from STM32 HAL:
 *   - No more HAL_UART_RxCpltCallback — ISR is in main.c.
 *   - JY61p_ParseByte() returns 1 when a frame completes; caller
 *     updates s_roll/s_pitch/s_yaw.
 *   - Added checksum validation (SUM of bytes [0..6] & 0xFF == byte[7]).
 */
#include "jy61p.h"
#include <string.h>

/* ---- Private state ---- */
static uint8_t  s_angle_raw[8];       /* [0..6]=data, [7]=checksum */
static volatile float s_roll  = 0.0f;
static volatile float s_pitch = 0.0f;
static volatile float s_yaw   = 0.0f;

/* ---- Public API ---- */

void JY61p_Init(void)
{
    memset(s_angle_raw, 0, sizeof(s_angle_raw));
}

/**
 * @brief  Feed one byte to the JY61P parser state machine.
 * @param  byte  received byte
 * @return 1 = a valid angle frame was completed, 0 = still accumulating
 */
uint8_t JY61p_ParseByte(uint8_t byte)
{
    static uint8_t sta = 0;
    static uint8_t idx = 0;
    uint8_t ready = 0;

    switch (sta)
    {
    case 0:  /* Wait for 0x55 header */
        if (byte == 0x55) sta = 1;
        break;

    case 1:  /* Wait for 0x53 (angle packet type) */
        if (byte == 0x53) {
            idx = 0;
            sta = 2;
        } else if (byte == 0x55) {
            sta = 1;   /* consecutive header */
        } else {
            sta = 0;
        }
        break;

    case 2:  /* Accumulate 7 data bytes */
        s_angle_raw[idx++] = byte;
        if (idx >= 7) sta = 3;
        break;

    case 3:  /* Checksum byte — validate and accept */
    {
        uint8_t sum = 0;
        for (uint8_t i = 0; i < 7; i++)
            sum += s_angle_raw[i];
        sta = 0;
        if (sum == byte) {
            /* Frame valid — parse angles */
            s_roll  = (int16_t)((s_angle_raw[1] << 8) | s_angle_raw[0]) / 32768.0f * 180.0f;
            s_pitch = (int16_t)((s_angle_raw[3] << 8) | s_angle_raw[2]) / 32768.0f * 180.0f;
            s_yaw   = (int16_t)((s_angle_raw[5] << 8) | s_angle_raw[4]) / 32768.0f * 180.0f;
            ready = 1;
        }
        /* else: checksum mismatch → discard silently */
        break;
    }

    default:
        sta = 0;
        break;
    }

    return ready;
}

void JY61p_GetAngle(float *Roll, float *Pitch, float *Yaw)
{
    /*
     * On Cortex-M0+ there is no atomic 32-bit load guarantee for
     * volatile floats across interrupt / mainline.  We accept the
     * extremely low risk of a torn read (IMU updates at 100 Hz vs
     * 25 Hz control loop).
     */
    *Roll  = s_roll;
    *Pitch = s_pitch;
    *Yaw   = s_yaw;
}
