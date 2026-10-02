/**
 * @file    track.c
 * @brief   7-sensor line tracking with continuous error proportional control
 *
 * Ported from STM32 version.  Key changes:
 *   D1..D7 macros → inline reads via DL_GPIO_readPins()
 *   All sensors on GPIOA → single port read for speed
 */
#include "main.h"
#include "track.h"
#include "motor.h"

/* ---- Tracking parameters (unchanged from STM32 version) ---- */
#define TRACK_BASE_SPEED            14
#define TRACK_LOST_SPEED            10
#define TRACK_CROSS_SPEED           12
#define TRACK_TURN_LIMIT            9
#define TRACK_ERR_SCALE             4
#define TRACK_EDGE_SPEED            11
#define TRACK_LOST_TURN_LIMIT       7

static int16_t s_last_track_turn = 0;

/* ---- Helpers ---- */

static int16_t track_limit_i16(int16_t x, int16_t min, int16_t max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

/* Read all 7 track sensors at once (all on GPIOA) and extract bits */
static uint8_t track_read_all(void)
{
    uint32_t raw = DL_GPIO_readPins(TRACK_PORT, TRACK_ALL_MASK);

    uint8_t bits = 0;
    if (raw & TRACK_D1_PIN) bits |= 0x01;
    if (raw & TRACK_D2_PIN) bits |= 0x02;
    if (raw & TRACK_D3_PIN) bits |= 0x04;
    if (raw & TRACK_D4_PIN) bits |= 0x08;
    if (raw & TRACK_D5_PIN) bits |= 0x10;
    if (raw & TRACK_D6_PIN) bits |= 0x20;
    if (raw & TRACK_D7_PIN) bits |= 0x40;
    return bits;
}

/* ---- Public API ---- */

void Track_Init(void)
{
    /* Configure all 7 sensors as input with pull-up */
    DL_GPIO_initDigitalInputFeatures(TRACK_D1_PIN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK_D2_PIN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK_D3_PIN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK_D4_PIN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK_D5_PIN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK_D6_PIN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(TRACK_D7_PIN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

uint8_t Track_AnyLine(void)
{
    uint32_t raw = DL_GPIO_readPins(TRACK_PORT, TRACK_ALL_MASK);
    return (raw & TRACK_ALL_MASK) ? 1 : 0;
}

void Track_move(void)
{
    uint8_t bits = track_read_all();
    uint8_t s1 = (bits >> 0) & 1;
    uint8_t s2 = (bits >> 1) & 1;
    uint8_t s3 = (bits >> 2) & 1;
    uint8_t s4 = (bits >> 3) & 1;
    uint8_t s5 = (bits >> 4) & 1;
    uint8_t s6 = (bits >> 5) & 1;
    uint8_t s7 = (bits >> 6) & 1;
    uint8_t count = s1 + s2 + s3 + s4 + s5 + s6 + s7;

    int16_t turn = 0;
    int16_t l = TRACK_BASE_SPEED;
    int16_t r = TRACK_BASE_SPEED;

    /* No line → use last known error direction */
    if (count == 0) {
        turn = track_limit_i16(s_last_track_turn,
                               -TRACK_LOST_TURN_LIMIT, TRACK_LOST_TURN_LIMIT);
        l = track_limit_i16((int16_t)(TRACK_LOST_SPEED + turn), 0, 99);
        r = track_limit_i16((int16_t)(TRACK_LOST_SPEED - turn), 0, 99);
        motor_start(l, r);
        return;
    }

    /* Wide line / crossing → go straight slow */
    if (count >= 4) {
        s_last_track_turn = 0;
        motor_start(TRACK_CROSS_SPEED, TRACK_CROSS_SPEED);
        return;
    }

    /* Weighted error sum: D1 leftmost → +6, D4 centre → 0, D7 rightmost → -6 */
    int16_t err_sum = (int16_t)(6 * s1 + 3 * s2 + 2 * s3
                              + 0 * s4
                              - 2 * s5 - 3 * s6 - 6 * s7);

    /* Proportional turning */
    turn = track_limit_i16((int16_t)((err_sum * TRACK_ERR_SCALE) / (int16_t)count),
                           -TRACK_TURN_LIMIT, TRACK_TURN_LIMIT);
    s_last_track_turn = turn;

    /* Edge detection → slow down */
    if (count == 1 && (s1 || s7)) {
        l = track_limit_i16((int16_t)(TRACK_EDGE_SPEED + turn), 0, 99);
        r = track_limit_i16((int16_t)(TRACK_EDGE_SPEED - turn), 0, 99);
    } else {
        l = track_limit_i16((int16_t)(TRACK_BASE_SPEED + turn), 0, 99);
        r = track_limit_i16((int16_t)(TRACK_BASE_SPEED - turn), 0, 99);
    }

    motor_start(l, r);
}
