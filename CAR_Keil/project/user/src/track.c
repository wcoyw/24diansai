/**
 * @file    track.c
 * @brief   7 路红外循迹传感器 — 加权误差比例控制
 *
 * 传感器分布在不同 PORT (PA/PB), 逐路读取。
 * 传感器: 高电平 = 线上 (白色), 低电平 = 线外 (黑色)
 */
#include "main.h"
#include "track.h"
#include "motor.h"

/* ---- 循迹参数 ---- */
#define TRACK_BASE_SPEED            14
#define TRACK_LOST_SPEED            10
#define TRACK_CROSS_SPEED           12
#define TRACK_TURN_LIMIT            9
#define TRACK_ERR_SCALE             4
#define TRACK_EDGE_SPEED            11
#define TRACK_LOST_TURN_LIMIT       7

static int16_t s_last_track_turn = 0;

/* 所有 7 路传感器引脚列表 (方便初始化) */
static const gpio_pin_enum track_pins[7] = {
    TRACK_D1_PIN,   /* PA12 */
    TRACK_D2_PIN,   /* PB19 */
    TRACK_D3_PIN,   /* PB17 */
    TRACK_D4_PIN,   /* PA16 */
    TRACK_D5_PIN,   /* PA14 */
    TRACK_D6_PIN,   /* PB20 */
    TRACK_D7_PIN,   /* PB25 */
};

static int16_t track_limit_i16(int16_t x, int16_t min, int16_t max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

/* 读取单路传感器 (高=线上) */
static uint8_t track_read_one(gpio_pin_enum pin)
{
    return gpio_get_level(pin);
}

/* 读取全部 7 路, 返回位掩码 */
static uint8_t track_read_all(void)
{
    uint8_t bits = 0;
    if (track_read_one(TRACK_D1_PIN)) bits |= 0x01;
    if (track_read_one(TRACK_D2_PIN)) bits |= 0x02;
    if (track_read_one(TRACK_D3_PIN)) bits |= 0x04;
    if (track_read_one(TRACK_D4_PIN)) bits |= 0x08;
    if (track_read_one(TRACK_D5_PIN)) bits |= 0x10;
    if (track_read_one(TRACK_D6_PIN)) bits |= 0x20;
    if (track_read_one(TRACK_D7_PIN)) bits |= 0x40;
    return bits;
}

/* ---- 公开 API ---- */

void Track_Init(void)
{
    for (int i = 0; i < 7; i++) {
        gpio_init(track_pins[i], GPI, GPIO_LOW, GPI_PULL_UP);
    }
}

uint8_t Track_AnyLine(void)
{
    for (int i = 0; i < 7; i++) {
        if (gpio_get_level(track_pins[i]))
            return 1;
    }
    return 0;
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

    /* 全线丢失 → 保持上次方向 */
    if (count == 0) {
        turn = track_limit_i16(s_last_track_turn,
                               -TRACK_LOST_TURN_LIMIT, TRACK_LOST_TURN_LIMIT);
        l = track_limit_i16((int16_t)(TRACK_LOST_SPEED + turn), 0, 99);
        r = track_limit_i16((int16_t)(TRACK_LOST_SPEED - turn), 0, 99);
        motor_start(l, r);
        return;
    }

    /* 宽线/路口 → 直走减速 */
    if (count >= 4) {
        s_last_track_turn = 0;
        motor_start(TRACK_CROSS_SPEED, TRACK_CROSS_SPEED);
        return;
    }

    /* 加权误差: D1(+6) ← 中心 → D7(-6) */
    int16_t err_sum = (int16_t)(6 * s1 + 3 * s2 + 2 * s3
                              + 0 * s4
                              - 2 * s5 - 3 * s6 - 6 * s7);

    turn = track_limit_i16((int16_t)((err_sum * TRACK_ERR_SCALE) / (int16_t)count),
                           -TRACK_TURN_LIMIT, TRACK_TURN_LIMIT);
    s_last_track_turn = turn;

    /* 边缘检测 → 减速 */
    if (count == 1 && (s1 || s7)) {
        l = track_limit_i16((int16_t)(TRACK_EDGE_SPEED + turn), 0, 99);
        r = track_limit_i16((int16_t)(TRACK_EDGE_SPEED - turn), 0, 99);
    } else {
        l = track_limit_i16((int16_t)(TRACK_BASE_SPEED + turn), 0, 99);
        r = track_limit_i16((int16_t)(TRACK_BASE_SPEED - turn), 0, 99);
    }

    motor_start(l, r);
}
