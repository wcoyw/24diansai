/**
 * @file    motor.c
 * @brief   电机控制 — TIMA0 PWM + 方向 GPIO + GPIO EXTI 编码器
 *
 * 逐飞库 API:
 *   pwm_init(pin, freq, duty)  — 初始化 PWM 通道
 *   pwm_set_duty(pin, duty)    — 设置占空比 (0~10000 = 0%~100%)
 *   gpio_init(pin, dir, level, mode) — 初始化 GPIO
 *   gpio_set_level(pin, level) — 设置 GPIO 电平
 *   exti_init(pin, trigger, callback, ptr) — 初始化 EXTI 中断
 *
 * 编码器: 不使用硬件 QEI, 改用 GPIO EXTI 双边沿触发计数。
 *   左编码器: PA28 (A), PA29 (B)
 *   右编码器: PB14 (A), PB18 (B)
 *   每个 EXTI 触发时, 读取 B 相确定方向, 累加/减计数。
 */
#include "main.h"
#include "motor.h"

/* ================================================================== */
/*  编码器计数 (在 EXTI ISR 中修改)                                     */
/* ================================================================== */
static volatile int32_t s_enc_l_count = 0;
static volatile int32_t s_enc_r_count = 0;

/* 方向信号缓存 (在 ISR 中读取) */
static uint8_t enc_l_b_state(void) { return gpio_get_level(ENC_L_B_PIN); }
static uint8_t enc_r_b_state(void) { return gpio_get_level(ENC_R_B_PIN); }

/* ================================================================== */
/*  编码器 EXTI 回调                                                   */
/* ================================================================== */

/**
 * 左编码器 A 相 (PA28) — 双边沿触发
 * 上升沿: B 高 → 正转加, B 低 → 反转减
 * 下降沿: B 低 → 正转加, B 高 → 反转减
 */
static void enc_l_a_callback(uint32_t pin, void *ptr)
{
    UNUSED(ptr);
    if (gpio_get_level(pin)) {
        /* 上升沿 */
        if (enc_l_b_state()) s_enc_l_count++;
        else                 s_enc_l_count--;
    } else {
        /* 下降沿 */
        if (enc_l_b_state()) s_enc_l_count--;
        else                 s_enc_l_count++;
    }
}

/**
 * 左编码器 B 相 (PA29) — 双边沿触发
 */
static void enc_l_b_callback(uint32_t pin, void *ptr)
{
    UNUSED(ptr);
    if (gpio_get_level(pin)) {
        /* 上升沿 */
        if (gpio_get_level(ENC_L_A_PIN)) s_enc_l_count--;
        else                             s_enc_l_count++;
    } else {
        /* 下降沿 */
        if (gpio_get_level(ENC_L_A_PIN)) s_enc_l_count++;
        else                             s_enc_l_count--;
    }
}

/**
 * 右编码器 A 相 (PB14) — 双边沿触发
 */
static void enc_r_a_callback(uint32_t pin, void *ptr)
{
    UNUSED(ptr);
    if (gpio_get_level(pin)) {
        if (enc_r_b_state()) s_enc_r_count++;
        else                 s_enc_r_count--;
    } else {
        if (enc_r_b_state()) s_enc_r_count--;
        else                 s_enc_r_count++;
    }
}

/**
 * 右编码器 B 相 (PB18) — 双边沿触发
 */
static void enc_r_b_callback(uint32_t pin, void *ptr)
{
    UNUSED(ptr);
    if (gpio_get_level(pin)) {
        if (gpio_get_level(ENC_R_A_PIN)) s_enc_r_count--;
        else                             s_enc_r_count++;
    } else {
        if (gpio_get_level(ENC_R_A_PIN)) s_enc_r_count++;
        else                             s_enc_r_count--;
    }
}

/* ================================================================== */
/*  编码器初始化                                                       */
/* ================================================================== */
static void encoder_init(void)
{
    /* ---- 左编码器 ---- */
    /* A 相 PA28: 输入 + 上拉 + EXTI 双边沿 */
    gpio_init(ENC_L_A_PIN, GPI, GPIO_LOW, GPI_PULL_UP);
    exti_init(ENC_L_A_PIN, EXTI_TRIGGER_BOTH, enc_l_a_callback, NULL);

    /* B 相 PA29: 输入 + 上拉 + EXTI 双边沿 */
    gpio_init(ENC_L_B_PIN, GPI, GPIO_LOW, GPI_PULL_UP);
    exti_init(ENC_L_B_PIN, EXTI_TRIGGER_BOTH, enc_l_b_callback, NULL);

    /* ---- 右编码器 ---- */
    /* A 相 PB14: 输入 + 上拉 + EXTI 双边沿 */
    gpio_init(ENC_R_A_PIN, GPI, GPIO_LOW, GPI_PULL_UP);
    exti_init(ENC_R_A_PIN, EXTI_TRIGGER_BOTH, enc_r_a_callback, NULL);

    /* B 相 PB18: 输入 + 上拉 + EXTI 双边沿 */
    gpio_init(ENC_R_B_PIN, GPI, GPIO_LOW, GPI_PULL_UP);
    exti_init(ENC_R_B_PIN, EXTI_TRIGGER_BOTH, enc_r_b_callback, NULL);
}

/* ================================================================== */
/*  方向 GPIO 初始化                                                   */
/* ================================================================== */
static void direction_gpio_init(void)
{
    gpio_init(AIN1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(AIN2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(BIN1_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(BIN2_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

/* ================================================================== */
/*  PWM 初始化                                                         */
/* ================================================================== */
static void pwm_init_all(void)
{
    /* 左轮: PA21 = TIMA0 CH0, 20kHz, 初始占空比 0% */
    pwm_init(PWM_L_CH, PWM_FREQ_HZ, 0);
    /* 右轮: PA07 = TIMA0 CH1, 20kHz, 初始占空比 0% */
    pwm_init(PWM_R_CH, PWM_FREQ_HZ, 0);
}

/* ================================================================== */
/*  电机速度控制                                                       */
/* ================================================================== */
/*
 * PWM 占空比缩放因子。
 * 逐飞库 pwm_set_duty 范围 0~10000 = 0%~100%。
 * 原始 STM32 代码用 0~99 范围(Period=100), 乘以 100 映射到 0~10000。
 * 例: speed=14 → duty=1400 → 14% 占空比。
 */
#define PWM_DUTY_SCALE          100

static void motor_speed_left(int16_t speed)
{
    int16_t abs_speed = (speed >= 0) ? speed : (int16_t)(-speed);
    uint32_t duty = (uint32_t)(abs_speed * PWM_DUTY_SCALE);
    if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;

    if (speed >= 0) {
        gpio_low(AIN1_PIN);
        gpio_high(AIN2_PIN);
    } else {
        gpio_high(AIN1_PIN);
        gpio_low(AIN2_PIN);
    }
    pwm_set_duty(PWM_L_CH, duty);
}

static void motor_speed_right(int16_t speed)
{
    int16_t abs_speed = (speed >= 0) ? speed : (int16_t)(-speed);
    uint32_t duty = (uint32_t)(abs_speed * PWM_DUTY_SCALE);
    if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;

    if (speed >= 0) {
        gpio_high(BIN1_PIN);
        gpio_low(BIN2_PIN);
    } else {
        gpio_low(BIN1_PIN);
        gpio_high(BIN2_PIN);
    }
    pwm_set_duty(PWM_R_CH, duty);
}

/* ================================================================== */
/*  公开 API                                                           */
/* ================================================================== */
void motor_Init(void)
{
    direction_gpio_init();
    pwm_init_all();
    encoder_init();
}

void motor_start(int16_t left_pwm, int16_t right_pwm)
{
    motor_speed_left(left_pwm);
    motor_speed_right(right_pwm);
}

int16_t encoder_get_left(void)
{
    return (int16_t)s_enc_l_count;
}

int16_t encoder_get_right(void)
{
    return -(int16_t)s_enc_r_count;  /* 右编码器取反 (机械对称) */
}

void encoder_clear_both(void)
{
    s_enc_l_count = 0;
    s_enc_r_count = 0;
}
