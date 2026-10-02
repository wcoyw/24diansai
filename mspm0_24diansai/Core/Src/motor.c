/**
 * @file    motor.c
 * @brief   Motor control — TIMA0 PWM + direction GPIO
 *
 * STM32 → MSPM0 mapping:
 *   HAL_TIM_PWM_Start()      → DL_Timer_initPWMMode() + DL_Timer_startCounter()
 *   __HAL_TIM_SET_COMPARE()  → DL_Timer_setCaptureCompareValue()
 *   HAL_GPIO_WritePin()      → DL_GPIO_setPins() / DL_GPIO_clearPins()
 *
 * Motor driver: 2-channel H-bridge with IN1/IN2 (left), IN3/IN4 (right).
 * Direction is set by GPIO; speed is set by PWM duty cycle.
 */
#include "main.h"
#include "motor.h"

/* ---- Internal helpers ---- */

static void pwm_set_left(uint16_t duty)
{
    DL_Timer_setCaptureCompareValue(TIMA0, duty, PWM_L_CH);
}

static void pwm_set_right(uint16_t duty)
{
    DL_Timer_setCaptureCompareValue(TIMA0, duty, PWM_R_CH);
}

static void motor_speed1(int16_t speed)
{
    if (speed >= 0) {
        /* Left motor forward */
        DL_GPIO_clearPins(IN1_PORT, IN1_PIN);
        DL_GPIO_setPins(IN2_PORT, IN2_PIN);
        pwm_set_left((uint16_t)speed);
    } else {
        /* Left motor reverse */
        DL_GPIO_setPins(IN1_PORT, IN1_PIN);
        DL_GPIO_clearPins(IN2_PORT, IN2_PIN);
        pwm_set_left((uint16_t)(-speed));
    }
}

static void motor_speed2(int16_t speed)
{
    if (speed >= 0) {
        /* Right motor forward */
        DL_GPIO_setPins(IN3_PORT, IN3_PIN);
        DL_GPIO_clearPins(IN4_PORT, IN4_PIN);
        pwm_set_right((uint16_t)speed);
    } else {
        /* Right motor reverse */
        DL_GPIO_clearPins(IN3_PORT, IN3_PIN);
        DL_GPIO_setPins(IN4_PORT, IN4_PIN);
        pwm_set_right((uint16_t)(-speed));
    }
}

/* ---- Direction GPIO init ---- */
static void direction_gpio_init(void)
{
    DL_GPIO_initDigitalOutput(IN1_IOMUX);
    DL_GPIO_initDigitalOutput(IN2_IOMUX);
    DL_GPIO_initDigitalOutput(IN3_IOMUX);
    DL_GPIO_initDigitalOutput(IN4_IOMUX);

    /* Default: all direction pins low */
    DL_GPIO_clearPins(IN1_PORT, IN1_PIN);
    DL_GPIO_clearPins(IN2_PORT, IN2_PIN);
    DL_GPIO_clearPins(IN3_PORT, IN3_PIN);
    DL_GPIO_clearPins(IN4_PORT, IN4_PIN);
}

/* ---- TIMA0 PWM init (2 channels, edge-aligned, 10 kHz) ---- */
void Motor_PWM_Init(void)
{
    /* Configure PWM channel 0 (left motor) */
    DL_Timer_initPWMMode(TIMA0, PWM_L_CH, DL_TIMER_PWM_MODE_EDGE_ALIGN);
    DL_Timer_setPeriod(TIMA0, PWM_PERIOD_TICKS);
    DL_Timer_setCaptureCompareValue(TIMA0, 0, PWM_L_CH);

    /* Configure PWM channel 1/2 (right motor) */
    DL_Timer_initPWMMode(TIMA0, PWM_R_CH, DL_TIMER_PWM_MODE_EDGE_ALIGN);
    DL_Timer_setCaptureCompareValue(TIMA0, 0, PWM_R_CH);

    DL_Timer_startCounter(TIMA0);
}

/* ---- Encoder QEI init (TIMG0 = left, TIMG1 = right) ---- */
void Encoder_QEI_Init(void)
{
    /*
     * TIMG QEI mode: Phase A + Phase B quadrature decoding, no index.
     * Period = 65535 (16-bit), works identically to STM32 TIM3/TIM4 setup.
     */
    /* Left encoder — TIMG0 */
    DL_Timer_initQEI(ENC_L_TIMG, &(DL_Timer_QEIConfig){
        .period       = ENC_PERIOD,
        .captureMode  = DL_TIMER_QEI_CAPTURE_MODE_QUADRATURE,
        .xcrMode      = DL_TIMER_QEI_XCR_MODE_NO_XCR,
        .encoderMode  = DL_TIMER_QEI_ENCODER_MODE_QUADRATURE,
        .indexPolarity = DL_TIMER_QEI_INDEX_POLARITY_RISING,
        .swapPolarity  = DL_TIMER_QEI_SWAP_POLARITY_NON_SWAP,
    });
    DL_Timer_setQEIChannelControl(ENC_L_TIMG,
        DL_TIMER_QEI_XOR_DISABLE,
        DL_TIMER_QEI_INDEX_DISABLE,
        DL_TIMER_QEI_MODE_QUADRATURE);
    DL_Timer_startCounter(ENC_L_TIMG);

    /* Right encoder — TIMG1 */
    DL_Timer_initQEI(ENC_R_TIMG, &(DL_Timer_QEIConfig){
        .period       = ENC_PERIOD,
        .captureMode  = DL_TIMER_QEI_CAPTURE_MODE_QUADRATURE,
        .xcrMode      = DL_TIMER_QEI_XCR_MODE_NO_XCR,
        .encoderMode  = DL_TIMER_QEI_ENCODER_MODE_QUADRATURE,
        .indexPolarity = DL_TIMER_QEI_INDEX_POLARITY_RISING,
        .swapPolarity  = DL_TIMER_QEI_SWAP_POLARITY_NON_SWAP,
    });
    DL_Timer_setQEIChannelControl(ENC_R_TIMG,
        DL_TIMER_QEI_XOR_DISABLE,
        DL_TIMER_QEI_INDEX_DISABLE,
        DL_TIMER_QEI_MODE_QUADRATURE);
    DL_Timer_startCounter(ENC_R_TIMG);
}

/* ---- Public API ---- */

void motor_Init(void)
{
    direction_gpio_init();
    Motor_PWM_Init();
    Encoder_QEI_Init();
}

/**
 * @brief  Set left + right motor speed.
 * @param  left_pwm   PWM duty for left motor  (-9999 … +9999, sign = direction)
 * @param  right_pwm  PWM duty for right motor (-9999 … +9999, sign = direction)
 */
void motor_start(int16_t left_pwm, int16_t right_pwm)
{
    motor_speed1(left_pwm);
    motor_speed2(right_pwm);
}
