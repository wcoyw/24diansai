/**
 * @file    main.c
 * @brief   MSPM0G3507 24diansai — main state machine
 *
 * Ported from STM32F103C8 (STM32 HAL) to MSPM0G3507 (TI DriverLib).
 *
 * Key HAL → DL mappings:
 *   HAL_Init()                           → SystemClock_Config()
 *   HAL_TIM_Base_Start_IT()              → DL_Timer_startCounter() + enableInterrupt
 *   HAL_TIM_PeriodElapsedCallback()      → TIMG2_IRQHandler (manual ISR)
 *   HAL_UART_Receive_IT()                → UART1_IRQHandler (direct byte read)
 *   __HAL_TIM_GET_COUNTER()              → DL_Timer_getQEIValue()
 *   __HAL_TIM_SET_COUNTER(x, 0)          → DL_Timer_resetQEICounter()
 *   HAL_GPIO_WritePin()                  → DL_GPIO_setPins() / clearPins()
 *   __disable_irq() / __enable_irq()     → same (CMSIS)
 *   HAL_Delay()                          → removed (non-blocking everywhere)
 *   fputc (USART2→DR)                    → DL_UART_transmitDataBlocking(UART0, ch)
 */

#include "main.h"
#include "pid.h"
#include "motor.h"
#include "jy61p.h"
#include "track.h"
#include "button.h"
#include "buzzer.h"
#include "led.h"
#include <stdio.h>

/* ================================================================== */
/*  Debug print control (unchanged)                                    */
/* ================================================================== */
#define CONTROL_DEBUG_ENABLE         0

#if CONTROL_DEBUG_ENABLE
#define CONTROL_DEBUG_PRINTF(...)    printf(__VA_ARGS__)
#else
#define CONTROL_DEBUG_PRINTF(...)    ((void)0)
#endif

/* ================================================================== */
/*  Global variables (unchanged from STM32 version)                    */
/* ================================================================== */
static int32_t current_encoder1 = 0;
static int32_t current_encoder2 = 0;
static int32_t location = 0;

static PID_t Inner = {
    .Target = 15.0f, .Kp = 2.0f, .Ki = 0.05f, .Kd = 0.0f,
    .OutMax = 30.0f, .OutMin = -30.0f, .IntMax = 80.0f, .IntMin = -80.0f,
    .ErrHighThresh = 30.0f, .ErrLowThresh = 5.0f,
};
static PID_t Inner1 = {
    .Target = 15.0f, .Kp = 2.0f, .Ki = 0.05f, .Kd = 0.0f,
    .OutMax = 30.0f, .OutMin = -30.0f, .IntMax = 80.0f, .IntMin = -80.0f,
    .ErrHighThresh = 30.0f, .ErrLowThresh = 5.0f,
};

/* ================================================================== */
/*  Task / State enums (unchanged)                                     */
/* ================================================================== */
typedef enum {
    TASK_NONE = 0,
    TASK_Q1,
    TASK_Q2,
    TASK_Q3,
    TASK_Q4,
} TaskMode_t;

typedef enum {
    ST_TURN_TO_33 = 0,
    ST_LINE_33,
    ST_TRACK,
    ST_TURN_TO_147,
    ST_LINE_147,
    ST_TRACK2,
    ST_DONE,
} State_t;

typedef enum {
    Q2_LINE_0 = 0,
    Q2_TRACK_1,
    Q2_LINE_180,
    Q2_TRACK_2,
} Q2State_t;

/* ================================================================== */
/*  State variables                                                     */
/* ================================================================== */
static volatile TaskMode_t s_task = TASK_NONE;
static State_t    s_state      = ST_DONE;
static Q2State_t  s_q2_state   = Q2_LINE_0;

static uint8_t  s_no_line_cnt      = 0;
static uint8_t  s_ignore_line_cnt  = 0;
static uint8_t  s_line_cnt         = 0;
static float    s_yaw_corr         = 0.0f;
static uint8_t  s_lap              = 0;
static uint8_t  s_target_laps      = 4;
static uint8_t  s_turn_stable_cnt  = 0;
static uint8_t  s_drive_boost_ticks = 0;
static float    s_turn33_target    = 0.0f;
static float    s_turn147_target   = 0.0f;
static int32_t  s_segment_ticks    = 0;
static volatile uint16_t s_alert_ticks = 0;
static uint8_t  s_task_start_pending = 0;   /* button result from ISR */
static volatile uint8_t s_imu_rx_byte;       /* JY61P rx buffer        */

/* ================================================================== */
/*  Parameters (unchanged)                                             */
/* ================================================================== */
#define NO_LINE_EXIT_CNT             4
#define LINE_IGNORE_CNT              1
#define LINE_CONFIRM_CNT             1
#define Q2_LINE180_MIN_TICKS         180
#define Q2_FINISH_NO_LINE_CNT        10
#define TRACK2_FINISH_MIN_TICKS      80

#define YAW33_MIN                    -60.0f
#define YAW33_MAX                    -20.0f
#define YAW33_TARGET                 -33.7f
#define YAW147_MIN                   -160.0f
#define YAW147_MAX                   -120.0f
#define YAW147_TARGET                -147.0f
#define YAW_TURN_TOLERANCE           3.0f
#define YAW_TURN_KP                  0.35f
#define YAW_TURN_MIN_PWM             10.0f
#define YAW_TURN_MAX_PWM             16.0f
#define YAW_TURN_STABLE_CNT          2
#define YAW_DEADBAND                 2.0f
#define YAW_CORR_KP                  0.25f
#define YAW_CORR_STEP                1.5f
#define DRIVE_START_BOOST_TICKS      4
#define DRIVE_START_MIN_PWM          16
#define TURN_TARGET_COMP_LIMIT       10.0f
#define YAW_OFFSET                   2.2f
#define MOTOR_TRIM                   0.0f

#define ALERT_PASS_TICKS             4
#define ALERT_STOP_TICKS             15

/* ================================================================== */
/*  Math helpers (unchanged)                                           */
/* ================================================================== */
static int32_t abs_i32(int32_t x) { return (x < 0) ? -x : x; }

static float limit_float(float x, float min, float max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

static float normalize_angle(float angle)
{
    if (angle >  180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float approach_float(float now, float target, float step)
{
    if (now + step < target) return now + step;
    if (now - step > target) return now - step;
    return target;
}

static float wrap_angle_err(float target, float yaw)
{
    float err = target - (yaw + YAW_OFFSET);
    return normalize_angle(err);
}

static float build_turn33_target_from_A(float yaw)
{
    float corrected_yaw = normalize_angle(yaw + YAW_OFFSET);
    float heading_err   = corrected_yaw;
    float comp = limit_float(heading_err, -TURN_TARGET_COMP_LIMIT, TURN_TARGET_COMP_LIMIT);
    return normalize_angle(YAW33_TARGET + 0.11f * comp);
}

static float build_turn147_target_from_B(float yaw)
{
    float corrected_yaw = normalize_angle(yaw + YAW_OFFSET);
    float heading_err   = (corrected_yaw >= 0.0f)
                        ? (corrected_yaw - 180.0f)
                        : (corrected_yaw + 180.0f);
    float comp = limit_float(heading_err, -TURN_TARGET_COMP_LIMIT, TURN_TARGET_COMP_LIMIT);
    return normalize_angle(YAW147_TARGET - 0.15f * comp);
}

/* ================================================================== */
/*  Motor / PID helpers (unchanged except encoder read)                */
/* ================================================================== */
static void motor_stop(void) { motor_start(0, 0); }

static void reset_drive_pid(void)
{
    Inner.Error0 = Inner.Error1 = Inner.ErrorInt = Inner.Out = 0.0f;
    Inner1.Error0 = Inner1.Error1 = Inner1.ErrorInt = Inner1.Out = 0.0f;
}

static void reset_line_state(uint8_t ignore_line)
{
    s_no_line_cnt    = 0;
    s_line_cnt       = 0;
    s_ignore_line_cnt = ignore_line;
}

static uint8_t line_confirmed(void)
{
    if (Track_AnyLine()) {
        if (s_line_cnt < LINE_CONFIRM_CNT) s_line_cnt++;
    } else {
        s_line_cnt = 0;
    }
    return (s_line_cnt >= LINE_CONFIRM_CNT) ? 1 : 0;
}

static void update_no_line_counter(void)
{
    if (Track_AnyLine()) s_no_line_cnt = 0;
    else if (s_no_line_cnt < 200) s_no_line_cnt++;
}

static void trigger_alert(uint16_t ticks)
{
    s_alert_ticks = ticks;
    LED_ON();
    Buzzer_ON();
}

/* ================================================================== */
/*  Steering & drive (unchanged logic)                                 */
/* ================================================================== */
static void act_turn_closed_loop(float yaw, float target)
{
    float turn_pwm = wrap_angle_err(target, yaw) * YAW_TURN_KP;
    turn_pwm = limit_float(turn_pwm, -YAW_TURN_MAX_PWM, YAW_TURN_MAX_PWM);

    if (turn_pwm > 0.0f && turn_pwm < YAW_TURN_MIN_PWM)
        turn_pwm = YAW_TURN_MIN_PWM;
    else if (turn_pwm < 0.0f && turn_pwm > -YAW_TURN_MIN_PWM)
        turn_pwm = -YAW_TURN_MIN_PWM;

    motor_start((int16_t)(-turn_pwm), (int16_t)turn_pwm);
}

static void act_drive_yaw(float yaw, float target, float speed, float corr_max)
{
    float yaw_err = (yaw + YAW_OFFSET) - target;
    if (yaw_err >  180.0f) yaw_err -= 360.0f;
    if (yaw_err < -180.0f) yaw_err += 360.0f;

    float yaw_corr_target = 0.0f;
    if (yaw_err > YAW_DEADBAND || yaw_err < -YAW_DEADBAND)
        yaw_corr_target = limit_float(yaw_err * YAW_CORR_KP, -corr_max, corr_max);

    s_yaw_corr = approach_float(s_yaw_corr, yaw_corr_target, YAW_CORR_STEP);

    Inner.Target  = speed;
    Inner1.Target = speed;
    Inner.Actual  = (float)current_encoder1;
    Inner1.Actual = (float)current_encoder2;
    PID_Update(&Inner);
    PID_Update(&Inner1);

    int16_t left_pwm  = (int16_t)(Inner.Out  + s_yaw_corr + MOTOR_TRIM);
    int16_t right_pwm = (int16_t)(Inner1.Out - s_yaw_corr - MOTOR_TRIM);

    if (s_drive_boost_ticks > 0) {
        if (left_pwm  < DRIVE_START_MIN_PWM) left_pwm  = DRIVE_START_MIN_PWM;
        if (right_pwm < DRIVE_START_MIN_PWM) right_pwm = DRIVE_START_MIN_PWM;
        s_drive_boost_ticks--;
    }

    motor_start(left_pwm, right_pwm);
}

/* ================================================================== */
/*  Yaw check helpers (unchanged)                                      */
/* ================================================================== */
static uint8_t yaw_reached_target(float yaw, float target)
{
    float yaw_err = wrap_angle_err(target, yaw);
    if (yaw_err < YAW_TURN_TOLERANCE && yaw_err > -YAW_TURN_TOLERANCE) {
        if (s_turn_stable_cnt < YAW_TURN_STABLE_CNT) s_turn_stable_cnt++;
    } else {
        s_turn_stable_cnt = 0;
    }
    return (s_turn_stable_cnt >= YAW_TURN_STABLE_CNT) ? 1 : 0;
}

static uint8_t yaw_out_of_window(float yaw, float target, float tolerance)
{
    float yaw_err = wrap_angle_err(target, yaw);
    return (yaw_err > tolerance || yaw_err < -tolerance) ? 1 : 0;
}

/* ================================================================== */
/*  State transitions (unchanged)                                      */
/* ================================================================== */
static void finish_task(void)
{
    s_task  = TASK_NONE;
    s_state = ST_DONE;
    motor_stop();
    reset_drive_pid();
    s_yaw_corr = 0.0f;
    trigger_alert(ALERT_STOP_TICKS);
}

static void enter_line_33(void)
{
    s_state = ST_LINE_33;
    reset_line_state(LINE_IGNORE_CNT);
    s_yaw_corr          = 0.0f;
    s_turn_stable_cnt   = 0;
    s_drive_boost_ticks = DRIVE_START_BOOST_TICKS;
    reset_drive_pid();
}

static void enter_turn_33(float yaw)
{
    s_state = ST_TURN_TO_33;
    reset_line_state(0);
    s_yaw_corr        = 0.0f;
    s_turn_stable_cnt = 0;
    s_turn33_target   = build_turn33_target_from_A(yaw);
    reset_drive_pid();
}

static void enter_turn_147(float yaw)
{
    s_state = ST_TURN_TO_147;
    reset_line_state(0);
    s_yaw_corr        = 0.0f;
    s_turn_stable_cnt = 0;
    s_turn147_target  = build_turn147_target_from_B(yaw);
    reset_drive_pid();
}

static void enter_line_147(void)
{
    s_state = ST_LINE_147;
    reset_line_state(LINE_IGNORE_CNT);
    s_yaw_corr          = 0.0f;
    s_turn_stable_cnt   = 0;
    s_drive_boost_ticks = DRIVE_START_BOOST_TICKS;
    reset_drive_pid();
}

/* ================================================================== */
/*  Q3 / Q4 state machine (unchanged)                                  */
/* ================================================================== */
static void path34_step(float yaw)
{
    switch (s_state)
    {
    case ST_TURN_TO_33:
        if (yaw_reached_target(yaw, s_turn33_target)) enter_line_33();
        break;

    case ST_LINE_33:
        if (s_ignore_line_cnt > 0) { s_ignore_line_cnt--; break; }
        if (line_confirmed()) {
            s_state = ST_TRACK;
            trigger_alert(ALERT_PASS_TICKS);
            reset_line_state(0);
            break;
        }
        if (yaw_out_of_window(yaw, s_turn33_target, 27.0f)) {
            enter_turn_33(yaw);
            s_line_cnt = 0;
        }
        break;

    case ST_TRACK:
        update_no_line_counter();
        if (s_no_line_cnt >= NO_LINE_EXIT_CNT &&
            (yaw > 130.0f || yaw < -150.0f)) {
            trigger_alert(ALERT_PASS_TICKS);
            enter_turn_147(yaw);
        }
        break;

    case ST_TURN_TO_147:
        if (yaw_reached_target(yaw, s_turn147_target)) enter_line_147();
        break;

    case ST_LINE_147:
        if (s_ignore_line_cnt > 0) { s_ignore_line_cnt--; break; }
        if (line_confirmed()) {
            s_state = ST_TRACK2;
            trigger_alert(ALERT_PASS_TICKS);
            reset_line_state(0);
            s_segment_ticks = 0;
            break;
        }
        if (yaw_out_of_window(yaw, s_turn147_target, 21.0f))
            enter_turn_147(yaw);
        break;

    case ST_TRACK2:
        update_no_line_counter();
        if (s_segment_ticks >= TRACK2_FINISH_MIN_TICKS &&
            s_no_line_cnt >= NO_LINE_EXIT_CNT) {
            s_lap++;
            trigger_alert(ALERT_PASS_TICKS);
            reset_line_state(0);
            if (s_lap >= s_target_laps)
                finish_task();
            else
                enter_turn_33(yaw);
        }
        break;

    case ST_DONE:
        finish_task();
        break;
    }
}

static void path34_run(float yaw)
{
    switch (s_state)
    {
    case ST_TURN_TO_33:
        act_turn_closed_loop(yaw, s_turn33_target);
        CONTROL_DEBUG_PRINTF("Q%d TURN1 Yaw:%.2f\r\n",
            (s_task == TASK_Q4) ? 4 : 3, (double)yaw);
        break;
    case ST_LINE_33:
        act_drive_yaw(yaw, s_turn33_target, 29.0f, 8.0f);
        CONTROL_DEBUG_PRINTF("Q%d STR1 Yaw:%.2f\r\n",
            (s_task == TASK_Q4) ? 4 : 3, (double)yaw);
        break;
    case ST_TRACK:
        Track_move();
        CONTROL_DEBUG_PRINTF("Q%d TRACK Yaw:%.2f\r\n",
            (s_task == TASK_Q4) ? 4 : 3, (double)yaw);
        break;
    case ST_TURN_TO_147:
        act_turn_closed_loop(yaw, s_turn147_target);
        CONTROL_DEBUG_PRINTF("Q%d TURN2 Yaw:%.2f\r\n",
            (s_task == TASK_Q4) ? 4 : 3, (double)yaw);
        break;
    case ST_LINE_147:
        act_drive_yaw(yaw, s_turn147_target, 22.0f, 5.0f);
        CONTROL_DEBUG_PRINTF("Q%d STR2 Yaw:%.2f\r\n",
            (s_task == TASK_Q4) ? 4 : 3, (double)yaw);
        break;
    case ST_TRACK2:
        Track_move();
        CONTROL_DEBUG_PRINTF("Q%d TRACK2 Lap:%d Yaw:%.2f\r\n",
            (s_task == TASK_Q4) ? 4 : 3, (int)s_lap, (double)yaw);
        break;
    case ST_DONE:
        motor_stop();
        break;
    }
}

/* ================================================================== */
/*  Q1 state machine (unchanged)                                       */
/* ================================================================== */
static void task1_step(float yaw)
{
    act_drive_yaw(yaw, 0.0f, 29.0f, 8.0f);
    CONTROL_DEBUG_PRINTF("Q1 STR0 Yaw:%.2f\r\n", (double)yaw);

    if (s_ignore_line_cnt > 0) { s_ignore_line_cnt--; s_line_cnt = 0; return; }
    if (line_confirmed()) {
        trigger_alert(ALERT_PASS_TICKS);
        finish_task();
    }
}

/* ================================================================== */
/*  Q2 state machine (unchanged)                                       */
/* ================================================================== */
static void enter_task2_state(Q2State_t next, uint8_t ignore_line)
{
    s_q2_state         = next;
    reset_line_state(ignore_line);
    s_drive_boost_ticks = 0;
    s_segment_ticks     = 0;
    s_yaw_corr          = 0.0f;
    reset_drive_pid();
}

static void task2_step(float yaw)
{
    switch (s_q2_state)
    {
    case Q2_LINE_0:
        act_drive_yaw(yaw, 0.0f, 29.0f, 8.0f);
        CONTROL_DEBUG_PRINTF("Q2 LINE0 Yaw:%.2f\r\n", (double)yaw);
        if (s_ignore_line_cnt > 0) { s_ignore_line_cnt--; s_line_cnt = 0; break; }
        if (line_confirmed()) {
            trigger_alert(ALERT_PASS_TICKS);
            enter_task2_state(Q2_TRACK_1, 0);
        }
        break;

    case Q2_TRACK_1:
        Track_move();
        update_no_line_counter();
        CONTROL_DEBUG_PRINTF("Q2 TRACK1 Yaw:%.2f\r\n", (double)yaw);
        if (s_no_line_cnt >= NO_LINE_EXIT_CNT &&
            (yaw > 130.0f || yaw < -150.0f)) {
            trigger_alert(ALERT_PASS_TICKS);
            enter_task2_state(Q2_LINE_180, LINE_IGNORE_CNT);
        }
        break;

    case Q2_LINE_180:
    {
        float target = (yaw >= 0.0f) ? 180.0f : -180.0f;
        act_drive_yaw(yaw, target, 29.0f, 8.0f);
        CONTROL_DEBUG_PRINTF("Q2 LINE180 Yaw:%.2f\r\n", (double)yaw);
        if (s_ignore_line_cnt > 0) { s_ignore_line_cnt--; s_line_cnt = 0; break; }
        if (s_segment_ticks >= Q2_LINE180_MIN_TICKS && line_confirmed()) {
            trigger_alert(ALERT_PASS_TICKS);
            enter_task2_state(Q2_TRACK_2, 0);
        }
        break;
    }

    case Q2_TRACK_2:
        Track_move();
        update_no_line_counter();
        CONTROL_DEBUG_PRINTF("Q2 TRACK2 Yaw:%.2f\r\n", (double)yaw);
        if (s_no_line_cnt >= Q2_FINISH_NO_LINE_CNT) {
            trigger_alert(ALERT_PASS_TICKS);
            finish_task();
        }
        break;
    }
}

/* ================================================================== */
/*  Task management (unchanged except button + encoder reset)          */
/* ================================================================== */
static void reset_task_common(void)
{
    s_state      = ST_DONE;
    s_q2_state   = Q2_LINE_0;
    reset_line_state(0);
    s_yaw_corr        = 0.0f;
    s_lap             = 0;
    s_target_laps     = 4;
    s_turn33_target   = YAW33_TARGET;
    s_turn147_target  = YAW147_TARGET;
    s_turn_stable_cnt = 0;
    s_segment_ticks   = 0;
    location          = 0;
    current_encoder1  = 0;
    current_encoder2  = 0;
    reset_drive_pid();

    /* MSPM0 QEI reset — if supported, else rely on delta method */
    DL_Timer_resetQEICounter(ENC_L_TIMG);
    DL_Timer_resetQEICounter(ENC_R_TIMG);

    motor_stop();
}

static void start_task_unlocked(TaskMode_t task)
{
    reset_task_common();

    switch (task)
    {
    case TASK_Q1:
        reset_line_state(LINE_IGNORE_CNT);
        s_task = TASK_Q1;
        trigger_alert(ALERT_PASS_TICKS);
        break;

    case TASK_Q2:
        s_q2_state = Q2_LINE_0;
        reset_line_state(LINE_IGNORE_CNT);
        s_task = TASK_Q2;
        trigger_alert(ALERT_PASS_TICKS);
        break;

    case TASK_Q3:
        s_state       = ST_TURN_TO_33;
        s_target_laps = 1;
        s_task        = TASK_Q3;
        trigger_alert(ALERT_PASS_TICKS);
        break;

    case TASK_Q4:
        s_state       = ST_TURN_TO_33;
        s_target_laps = 4;
        s_task        = TASK_Q4;
        trigger_alert(ALERT_PASS_TICKS);
        break;

    default:
        s_task = TASK_NONE;
        break;
    }
}

static void start_task(TaskMode_t task)
{
    __disable_irq();
    start_task_unlocked(task);
    __enable_irq();
}

/* ================================================================== */
/*  UART init helpers                                                  */
/* ================================================================== */

static void Debug_UART_Init(void)
{
    /* UART0: 115200, 8N1 — printf via CH340E USB-serial */
    DL_UART_ClockConfig clockCfg = {
        .clockSel    = DL_UART_CLOCK_BUSCLK,
        .divideRatio = DL_UART_CLOCK_DIVIDE_RATIO_1,
    };
    DL_UART_MainConfig mainCfg = {
        .mode        = DL_UART_MAIN_MODE_NORMAL,
        .direction   = DL_UART_DIRECTION_TX_RX,
        .flowControl = DL_UART_FLOW_CONTROL_NONE,
        .parity      = DL_UART_PARITY_NONE,
        .wordLength  = DL_UART_WORD_LENGTH_8_BITS,
        .stopBits    = DL_UART_STOP_BITS_1,
    };
    DL_UART_ExtendedConfig extCfg = {
        .oversampling = DL_UART_OVERSAMPLING_RATE_16,
        .lsbFirst     = DL_UART_LSB_FIRST_ENABLE,
        .rxFifoDepth  = DL_UART_RX_FIFO_DEPTH_1_4,
        .txFifoDepth  = DL_UART_TX_FIFO_DEPTH_1_4,
    };

    /* Pin mux */
    DL_GPIO_initPeripheralOutput(DEBUG_TX_IOMUX);
    DL_GPIO_initPeripheralInput(DEBUG_RX_IOMUX);

    DL_UART_init(DEBUG_UART, &clockCfg);
    DL_UART_initMain(DEBUG_UART, &mainCfg);
    DL_UART_initExtended(DEBUG_UART, &extCfg);

    /* Baud rate: 80 MHz / (16 × BR) = 115200 → BR ≈ 43.4 */
    DL_UART_setOversampling(DEBUG_UART, DL_UART_OVERSAMPLING_RATE_16);
    DL_UART_setBaudRateDivisor(DEBUG_UART,
        (uint32_t)(SYSTEM_CLOCK_HZ / (16UL * 115200UL)),
        (uint32_t)(SYSTEM_CLOCK_HZ % (16UL * 115200UL)));

    DL_UART_enable(DEBUG_UART);
}

static void IMU_UART_Init(void)
{
    /* UART1: 115200, 8N1 — JY61P IMU with RX interrupt */
    DL_UART_ClockConfig clockCfg = {
        .clockSel    = DL_UART_CLOCK_BUSCLK,
        .divideRatio = DL_UART_CLOCK_DIVIDE_RATIO_1,
    };
    DL_UART_MainConfig mainCfg = {
        .mode        = DL_UART_MAIN_MODE_NORMAL,
        .direction   = DL_UART_DIRECTION_TX_RX,
        .flowControl = DL_UART_FLOW_CONTROL_NONE,
        .parity      = DL_UART_PARITY_NONE,
        .wordLength  = DL_UART_WORD_LENGTH_8_BITS,
        .stopBits    = DL_UART_STOP_BITS_1,
    };
    DL_UART_ExtendedConfig extCfg = {
        .oversampling = DL_UART_OVERSAMPLING_RATE_16,
        .lsbFirst     = DL_UART_LSB_FIRST_ENABLE,
        .rxFifoDepth  = DL_UART_RX_FIFO_DEPTH_1_4,
        .txFifoDepth  = DL_UART_TX_FIFO_DEPTH_1_4,
    };

    DL_GPIO_initPeripheralOutput(IMU_TX_IOMUX);
    DL_GPIO_initPeripheralInput(IMU_RX_IOMUX);

    DL_UART_init(IMU_UART, &clockCfg);
    DL_UART_initMain(IMU_UART, &mainCfg);
    DL_UART_initExtended(IMU_UART, &extCfg);

    DL_UART_setBaudRateDivisor(IMU_UART,
        (uint32_t)(SYSTEM_CLOCK_HZ / (16UL * 115200UL)),
        (uint32_t)(SYSTEM_CLOCK_HZ % (16UL * 115200UL)));

    /* Enable RX interrupt (no TX interrupt needed) */
    DL_UART_enableInterrupt(IMU_UART, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(IMU_UART_INT);

    DL_UART_enable(IMU_UART);
}

/* ================================================================== */
/*  Control timer init (TIMG2, 25 Hz)                                  */
/* ================================================================== */
void Control_Timer_Init(void)
{
    DL_Timer_initBasicMode(CONTROL_TIMG,
        DL_TIMER_CLOCK_BUS, CONTROL_PRESCALER, DL_TIMER_COUNT_DIR_UP);
    DL_Timer_setPeriod(CONTROL_TIMG, CONTROL_PERIOD);
    DL_Timer_enableInterrupt(CONTROL_TIMG, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_EnableIRQ(CONTROL_TIMG_INT);
    DL_Timer_startCounter(CONTROL_TIMG);
}

/* ================================================================== */
/*  fputc redirect for printf → UART0                                  */
/* ================================================================== */
int fputc(int ch, FILE *f)
{
    UNUSED(f);
    DL_UART_transmitDataBlocking(DEBUG_UART, (uint8_t)ch);
    return ch;
}

/* ================================================================== */
/*  MAIN                                                               */
/* ================================================================== */
int main(void)
{
    /* ---- System init ---- */
    SystemClock_Config();

    /* ---- GPIO init ---- */
    LED_Init();
    Buzzer_Init();

    /* ---- Motors & encoders ---- */
    motor_Init();

    /* ---- UART ---- */
    Debug_UART_Init();
    IMU_UART_Init();

    /* ---- Sensors ---- */
    Track_Init();
    Button_Init();
    JY61p_Init();

    /* ---- Control timer (starts 25 Hz ISR) ---- */
    Control_Timer_Init();

    /* ---- Welcome ---- */
    printf("MSPM0G3507 24diansai ready\r\n");

    motor_stop();

    while (1) {
        /*
         * Buttons are now scanned inside the 25 Hz ISR.
         * The result is posted via s_task_start_pending.
         * We process it here in the main loop (outside ISR).
         */
        if (s_task_start_pending != 0) {
            TaskMode_t pending = (TaskMode_t)s_task_start_pending;
            s_task_start_pending = 0;
            start_task(pending);
        }
    }
}

/* ================================================================== */
/*  Interrupt Service Routines                                         */
/* ================================================================== */

/**
 * @brief  TIMG2 ISR — 25 Hz control loop
 *
 * Replaces STM32 TIM5 HAL_TIM_PeriodElapsedCallback().
 */
void TIMG2_IRQHandler(void)
{
    /* Check which interrupt fired */
    uint32_t status = DL_Timer_getPendingInterrupt(CONTROL_TIMG);

    if (status & DL_TIMER_INTERRUPT_ZERO_EVENT) {
        DL_Timer_clearInterruptStatus(CONTROL_TIMG, DL_TIMER_INTERRUPT_ZERO_EVENT);

        /* ---- Read encoders ---- */
        /*
         * MSPM0 QEI read.  If DL_Timer_getQEIValue returns signed 16-bit
         * or the counter wraps at ENC_PERIOD, use delta method.
         */
        current_encoder1 =  (int16_t)DL_Timer_getQEIValue(ENC_L_TIMG);
        current_encoder2 = -(int16_t)DL_Timer_getQEIValue(ENC_R_TIMG);
        location += current_encoder1;
        s_segment_ticks += (abs_i32(current_encoder1) + abs_i32(current_encoder2)) / 2;

        /* Reset encoder counters — if hw reset not available, use delta method */
        DL_Timer_resetQEICounter(ENC_L_TIMG);
        DL_Timer_resetQEICounter(ENC_R_TIMG);

        /* ---- Read IMU ---- */
        float roll, pitch, yaw;
        JY61p_GetAngle(&roll, &pitch, &yaw);

        /* ---- Button scan ---- */
        uint8_t btn = Button_Scan();
        if (btn != BTN_NONE && s_task == TASK_NONE) {
            switch (btn) {
            case BTN_1: s_task_start_pending = (uint8_t)TASK_Q1; break;
            case BTN_2: s_task_start_pending = (uint8_t)TASK_Q2; break;
            case BTN_3: s_task_start_pending = (uint8_t)TASK_Q3; break;
            case BTN_4: s_task_start_pending = (uint8_t)TASK_Q4; break;
            }
        }

        /* ---- Task state machine ---- */
        switch (s_task)
        {
        case TASK_Q1:
            task1_step(yaw);
            break;

        case TASK_Q2:
            task2_step(yaw);
            break;

        case TASK_Q3:
        case TASK_Q4:
            path34_step(yaw);
            if (s_task != TASK_NONE)
                path34_run(yaw);
            break;

        case TASK_NONE:
        default:
            motor_stop();
            break;
        }

        /* ---- Alert timer (non-blocking LED + Buzzer) ---- */
        if (s_alert_ticks > 0) {
            s_alert_ticks--;
            if (s_alert_ticks == 0) {
                LED_OFF();
                Buzzer_OFF();
            }
        }
    }
}

/**
 * @brief  UART1 ISR — JY61P IMU byte receiver
 *
 * Replaces STM32 HAL_UART_RxCpltCallback().
 */
void UART1_IRQHandler(void)
{
    uint32_t status = DL_UART_getPendingInterrupt(IMU_UART);

    if (status & DL_UART_INTERRUPT_RX) {
        DL_UART_clearInterruptStatus(IMU_UART, DL_UART_INTERRUPT_RX);
        uint8_t byte = DL_UART_receiveData(IMU_UART);
        JY61p_ParseByte(byte);
		}
}

/* ================================================================== */
/*  Error handler                                                      */
/* ================================================================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Flash LED rapidly to signal error */
        LED_Toggle();
        for (volatile uint32_t d = 0; d < 1000000; d++) { }
    }
}

/* ================================================================== */
/*  Fault handlers for Cortex-M0+ (minimal)                            */
/* ================================================================== */
void NMI_Handler(void)           { while (1) { } }
void HardFault_Handler(void)     { while (1) { } }
void SVC_Handler(void)           { }
void PendSV_Handler(void)        { }
void SysTick_Handler(void)       { }
