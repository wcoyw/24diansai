/**
 * @file    main.c
 * @brief   CAR 主状态机 — MSPM0G3507 + 逐飞库 | 25Hz 控制循环
 *
 * 从 STM32F103C8 (HAL) 移植到 MSPM0G3507 (逐飞库)。
 *
 * 关键变化:
 *   JY61P (UART)    → ICM-42688-P (I2C)
 *   硬件 QEI (TIM3/4) → GPIO EXTI 编码器
 *   HAL_Delay 阻塞   → 全部非阻塞
 *   printf 重定向     → uart_write_string / uart_write_buffer
 */
#include "main.h"
#include "pid.h"
#include "motor.h"
#include "track.h"
#include "button.h"
#include "buzzer.h"
#include "led.h"
#include "icm42688.h"
#include "imu.h"
#include <math.h>

/* ================================================================== */
/*  调试打印控制                                                       */
/* ================================================================== */
#define CONTROL_DEBUG_ENABLE         0

#if CONTROL_DEBUG_ENABLE
#define CONTROL_DEBUG_PRINTF(...)    printf(__VA_ARGS__)
#else
#define CONTROL_DEBUG_PRINTF(...)    ((void)0)
#endif

/* ================================================================== */
/*  全局变量                                                           */
/* ================================================================== */
int32_t current_encoder1 = 0;
int32_t current_encoder2 = 0;
int32_t location = 0;

/* ================================================================== */
/*  PID 对象                                                           */
/* ================================================================== */
static PID_t Inner = {
    .Target = 15.0f, .Kp = 2.0f, .Ki = 0.05f, .Kd = 0.0f,
    .OutMax = 30.0f, .OutMin = -30.0f,
    .IntMax = 80.0f, .IntMin = -80.0f,
    .ErrHighThresh = 30.0f, .ErrLowThresh = 5.0f,
};
static PID_t Inner1 = {
    .Target = 15.0f, .Kp = 2.0f, .Ki = 0.05f, .Kd = 0.0f,
    .OutMax = 30.0f, .OutMin = -30.0f,
    .IntMax = 80.0f, .IntMin = -80.0f,
    .ErrHighThresh = 30.0f, .ErrLowThresh = 5.0f,
};

/* ================================================================== */
/*  任务/状态枚举                                                      */
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
/*  状态变量                                                           */
/* ================================================================== */
static volatile TaskMode_t s_task = TASK_NONE;
static State_t             s_state      = ST_DONE;
static Q2State_t           s_q2_state   = Q2_LINE_0;

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
static volatile uint8_t  s_task_start_pending = 0;

/* 当前 IMU 姿态 (在控制 ISR 中更新, ISR 独占, 无需 volatile) */
static float s_roll  = 0.0f;
static float s_pitch = 0.0f;
static float s_yaw   = 0.0f;

/* long 字符缓冲 (printf 替代) */
static char s_print_buf[128];

/* ================================================================== */
/*  参数常量                                                           */
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
/*  数学辅助                                                           */
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
/*  电机/PID 辅助                                                      */
/* ================================================================== */
static void motor_stop(void) { motor_start(0, 0); }

static void reset_drive_pid(void)
{
    Inner.Error0 = Inner.Error1 = Inner.ErrorInt = Inner.Out = 0.0f;
    Inner1.Error0 = Inner1.Error1 = Inner1.ErrorInt = Inner1.Out = 0.0f;
}

static void reset_line_state(uint8_t ignore_line)
{
    s_no_line_cnt     = 0;
    s_line_cnt        = 0;
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
    if (Track_AnyLine())
        s_no_line_cnt = 0;
    else if (s_no_line_cnt < 200)
        s_no_line_cnt++;
}

static void trigger_alert(uint16_t ticks)
{
    s_alert_ticks = ticks;
    LED_ON();
    Buzzer_ON();
}

/* ================================================================== */
/*  转向与直线驱动                                                     */
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
/*  Yaw 判定                                                           */
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
/*  状态转移                                                           */
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
/*  Q3 / Q4 状态机                                                     */
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
        break;
    case ST_LINE_33:
        act_drive_yaw(yaw, s_turn33_target, 29.0f, 8.0f);
        break;
    case ST_TRACK:
        Track_move();
        break;
    case ST_TURN_TO_147:
        act_turn_closed_loop(yaw, s_turn147_target);
        break;
    case ST_LINE_147:
        act_drive_yaw(yaw, s_turn147_target, 22.0f, 5.0f);
        break;
    case ST_TRACK2:
        Track_move();
        break;
    case ST_DONE:
        motor_stop();
        break;
    }
}

/* ================================================================== */
/*  Q1 状态机                                                          */
/* ================================================================== */
static void task1_step(float yaw)
{
    act_drive_yaw(yaw, 0.0f, 29.0f, 8.0f);

    if (s_ignore_line_cnt > 0) { s_ignore_line_cnt--; s_line_cnt = 0; return; }
    if (line_confirmed()) {
        trigger_alert(ALERT_PASS_TICKS);
        finish_task();
    }
}

/* ================================================================== */
/*  Q2 状态机                                                          */
/* ================================================================== */
static void enter_task2_state(Q2State_t next, uint8_t ignore_line)
{
    s_q2_state          = next;
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
        if (s_ignore_line_cnt > 0) { s_ignore_line_cnt--; s_line_cnt = 0; break; }
        if (line_confirmed()) {
            trigger_alert(ALERT_PASS_TICKS);
            enter_task2_state(Q2_TRACK_1, 0);
        }
        break;

    case Q2_TRACK_1:
        Track_move();
        update_no_line_counter();
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
        if (s_no_line_cnt >= Q2_FINISH_NO_LINE_CNT) {
            trigger_alert(ALERT_PASS_TICKS);
            finish_task();
        }
        break;
    }
}

/* ================================================================== */
/*  任务管理                                                           */
/* ================================================================== */
static void reset_task_common(void)
{
    s_state       = ST_DONE;
    s_q2_state    = Q2_LINE_0;
    reset_line_state(0);
    s_yaw_corr         = 0.0f;
    s_lap              = 0;
    s_target_laps      = 4;
    s_turn33_target    = YAW33_TARGET;
    s_turn147_target   = YAW147_TARGET;
    s_turn_stable_cnt  = 0;
    s_segment_ticks    = 0;
    location           = 0;
    current_encoder1   = 0;
    current_encoder2   = 0;
    reset_drive_pid();
    encoder_clear_both();
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
/*  PIT 回调 (在 TIMA1_IRQHandler 上下文中执行, 25Hz)                   */
/* ================================================================== */
static void control_pit_callback(uint32_t event, void *ptr);

/* ================================================================== */
/*  printf 替代 — UART0 调试输出                                       */
/* ================================================================== */
static void debug_print(const char *str)
{
    uart_write_string(DEBUG_UART_INDEX, str);
}

static void debug_println(const char *str)
{
    uart_write_string(DEBUG_UART_INDEX, str);
    uart_write_string(DEBUG_UART_INDEX, "\r\n");
}

/* 简易 printf 替代 (避免完整的 printf 拉入大量 libc 代码) */
#if CONTROL_DEBUG_ENABLE
static void debug_printf(const char *fmt, float yaw, int lap)
{
    /* 简化版: 仅支持固定格式, 减少代码体积 */
    int len = 0;
    const char *p = fmt;
    while (*p && len < (int)sizeof(s_print_buf) - 1) {
        if (*p == '%') {
            p++;
            if (*p == 'd' || *p == 'u') {
                /* lap 参数 */
                int val = lap;
                if (val < 0) { s_print_buf[len++] = '-'; val = -val; }
                if (val >= 100) s_print_buf[len++] = '0' + (val / 100) % 10;
                if (val >= 10)  s_print_buf[len++] = '0' + (val / 10) % 10;
                s_print_buf[len++] = '0' + (val % 10);
            } else if (*p == 'f') {
                /* yaw 参数 */
                int int_part = (int)yaw;
                int frac_part = (int)((yaw - (float)int_part) * 100.0f);
                if (frac_part < 0) frac_part = -frac_part;
                if (int_part < 0) { s_print_buf[len++] = '-'; int_part = -int_part; }
                if (int_part >= 100) s_print_buf[len++] = '0' + (int_part / 100) % 10;
                if (int_part >= 10)  s_print_buf[len++] = '0' + (int_part / 10) % 10;
                s_print_buf[len++] = '0' + (int_part % 10);
                s_print_buf[len++] = '.';
                s_print_buf[len++] = '0' + (frac_part / 10) % 10;
                s_print_buf[len++] = '0' + (frac_part % 10);
            }
            p++;
        } else if (*p == '\r' || *p == '\n') {
            p++;
        } else {
            s_print_buf[len++] = *p++;
        }
    }
    s_print_buf[len] = '\0';
    debug_print(s_print_buf);
    debug_print("\r\n");
}
#endif

/* ================================================================== */
/*  fputc 重定向 (给标准 printf 用, 保留兼容)                           */
/* ================================================================== */
/* fputc 由逐飞库 zf_common_debug.c 提供, 无需重复定义 */
#if 0
int fputc(int ch, FILE *f)
{
    UNUSED(f);
    uart_write_byte(DEBUG_UART_INDEX, (uint8_t)ch);
    return ch;
}
#endif

/* ================================================================== */
/*  MAIN                                                               */
/* ================================================================== */
int main(void)
{
    /* ---- 系统时钟 ---- */
    SystemClock_Config();

    /* ---- LED + 蜂鸣器 ---- */
    LED_Init();
    Buzzer_Init();

    /* ---- 电机 + 编码器 ---- */
    motor_Init();

    /* ---- UART 调试串口 ---- */
    uart_init(DEBUG_UART_INDEX, 115200, DEBUG_TX_PIN, DEBUG_RX_PIN);

    /* ---- UART 视觉串口 ---- */
    uart_init(VISION_UART_INDEX, 115200, VISION_TX_PIN, VISION_RX_PIN);

    /* ---- 循迹传感器 ---- */
    Track_Init();

    /* ---- 按键 ---- */
    Button_Init();

    /* ---- ICM-42688-P IMU (I2C) ---- */
    IMU_Init();

    /* ---- 启动提示 ---- */
    debug_println("CAR MSPM0G3507 ready (SeekFree Lib)");
    debug_println("  ICM-42688-P + SoftI2C + EXTI Encoder");

    /* ---- 控制定时器 25Hz (回调在 TIMA1_IRQHandler 上下文中执行) ---- */
    pit_ms_init(CONTROL_PIT, CONTROL_PERIOD_US / 1000, control_pit_callback, NULL);

    motor_stop();

    while (1) {
        /* 按键结果由控制 ISR 写入 s_task_start_pending, 主循环处理 */
        if (s_task_start_pending != 0) {
            TaskMode_t pending = (TaskMode_t)s_task_start_pending;
            s_task_start_pending = 0;
            start_task(pending);
        }
    }
}

/* ================================================================== */
/*  控制定时器 PIT 回调 (25Hz, TIMA1_IRQHandler 上下文)                  */
/*  PIT_TIM_A1 → TIMA1 → TIMA1_IRQHandler → pit_callback_list[1]()     */
/* ================================================================== */
static void control_pit_callback(uint32_t event, void *ptr)
{
    UNUSED(event);
    UNUSED(ptr);

    /* ---- 编码器 ---- */
    current_encoder1 = encoder_get_left();
    current_encoder2 = encoder_get_right();
    encoder_clear_both();
    location += current_encoder1;
    s_segment_ticks += (abs_i32(current_encoder1) + abs_i32(current_encoder2)) / 2;

    /* ---- IMU 姿态 ---- */
    IMU_Update(&s_roll, &s_pitch, &s_yaw);
    float yaw = s_yaw;

    /* ---- 按键扫描 ---- */
    uint8_t btn = Button_Scan();
    if (btn != BTN_NONE && s_task == TASK_NONE) {
        switch (btn) {
        case BTN_1: s_task_start_pending = (uint8_t)TASK_Q1; break;
        case BTN_2: s_task_start_pending = (uint8_t)TASK_Q2; break;
        case BTN_3: s_task_start_pending = (uint8_t)TASK_Q3; break;
        case BTN_4: s_task_start_pending = (uint8_t)TASK_Q4; break;
        }
    }

    /* ---- 状态机 ---- */
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

    /* ---- 声光提示计时 ---- */
    if (s_alert_ticks > 0) {
        s_alert_ticks--;
        if (s_alert_ticks == 0) {
            LED_OFF();
            Buzzer_OFF();
        }
    }
}

/* ================================================================== */
/*  系统时钟配置                                                       */
/* ================================================================== */
void SystemClock_Config(void)
{
    /* 使用逐飞库的默认时钟配置 (在 ti_msp_dl_config.h / system_mspm0g3507.c 中) */
    /* 若需 32MHz SYSOSC 无 PLL, 在 SysConfig 或 clock 初始化中设置: */
    /*   DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_32MHZ);            */
    /*   DL_SYSCTL_disablePLL();                                           */
    /* 此处信任逐飞库的默认配置, 后续可在 clock.c 中调优。                   */
}

/* ================================================================== */
/*  错误处理 + 异常向量                                                 */
/* ================================================================== */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        LED_Toggle();
        for (volatile uint32_t d = 0; d < 500000; d++) { }
    }
}

void NMI_Handler(void)           { while (1) { } }
void HardFault_Handler(void)     { while (1) { } }
void SVC_Handler(void)           { }
void PendSV_Handler(void)        { }
void SysTick_Handler(void)       { }
