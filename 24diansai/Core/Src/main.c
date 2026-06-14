#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "pid.h"
#include <stdio.h>
#include "motor.h"
#include "jy61p.h"
#include "track.h"
#include "button.h"
#include "buzzer.h"
#include "led.h"

/* 第4次改动说明：控制中断链路默认关闭实时串口打印，先保证循迹和姿态控制周期稳定。 */
#define CONTROL_DEBUG_ENABLE         0

#if CONTROL_DEBUG_ENABLE
#define CONTROL_DEBUG_PRINTF(...)    printf(__VA_ARGS__)
#else
/* 第4次改动说明：关闭调试打印后，所有控制路径里的日志调用都退化为空操作，避免阻塞中断。 */
#define CONTROL_DEBUG_PRINTF(...)    ((void)0)
#endif


int32_t current_encoder1 = 0;
int32_t current_encoder2 = 0;
int32_t location = 0;

PID_t Inner = {
    .Target = 15, .Kp = 2.0f, .Ki = 0.05f, .Kd = 0,
    .OutMax = 30, .OutMin = -30,.IntMax = 80, .IntMin = -80,
    .ErrHighThresh = 30, .ErrLowThresh = 5,
};
PID_t Inner1 = {
    .Target = 15, .Kp = 2.0f, .Ki = 0.05f, .Kd = 0,
    .OutMax = 30, .OutMin = -30, .IntMax = 80, .IntMin = -80,
    .ErrHighThresh = 30, .ErrLowThresh = 5,
};

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
    /* 第3次改动说明：把第二次定向单独拆成状态，避免从弧线出口直接硬切进第二段直线。 */
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

static volatile TaskMode_t s_task = TASK_NONE;
static State_t s_state = ST_DONE;
static Q2State_t s_q2_state = Q2_LINE_0;

static uint8_t s_no_line_cnt = 0;
static uint8_t s_ignore_line_cnt = 0;
static uint8_t s_line_cnt = 0;
static float s_yaw_corr = 0.0f;
static uint8_t s_lap = 0;
static uint8_t s_target_laps = 4;
/* 第2次改动说明：记录转向“连续稳定到位”的次数，避免 IMU 抖动导致单次命中目标角就提前切状态。 */
static uint8_t s_turn_stable_cnt = 0;
static uint8_t s_drive_boost_ticks = 0;
/* 第16次修改说明：每次进入闭环转向时保存本次动态目标角，避免多圈运行时一直套用固定绝对角导致误差累计。 */
static float s_turn33_target = 0.0f;
static float s_turn147_target = 0.0f;

static int32_t s_segment_ticks = 0;

#define NO_LINE_EXIT_CNT             4
/* 第21次修改说明：进入循迹偏晚，先把转向后直线保持的忽略拍数从 6 降到 4，让切入循迹提前约 80ms。 */
#define LINE_IGNORE_CNT              1
#define LINE_CONFIRM_CNT             1
#define Q2_LINE180_MIN_TICKS         180
#define Q2_FINISH_NO_LINE_CNT        10
/* 第15次修改说明：D->A 循迹刚开始时不允许立刻用丢线结束一圈，先走过一小段编码器距离再开放 A 点判定。 */
#define TRACK2_FINISH_MIN_TICKS      80

#define YAW33_MIN                    -60.0f
#define YAW33_MAX                    -20.0f
/* 第2次改动说明：Q3/Q4 的第一次定向目标角设置为 -33 度，后续闭环转向和到位判定都围绕这个目标执行。 */
#define YAW33_TARGET                 -33.1f
#define YAW147_MIN                   -160.0f
#define YAW147_MAX                   -120.0f
/* 第3次改动说明：第二次定向目标角设置为 -141 度，与第二段直线的 yaw 保持一致。 */
/* 临时调试：按实测要求将 B->D 第二次定向目标角调到 -142，不计入正式修改编号。 */
#define YAW147_TARGET                -147.0f
/* 第2次改动说明：允许转向结束时保留 ±3 度误差，给 IMU 噪声和机械惯性留出可接受范围。 */
#define YAW_TURN_TOLERANCE           3.0f
/* 第2次改动说明：转向采用最简单的 P 控制，误差越大给的原地转向 PWM 越大。 */
#define YAW_TURN_KP                  0.35f
/* 第2次改动说明：接近目标角时仍保留最小 PWM，避免电机因为静摩擦卡住不转。 */
/* 第12次修改说明：原来的 6.0f 容易落入电机静摩擦死区，导致还没真正转到目标角就停住；先提高到 10.0f 让闭环转向末段仍有足够力矩。 */
#define YAW_TURN_MIN_PWM             10.0f
/* 第2次改动说明：限制最大转向 PWM，防止原地转过猛造成过冲。 */
#define YAW_TURN_MAX_PWM             16.0f
/* 第2次改动说明：必须连续 3 个控制周期都进入容差，才认为第一次转向真正稳定完成。 */
#define YAW_TURN_STABLE_CNT          3
#define YAW_DEADBAND                 2.0f
#define YAW_CORR_KP                  0.25f
#define YAW_CORR_STEP                1.5f
#define DRIVE_START_BOOST_TICKS      4
#define DRIVE_START_MIN_PWM          16
/* 第16次修改说明：转向点姿态补偿最多只修正 10 度，防止某次入点过歪或 IMU 抖动把目标角带飞。 */
#define TURN_TARGET_COMP_LIMIT       10.0f
/*
 * IMU yaw 零点校准值。车物理朝前但 IMU 读数偏负时，角度环会
 * 主动向左打，将此值调大（正值）可抵消。实测决定具体数值。
 */
#define YAW_OFFSET                   2.2f
/*
 * 左右轮静态补偿。正值=左轮加速+右轮减速，车向右转，抵消左偏。
 * 机械不对称导致缓慢漂移时用，实测决定具体数值。
 */
#define MOTOR_TRIM                   0.0f

/*
 * 声光提示计数器。TIM5=100Hz，控制周期 4 分频 = 25Hz（40ms/周期）。
 * ALERT_PASS_TICKS = 6   → ~250ms 短促提示
 * ALERT_STOP_TICKS = 25  → ~1s 长提示
 */
#define ALERT_PASS_TICKS    4
#define ALERT_STOP_TICKS   15

/*
 * 非0时LED+蜂鸣器持续打开，每个控制周期（40ms）减1，
 * 归零时自动关闭。中断上下文中操作。
 */
static volatile uint16_t s_alert_ticks = 0;

/* 触发声光提示，ticks 为 40ms 控制周期数，不阻塞。 */
  static void trigger_alert(uint16_t ticks)
  {
      s_alert_ticks = ticks;
      LED_ON();
      Buzzer_ON();
  }

void SystemClock_Config(void);

static int32_t abs_i32(int32_t x)
{
    return (x < 0) ? -x : x;
}

static float limit_float(float x, float min, float max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

/* 第16次修改说明：把角度统一压回 [-180, 180]，动态目标角和姿态误差都用同一套角度环绕规则。 */
static float normalize_angle(float angle)
{
    if (angle > 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float approach_float(float now, float target, float step)
{
    if (now + step < target) return now + step;
    if (now - step > target) return now - step;
    return target;
}

/* 第2次改动说明：把目标角与当前 yaw 的差值压到 [-180, 180]，避免角度跨过 ±180 度时误差突变。 */
static float wrap_angle_err(float target, float yaw)
{
    float err = target - (yaw + YAW_OFFSET);
    return normalize_angle(err);
}

/* 第16次修改说明：进入转向点时按当前姿态相对理想方向的误差，生成本次转向目标角，减少多圈姿态误差累计。 */
static float build_turn33_target_from_A(float yaw)
{
    float corrected_yaw = normalize_angle(yaw + YAW_OFFSET);
    /* 第18次修改说明：A 点理论车头为 0 度，直接用校正后的 yaw 作为 A 点姿态误差。 */
    float heading_err = corrected_yaw;
    float comp = limit_float(heading_err, -TURN_TARGET_COMP_LIMIT, TURN_TARGET_COMP_LIMIT);
    /* 第17次修改说明：补偿要抵消到点姿态误差，所以目标角用 base_target - comp，而不是继续跟随误差加大偏差。 */
    return normalize_angle(YAW33_TARGET + 0.11f * comp);
}

/* 第18次修改说明：B 点理论车头为 180/-180 度，先把 yaw 换算成相对 180 度的小误差。 */
static float build_turn147_target_from_B(float yaw)
{
    float corrected_yaw = normalize_angle(yaw + YAW_OFFSET);
    float heading_err = (corrected_yaw >= 0.0f) ?
                        (corrected_yaw - 180.0f) :
                        (corrected_yaw + 180.0f);
    float comp = limit_float(heading_err, -TURN_TARGET_COMP_LIMIT, TURN_TARGET_COMP_LIMIT);
    /* 第18次修改说明：B 点也用目标角减误差的方式抵消左偏/右偏，参考角按 corrected_yaw 选 ±180。 */
    return normalize_angle(YAW147_TARGET - 0.15f*comp);
}

static void motor_stop(void)
{
    motor_start(0, 0);
}

static void reset_drive_pid(void)
{
    Inner.Error0 = 0.0f;
    Inner.Error1 = 0.0f;
    Inner.ErrorInt = 0.0f;
    Inner.Out = 0.0f;

    Inner1.Error0 = 0.0f;
    Inner1.Error1 = 0.0f;
    Inner1.ErrorInt = 0.0f;
    Inner1.Out = 0.0f;
}

static void reset_line_state(uint8_t ignore_line)
{
    s_no_line_cnt = 0;
    s_line_cnt = 0;
    s_ignore_line_cnt = ignore_line;
}

static uint8_t line_confirmed(void)
{
    if (Track_AnyLine()) {
        if (s_line_cnt < LINE_CONFIRM_CNT)
            s_line_cnt++;
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

/* 第1/2次改动说明：这里替代原来的固定 PWM 开环转向。
 * 运行过程是：先算目标角误差，再按比例换算成转向 PWM，最后左右轮反向输出实现原地闭环转向。
 * 这样误差大时转得快，误差小时自动减速，比 motor_start(10, -10) 的开环方式稳定得多。 */
static void act_turn_closed_loop(float yaw, float target)
{
    float turn_pwm = wrap_angle_err(target, yaw) * YAW_TURN_KP;
    turn_pwm = limit_float(turn_pwm, -YAW_TURN_MAX_PWM, YAW_TURN_MAX_PWM);

    /* 第2次改动说明：如果误差已经很小，P 控制算出来的 PWM 也会很小。
     * 这里强制兜底一个最小 PWM，避免电机停在“理论该转、实际上转不动”的死区里。 */
    if (turn_pwm > 0.0f && turn_pwm < YAW_TURN_MIN_PWM)
        turn_pwm = YAW_TURN_MIN_PWM;
    else if (turn_pwm < 0.0f && turn_pwm > -YAW_TURN_MIN_PWM)
        turn_pwm = -YAW_TURN_MIN_PWM;

    /* 第2次改动说明：左右轮反向输出，车辆原地转向，不引入额外前进位移。 */
    /* 第9次改动说明：闭环转向输出方向要与原开环 motor_start(10, -10) 一致。 */
    motor_start((int16_t)(-turn_pwm), (int16_t)turn_pwm);
}

static void act_drive_yaw(float yaw, float target, float speed, float corr_max)
{
    float yaw_err = (yaw + YAW_OFFSET) - target;
    if (yaw_err >  180.0f) yaw_err -= 360.0f;
    if (yaw_err < -180.0f) yaw_err += 360.0f;

    float yaw_corr_target = 0;
    if (yaw_err > YAW_DEADBAND || yaw_err < -YAW_DEADBAND)
        yaw_corr_target = limit_float(yaw_err * YAW_CORR_KP, -corr_max, corr_max);

    s_yaw_corr = approach_float(s_yaw_corr, yaw_corr_target, YAW_CORR_STEP);

    Inner.Target  = speed;
    Inner1.Target = speed;
    Inner.Actual  = current_encoder1;
    Inner1.Actual = current_encoder2;
    PID_Update(&Inner);
    PID_Update(&Inner1);
    int16_t left_pwm = (int16_t)(Inner.Out + s_yaw_corr + MOTOR_TRIM);
    int16_t right_pwm = (int16_t)(Inner1.Out - s_yaw_corr - MOTOR_TRIM);

    /* 第10次改动说明：转向刚切到直线时，前几拍给最低前进 PWM，避免静止起步卡住。 */
    if (s_drive_boost_ticks > 0) {
        if (left_pwm < DRIVE_START_MIN_PWM)
            left_pwm = DRIVE_START_MIN_PWM;
        if (right_pwm < DRIVE_START_MIN_PWM)
            right_pwm = DRIVE_START_MIN_PWM;
        s_drive_boost_ticks--;
    }

    motor_start(left_pwm, right_pwm);
}

/* 第1/2次改动说明：这里不再用“当前这一拍进窗口就切状态”的判断。
 * 只有当 yaw 连续多个控制周期都落在目标角容差内，才允许从 ST_TURN_TO_33 切到 ST_LINE_33，
 * 这样可以过滤 IMU 抖动和转向惯性带来的瞬时误判。 */
static uint8_t yaw_reached_target(float yaw, float target)
{
    float yaw_err = wrap_angle_err(target, yaw);

    if (yaw_err < YAW_TURN_TOLERANCE && yaw_err > -YAW_TURN_TOLERANCE) {
        if (s_turn_stable_cnt < YAW_TURN_STABLE_CNT)
            s_turn_stable_cnt++;
    } else {
        s_turn_stable_cnt = 0;
    }

    return (s_turn_stable_cnt >= YAW_TURN_STABLE_CNT) ? 1 : 0;
}

/* 第11次改动说明：直线段姿态窗口也统一用 wrap_angle_err()，避免到位判断用 offset、回退判断不用 offset。 */
static uint8_t yaw_out_of_window(float yaw, float target, float tolerance)
{
    float yaw_err = wrap_angle_err(target, yaw);
    return (yaw_err > tolerance || yaw_err < -tolerance) ? 1 : 0;
}

static void finish_task(void)
{
    s_task = TASK_NONE;
    s_state = ST_DONE;
    motor_stop();
    reset_drive_pid();
    s_yaw_corr = 0.0f;
    /* 停车声光提示 */
    trigger_alert(ALERT_STOP_TICKS);
}

static void enter_line_33(void)
{
    s_state = ST_LINE_33;
    reset_line_state(LINE_IGNORE_CNT);
    s_yaw_corr = 0.0f;
    /* 第2次改动说明：转向状态结束后立刻清零稳定计数，避免旧计数污染后续状态。 */
    s_turn_stable_cnt = 0;
    /* 第10次改动说明：第一次转向进入直线后给短促起步兜底，克服静止起步死区。 */
    s_drive_boost_ticks = DRIVE_START_BOOST_TICKS;
    reset_drive_pid();
}

/* 第8次改动说明：Q4 每一圈回到 A 后必须重新进入第一次定向，不能直接跳到 ST_LINE_33。 */
static void enter_turn_33(float yaw)
{
    s_state = ST_TURN_TO_33;
    reset_line_state(0);
    s_yaw_corr = 0.0f;
    s_turn_stable_cnt = 0;
    /* 第16次修改说明：A 点理论车头接近 0 度，按当前姿态误差修正本次 A->C 的目标角。 */
    s_turn33_target = build_turn33_target_from_A(yaw);
    reset_drive_pid();
}

/* 第3次改动说明：从第一段弧线出来后，先做第二次原地闭环定向，再切到第二段直线。 */
static void enter_turn_147(float yaw)
{
    s_state = ST_TURN_TO_147;
    reset_line_state(0);
    s_yaw_corr = 0.0f;
    s_turn_stable_cnt = 0;
    /* 第16次修改说明：B 点理论车头接近 180/-180 度，按当前姿态误差修正本次 B->D 的目标角。 */
    s_turn147_target = build_turn147_target_from_B(yaw);
    reset_drive_pid();
}

static void enter_line_147(void)
{
    s_state = ST_LINE_147;
    reset_line_state(LINE_IGNORE_CNT);
    s_yaw_corr = 0.0f;
    /* 第3次改动说明：第二次转向完成后同样清零稳定计数，避免影响第二段直线。 */
    s_turn_stable_cnt = 0;
    s_drive_boost_ticks = DRIVE_START_BOOST_TICKS;
    reset_drive_pid();
}


static void path34_step(float yaw)
{
    switch (s_state)
    {
    case ST_TURN_TO_33:
        /* 第2次改动说明：第一次转向不再使用宽窗口直接放行，而是判断是否稳定到达 -33 度目标角。 */
        if (yaw_reached_target(yaw, s_turn33_target)) enter_line_33();
        break;

    case ST_LINE_33:
        if (s_ignore_line_cnt > 0) {
            s_ignore_line_cnt--;
            break;
        }
        if (line_confirmed()) {
            s_state = ST_TRACK;
					  /* 到达C点 */
						trigger_alert(ALERT_PASS_TICKS);
            reset_line_state(0);
            break;
        }
        /* 第2次改动说明：直线段如果姿态重新偏出允许范围，退回 ST_TURN_TO_33 重新校正第一次定向。 */
        if (yaw_out_of_window(yaw, s_turn33_target, 27.0f)) {
            enter_turn_33(yaw);
            /* 第2次改动说明：退回转向状态时必须重新累计稳定次数，不能沿用上一次结果。 */
            s_line_cnt = 0;
        }
        break;

    case ST_TRACK:
        update_no_line_counter();
        if (s_no_line_cnt >= NO_LINE_EXIT_CNT &&
            (yaw > 130.0f || yaw < -150.0f))
        {
					/* 到达B点 */
						trigger_alert(ALERT_PASS_TICKS);
            /* 第3次改动说明：弧线出口后不再直接硬切进第二段直线，先进入第二次闭环定向。 */
            enter_turn_147(yaw);
        }
        break;

    case ST_TURN_TO_147:
        /* 第3次改动说明：第二次转向沿用第一次的稳定到位判定，确认姿态稳定后再进入第二段直线。 */
        if (yaw_reached_target(yaw, s_turn147_target)) enter_line_147();
        break;

    case ST_LINE_147:
        if (s_ignore_line_cnt > 0) {
            s_ignore_line_cnt--;
            break;
        }
        if (line_confirmed()) {
            s_state = ST_TRACK2;
					/* 到达D点 */
            trigger_alert(ALERT_PASS_TICKS);
            reset_line_state(0);
            /* 第15次修改说明：从 D 点切入 D->A 循迹时重置本段距离，后续用它过滤刚入弯的短暂丢线误判。 */
            s_segment_ticks = 0;
            /* 第16次修改说明：确认到 D 点后已经切入 D->A 循迹，本拍不再继续执行第二段直线的姿态回退判断。 */
            break;
        }
        /* 第3次改动说明：第二段直线如果偏出允许姿态范围，退回第二次转向状态重新校正。 */
        if (yaw_out_of_window(yaw, s_turn147_target, 21.0f)) {
            enter_turn_147(yaw);
        }
        break;

    case ST_TRACK2:
        update_no_line_counter();
        /* 第15次修改说明：D->A 先走过一小段距离后才允许丢线结束本圈，避免入弯丢线被误判成回到 A。 */
        if (s_segment_ticks >= TRACK2_FINISH_MIN_TICKS &&
            s_no_line_cnt >= NO_LINE_EXIT_CNT)
        {
            s_lap++;
					/* 回到A点 */
            trigger_alert(ALERT_PASS_TICKS);
            reset_line_state(0);
            if (s_lap >= s_target_laps)
                finish_task();
            else
                /* 第8次改动说明：多圈任务下一圈从第一次闭环转向开始，避免直接进直线导致转向位置错误。 */
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
        /* 第2次改动说明：这里正式接入闭环转向执行函数，控制量由当前 yaw 和目标角共同决定。 */
        act_turn_closed_loop(yaw, s_turn33_target);
        /* 第4次改动说明：控制回调中的逐周期打印会拖慢中断，这里改为受宏控制的可关闭日志。 */
        CONTROL_DEBUG_PRINTF("Q%d TURN1 Yaw:%.2f\r\n", (s_task == TASK_Q4) ? 4 : 3, (double)yaw);
        break;
    case ST_LINE_33:
        act_drive_yaw(yaw, s_turn33_target, 29.0f, 8.0f);
        CONTROL_DEBUG_PRINTF("Q%d STR1 Yaw:%.2f\r\n", (s_task == TASK_Q4) ? 4 : 3, (double)yaw);
        break;
    case ST_TRACK:
        Track_move();
        CONTROL_DEBUG_PRINTF("Q%d TRACK Yaw:%.2f\r\n", (s_task == TASK_Q4) ? 4 : 3, (double)yaw);
        break;
    case ST_TURN_TO_147:
        /* 第3次改动说明：第二次转向也切换为显式闭环执行，和第一次保持同一套控制方式。 */
        act_turn_closed_loop(yaw, s_turn147_target);
        CONTROL_DEBUG_PRINTF("Q%d TURN2 Yaw:%.2f\r\n", (s_task == TASK_Q4) ? 4 : 3, (double)yaw);
        break;
    case ST_LINE_147:
        act_drive_yaw(yaw, s_turn147_target, 22.0f, 5.0f);
        CONTROL_DEBUG_PRINTF("Q%d STR2 Yaw:%.2f\r\n", (s_task == TASK_Q4) ? 4 : 3, (double)yaw);
        break;
    case ST_TRACK2:
        Track_move();
        CONTROL_DEBUG_PRINTF("Q%d TRACK2 Lap:%d Yaw:%.2f\r\n", (s_task == TASK_Q4) ? 4 : 3, (int)s_lap, (double)yaw);
        break;
    case ST_DONE:
        motor_stop();
        break;
    }
}

static void task1_step(float yaw)
{
    act_drive_yaw(yaw, 0.0f, 29.0f, 8.0f);
    CONTROL_DEBUG_PRINTF("Q1 STR0 Yaw:%.2f\r\n", (double)yaw);

    if (s_ignore_line_cnt > 0) {
        s_ignore_line_cnt--;
        s_line_cnt = 0;
        return;
    }

    if (line_confirmed()) {
        /* 到达B点 */
        trigger_alert(ALERT_PASS_TICKS);
        finish_task();
    }
}

static void enter_task2_state(Q2State_t next_state, uint8_t ignore_line)
{
    s_q2_state = next_state;
    reset_line_state(ignore_line);
    s_drive_boost_ticks = 0;
    s_segment_ticks = 0;
    s_yaw_corr = 0.0f;
    reset_drive_pid();
}

static void task2_step(float yaw)
{
    switch (s_q2_state)
    {
    case Q2_LINE_0:
        act_drive_yaw(yaw, 0.0f, 29.0f, 8.0f);
        CONTROL_DEBUG_PRINTF("Q2 LINE0 Yaw:%.2f\r\n", (double)yaw);

        if (s_ignore_line_cnt > 0) {
            s_ignore_line_cnt--;
            s_line_cnt = 0;
            break;
        }
        if (line_confirmed()) {
            /* 到达B点 */
            trigger_alert(ALERT_PASS_TICKS);
            enter_task2_state(Q2_TRACK_1, 0);
        }
        break;

    case Q2_TRACK_1:
        Track_move();
        update_no_line_counter();
        CONTROL_DEBUG_PRINTF("Q2 TRACK1 Yaw:%.2f\r\n", (double)yaw);
        if (s_no_line_cnt >= NO_LINE_EXIT_CNT &&
            (yaw > 130.0f || yaw < -150.0f))
        {
            /* 到达C点 */
            trigger_alert(ALERT_PASS_TICKS);
            enter_task2_state(Q2_LINE_180, LINE_IGNORE_CNT);
        }
        break;

    case Q2_LINE_180:
    {
        float target = (yaw >= 0.0f) ? 180.0f : -180.0f;
        act_drive_yaw(yaw, target, 29.0f, 8.0f);
        CONTROL_DEBUG_PRINTF("Q2 LINE180 Yaw:%.2f\r\n", (double)yaw);

        if (s_ignore_line_cnt > 0) {
            s_ignore_line_cnt--;
            s_line_cnt = 0;
            break;
        }
        if (s_segment_ticks >= Q2_LINE180_MIN_TICKS && line_confirmed()) {
            /* 到达D点 */
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
            /* 回到A点 */
            trigger_alert(ALERT_PASS_TICKS);
            finish_task();
        }
        break;
    }
}

static void reset_task_common(void)
{
    s_state = ST_DONE;
    s_q2_state = Q2_LINE_0;
    reset_line_state(0);
    s_yaw_corr = 0.0f;
    s_lap = 0;
    s_target_laps = 4;
    /* 第16次修改说明：任务重启时先恢复固定理论目标，进入转向点后再按当前姿态生成动态目标。 */
    s_turn33_target = YAW33_TARGET;
    s_turn147_target = YAW147_TARGET;
    /* 第2次改动说明：每次重新开始任务前清零转向稳定计数，保证本轮第一次转向从干净状态开始。 */
    s_turn_stable_cnt = 0;
    s_segment_ticks = 0;
    location = 0;
    current_encoder1 = 0;
    current_encoder2 = 0;
    reset_drive_pid();
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
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
        s_state = ST_TURN_TO_33;
        s_target_laps = 1;
        s_task = TASK_Q3;
				trigger_alert(ALERT_PASS_TICKS);
        break;

    case TASK_Q4:
        s_state = ST_TURN_TO_33;
        s_target_laps = 4;
        s_task = TASK_Q4;
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

static void poll_start_buttons(void)
{
    if (s_task != TASK_NONE)
        return;

    if (Button1_Pressed()) {
        start_task(TASK_Q1);
    } else if (Button2_Pressed()) {
        start_task(TASK_Q2);
    } else if (Button3_Pressed()) {
        start_task(TASK_Q3);
    } else if (Button4_Pressed()) {
        start_task(TASK_Q4);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
	  Buzzer_OFF();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();

    HAL_TIM_Base_Start_IT(&htim5);
    motor_Init();
    JY61p_Init();
    Track_Init();
    Button_Init();
    motor_stop();

    while (1) {
        poll_start_buttons();
    }
}

int fputc(int ch, FILE *f)
{
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == RESET);
    huart2.Instance->DR = (uint8_t)ch;
    return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint16_t counter = 0;
    if (htim->Instance != TIM5) return;

    if (++counter < 4) {
        __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
        return;
    }
    counter = 0;

    current_encoder1 =  (int16_t)(__HAL_TIM_GET_COUNTER(&htim3));
    current_encoder2 = -(int16_t)(__HAL_TIM_GET_COUNTER(&htim4));
    location += current_encoder1;
    s_segment_ticks += (abs_i32(current_encoder1) + abs_i32(current_encoder2)) / 2;

    float roll, pitch, yaw;
    JY61p_GetAngle(&roll, &pitch, &yaw);

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

    /* 声光提示计时：非阻塞驱动LED和蜂鸣器 */
    if (s_alert_ticks > 0) {
        s_alert_ticks--;
        if (s_alert_ticks == 0) {
            LED_OFF();
            Buzzer_OFF();
        }
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
