#include "track.h"
#include "motor.h"
#include "led.h"
#include "buzzer.h"

/* 第5次改动说明：为循迹比例控制提供统一参数，替代原来的固定档位表。 */
#define TRACK_BASE_SPEED            14
#define TRACK_LOST_SPEED            10
#define TRACK_CROSS_SPEED           12
#define TRACK_TURN_LIMIT            9
/* 第20次修改说明：D1/D7 单边时容易过冲丢线，先把整体比例修正回 4，再配合单边低速找线。 */
#define TRACK_ERR_SCALE             4
/* 第20次修改说明：单边最外侧探头触发时，降低基础速度，避免差速打满后直接甩过赛道。 */
#define TRACK_EDGE_SPEED            11
/* 第15次修改说明：短暂丢线时不再直接直走，而是按最后一次循迹偏差小幅继续找线，帮助 D4 重新压回赛道。 */
#define TRACK_LOST_TURN_LIMIT       7

static int16_t s_last_track_turn = 0;

static int16_t track_limit_i16(int16_t x, int16_t min, int16_t max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

void Track_Init(void)
{
    motor_Init();
    LED_Init();
    Buzzer_Init();
}

/* 任意一个传感器检测到线就返回 1。 */
uint8_t Track_AnyLine(void)
{
    return (D1 || D2 || D3 || D4 || D5 || D6 || D7) ? 1 : 0;
}

/* 第5次改动说明：把原来的固定档位循迹改成连续误差循迹。
 * 做法是把 7 路数字传感器映射成一个横向误差，再把误差换算成左右轮差速。 */
void Track_move(void)
{
    uint8_t s1 = (uint8_t)D1;
    uint8_t s2 = (uint8_t)D2;
    uint8_t s3 = (uint8_t)D3;
    uint8_t s4 = (uint8_t)D4;
    uint8_t s5 = (uint8_t)D5;
    uint8_t s6 = (uint8_t)D6;
    uint8_t s7 = (uint8_t)D7;
    uint8_t count = (uint8_t)(s1 + s2 + s3 + s4 + s5 + s6 + s7);
    int16_t err_sum = 0;
    int16_t turn = 0;
    int16_t l = TRACK_BASE_SPEED;
    int16_t r = TRACK_BASE_SPEED;

    /* 第5次改动说明：丢线时先降速直行，不再维持较高速度前冲，给重捕获或状态机退出留时间。 */
    if (count == 0) {
        /* 第15次修改说明：短暂丢线时沿用上一拍差速方向找线，不再直走，避免 D->A 弧线中 D4 更难重新压回赛道。 */
        turn = track_limit_i16(s_last_track_turn, -TRACK_LOST_TURN_LIMIT, TRACK_LOST_TURN_LIMIT);
        l = track_limit_i16((int16_t)(TRACK_LOST_SPEED + turn), 0, 99);
        r = track_limit_i16((int16_t)(TRACK_LOST_SPEED - turn), 0, 99);
        motor_start(l, r);
        return;
    }

    /* 第5次改动说明：多探头同时压线通常出现在节点或宽线区域，这里低速通过，减少误冲。 */
    if (count >= 4) {
        /* 第15次修改说明：宽线/节点处清掉上一拍找线记忆，避免离开节点后继承错误转向。 */
        s_last_track_turn = 0;
        motor_start(TRACK_CROSS_SPEED, TRACK_CROSS_SPEED);
        return;
    }

    /* 第7次改动说明：权重方向按原固定档位表校正。
     * 原逻辑是 D1 -> 左轮快右轮慢，D7 -> 左轮慢右轮快；这里保持同样的修正方向。 */
    /* 第16次修改说明：D1/D7 已经是赛道边缘信号，权重从 3 提到 4，让快冲出赛道时的救车响应更强。 */
    err_sum = (int16_t)(6 * s1 + 3 * s2 + 2 * s3 + 0 * s4 - 2 * s5 - 3 * s6 - 6 * s7);

    /* 第5次改动说明：误差越大，给的差速越大；误差越小，修正越柔和。 */
    /* 第14次修改说明：差速先乘后除，保留 D3+D4、D4+D5 这类轻微偏线的修正量，避免整数除法把小误差抹成 0。 */
    turn = track_limit_i16((int16_t)((err_sum * TRACK_ERR_SCALE) / (int16_t)count), -TRACK_TURN_LIMIT, TRACK_TURN_LIMIT);
    /* 第15次修改说明：记录有效循迹差速，下一拍若短暂丢线，就按这个方向继续修正找线。 */
    s_last_track_turn = turn;

    /* 第5次改动说明：连续差速替代固定档位跳变，提升 A->C 和弧线入口的循迹细度。 */
    /* 第20次修改说明：只剩 D1 或 D7 时说明车已经跑到赛道最边缘，这一拍先降速，再按原方向修正，减少过冲丢线。 */
    if (count == 1 && (s1 || s7)) {
        l = track_limit_i16((int16_t)(TRACK_EDGE_SPEED + turn), 0, 99);
        r = track_limit_i16((int16_t)(TRACK_EDGE_SPEED - turn), 0, 99);
    } else {
        l = track_limit_i16((int16_t)(TRACK_BASE_SPEED + turn), 0, 99);
        r = track_limit_i16((int16_t)(TRACK_BASE_SPEED - turn), 0, 99);
    }

    motor_start(l, r);
}
