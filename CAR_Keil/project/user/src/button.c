/**
 * @file    button.c
 * @brief   非阻塞按键扫描 — 4 路独立状态机消抖
 *
 * 在 25Hz 控制 ISR 中每 40ms 调用一次。
 * 消抖: 连续 2 次低 = 80ms 确认按下。
 * 触发后等待释放才返回空闲。
 */
#include "main.h"
#include "button.h"

/* 按键引脚列表 */
static const gpio_pin_enum btn_pins[4] = {
    SW1_PIN, SW2_PIN, SW3_PIN, SW4_PIN,
};

/* 按键状态机 */
typedef enum {
    BTN_ST_IDLE = 0,
    BTN_ST_DEBOUNCE,
    BTN_ST_HOLD,
} btn_state_t;

static btn_state_t s_btn_state[4] = { BTN_ST_IDLE, BTN_ST_IDLE, BTN_ST_IDLE, BTN_ST_IDLE };
static uint8_t     s_btn_cnt[4]   = { 0, 0, 0, 0 };

void Button_Init(void)
{
    for (int i = 0; i < 4; i++) {
        gpio_init(btn_pins[i], GPI, GPIO_LOW, GPI_PULL_UP);
    }
}

uint8_t Button_Scan(void)
{
    for (int i = 0; i < 4; i++) {
        uint8_t pressed = (gpio_get_level(btn_pins[i]) == 0);  /* 低有效 */

        switch (s_btn_state[i])
        {
        case BTN_ST_IDLE:
            if (pressed) {
                s_btn_state[i] = BTN_ST_DEBOUNCE;
                s_btn_cnt[i]   = 1;
            }
            break;

        case BTN_ST_DEBOUNCE:
            if (pressed) {
                s_btn_cnt[i]++;
                if (s_btn_cnt[i] >= 2) {  /* 2 × 40ms = 80ms 消抖 */
                    s_btn_state[i] = BTN_ST_HOLD;
                    return (uint8_t)(BTN_1 + i);  /* BTN_1 ~ BTN_4 */
                }
            } else {
                s_btn_state[i] = BTN_ST_IDLE;
                s_btn_cnt[i]   = 0;
            }
            break;

        case BTN_ST_HOLD:
            if (!pressed) {
                s_btn_state[i] = BTN_ST_IDLE;
                s_btn_cnt[i]   = 0;
            }
            break;
        }
    }

    return BTN_NONE;
}
