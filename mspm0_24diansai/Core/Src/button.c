/**
 * @file    button.c
 * @brief   Non-blocking button scanner (state-machine debounce)
 *
 * Replaces the HAL_Delay()-based blocking scan.
 *
 * Called at 25 Hz (40 ms per tick) from the control ISR.
 * Debounce: 2 ticks = 80 ms confirm, then wait for release.
 */
#include "main.h"
#include "button.h"

/* ---- Per-button state ---- */
typedef enum {
    BTN_ST_IDLE = 0,
    BTN_ST_DEBOUNCE,
    BTN_ST_HOLD,
} btn_state_t;

static struct {
    btn_state_t state;
    uint8_t     cnt;
} s_btn[4];

/* ---- Public API ---- */

void Button_Init(void)
{
    /* Configure button pins as input with pull-up */
    DL_GPIO_initDigitalInputFeatures(BUTTON1_PIN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BUTTON2_PIN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BUTTON3_PIN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(BUTTON4_PIN_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    for (uint8_t i = 0; i < 4; i++) {
        s_btn[i].state = BTN_ST_IDLE;
        s_btn[i].cnt   = 0;
    }
}

/**
 * @brief  Scan all 4 buttons (call at 25 Hz).
 * @return BTN_1..4 when a button is confirmed pressed, BTN_NONE otherwise.
 *         Each button must be released before it can trigger again.
 */
uint8_t Button_Scan(void)
{
    uint32_t raw = DL_GPIO_readPins(BUTTON_PORT, BUTTON_ALL_MASK);
    uint8_t  pin_state[4];
    pin_state[0] = (raw & BUTTON1_PIN) ? 1 : 0;   /* 1 = released (pull-up) */
    pin_state[1] = (raw & BUTTON2_PIN) ? 1 : 0;
    pin_state[2] = (raw & BUTTON3_PIN) ? 1 : 0;
    pin_state[3] = (raw & BUTTON4_PIN) ? 1 : 0;

    uint8_t result = BTN_NONE;

    for (uint8_t i = 0; i < 4; i++) {
        uint8_t pressed = (pin_state[i] == 0);  /* active-low */
        switch (s_btn[i].state) {
        case BTN_ST_IDLE:
            if (pressed) {
                s_btn[i].state = BTN_ST_DEBOUNCE;
                s_btn[i].cnt   = 1;
            }
            break;

        case BTN_ST_DEBOUNCE:
            if (pressed) {
                s_btn[i].cnt++;
                if (s_btn[i].cnt >= 2) {     /* 2 × 40 ms = 80 ms */
                    s_btn[i].state = BTN_ST_HOLD;
                    result = (uint8_t)(i + 1);  /* BTN_1 .. BTN_4 */
                }
            } else {
                s_btn[i].state = BTN_ST_IDLE;
                s_btn[i].cnt   = 0;
            }
            break;

        case BTN_ST_HOLD:
            if (!pressed) {
                s_btn[i].state = BTN_ST_IDLE;
                s_btn[i].cnt   = 0;
            }
            break;
        }
    }

    return result;
}
