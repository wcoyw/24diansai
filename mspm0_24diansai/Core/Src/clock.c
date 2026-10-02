/**
 * @file    clock.c
 * @brief   MSPM0G3507 system / bus clock configuration (80 MHz)
 *
 * Equivalent to STM32's SystemClock_Config().
 * Uses internal SYSOSC + PLL — no external crystal required.
 */
#include "ti_msp_dl_config.h"

void SystemClock_Config(void)
{
    /*
     * Default power-on: SYSOSC @ 32 MHz, MCLK = SYSOSC.
     *
     * Target: SYSOSC → PLL → 80 MHz MCLK / UCLK.
     *
     * Datasheet constraints:
     *   PLL reference must be 4–48 MHz  → SYSOSC 32 MHz is fine.
     *   PLL VCO range is 320–640 MHz.
     *   f_VCO = f_REF × (REFDIV+1) / (PREDIV+1) × FB_DIV
     *
     * For 80 MHz output from 32 MHz reference:
     *   PREDIV = 1 → f_in = 16 MHz
     *   FB_DIV  = 40 → f_VCO = 16 × 40 = 640 MHz  ✓
     *   ODIV    = 8  → f_out = 640 / 8 = 80 MHz   ✓
     *
     * Simplified: use DL_SYSCTL helper functions.
     */
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_32MHZ);

    /* Enable PLL with 80 MHz output */
    DL_SYSCTL_enablePLL();
    DL_SYSCTL_setPLLFreq(DL_SYSCTL_PLL_FREQ_80MHZ);

    /* Switch MCLK source to PLL output */
    DL_SYSCTL_setMCLKSel(DL_SYSCTL_MCLK_SEL_PLL_OUT0);

    /* MCLK divider = /1 → 80 MHz */
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_1);

    /* UCLK (bus clock for UART/TIMER) = MCLK / 1 → 80 MHz */
    DL_SYSCTL_setUCLKDivider(DL_SYSCTL_UCLK_DIVIDER_1);

    /* Run mode — everything on */
    DL_SYSCTL_setPowerMode(DL_SYSCTL_POWER_MODE_RUN);

    /*
     * Enable peripheral bus clocks for the modules we use.
     * On MSPM0, peripherals are powered by PD1 (RUN mode).
     * Individual peripheral clocks are gated through SYSCTL.
     */

    /* GPIO banks */
    DL_SYSCTL_enablePD1();
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
}
