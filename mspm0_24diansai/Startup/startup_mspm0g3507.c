/**
 * @file    startup_mspm0g3507.c
 * @brief   CMSIS startup for MSPM0G3507 (Cortex-M0+)
 *
 * Minimal startup: vector table, stack pointer init, bss/data init,
 * then call main().
 *
 * The linker script (MSPM0G3507.cmd) places the .vectors section
 * at 0x00000000 (flash base).
 */

#include <stdint.h>

/* ---- Symbols defined by linker ---- */
extern uint32_t _stack_top;          /* top of main stack (linker symbol) */
extern int main(void);

/* Default empty handler */
void Default_Handler(void) { while (1) { } }

/* Weak aliases for all exception handlers */
void NMI_Handler(void)           __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)           __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)       __attribute__((weak, alias("Default_Handler")));

/* Peripheral interrupts — weak aliases */
void TIMG2_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void TIMG0_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void TIMG1_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void UART0_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void UART1_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));

/*
 * The vector table must be placed in a section named ".vectors"
 * (or ".intvecs" for TI compiler).  The linker command file maps
 * this section to address 0x00000000.
 */
__attribute__((section(".vectors"), used))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))((uint32_t)&_stack_top),  /* [ 0] Initial SP       */
    (void (*)(void))((uint32_t)&main),         /* [ 1] Reset           */
    NMI_Handler,                               /* [ 2] NMI             */
    HardFault_Handler,                         /* [ 3] HardFault       */
    0,                                         /* [ 4] — reserved      */
    0,                                         /* [ 5] — reserved      */
    0,                                         /* [ 6] — reserved      */
    0,                                         /* [ 7] — reserved      */
    0,                                         /* [ 8] — reserved      */
    0,                                         /* [ 9] — reserved      */
    0,                                         /* [10] — reserved      */
    SVC_Handler,                               /* [11] SVCall          */
    0,                                         /* [12] — reserved      */
    0,                                         /* [13] — reserved      */
    PendSV_Handler,                            /* [14] PendSV          */
    SysTick_Handler,                           /* [15] SysTick         */
    /* Peripheral interrupts — IRQ 0..31 */
    Default_Handler,  /*  0 */  Default_Handler,  /*  1 */
    Default_Handler,  /*  2 */  Default_Handler,  /*  3 */
    Default_Handler,  /*  4 */  Default_Handler,  /*  5 */
    Default_Handler,  /*  6 */  Default_Handler,  /*  7 */
    Default_Handler,  /*  8 */  Default_Handler,  /*  9 */
    Default_Handler,  /* 10 */  Default_Handler,  /* 11 */
    Default_Handler,  /* 12 */  Default_Handler,  /* 13 */
    Default_Handler,  /* 14 */  Default_Handler,  /* 15 */
    Default_Handler,  /* 16 */  Default_Handler,  /* 17 */
    Default_Handler,  /* 18 */  Default_Handler,  /* 19 */
    Default_Handler,  /* 20 */  Default_Handler,  /* 21 */
    Default_Handler,  /* 22 */  Default_Handler,  /* 23 */
    Default_Handler,  /* 24 */  Default_Handler,  /* 25 */
    Default_Handler,  /* 26 */  Default_Handler,  /* 27 */
    Default_Handler,  /* 28 */  Default_Handler,  /* 29 */
    Default_Handler,  /* 30 */  Default_Handler,  /* 31 */
};
