/**
 * @file    MSPM0G3507.cmd
 * @brief   Linker command file for MSPM0G3507 (128 KB Flash, 32 KB SRAM)
 *
 * Used by the TI ARM linker (armlnk) or GCC linker (arm-none-eabi-ld).
 * Adjust syntax for your toolchain.
 *
 * ===== Memory map =====
 * Flash: 0x0000_0000 – 0x0002_0000  (128 KB)
 * SRAM:  0x2000_0000 – 0x2000_8000  ( 32 KB)
 */

/* ================================================================== */
/*  For TI ARM Linker (armlnk / tiarmclang)                           */
/* ================================================================== */
#if defined(__TI_ARM__)

MEMORY
{
    FLASH  (RX) : origin = 0x00000000, length = 0x00020000
    SRAM   (RW) : origin = 0x20000000, length = 0x00008000
}

SECTIONS
{
    .vectors    : > FLASH, ALIGN(256)
    .text       : > FLASH
    .const      : > FLASH
    .cinit      : > FLASH
    .rodata     : > FLASH

    .stack      : > SRAM, ALIGN(8)
    .bss        : > SRAM
    .data       : > SRAM
    .sysmem     : > SRAM
}

/* Stack size */
__STACK_SIZE = 0x00000800;  /* 2 KB */
__STACK_TOP  = __stack + __STACK_SIZE;
_stack_top   = __STACK_TOP;

#endif /* __TI_ARM__ */

/* ================================================================== */
/*  For GCC Linker (arm-none-eabi-ld)                                 */
/* ================================================================== */
#if defined(__GNUC__) && !defined(__TI_ARM__)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 128K
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

ENTRY(Reset_Handler)

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS
{
    .vectors :
    {
        KEEP(*(.vectors))
    } > FLASH

    .text :
    {
        *(.text*)
        *(.rodata*)
    } > FLASH

    .ARM.extab   : { *(.ARM.extab*) } > FLASH
    .ARM.exidx   : { *(.ARM.exidx*) } > FLASH

    _sidata = LOADADDR(.data);
    .data : AT(_sidata)
    {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > SRAM

    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

    . = ALIGN(8);
    _stack_top = ORIGIN(SRAM) + LENGTH(SRAM);
}
#endif
