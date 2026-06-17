/*
 * Copyright (c) 2026 Armstrong Subero
 *
 * Program:  Assembly Blink
 * Platform: CH32V003 / RV32EC @ 48 MHz
 * Hardware: LED on PC1
 *
 * All program logic in assembly. Called from C via
 * app_init -> asm_main_init and app_run -> asm_main_run.
 * SDK handles startup, clocks, vector table.
 */

.globl asm_main_init
.globl asm_main_run

/* Register addresses */
.equ RCC_APB2PCENR, 0x40021018
.equ GPIOC_CFGLR,   0x40011000
.equ GPIOC_OUTDR,   0x4001100C
.equ IOPCEN,        (1 << 4)
.equ PC1_BIT,       (1 << 1)
.equ PC1_MASK,      0x000000F0
.equ PC1_PP_30MHZ,  0x00000030

/* Delay count: ~500ms at 48 MHz
 * 3 cycles per loop iteration: 48000000 / 3 / 2 = 8000000 */
.equ DELAY_COUNT,    8000000

.text

/* ---------------------------------------------------------------
 *  asm_main_init
 *  Enable GPIOC clock, configure PC1 as push-pull output
 * --------------------------------------------------------------- */
asm_main_init:
    /* Enable GPIOC clock */
    li      t0, RCC_APB2PCENR
    lw      t1, 0(t0)
    li      t2, IOPCEN
    or      t1, t1, t2
    sw      t1, 0(t0)

    /* PC1 = push-pull output, 30 MHz */
    li      t0, GPIOC_CFGLR
    lw      t1, 0(t0)
    li      t2, PC1_MASK
    not     t2, t2
    and     t1, t1, t2
    li      t2, PC1_PP_30MHZ
    or      t1, t1, t2
    sw      t1, 0(t0)

    ret

/* ---------------------------------------------------------------
 *  asm_main_run
 *  Toggle PC1, delay ~500ms
 * --------------------------------------------------------------- */
asm_main_run:
    /* Toggle PC1 */
    li      t0, GPIOC_OUTDR
    lw      t1, 0(t0)
    li      t2, PC1_BIT
    xor     t1, t1, t2
    sw      t1, 0(t0)

    /* Spin delay */
    li      t0, DELAY_COUNT
1:
    addi    t0, t0, -1
    bnez    t0, 1b

    ret