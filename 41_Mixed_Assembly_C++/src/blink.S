/*
 * Copyright (c) 2026 Armstrong Subero
 *
 * Assembly LED functions for mixed C/Assembly example.
 * Platform: CH32V003 / RV32EC
 * Hardware: LED on PC0
 */

.globl asm_blink_init
.globl asm_blink_toggle

.equ GPIOC_CFGLR,  0x40011000
.equ GPIOC_OUTDR,  0x4001100C
.equ RCC_APB2PCENR, 0x40021018
.equ IOPCEN,       (1 << 4)
.equ PC0_BIT,      (1 << 0)
.equ PC0_MASK,     0x0000000F
.equ PC0_PP_30MHZ, 0x00000003

.text

asm_blink_init:
    /* Enable GPIOC clock */
    li      t0, RCC_APB2PCENR
    lw      t1, 0(t0)
    li      t2, IOPCEN
    or      t1, t1, t2
    sw      t1, 0(t0)

    /* PC0 push-pull output 30 MHz */
    li      t0, GPIOC_CFGLR
    lw      t1, 0(t0)
    li      t2, PC0_MASK
    not     t2, t2
    and     t1, t1, t2
    li      t2, PC0_PP_30MHZ
    or      t1, t1, t2
    sw      t1, 0(t0)
    ret

asm_blink_toggle:
    li      t0, GPIOC_OUTDR
    lw      t1, 0(t0)
    li      t2, PC0_BIT
    xor     t1, t1, t2
    sw      t1, 0(t0)
    ret