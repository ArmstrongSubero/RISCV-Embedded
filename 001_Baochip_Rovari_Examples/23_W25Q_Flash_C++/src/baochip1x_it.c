/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Rovari - rvembedded.com
 *
 * baochip1x_it.c - Interrupt/exception dispatch for user code.
 *
 * The Baochip-1x uses a single trap entry point (crt0.S _trap_entry)
 * that saves all registers and calls trap_dispatch().
 *
 * Override trap_dispatch() here to handle interrupts and exceptions.
 * The default (in bao_stdlib.c, weak) lights PB1 on exception and halts.
 *
 * mcause bit 31 = 1: interrupt, bits [30:0] = interrupt number
 * mcause bit 31 = 0: exception, bits [30:0] = exception code
 *
 * Common interrupt numbers:
 *   20 = TickTimer
 *   30 = Timer0
 */

#include "bao/platform.h"
#include "hardware/gpio.h"

/* Uncomment and customize to handle interrupts:
 *
 * void trap_dispatch(void)
 * {
 *     uint32_t mcause;
 *     __asm__ volatile ("csrr %0, mcause" : "=r"(mcause));
 *
 *     if (mcause & 0x80000000) {
 *         uint32_t irq = mcause & 0x7FFFFFFF;
 *         switch (irq) {
 *         case 20:
 *             // TickTimer interrupt
 *             break;
 *         case 30:
 *             // Timer0 interrupt
 *             break;
 *         default:
 *             break;
 *         }
 *     } else {
 *         // Exception: halt
 *         for (;;) { }
 *     }
 * }
 */
