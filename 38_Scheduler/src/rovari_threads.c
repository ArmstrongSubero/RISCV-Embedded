/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 */

#include <stdint.h>
#include "rovari_threads.h"

extern uint32_t millis(void);

static struct thread main_thread;
static struct thread *active_thread;

static void __attribute__((naked)) ctx_switch(struct thread_ctx *prev,
                                               struct thread_ctx *next)
{
    asm volatile(
        "sw ra,  0(a0)\n"
        "sw sp,  4(a0)\n"
        "sw s0,  8(a0)\n"
        "sw s1, 12(a0)\n"
        "lw ra,  0(a1)\n"
        "lw sp,  4(a1)\n"
        "lw s0,  8(a1)\n"
        "lw s1, 12(a1)\n"
        "ret\n"
    );
}

static void __attribute__((naked)) thread_trampoline(void)
{
    asm volatile(
        "mv a0, s0\n"
        "jr s1\n"
    );
}

static void exit_handler(void)
{
    struct thread *t = active_thread;
    for (int i = 0; i < 32 && t->next != active_thread; i++)
        t = t->next;
    t->next = active_thread->next;
    thread_yield();
    while (1) {}
}

void threads_init(void)
{
    main_thread.next = &main_thread;
    active_thread    = &main_thread;
}

void thread_start(struct thread *t, thread_fn_t fn, void *data,
                  void *stack, uint32_t size)
{
    uint32_t sp_top = (uint32_t)((uint8_t *)stack + size);
    sp_top &= ~0x3u;

    t->ctx.ra  = (uint32_t)thread_trampoline;
    t->ctx.sp  = sp_top;
    t->ctx.s0  = (uint32_t)data;
    t->ctx.s1  = (uint32_t)fn;
    t->next    = active_thread->next;
    active_thread->next = t;
}

void thread_yield(void)
{
    struct thread *next = active_thread->next;
    if (next != active_thread) {
        struct thread *prev = active_thread;
        active_thread = next;
        ctx_switch(&prev->ctx, &next->ctx);
    }
}

void thread_sleep(uint32_t ms)
{
    uint32_t target = millis() + ms;
    while ((int32_t)(target - millis()) > 0) {
        thread_yield();
    }
}