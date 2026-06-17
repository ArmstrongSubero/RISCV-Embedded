/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 *
 * rovari_threads.h
 * Cooperative multitasking for CH32V003
 */

#ifndef ROVARI_THREADS_H
#define ROVARI_THREADS_H

#include <stdint.h>

typedef void     (*thread_fn_t)(void *data);

struct thread_ctx
{
    uint32_t ra;
    uint32_t sp;
    uint32_t s0;
    uint32_t s1;
};

struct thread
{
    struct thread_ctx ctx;
    struct thread    *next;
};

#ifdef __cplusplus
extern "C" {
#endif

void threads_init(void);
void thread_start(struct thread *t, thread_fn_t fn, void *data,
                  void *stack, uint32_t size);
void thread_yield(void);
void thread_sleep(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif