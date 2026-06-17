/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 */

/**
 * @file rovari_fall.c
 * @brief Fall detection state machine for CH32V003 using MPU6050.
 *
 * Four trigger sequence:
 *   1. Freefall: accel magnitude drops below lower threshold
 *   2. Impact: accel magnitude exceeds upper threshold 
 *   3. Orientation change: gyro magnitude 30..400 dps
 *   4. Stillness: gyro magnitude < 10 dps sustained
 *
 * All integer, no floats, no sqrt (uses squared magnitudes).
 *
 * @req REQ-ROVARI-FALL-0010
 */

#include <stdint.h>
#include "rovari_fall.h"

/* -----------------------------------------------------------------------
 *  State
 * ----------------------------------------------------------------------- */
typedef enum {
    ST_WATCH,
    ST_FREEFALL,
    ST_IMPACT,
    ST_VERIFY,
    ST_ALERT
} fall_state_t;

static fall_state_t s_state;
static uint8_t      s_count;
static uint8_t      s_verify_count;
static uint8_t      s_flag;

/* Runtime thresholds (in mg and mdps) */
static uint16_t s_freefall_mg  = 200;   /* 0.2g */
static uint16_t s_impact_mg    = 1200;  /* 1.2g */
static uint16_t s_orient_lo    = 30000; /* 30 dps in mdps */
static uint16_t s_orient_hi    = 400000;/* 400 dps in mdps (stored as uint32) */
static uint16_t s_still_mdps   = 10000; /* 10 dps in mdps */
static uint8_t  s_timeout      = 6;     /* 0.5s at ~10ms/sample */

/* -----------------------------------------------------------------------
 *  Helpers
 * ----------------------------------------------------------------------- */
static inline int32_t iabs(int32_t v) { return v < 0 ? -v : v; }

static uint32_t isqrt(uint32_t x)
{
    uint32_t op = x, res = 0, one = 1uL << 30;
    while (one > op) one >>= 2;
    while (one) {
        if (op >= res + one) { op -= res + one; res = (res >> 1) + one; }
        else res >>= 1;
        one >>= 2;
    }
    return res;
}

/** Accel magnitude in mg */
static uint32_t accel_mag(const mpu6050_data_t *d)
{
    int64_t a2 = (int64_t)d->ax * d->ax
               + (int64_t)d->ay * d->ay
               + (int64_t)d->az * d->az;
    return isqrt((uint32_t)a2);
}

/** Gyro magnitude in mdps */
static uint32_t gyro_mag(const mpu6050_data_t *d)
{
    int64_t g2 = (int64_t)d->gx * d->gx
               + (int64_t)d->gy * d->gy
               + (int64_t)d->gz * d->gz;
    return isqrt((uint32_t)g2);
}

/* -----------------------------------------------------------------------
 *  Public API
 * ----------------------------------------------------------------------- */

void fall_init(void)
{
    s_state = ST_WATCH;
    s_count = 0;
    s_verify_count = 0;
    s_flag = 0;
}

void fall_set_thresholds(uint16_t impact_mg, uint16_t freefall_mg)
{
    s_impact_mg   = impact_mg;
    s_freefall_mg = freefall_mg;
}

void fall_update(const mpu6050_data_t *d)
{
    uint32_t am = accel_mag(d);
    uint32_t gm = gyro_mag(d);

    switch (s_state) {

    case ST_WATCH:
        /* Trigger 1: accel drops below freefall threshold */
        if (am <= s_freefall_mg) {
            s_state = ST_FREEFALL;
            s_count = 0;
        }
        break;

    case ST_FREEFALL:
        s_count++;
        /* Trigger 2: accel exceeds impact threshold */
        if (am >= s_impact_mg) {
            s_state = ST_IMPACT;
            s_count = 0;
        }
        /* Timeout: no impact within window */
        else if (s_count >= s_timeout) {
            s_state = ST_WATCH;
        }
        break;

    case ST_IMPACT:
        s_count++;
        /* Trigger 3: gyro shows orientation change (30..400 dps) */
        if (gm >= s_orient_lo && gm <= s_orient_hi) {
            s_state = ST_VERIFY;
            s_verify_count = 0;
        }
        /* Timeout: no orientation change */
        else if (s_count >= s_timeout) {
            s_state = ST_WATCH;
        }
        break;

    case ST_VERIFY:
        s_verify_count++;
        if (s_verify_count >= 10) {
            /* Check if person is now still (gyro < 10 dps) */
            if (gm <= s_still_mdps) {
                s_flag = 1;
                s_state = ST_ALERT;
            } else {
                /* Person regained balance */
                s_state = ST_WATCH;
            }
        }
        break;

    case ST_ALERT:
        break;
    }
}

uint8_t fall_detected(void)
{
    if (s_flag) {
        s_flag = 0;
        s_state = ST_WATCH;
        return 1;
    }
    return 0;
}

uint8_t fall_get_state(void)
{
    return (uint8_t)s_state;
}

const char* fall_state_str(uint8_t state)
{
    switch (state) {
        case ST_WATCH:    return "WATCH";
        case ST_FREEFALL: return "FREEFALL";
        case ST_IMPACT:   return "IMPACT";
        case ST_VERIFY:   return "VERIFY";
        case ST_ALERT:    return "ALERT";
        default:          return "?";
    }
}

void fall_reset(void)
{
    fall_init();
}