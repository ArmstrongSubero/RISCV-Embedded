/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 */

/**
 * @file rovari_ahrs.c
 * @brief Roll/Pitch/Yaw orientation from MPU6050 for CH32V003.
 *
 * Ported from working CH32V003 bare-metal roll/pitch/yaw driver.
 * All integer math, no libm, no floats.
 *
 * Roll/pitch: accelerometer via Q15 polynomial atan2.
 * Yaw: gyro Z integration with adaptive stationary drift zeroing.
 *
 * @req REQ-ROVARI-AHRS-0010
 */

#include <stdint.h>
#include "rovari_ahrs.h"
#include "rovari_mpu6050.h"

extern uint32_t millis(void);

/* -----------------------------------------------------------------------
 *  Fixed point helpers
 * ----------------------------------------------------------------------- */
#define Q15_ONE  32768

static inline int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }

/** Bitwise integer square root */
static uint32_t isqrt(uint32_t x)
{
    uint32_t op = x, res = 0, one = 1uL << 30;
    while (one > op) one >>= 2;
    while (one) {
        if (op >= res + one) {
            op -= res + one;
            res = (res >> 1) + one;
        } else {
            res >>= 1;
        }
        one >>= 2;
    }
    return res;
}

/**
 * atan(z) approximation: 45*z + 15.64*z*(1-|z|)
 * z in Q15, result in centidegrees.
 */
static int32_t atan_cdeg_q15(int32_t z)
{
    int32_t az    = z < 0 ? -z : z;
    int32_t term1 = ((int64_t)4500 * z) >> 15;
    int32_t t     = ((int64_t)z * (Q15_ONE - az)) >> 15;
    int32_t term2 = ((int64_t)1564 * t) >> 15;
    return term1 + term2;
}

/** atan2(y, x) in centidegrees, full quadrant correction */
static int32_t atan2_cdeg(int32_t y, int32_t x)
{
    if (x == 0) {
        if (y > 0) return 9000;
        if (y < 0) return -9000;
        return 0;
    }

    int32_t ax = iabs32(x), ay = iabs32(y);
    int32_t a;

    if (ax >= ay) {
        int32_t z = (int32_t)(((int64_t)y << 15) / ax);
        a = atan_cdeg_q15(z);
    } else {
        int32_t z_abs = (int32_t)(((int64_t)ax << 15) / ay);
        int32_t a1 = atan_cdeg_q15(z_abs);
        int32_t signr = ((x ^ y) < 0) ? -1 : 1;
        a = signr * 9000 - a1;
    }

    if (x > 0) return a;
    return a + ((y >= 0) ? 18000 : -18000);
}

static int32_t wrap18000(int32_t a)
{
    while (a <= -18000) a += 36000;
    while (a > 18000)   a -= 36000;
    return a;
}

/* -----------------------------------------------------------------------
 *  Yaw zero manager (adaptive drift cancellation)
 * ----------------------------------------------------------------------- */
#define STILL_ACC_TOL     60     /* mg window around 1g */
#define STILL_GYRO_TOL    800    /* mdps per axis */
#define STILL_MIN_COUNT   50     /* ~0.5s at 100 Hz */
#define ZERO_MIN_MS       2000
#define ZERO_PERIOD_MS    500
#define BIAS_GAIN_NUM     1
#define BIAS_GAIN_DEN     2
#define DRIFT_CLAMP       5000   /* mdps */
#define DT_MS             10

static int32_t  s_bias_gz;

static struct {
    uint8_t  stationary;
    uint16_t count;
    uint32_t enter_ms;
    int32_t  yaw_start;
    uint32_t last_corr_ms;
} s_yz;

static uint8_t is_still(const mpu6050_data_t *d)
{
    int64_t a2 = (int64_t)d->ax * d->ax
               + (int64_t)d->ay * d->ay
               + (int64_t)d->az * d->az;
    int32_t lo = 1000 - STILL_ACC_TOL;
    int32_t hi = 1000 + STILL_ACC_TOL;
    if (a2 < (int64_t)lo * lo || a2 > (int64_t)hi * hi) return 0;
    if (iabs32(d->gx) > STILL_GYRO_TOL) return 0;
    if (iabs32(d->gy) > STILL_GYRO_TOL) return 0;
    if (iabs32(d->gz) > STILL_GYRO_TOL) return 0;
    return 1;
}

static void yaw_zero_step(uint32_t now, int32_t yaw, const mpu6050_data_t *d)
{
    if (is_still(d)) {
        if (s_yz.count < 0xFFFF) s_yz.count++;
    } else {
        s_yz.count = 0;
        s_yz.stationary = 0;
    }

    if (!s_yz.stationary && s_yz.count >= STILL_MIN_COUNT) {
        s_yz.stationary   = 1;
        s_yz.enter_ms     = now;
        s_yz.yaw_start    = yaw;
        s_yz.last_corr_ms = now;
    }

    if (s_yz.stationary) {
        uint32_t elapsed = now - s_yz.enter_ms;
        if (elapsed >= ZERO_MIN_MS &&
            (now - s_yz.last_corr_ms) >= ZERO_PERIOD_MS)
        {
            int32_t dpsi = wrap18000(yaw - s_yz.yaw_start);
            int32_t drift = (int32_t)(((int64_t)dpsi * 10000) / (int32_t)elapsed);
            if (drift >  DRIFT_CLAMP) drift =  DRIFT_CLAMP;
            if (drift < -DRIFT_CLAMP) drift = -DRIFT_CLAMP;
            s_bias_gz += (drift * BIAS_GAIN_NUM) / BIAS_GAIN_DEN;
            s_yz.last_corr_ms = now;
            s_yz.enter_ms     = now;
            s_yz.yaw_start    = yaw;
        }
    }
}

/* -----------------------------------------------------------------------
 *  Module state
 * ----------------------------------------------------------------------- */
static int32_t s_yaw;
static uint8_t s_level_latched;

/* -----------------------------------------------------------------------
 *  Public API
 * ----------------------------------------------------------------------- */

void ahrs_init(void)
{
    s_yaw = 0;
    s_bias_gz = 0;
    s_level_latched = 0;
    s_yz.stationary = 0;
    s_yz.count = 0;
}

int ahrs_update(ahrs_angles_t *angles)
{
    mpu6050_data_t d;
    if (mpu6050_read(&d) != 0) return -1;

    uint32_t now = millis();

    /* Roll = atan2(Ay, Az) */
    angles->roll = atan2_cdeg(d.ay, d.az);

    /* Pitch = atan2(-Ax, sqrt(Ay^2 + Az^2)) */
    uint32_t ay2 = (uint32_t)((int64_t)d.ay * d.ay);
    uint32_t az2 = (uint32_t)((int64_t)d.az * d.az);
    uint32_t den = isqrt(ay2 + az2);
    if (den == 0) den = 1;
    angles->pitch = atan2_cdeg(-d.ax, (int32_t)den);

    /* Yaw = integrate gyro Z with bias correction */
    int32_t gz_corrected = d.gz - s_bias_gz;
    int32_t yaw_step = (gz_corrected * DT_MS) / 10000;
    s_yaw = wrap18000(s_yaw + yaw_step);
    angles->yaw = s_yaw;

    /* Drift zeroing */
    yaw_zero_step(now, s_yaw, &d);

    /* Level detection with hysteresis */
    int32_t band_in  = AHRS_LEVEL_BAND;
    int32_t band_out = AHRS_LEVEL_BAND + AHRS_LEVEL_HYST;
    uint8_t inside   = (iabs32(angles->roll) <= band_in) &&
                       (iabs32(angles->pitch) <= band_in);
    uint8_t outside  = (iabs32(angles->roll) > band_out) ||
                       (iabs32(angles->pitch) > band_out);
    if (!s_level_latched && inside)  s_level_latched = 1;
    if (s_level_latched && outside)  s_level_latched = 0;

    return 0;
}

uint8_t ahrs_is_level(void)
{
    return s_level_latched;
}

void ahrs_reset_yaw(void)
{
    s_yaw = 0;
    s_bias_gz = 0;
}
