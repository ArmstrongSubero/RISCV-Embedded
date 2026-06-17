/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 */

/**
 * @file rovari_mpu6050.c
 * @brief MPU6050 6-axis IMU driver for CH32V003.
 *
 * Ported from working CH32V003 bare-metal driver.
 * DLPF 42 Hz, 100 Hz sample rate, +/- 2g accel, +/- 250 dps gyro.
 * FIFO disabled, DRDY interrupt polled for coherent reads.
 * 14-byte burst read with frame sanity check and retry.
 * Calibration: 200-sample offset average, Z assumes +1g.
 * Filtering: median-of-3 then IIR (accel alpha=1/8, gyro alpha=1/4).
 *
 * @req REQ-ROVARI-MPU6050-0010
 */

#include <stdint.h>
#include <stdbool.h>
#include "rovari_mpu6050.h"
#include "rovari_i2c.h"

extern uint32_t millis(void);
extern void Delay_Ms(uint32_t);

/* -----------------------------------------------------------------------
 *  Registers
 * ----------------------------------------------------------------------- */
#define REG_SMPLRT_DIV     0x19
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_FIFO_EN        0x23
#define REG_INT_PIN_CFG    0x37
#define REG_INT_ENABLE     0x38
#define REG_INT_STATUS     0x3A
#define REG_ACCEL_XOUT_H   0x3B
#define REG_USER_CTRL      0x6A
#define REG_PWR_MGMT_1     0x6B
#define REG_WHO_AM_I       0x75

#define BUS   I2C_1
#define ADDR  MPU6050_ADDR

/* -----------------------------------------------------------------------
 *  I2C helpers
 * ----------------------------------------------------------------------- */
static void wr8(uint8_t reg, uint8_t val)
{
    i2c_write_reg(BUS, ADDR, reg, val);
}

static uint8_t rd8(uint8_t reg)
{
    return i2c_read_reg(BUS, ADDR, reg);
}

static int rd_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    return i2c_read_buf(BUS, ADDR, reg, buf, len);
}

/* -----------------------------------------------------------------------
 *  Scaling (integer, no floats)
 *  Accel: +/- 2g = 16384 LSB/g -> mg = raw * 1000 / 16384
 *  Gyro:  +/- 250 dps = 131 LSB/dps -> mdps = raw * 1000 / 131
 *  Temp:  C = raw/340 + 36.53 -> centidegrees = raw*100/340 + 3653
 * ----------------------------------------------------------------------- */
static inline int32_t raw_to_mg(int16_t r)
{
    return ((int32_t)r * 1000 + (r >= 0 ? 8192 : -8192)) / 16384;
}

static inline int32_t raw_to_mdps(int16_t r)
{
    return ((int32_t)r * 1000 + (r >= 0 ? 65 : -65)) / 131;
}

static inline int32_t temp_to_c100(int16_t t)
{
    return ((int32_t)t * 100 + (t >= 0 ? 169 : -169)) / 340 + 3653;
}

/* -----------------------------------------------------------------------
 *  Frame sanity check
 * ----------------------------------------------------------------------- */
static int frame_sane(const uint8_t *b, uint8_t len)
{
    uint8_t orv = 0, andv = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        orv |= b[i];
        andv &= b[i];
    }
    if (orv == 0x00 || andv == 0xFF) return 0;
    int16_t traw = (int16_t)((b[6] << 8) | b[7]);
    if (traw < -15000 || traw > 15000) return 0;
    return 1;
}

/* -----------------------------------------------------------------------
 *  DRDY wait (poll INT_STATUS bit 0)
 * ----------------------------------------------------------------------- */
static int wait_drdy(uint32_t ms)
{
    uint32_t t = millis();
    while (millis() - t < ms) {
        uint8_t st = rd8(REG_INT_STATUS);
        if (st & 0x01) return 1;
        Delay_Ms(1);
    }
    return 0;
}

/* -----------------------------------------------------------------------
 *  Burst read with retry
 * ----------------------------------------------------------------------- */
static int read_14_robust(uint8_t *buf)
{
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        rd_burst(REG_ACCEL_XOUT_H, buf, 14);
        if (frame_sane(buf, 14))
            return 1;
        Delay_Ms(1);
    }
    return 0;
}

/* -----------------------------------------------------------------------
 *  Calibration and filter state
 * ----------------------------------------------------------------------- */
static int32_t s_ax_off, s_ay_off, s_az_off;
static int32_t s_gx_off, s_gy_off, s_gz_off;

#define ACC_SHIFT  3  /* alpha = 1/8 */
#define GYR_SHIFT  2  /* alpha = 1/4 */

static int32_t s_ax, s_ay, s_az;
static int32_t s_gx, s_gy, s_gz;
static int32_t s_temp;
static uint8_t s_filter_init;

static int32_t s_ax_hist[2], s_ay_hist[2], s_az_hist[2];
static int32_t s_gx_hist[2], s_gy_hist[2], s_gz_hist[2];

static inline int32_t median3(int32_t a, int32_t b, int32_t c)
{
    if (a > b) { int32_t t = a; a = b; b = t; }
    if (b > c) { int32_t t = b; b = c; c = t; }
    if (a > b) { int32_t t = a; a = b; b = t; }
    return b;
}

static inline int32_t clamp(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* -----------------------------------------------------------------------
 *  Parse raw bytes into struct
 * ----------------------------------------------------------------------- */
static void parse_raw(const uint8_t *b, mpu6050_raw_t *r)
{
    r->ax   = (int16_t)((b[0]  << 8) | b[1]);
    r->ay   = (int16_t)((b[2]  << 8) | b[3]);
    r->az   = (int16_t)((b[4]  << 8) | b[5]);
    r->temp = (int16_t)((b[6]  << 8) | b[7]);
    r->gx   = (int16_t)((b[8]  << 8) | b[9]);
    r->gy   = (int16_t)((b[10] << 8) | b[11]);
    r->gz   = (int16_t)((b[12] << 8) | b[13]);
}

/* -----------------------------------------------------------------------
 *  Filter step: median-of-3 then IIR
 * ----------------------------------------------------------------------- */
static void filter_step(const mpu6050_raw_t *r)
{
    int32_t ax = clamp(raw_to_mg(r->ax)   - s_ax_off, -4000, 4000);
    int32_t ay = clamp(raw_to_mg(r->ay)   - s_ay_off, -4000, 4000);
    int32_t az = clamp(raw_to_mg(r->az)   - s_az_off, -4000, 4000);
    int32_t gx = clamp(raw_to_mdps(r->gx) - s_gx_off, -250000, 250000);
    int32_t gy = clamp(raw_to_mdps(r->gy) - s_gy_off, -250000, 250000);
    int32_t gz = clamp(raw_to_mdps(r->gz) - s_gz_off, -250000, 250000);

    int32_t ax_m = median3(s_ax_hist[0], s_ax_hist[1], ax);
    int32_t ay_m = median3(s_ay_hist[0], s_ay_hist[1], ay);
    int32_t az_m = median3(s_az_hist[0], s_az_hist[1], az);
    int32_t gx_m = median3(s_gx_hist[0], s_gx_hist[1], gx);
    int32_t gy_m = median3(s_gy_hist[0], s_gy_hist[1], gy);
    int32_t gz_m = median3(s_gz_hist[0], s_gz_hist[1], gz);

    s_ax_hist[1] = s_ax_hist[0]; s_ax_hist[0] = ax;
    s_ay_hist[1] = s_ay_hist[0]; s_ay_hist[0] = ay;
    s_az_hist[1] = s_az_hist[0]; s_az_hist[0] = az;
    s_gx_hist[1] = s_gx_hist[0]; s_gx_hist[0] = gx;
    s_gy_hist[1] = s_gy_hist[0]; s_gy_hist[0] = gy;
    s_gz_hist[1] = s_gz_hist[0]; s_gz_hist[0] = gz;

    if (!s_filter_init) {
        s_ax = ax_m; s_ay = ay_m; s_az = az_m;
        s_gx = gx_m; s_gy = gy_m; s_gz = gz_m;
        s_filter_init = 1;
    } else {
        s_ax += (ax_m - s_ax) >> ACC_SHIFT;
        s_ay += (ay_m - s_ay) >> ACC_SHIFT;
        s_az += (az_m - s_az) >> ACC_SHIFT;
        s_gx += (gx_m - s_gx) >> GYR_SHIFT;
        s_gy += (gy_m - s_gy) >> GYR_SHIFT;
        s_gz += (gz_m - s_gz) >> GYR_SHIFT;
    }

    s_temp = temp_to_c100(r->temp);
}

/* -----------------------------------------------------------------------
 *  Public API
 * ----------------------------------------------------------------------- */

int mpu6050_init(void)
{
    uint8_t who = rd8(REG_WHO_AM_I);
    if ((who & 0x7E) != 0x68) return -1;

    /* Wake, PLL on X gyro */
    wr8(REG_PWR_MGMT_1, 0x01);
    Delay_Ms(2);

    /* DLPF 42 Hz */
    wr8(REG_CONFIG, 0x03);

    /* 100 Hz sample rate: 1kHz / (1+9) */
    wr8(REG_SMPLRT_DIV, 9);

    /* +/- 250 dps */
    wr8(REG_GYRO_CONFIG, 0x00);

    /* +/- 2g */
    wr8(REG_ACCEL_CONFIG, 0x00);

    /* Disable FIFO */
    wr8(REG_USER_CTRL, 0x00);
    wr8(REG_FIFO_EN, 0x00);

    /* DRDY interrupt, push-pull active high */
    wr8(REG_INT_PIN_CFG, 0x00);
    wr8(REG_INT_ENABLE, 0x01);

    return 0;
}

void mpu6050_calibrate(void)
{
    int64_t sax = 0, say = 0, saz = 0;
    int64_t sgx = 0, sgy = 0, sgz = 0;
    uint16_t count = 0;

    /* Use narrow DLPF for quieter calibration mean */
    wr8(REG_CONFIG, 0x06);
    Delay_Ms(50);

    for (uint16_t i = 0; i < 200; i++) {
        if (!wait_drdy(20)) continue;
        uint8_t buf[14];
        if (!read_14_robust(buf)) continue;

        mpu6050_raw_t r;
        parse_raw(buf, &r);

        sax += raw_to_mg(r.ax);
        say += raw_to_mg(r.ay);
        saz += raw_to_mg(r.az);
        sgx += raw_to_mdps(r.gx);
        sgy += raw_to_mdps(r.gy);
        sgz += raw_to_mdps(r.gz);
        count++;
        Delay_Ms(5);
    }

    if (count > 0) {
        s_ax_off = (int32_t)(sax / count);
        s_ay_off = (int32_t)(say / count);
        s_az_off = (int32_t)(saz / count) - 1000; /* Z expects +1g flat */
        s_gx_off = (int32_t)(sgx / count);
        s_gy_off = (int32_t)(sgy / count);
        s_gz_off = (int32_t)(sgz / count);
    }

    /* Restore normal DLPF */
    wr8(REG_CONFIG, 0x03);
    Delay_Ms(10);

    /* Seed filter */
    s_ax = 0; s_ay = 0; s_az = 1000;
    s_gx = 0; s_gy = 0; s_gz = 0;
    s_filter_init = 0;

    /* Read a few frames to prime the filter */
    for (uint8_t i = 0; i < 3; i++) {
        if (!wait_drdy(20)) continue;
        uint8_t buf[14];
        if (read_14_robust(buf)) {
            mpu6050_raw_t r;
            parse_raw(buf, &r);
            filter_step(&r);
        }
        Delay_Ms(5);
    }
}

int mpu6050_read_raw(mpu6050_raw_t *raw)
{
    uint8_t buf[14];
    if (!read_14_robust(buf)) return -1;
    parse_raw(buf, raw);
    return 0;
}

int mpu6050_read(mpu6050_data_t *data)
{
    uint8_t buf[14];
    if (!read_14_robust(buf)) return -1;

    mpu6050_raw_t r;
    parse_raw(buf, &r);
    filter_step(&r);

    data->ax   = s_ax;
    data->ay   = s_ay;
    data->az   = s_az;
    data->gx   = s_gx;
    data->gy   = s_gy;
    data->gz   = s_gz;
    data->temp = s_temp;

    return 0;
}
