/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 */

/**
 * @file rovari_vl53l0x.c
 * @brief VL53L0X time-of-flight distance sensor driver for CH32V003.
 *
 * Full init with SPAD calibration, tuning table, timing budget,
 * and reference calibration. Ported from working PIC16F1718 driver.
 *
 * @req REQ-ROVARI-VL53L0X-0010
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "rovari_vl53l0x.h"
#include "rovari_i2c.h"

extern uint32_t millis(void);

/* -----------------------------------------------------------------------
 *  Registers
 * ----------------------------------------------------------------------- */
#define SYSRANGE_START                       0x00
#define SYSTEM_SEQUENCE_CONFIG               0x01
#define SYSTEM_INTERMEASUREMENT_PERIOD       0x04
#define SYSTEM_INTERRUPT_CONFIG_GPIO         0x0A
#define SYSTEM_INTERRUPT_CLEAR               0x0B
#define RESULT_INTERRUPT_STATUS              0x13
#define RESULT_RANGE_STATUS                  0x14
#define MSRC_CONFIG_CONTROL                  0x60
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE    0x44
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH  0x48
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW   0x47
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH    0x57
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW     0x56
#define PRE_RANGE_CONFIG_VCSEL_PERIOD        0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI   0x51
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD      0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71
#define MSRC_CONFIG_TIMEOUT_MACROP           0x46
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0     0xB0
#define GLOBAL_CONFIG_REF_EN_START_SELECT    0xB6
#define GLOBAL_CONFIG_VCSEL_WIDTH            0x32
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD  0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET     0x4F
#define VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV     0x89
#define GPIO_HV_MUX_ACTIVE_HIGH              0x84
#define OSC_CALIBRATE_VAL                    0xF8
#define ALGO_PHASECAL_LIM                    0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT         0x30

/* -----------------------------------------------------------------------
 *  Macros
 * ----------------------------------------------------------------------- */
#define decodeVcselPeriod(r)      (((r) + 1) << 1)
#define encodeVcselPeriod(p)      (((p) >> 1) - 1)
#define calcMacroPeriod(v) \
    ((((uint32_t)2304 * (v) * 1655) + 500) / 1000)

/* -----------------------------------------------------------------------
 *  Internal state
 * ----------------------------------------------------------------------- */
#define BUS   I2C_1
#define ADDR  VL53L0X_ADDR_DEFAULT

static uint8_t  s_stop_var;
static uint32_t s_timing_budget_us;

typedef struct {
    bool tcc, msrc, dss, pre_range, final_range;
} seq_en_t;

typedef struct {
    uint16_t pre_range_vcsel_pclks, final_range_vcsel_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} seq_to_t;

/* -----------------------------------------------------------------------
 *  I2C helpers using proper multi-byte transactions
 * ----------------------------------------------------------------------- */
static void wr8(uint8_t reg, uint8_t val)
{
    i2c_write_reg(BUS, ADDR, reg, val);
}

static uint8_t rd8(uint8_t reg)
{
    return i2c_read_reg(BUS, ADDR, reg);
}

static void wr16(uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    i2c_write_buf(BUS, ADDR, reg, buf, 2);
}

static uint16_t rd16(uint8_t reg)
{
    uint8_t buf[2] = {0};
    i2c_read_buf(BUS, ADDR, reg, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static void wr32(uint8_t reg, uint32_t val)
{
    uint8_t buf[4] = {
        (uint8_t)(val >> 24), (uint8_t)(val >> 16),
        (uint8_t)(val >> 8),  (uint8_t)(val)
    };
    i2c_write_buf(BUS, ADDR, reg, buf, 4);
}

static void rd_multi(uint8_t reg, uint8_t *dst, uint8_t len)
{
    i2c_read_buf(BUS, ADDR, reg, dst, len);
}

static void wr_multi(uint8_t reg, const uint8_t *src, uint8_t len)
{
    i2c_write_buf(BUS, ADDR, reg, src, len);
}

/* -----------------------------------------------------------------------
 *  Timeout encode / decode / convert
 * ----------------------------------------------------------------------- */
static uint16_t decode_timeout(uint16_t rv)
{
    return (uint16_t)((rv & 0x00FF) << (uint16_t)((rv & 0xFF00) >> 8)) + 1;
}

static uint16_t encode_timeout(uint16_t mclks)
{
    uint32_t ls = 0;
    uint16_t ms = 0;
    if (mclks > 0) {
        ls = mclks - 1;
        while ((ls & 0xFFFFFF00) > 0) { ls >>= 1; ms++; }
        return (ms << 8) | (ls & 0xFF);
    }
    return 0;
}

static uint32_t mclks_to_us(uint16_t mclks, uint8_t vcsel)
{
    uint32_t mp = calcMacroPeriod(vcsel);
    return ((mclks * mp) + (mp / 2)) / 1000;
}

static uint32_t us_to_mclks(uint32_t us, uint8_t vcsel)
{
    uint32_t mp = calcMacroPeriod(vcsel);
    return (((us * 1000) + (mp / 2)) / mp);
}

/* -----------------------------------------------------------------------
 *  Sequence step enables and timeouts
 * ----------------------------------------------------------------------- */
static void get_seq_enables(seq_en_t *en)
{
    uint8_t sc = rd8(SYSTEM_SEQUENCE_CONFIG);
    en->tcc         = (sc >> 4) & 1;
    en->dss         = (sc >> 3) & 1;
    en->msrc        = (sc >> 2) & 1;
    en->pre_range   = (sc >> 6) & 1;
    en->final_range = (sc >> 7) & 1;
}

static uint8_t get_vcsel(uint8_t type)
{
    if (type == 0) return decodeVcselPeriod(rd8(PRE_RANGE_CONFIG_VCSEL_PERIOD));
    if (type == 1) return decodeVcselPeriod(rd8(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    return 255;
}

static void get_seq_timeouts(seq_en_t *en, seq_to_t *to)
{
    to->pre_range_vcsel_pclks   = get_vcsel(0);
    to->msrc_dss_tcc_mclks     = rd8(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    to->msrc_dss_tcc_us        = mclks_to_us(to->msrc_dss_tcc_mclks,
                                              to->pre_range_vcsel_pclks);
    to->pre_range_mclks        = decode_timeout(rd16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    to->pre_range_us           = mclks_to_us(to->pre_range_mclks,
                                              to->pre_range_vcsel_pclks);
    to->final_range_vcsel_pclks = get_vcsel(1);
    to->final_range_mclks      = decode_timeout(rd16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    if (en->pre_range) to->final_range_mclks -= to->pre_range_mclks;
    to->final_range_us         = mclks_to_us(to->final_range_mclks,
                                              to->final_range_vcsel_pclks);
}

/* -----------------------------------------------------------------------
 *  Timing budget
 * ----------------------------------------------------------------------- */
static uint32_t get_timing_budget(void)
{
    seq_en_t en; seq_to_t to;
    uint32_t b = 1910 + 960;
    get_seq_enables(&en);
    get_seq_timeouts(&en, &to);
    if (en.tcc)        b += to.msrc_dss_tcc_us + 590;
    if (en.dss)        b += 2 * (to.msrc_dss_tcc_us + 690);
    else if (en.msrc)  b += to.msrc_dss_tcc_us + 660;
    if (en.pre_range)  b += to.pre_range_us + 660;
    if (en.final_range) b += to.final_range_us + 550;
    s_timing_budget_us = b;
    return b;
}

static bool set_timing_budget(uint32_t budget_us)
{
    if (budget_us < 20000) return false;
    seq_en_t en; seq_to_t to;
    uint32_t used = 1320 + 960;
    get_seq_enables(&en);
    get_seq_timeouts(&en, &to);
    if (en.tcc)        used += to.msrc_dss_tcc_us + 590;
    if (en.dss)        used += 2 * (to.msrc_dss_tcc_us + 690);
    else if (en.msrc)  used += to.msrc_dss_tcc_us + 660;
    if (en.pre_range)  used += to.pre_range_us + 660;
    if (en.final_range) {
        used += 550;
        if (used > budget_us) return false;
        uint32_t fr_us = budget_us - used;
        uint16_t fr_mclks = us_to_mclks(fr_us, to.final_range_vcsel_pclks);
        if (en.pre_range) fr_mclks += to.pre_range_mclks;
        wr16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encode_timeout(fr_mclks));
        s_timing_budget_us = budget_us;
    }
    return true;
}

/* -----------------------------------------------------------------------
 *  VCSEL pulse period
 * ----------------------------------------------------------------------- */
static bool set_vcsel_period(uint8_t type, uint8_t period_pclks)
{
    uint8_t vreg = encodeVcselPeriod(period_pclks);
    seq_en_t en; seq_to_t to;
    get_seq_enables(&en);
    get_seq_timeouts(&en, &to);

    if (type == 0) { /* PreRange */
        switch (period_pclks) {
            case 12: wr8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
            case 14: wr8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
            case 16: wr8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
            case 18: wr8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
            default: return false;
        }
        wr8(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
        wr8(PRE_RANGE_CONFIG_VCSEL_PERIOD, vreg);
        uint16_t new_pr = us_to_mclks(to.pre_range_us, period_pclks);
        wr16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encode_timeout(new_pr));
        uint16_t new_msrc = us_to_mclks(to.msrc_dss_tcc_us, period_pclks);
        wr8(MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc > 256) ? 255 : (new_msrc - 1));
    } else if (type == 1) { /* FinalRange */
        switch (period_pclks) {
            case 8:
                wr8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                wr8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                wr8(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                wr8(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
                wr8(0xFF, 0x01); wr8(ALGO_PHASECAL_LIM, 0x30); wr8(0xFF, 0x00);
                break;
            case 10:
                wr8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                wr8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                wr8(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                wr8(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
                wr8(0xFF, 0x01); wr8(ALGO_PHASECAL_LIM, 0x20); wr8(0xFF, 0x00);
                break;
            case 12:
                wr8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                wr8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                wr8(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                wr8(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
                wr8(0xFF, 0x01); wr8(ALGO_PHASECAL_LIM, 0x20); wr8(0xFF, 0x00);
                break;
            case 14:
                wr8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
                wr8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                wr8(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                wr8(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
                wr8(0xFF, 0x01); wr8(ALGO_PHASECAL_LIM, 0x20); wr8(0xFF, 0x00);
                break;
            default: return false;
        }
        wr8(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vreg);
        uint16_t new_fr = us_to_mclks(to.final_range_us, period_pclks);
        if (en.pre_range) new_fr += to.pre_range_mclks;
        wr16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encode_timeout(new_fr));
    } else {
        return false;
    }

    set_timing_budget(s_timing_budget_us);

    /* Phase recalibration */
    uint8_t seq = rd8(SYSTEM_SEQUENCE_CONFIG);
    wr8(SYSTEM_SEQUENCE_CONFIG, 0x02);
    wr8(SYSRANGE_START, 0x01);
    uint32_t t = millis();
    while ((rd8(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (millis() - t > VL53L0X_TIMEOUT_MS) break;
    }
    wr8(SYSTEM_INTERRUPT_CLEAR, 0x01);
    wr8(SYSRANGE_START, 0x00);
    wr8(SYSTEM_SEQUENCE_CONFIG, seq);
    return true;
}

/* -----------------------------------------------------------------------
 *  SPAD info
 * ----------------------------------------------------------------------- */
static bool get_spad_info(uint8_t *count, bool *is_aperture)
{
    wr8(0x80, 0x01);
    wr8(0xFF, 0x01);
    wr8(0x00, 0x00);
    wr8(0xFF, 0x06);
    wr8(0x83, rd8(0x83) | 0x04);
    wr8(0xFF, 0x07);
    wr8(0x81, 0x01);
    wr8(0x80, 0x01);
    wr8(0x94, 0x6B);
    wr8(0x83, 0x00);

    uint32_t t = millis();
    while (rd8(0x83) == 0x00) {
        if (millis() - t > VL53L0X_TIMEOUT_MS) return false;
    }

    wr8(0x83, 0x01);
    uint8_t tmp = rd8(0x92);
    *count = tmp & 0x7F;
    *is_aperture = (tmp >> 7) & 0x01;

    wr8(0x81, 0x00);
    wr8(0xFF, 0x06);
    wr8(0x83, rd8(0x83) & ~0x04);
    wr8(0xFF, 0x01);
    wr8(0x00, 0x01);
    wr8(0xFF, 0x00);
    wr8(0x80, 0x00);
    return true;
}

/* -----------------------------------------------------------------------
 *  Reference calibration
 * ----------------------------------------------------------------------- */
static bool ref_cal(uint8_t vhv_init)
{
    wr8(SYSRANGE_START, 0x01 | vhv_init);

    uint32_t t = millis();
    while ((rd8(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (millis() - t > VL53L0X_TIMEOUT_MS) return false;
    }

    wr8(SYSTEM_INTERRUPT_CLEAR, 0x01);
    wr8(SYSRANGE_START, 0x00);
    return true;
}

/* -----------------------------------------------------------------------
 *  Tuning table
 * ----------------------------------------------------------------------- */
static const uint8_t tuning[] = {
    0xFF,0x01, 0x00,0x00,
    0xFF,0x00, 0x09,0x00, 0x10,0x00, 0x11,0x00,
    0x24,0x01, 0x25,0xFF, 0x75,0x00,
    0xFF,0x01, 0x4E,0x2C, 0x48,0x00, 0x30,0x20,
    0xFF,0x00, 0x30,0x09, 0x54,0x00, 0x31,0x04,
    0x32,0x03, 0x40,0x83, 0x46,0x25, 0x60,0x00,
    0x27,0x00, 0x50,0x06, 0x51,0x00, 0x52,0x96,
    0x56,0x08, 0x57,0x30, 0x61,0x00, 0x62,0x00,
    0x64,0x00, 0x65,0x00, 0x66,0xA0,
    0xFF,0x01, 0x22,0x32, 0x47,0x14, 0x49,0xFF,
    0x4A,0x00,
    0xFF,0x00, 0x7A,0x0A, 0x7B,0x00, 0x78,0x21,
    0xFF,0x01, 0x23,0x34, 0x42,0x00, 0x44,0xFF,
    0x45,0x26, 0x46,0x05, 0x40,0x40, 0x0E,0x06,
    0x20,0x1A, 0x43,0x40,
    0xFF,0x00, 0x34,0x03, 0x35,0x44,
    0xFF,0x01, 0x31,0x04, 0x4B,0x09, 0x4C,0x05,
    0x4D,0x04,
    0xFF,0x00, 0x44,0x00, 0x45,0x20, 0x47,0x08,
    0x48,0x28, 0x67,0x00, 0x70,0x04, 0x71,0x01,
    0x72,0xFE, 0x76,0x00, 0x77,0x00,
    0xFF,0x01, 0x0D,0x01,
    0xFF,0x00, 0x80,0x01, 0x01,0xF8,
    0xFF,0x01, 0x8E,0x01, 0x00,0x01,
    0xFF,0x00, 0x80,0x00,
};

static void load_tuning(void)
{
    for (uint16_t i = 0; i < sizeof(tuning); i += 2) {
        wr8(tuning[i], tuning[i + 1]);
    }
}

/* -----------------------------------------------------------------------
 *  Public API: init
 * ----------------------------------------------------------------------- */
int vl53l0x_init(Vl53l0xMode mode)
{
    /* Data init */
    wr8(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
        rd8(VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV) | 0x01);
    wr8(0x88, 0x00);

    wr8(0x80, 0x01);
    wr8(0xFF, 0x01);
    wr8(0x00, 0x00);
    s_stop_var = rd8(0x91);
    wr8(0x00, 0x01);
    wr8(0xFF, 0x00);
    wr8(0x80, 0x00);

    wr8(MSRC_CONFIG_CONTROL, rd8(MSRC_CONFIG_CONTROL) | 0x12);
    wr16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE, 32);
    wr8(SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* SPAD management */
    uint8_t spad_count;
    bool spad_aperture;
    if (!get_spad_info(&spad_count, &spad_aperture)) return -1;

    uint8_t spad_map[6];
    rd_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, 6);

    wr8(0xFF, 0x01);
    wr8(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    wr8(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    wr8(0xFF, 0x00);
    wr8(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first = spad_aperture ? 12 : 0;
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first || enabled == spad_count)
            spad_map[i / 8] &= ~(1 << (i % 8));
        else if ((spad_map[i / 8] >> (i % 8)) & 0x01)
            enabled++;
    }
    wr_multi(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, 6);

    /* Tuning */
    load_tuning();

    /* GPIO interrupt */
    wr8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    wr8(GPIO_HV_MUX_ACTIVE_HIGH, rd8(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
    wr8(SYSTEM_INTERRUPT_CLEAR, 0x01);

    /* Timing budget */
    s_timing_budget_us = get_timing_budget();
    wr8(SYSTEM_SEQUENCE_CONFIG, 0xE8);
    set_timing_budget(s_timing_budget_us);

    /* Reference calibration: VHV then phase */
    wr8(SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!ref_cal(0x40)) return -1;
    wr8(SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!ref_cal(0x00)) return -1;
    wr8(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* Apply mode */
    if (mode == VL53L0X_LONG_RANGE) {
        wr16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE, 13);
        set_vcsel_period(0, 18);
        set_vcsel_period(1, 14);
    } else if (mode == VL53L0X_HIGH_SPEED) {
        set_timing_budget(20000);
    } else if (mode == VL53L0X_HIGH_ACCURACY) {
        set_timing_budget(200000);
    }

    return 0;
}

/* -----------------------------------------------------------------------
 *  Public API: single-shot read
 * ----------------------------------------------------------------------- */
uint16_t vl53l0x_read_mm(void)
{
    wr8(0x80, 0x01);
    wr8(0xFF, 0x01);
    wr8(0x00, 0x00);
    wr8(0x91, s_stop_var);
    wr8(0x00, 0x01);
    wr8(0xFF, 0x00);
    wr8(0x80, 0x00);
    wr8(SYSRANGE_START, 0x01);

    /* @sevs-bound: wait for start bit clear */
    uint32_t t = millis();
    while (rd8(SYSRANGE_START) & 0x01) {
        if (millis() - t > VL53L0X_TIMEOUT_MS) return 0;
    }

    /* @sevs-bound: wait for data ready */
    t = millis();
    while ((rd8(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (millis() - t > VL53L0X_TIMEOUT_MS) return 0;
    }

    /* Read range and status in one transaction */
    uint8_t buf[12];
    rd_multi(RESULT_RANGE_STATUS, buf, 12);

    wr8(SYSTEM_INTERRUPT_CLEAR, 0x01);

    /* Check range status (upper nibble of byte 0) */
    uint8_t range_status = (buf[0] & 0xF8) >> 3;
    uint16_t range = ((uint16_t)buf[10] << 8) | buf[11];

    if (range_status == 11 && range < 8190) {
        return range;
    }

    return 0;
}

/* -----------------------------------------------------------------------
 *  Public API: continuous ranging
 * ----------------------------------------------------------------------- */
void vl53l0x_start_continuous(uint32_t period_ms)
{
    wr8(0x80, 0x01);
    wr8(0xFF, 0x01);
    wr8(0x00, 0x00);
    wr8(0x91, s_stop_var);
    wr8(0x00, 0x01);
    wr8(0xFF, 0x00);
    wr8(0x80, 0x00);

    if (period_ms != 0) {
        uint16_t osc = rd16(OSC_CALIBRATE_VAL);
        if (osc != 0) period_ms *= osc;
        wr32(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
        wr8(SYSRANGE_START, 0x04);
    } else {
        wr8(SYSRANGE_START, 0x02);
    }
}

uint16_t vl53l0x_read_continuous_mm(void)
{
    uint32_t t = millis();
    while ((rd8(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (millis() - t > VL53L0X_TIMEOUT_MS) return 0;
    }

    uint8_t buf[12];
    rd_multi(RESULT_RANGE_STATUS, buf, 12);

    wr8(SYSTEM_INTERRUPT_CLEAR, 0x01);

    uint8_t range_status = (buf[0] & 0xF8) >> 3;
    uint16_t range = ((uint16_t)buf[10] << 8) | buf[11];

    if (range_status == 11 && range < 8190) {
        return range;
    }

    return 0;
}

void vl53l0x_stop_continuous(void)
{
    wr8(SYSRANGE_START, 0x01);
    wr8(0xFF, 0x01);
    wr8(0x00, 0x00);
    wr8(0x91, 0x00);
    wr8(0x00, 0x01);
    wr8(0xFF, 0x00);
}