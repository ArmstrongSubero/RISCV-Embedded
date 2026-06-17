/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 */

/**
 * @file rovari_mcp9700.c
 * @brief MCP9700/MCP9700A analog temperature sensor driver for CH32V003.
 *
 * Transfer function: Vout = 500 mV + 10 mV/C * T
 * Inversion:         T(C) = (Vout_mV - 500) / 10
 * In tenths:         T_c10 = Vout_mV - 500
 *
 * Uses 16-sample averaging for stable readings.
 * VREF is stored at init so the mV conversion is accurate
 * regardless of compile-time ADC_VREF_MV.
 *
 * @req REQ-ROVARI-MCP9700-0010
 */

#include <stdint.h>
#include "rovari_mcp9700.h"
#include "rovari_adc.h"

/* MCP9700 output at 0 C */
#define OFFSET_MV  500

/* Number of ADC samples to average */
#define MCP9700_AVG_SAMPLES  16

/* Runtime VREF set by mcp9700_init() */
static uint16_t s_vref_mv = 5000;

void mcp9700_init(pin_t pin, uint16_t vref_mv)
{
    s_vref_mv = vref_mv;
    adc_init(pin);
}

/**
 * @brief Read averaged raw ADC value (16 samples).
 * @sevs-bound: exactly MCP9700_AVG_SAMPLES iterations.
 */
static uint16_t read_averaged(pin_t pin)
{
    uint32_t acc = 0;
    for (uint8_t i = 0; i < MCP9700_AVG_SAMPLES; i++) {
        acc += analog_read(pin);
    }
    return (uint16_t)(acc / MCP9700_AVG_SAMPLES);
}

uint16_t mcp9700_read_mv(pin_t pin)
{
    uint16_t raw = read_averaged(pin);
    return (uint16_t)(((uint32_t)raw * s_vref_mv + 511u) / ADC_MAX_VALUE);
}

int16_t mcp9700_read_c10(pin_t pin)
{
    return (int16_t)mcp9700_read_mv(pin) - OFFSET_MV;
}

int16_t mcp9700_read_c(pin_t pin)
{
    return mcp9700_read_c10(pin) / 10;
}