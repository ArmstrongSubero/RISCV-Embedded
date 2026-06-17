/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 */

/**
 * @file rovari_joystick.c
 * @brief 2-axis analog joystick with button for CH32V003.
 *
 * Reads two ADC channels for X/Y axes, one digital input for button.
 * Auto-calibrates center from 16-sample average at rest.
 * 8-sample averaging per read, deadzone filtering, 8-direction output.
 *
 * Ported from working PIC16F1718 joystick driver.
 *
 * @req REQ-ROVARI-JOYSTICK-0010
 */

#include <stdint.h>
#include "rovari_joystick.h"
#include "rovari_adc.h"
#include "rovari_gpio.h"

/* Stored pin assignments */
static pin_t s_x_pin;
static pin_t s_y_pin;
static pin_t s_btn_pin;

/* Calibrated center values */
static uint16_t s_center_x = 512;
static uint16_t s_center_y = 512;

/* -----------------------------------------------------------------------
 *  Averaged ADC read
 * ----------------------------------------------------------------------- */

/**
 * @brief Read ADC with N-sample averaging.
 * @sevs-bound: exactly n iterations.
 */
static uint16_t read_averaged(pin_t pin, uint8_t n)
{
    uint32_t acc = 0;
    for (uint8_t i = 0; i < n; i++) {
        acc += analog_read(pin);
    }
    return (uint16_t)(acc / n);
}

/* -----------------------------------------------------------------------
 *  Public API
 * ----------------------------------------------------------------------- */

void joystick_init(pin_t x_pin, pin_t y_pin, pin_t btn_pin)
{
    s_x_pin   = x_pin;
    s_y_pin   = y_pin;
    s_btn_pin = btn_pin;

    adc_init(x_pin);
    adc_init(y_pin);
    pin_mode(btn_pin, InputPullUp);
}

void joystick_calibrate(void)
{
    s_center_x = read_averaged(s_x_pin, 16);
    s_center_y = read_averaged(s_y_pin, 16);
}

void joystick_read(joystick_t *joy)
{
    /* 8-sample average for stable readings */
    joy->raw_x = read_averaged(s_x_pin, 8);
    joy->raw_y = read_averaged(s_y_pin, 8);

    /* Center around calibrated zero */
    joy->x = (int16_t)joy->raw_x - (int16_t)s_center_x;
    joy->y = (int16_t)joy->raw_y - (int16_t)s_center_y;

    /* Button: active low with pull-up */
    joy->button = (digital_read(s_btn_pin) == Low) ? 1 : 0;

    /* Direction with deadzone */
    uint8_t dir = JOY_CENTER;

    if (joy->x > JOYSTICK_DEADZONE)
        dir |= JOY_RIGHT;
    else if (joy->x < -JOYSTICK_DEADZONE)
        dir |= JOY_LEFT;

    if (joy->y > JOYSTICK_DEADZONE)
        dir |= JOY_DOWN;
    else if (joy->y < -JOYSTICK_DEADZONE)
        dir |= JOY_UP;

    joy->direction = dir;
}

const char* joystick_direction_str(uint8_t direction)
{
    switch (direction) {
        case JOY_UP:                    return "UP";
        case JOY_DOWN:                  return "DOWN";
        case JOY_LEFT:                  return "LEFT";
        case JOY_RIGHT:                 return "RIGHT";
        case (JOY_UP   | JOY_LEFT):     return "UP-LEFT";
        case (JOY_UP   | JOY_RIGHT):    return "UP-RIGHT";
        case (JOY_DOWN | JOY_LEFT):     return "DN-LEFT";
        case (JOY_DOWN | JOY_RIGHT):    return "DN-RIGHT";
        default:                        return "CENTER";
    }
}
