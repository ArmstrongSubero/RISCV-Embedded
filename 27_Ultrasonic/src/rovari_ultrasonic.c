/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 */

/**
 * @file rovari_ultrasonic.c
 * @brief HC-SR04 ultrasonic distance sensor driver for CH32V003.
 *
 * Uses the SDK's micros() (TIM2-based, 1 us resolution) for echo
 * pulse timing. All waits are bounded by ULTRASONIC_TIMEOUT_MS.
 *
 * Measurement sequence:
 *   1. Ensure ECHO is idle low (wait with timeout).
 *   2. Drive TRIG high for 12 us (spec minimum 10 us).
 *   3. Wait for ECHO rising edge (with timeout).
 *   4. Time ECHO high period (with timeout).
 *   5. Convert pulse width to distance:
 *      mm = (pulse_us * 343 + 999) / 2000   (rounded, 343 m/s at 20 C)
 *
 * RAM: 6 bytes per sensor (2 pin_t + 1 flag + padding), max 2 sensors.
 */

#include <stdint.h>
#include <stddef.h>
#include "sevs_runtime.h"
#include "debug.h"
#include "rovari_ultrasonic.h"
#include "rovari_gpio.h"

extern uint32_t micros(void);

/* Timeout converted to microseconds for internal use */
#define TIMEOUT_US  ((uint32_t)ULTRASONIC_TIMEOUT_MS * 1000U)

/* Speed of sound constant: 343 m/s = 343000 mm/s.
 * Round trip: distance_mm = pulse_us * 343000 / (1000000 * 2)
 *                         = pulse_us * 343 / 2000              */
#define SPEED_NUMER  343U
#define SPEED_DENOM  2000U

/* -----------------------------------------------------------------------
 *  Pin pair table
 * ----------------------------------------------------------------------- */
typedef struct {
    pin_t   trig;
    pin_t   echo;
    uint8_t active;
} us_entry_t;

static us_entry_t s_sensors[ULTRASONIC_MAX];

/**
 * @brief Find an entry by trigger pin.
 */
static us_entry_t* find_entry(pin_t trig)
{
    for (uint8_t i = 0; i < ULTRASONIC_MAX; i++) {
        if (s_sensors[i].active && s_sensors[i].trig == trig) {
            return &s_sensors[i];
        }
    }
    return NULL;
}

/* -----------------------------------------------------------------------
 *  Public C API
 * ----------------------------------------------------------------------- */

/**
 * @brief Register a sensor and configure its GPIO.
 * @req REQ-ROVARI-ULTRASONIC-0010
 */
void ultrasonic_init(pin_t trig, pin_t echo)
{
    /* Already registered? */
    if (find_entry(trig) != NULL) {
        return;
    }

    /* Find an empty slot */
    for (uint8_t i = 0; i < ULTRASONIC_MAX; i++) {
        if (!s_sensors[i].active) {
            s_sensors[i].trig   = trig;
            s_sensors[i].echo   = echo;
            s_sensors[i].active = 1;

            pin_mode(trig, Output);
            digital_write(trig, Low);
            pin_mode(echo, InputPullDown);
            return;
        }
    }
    /* No slot available, silently ignored */
}

/**
 * @brief Measure echo pulse width in microseconds.
 *
 * All waits are bounded by TIMEOUT_US. Returns 0 if the sensor
 * is not registered, no echo is received, or the timeout expires.
 *
 * @req REQ-ROVARI-ULTRASONIC-0011
 */
uint32_t ultrasonic_read_us(pin_t trig)
{
    us_entry_t* e = find_entry(trig);
    if (e == NULL) {
        return 0;
    }

    pin_t echo = e->echo;
    uint32_t t;

    /* @sevs-bound: wait for ECHO idle low (previous pulse drained) */
    t = micros();
    while (digital_read(echo)) {
        if (micros() - t > TIMEOUT_US) {
            return 0;
        }
    }

    /* Send 12 us trigger pulse (spec minimum 10 us) */
    digital_write(trig, Low);
    Delay_Us(2);
    digital_write(trig, High);
    Delay_Us(12);
    digital_write(trig, Low);

    /* @sevs-bound: wait for ECHO rising edge */
    t = micros();
    while (!digital_read(echo)) {
        if (micros() - t > TIMEOUT_US) {
            return 0;
        }
    }

    /* @sevs-bound: measure ECHO high time */
    uint32_t t0 = micros();
    while (digital_read(echo)) {
        if (micros() - t0 > TIMEOUT_US) {
            return 0;
        }
    }

    return micros() - t0;
}

/**
 * @brief Measure distance in millimeters.
 *
 * Calls ultrasonic_read_us() and converts:
 *   mm = (pulse_us * 343 + 999) / 2000
 * The +999 rounds up to the nearest mm.
 *
 * @req REQ-ROVARI-ULTRASONIC-0012
 */
uint32_t ultrasonic_read_mm(pin_t trig)
{
    uint32_t us = ultrasonic_read_us(trig);
    if (us == 0) {
        return 0;
    }
    return (us * SPEED_NUMER + (SPEED_DENOM - 1U)) / SPEED_DENOM;
}
