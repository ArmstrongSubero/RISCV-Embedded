/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 *
 * rovari_joystick.h: 2-axis analog joystick with button for CH32V003
 *
 * Two analog axes (potentiometers) and one digital button (active low).
 * Auto-calibrates center at startup. Deadzone filtering with 8 cardinal
 * and diagonal directions.
 *
 * Wiring:
 *   VRx  -> any ADC pin (e.g. PA2)
 *   VRy  -> any ADC pin (e.g. PA1)
 *   SW   -> any GPIO pin (e.g. PC0), active low
 *   VCC  -> 3.3V or 5V
 *   GND  -> GND
 *
 * C usage:
 *   joystick_init(PA2, PA1, PC0);
 *   joystick_calibrate();
 *   joystick_t joy;
 *   joystick_read(&joy);
 *
 * C++ usage:
 *   Joystick stick(PA2, PA1, PC0);
 *   stick.calibrate();
 *   joystick_t joy;
 *   stick.read(&joy);
 */

#ifndef ROVARI_JOYSTICK_H
#define ROVARI_JOYSTICK_H

#include "rovari_defs.h"

/* Dead zone threshold around calibrated center */
#ifndef JOYSTICK_DEADZONE
#define JOYSTICK_DEADZONE  80
#endif

/* Direction bitmask values (can be OR'd for diagonals) */
#define JOY_CENTER  0x00
#define JOY_UP      0x01
#define JOY_DOWN    0x02
#define JOY_LEFT    0x04
#define JOY_RIGHT   0x08

/* Joystick state */
typedef struct {
    uint16_t raw_x;       /* Raw ADC X (0-1023) */
    uint16_t raw_y;       /* Raw ADC Y (0-1023) */
    int16_t  x;           /* Centered X (negative=left, positive=right) */
    int16_t  y;           /* Centered Y (negative=up, positive=down) */
    uint8_t  direction;   /* Direction bitmask */
    uint8_t  button;      /* 1 = pressed, 0 = released */
} joystick_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize joystick pins.
 * Configures ADC for both axes and input pull-up for button.
 *
 * @param[in] x_pin   Analog pin for X axis.
 * @param[in] y_pin   Analog pin for Y axis.
 * @param[in] btn_pin Digital pin for button (active low).
 */
void joystick_init(pin_t x_pin, pin_t y_pin, pin_t btn_pin);

/**
 * Calibrate center position. Call at startup with joystick at rest.
 * Averages 16 samples per axis.
 */
void joystick_calibrate(void);

/**
 * Read joystick state into the provided struct.
 * Averages 8 samples per axis, applies deadzone, determines direction.
 *
 * @param[out] joy  Pointer to joystick state struct.
 */
void joystick_read(joystick_t *joy);

/**
 * Get direction string for display.
 *
 * @param[in] direction  Direction bitmask from joystick_t.
 * @return Constant string (e.g. "UP", "DN-LEFT", "CENTER").
 */
const char* joystick_direction_str(uint8_t direction);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Joystick {
public:
    Joystick(pin_t x_pin, pin_t y_pin, pin_t btn_pin)
    {
        joystick_init(x_pin, y_pin, btn_pin);
    }

    void calibrate()                 { joystick_calibrate(); }
    void read(joystick_t *joy)       { joystick_read(joy); }
    const char* dirStr(uint8_t dir)  { return joystick_direction_str(dir); }

private:
};

#endif

#endif /* ROVARI_JOYSTICK_H */
