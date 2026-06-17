/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 *
 * rovari_ultrasonic.h: HC-SR04 ultrasonic distance sensor for CH32V003
 *
 * Any two GPIO pins can be used for TRIG (output) and ECHO (input).
 * Supports up to 2 sensors simultaneously (4 pins total).
 *
 * Wiring:
 *   VCC  -> 5V (sensor supply)
 *   GND  -> GND
 *   TRIG -> any GPIO (e.g. PC1)
 *   ECHO -> any GPIO (e.g. PC2)
 *
 * If the MCU runs at 3.3V, a voltage divider on the ECHO line is
 * recommended since the HC-SR04 outputs a 5V pulse. The TRIG input
 * accepts 3.3V logic. Running the CH32V003 at 5V avoids this issue.
 *
 * Speed of sound: 343 m/s (at ~20 C). Integer arithmetic only.
 *
 * C usage:
 *   ultrasonic_init(PC1, PC2);
 *   uint32_t mm = ultrasonic_read_mm(PC1);
 *
 * C++ usage:
 *   Ultrasonic sonar(PC1, PC2);
 *   uint32_t mm = sonar.readMm();
 */

#ifndef ROVARI_ULTRASONIC_H
#define ROVARI_ULTRASONIC_H

#include "rovari_defs.h"

/* Maximum simultaneous sensors */
#define ULTRASONIC_MAX  2

/* Measurement timeout in milliseconds.
 * 50 ms covers ~8.5 m round trip, well beyond the HC-SR04's 4 m max. */
#define ULTRASONIC_TIMEOUT_MS  50

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Register and configure a sensor's pins.
 * TRIG is set to push-pull output (driven low).
 * ECHO is set to input pull-down (idles low).
 * Call once per sensor in app_init().
 *
 * @param[in] trig  Trigger pin (also used as sensor ID in read calls).
 * @param[in] echo  Echo pin.
 */
void ultrasonic_init(pin_t trig, pin_t echo);

/**
 * Measure the echo pulse width in microseconds.
 * Sends a 12 us trigger pulse, then times the echo high period.
 *
 * @param[in] trig  Trigger pin (same pin passed to ultrasonic_init).
 * @return Pulse width in microseconds, or 0 on timeout / no object.
 */
uint32_t ultrasonic_read_us(pin_t trig);

/**
 * Measure distance in millimeters.
 * Convenience wrapper: mm = pulse_us * 343 / 2000 (rounded).
 *
 * @param[in] trig  Trigger pin.
 * @return Distance in mm, or 0 on timeout / no object.
 */
uint32_t ultrasonic_read_mm(pin_t trig);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Ultrasonic {
public:
    Ultrasonic(pin_t trig, pin_t echo) : _trig(trig)
    {
        ultrasonic_init(trig, echo);
    }

    uint32_t readUs()  { return ultrasonic_read_us(_trig); }
    uint32_t readMm()  { return ultrasonic_read_mm(_trig); }

private:
    pin_t _trig;
};

#endif

#endif /* ROVARI_ULTRASONIC_H */
