/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 *
 * rovari_mcp9700.h: MCP9700/MCP9700A analog temperature sensor
 *
 * Linear output: 500 mV at 0 C, 10 mV per degree C.
 * Single analog pin, no configuration. Works at 3.3V or 5V.
 *
 * Wiring:
 *   VDD  -> 3.3V or 5V (same rail as MCU)
 *   VSS  -> GND
 *   VOUT -> any ADC pin (e.g. PA2)
 *
 * C usage:
 *   mcp9700_init(PA2, 4700);
 *   int16_t c10 = mcp9700_read_c10(PA2);   // 235 = 23.5 C
 *
 * C++ usage:
 *   Mcp9700 temp(PA2, 4700);
 *   int16_t c10 = temp.readC10();
 */

#ifndef ROVARI_MCP9700_H
#define ROVARI_MCP9700_H

#include "rovari_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the ADC channel and set the reference voltage.
 * Call once in app_init().
 *
 * @param[in] pin      Analog-capable pin connected to VOUT.
 * @param[in] vref_mv  Actual VDD in millivolts (measure with a meter).
 */
void mcp9700_init(pin_t pin, uint16_t vref_mv);

/**
 * Read the raw sensor output voltage in millivolts.
 *
 * @param[in] pin  Sensor pin.
 * @return Output voltage in mV.
 */
uint16_t mcp9700_read_mv(pin_t pin);

/**
 * Read temperature in tenths of a degree C.
 * Example: 235 = 23.5 C, -15 = -1.5 C.
 *
 * @param[in] pin  Sensor pin.
 * @return Temperature in tenths of degrees C.
 */
int16_t mcp9700_read_c10(pin_t pin);

/**
 * Read temperature in whole degrees C (truncated toward zero).
 *
 * @param[in] pin  Sensor pin.
 * @return Temperature in degrees C.
 */
int16_t mcp9700_read_c(pin_t pin);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Mcp9700 {
public:
    Mcp9700(pin_t pin, uint16_t vref_mv) : _pin(pin)
    {
        mcp9700_init(pin, vref_mv);
    }

    uint16_t readMv()   { return mcp9700_read_mv(_pin); }
    int16_t  readC10()  { return mcp9700_read_c10(_pin); }
    int16_t  readC()    { return mcp9700_read_c(_pin); }

private:
    pin_t _pin;
};

#endif

#endif /* ROVARI_MCP9700_H */