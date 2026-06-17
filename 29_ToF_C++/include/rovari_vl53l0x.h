/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 *
 * rovari_vl53l0x.h: VL53L0X time-of-flight distance sensor for CH32V003
 *
 * I2C laser ranging, ~30 mm to ~2000 mm.
 * Uses I2C1 (SCL=PC2, SDA=PC1). Default address 0x29.
 *
 * Wiring (V2 breakout):
 *   VIN  -> 3.3V or 5V (module has regulator)
 *   GND  -> GND
 *   SCL  -> PC2 (4.7k pullup)
 *   SDA  -> PC1 (4.7k pullup)
 *   XSHUT -> optional GPIO for reset control
 *
 * C usage:
 *   i2c_init(I2C_1, 400000);
 *   if (vl53l0x_init(VL53L0X_HIGH_SPEED) == 0) {
 *       uint16_t mm = vl53l0x_read_mm();
 *   }
 *
 * C++ usage:
 *   I2c bus(I2C_1, 400000);
 *   Vl53l0x tof;
 *   if (tof.init(VL53L0X_HIGH_SPEED) == 0) {
 *       uint16_t mm = tof.readMm();
 *   }
 */

#ifndef ROVARI_VL53L0X_H
#define ROVARI_VL53L0X_H

#include "rovari_defs.h"

/* Default 7-bit I2C address */
#define VL53L0X_ADDR_DEFAULT  0x29

/* I/O timeout in milliseconds */
#define VL53L0X_TIMEOUT_MS    500

/* Ranging mode */
typedef enum {
    VL53L0X_DEFAULT,       /* Default timing budget */
    VL53L0X_HIGH_SPEED,    /* 20 ms budget, faster but noisier */
    VL53L0X_HIGH_ACCURACY, /* 200 ms budget, slower but precise */
    VL53L0X_LONG_RANGE     /* Lower signal rate, longer VCSEL periods */
} Vl53l0xMode;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Full sensor init: data init, SPAD calibration, tuning, timing
 * budget, and reference calibration. I2C must already be initialized.
 *
 * @param[in] mode  Ranging mode.
 * @return 0 on success, -1 on failure.
 */
int vl53l0x_init(Vl53l0xMode mode);

/**
 * Single-shot range measurement.
 *
 * @return Distance in mm, or 0 on timeout / out of range.
 */
uint16_t vl53l0x_read_mm(void);

/**
 * Start continuous back-to-back ranging.
 *
 * @param[in] period_ms  Inter-measurement period. 0 for back-to-back.
 */
void vl53l0x_start_continuous(uint32_t period_ms);

/**
 * Read one result from continuous ranging.
 *
 * @return Distance in mm, or 0 on timeout.
 */
uint16_t vl53l0x_read_continuous_mm(void);

/**
 * Stop continuous ranging.
 */
void vl53l0x_stop_continuous(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Vl53l0x {
public:
    int      init(Vl53l0xMode mode = VL53L0X_DEFAULT) { return vl53l0x_init(mode); }
    uint16_t readMm()                   { return vl53l0x_read_mm(); }
    void     startContinuous(uint32_t p) { vl53l0x_start_continuous(p); }
    uint16_t readContinuousMm()          { return vl53l0x_read_continuous_mm(); }
    void     stopContinuous()            { vl53l0x_stop_continuous(); }
};

#endif

#endif /* ROVARI_VL53L0X_H */