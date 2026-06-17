/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 *
 * rovari_mpu6050.h
 * MPU6050 6-axis accelerometer/gyroscope for CH32V003
 *
 * I2C interface, default address 0x68 (AD0 low).
 * Accelerometer in mg (milligravity), gyroscope in mdps
 * (millidegrees per second), temperature in centidegrees C.
 * All integer, no floats.
 *
 * Wiring:
 *   VCC -> 3.3V
 *   GND -> GND
 *   SCL -> PC2 (4.7k pullup)
 *   SDA -> PC1 (4.7k pullup)
 *   AD0 -> GND (address 0x68) or VCC (address 0x69)
 *
 * C usage:
 *   i2c_init(I2C_1, 400000);
 *   mpu6050_init();
 *   mpu6050_calibrate();
 *   mpu6050_data_t d;
 *   mpu6050_read(&d);
 *
 * C++ usage:
 *   I2c bus(I2C_1, 400000);
 *   Mpu6050 imu;
 *   imu.init();
 *   imu.calibrate();
 *   mpu6050_data_t d;
 *   imu.read(&d);
 */

#ifndef ROVARI_MPU6050_H
#define ROVARI_MPU6050_H

#include "rovari_defs.h"

/* 7-bit I2C address (AD0 low = 0x68, AD0 high = 0x69) */
#ifndef MPU6050_ADDR
#define MPU6050_ADDR  0x68
#endif

/* Raw sensor data (14-byte burst) */
typedef struct {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
} mpu6050_raw_t;

/* Filtered and scaled sensor data */
typedef struct {
    int32_t ax, ay, az;   /* milligravity (mg) */
    int32_t gx, gy, gz;   /* millidegrees per second (mdps) */
    int32_t temp;          /* centidegrees C (e.g. 3653 = 36.53 C) */
} mpu6050_data_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize MPU6050. I2C must already be initialized.
 * Checks WHO_AM_I, wakes sensor, configures DLPF at 42 Hz,
 * sample rate 100 Hz, accel +/- 2g, gyro +/- 250 dps.
 * FIFO disabled, DRDY interrupt enabled.
 *
 * @return 0 on success, -1 on failure.
 */
int mpu6050_init(void);

/**
 * Calibrate offsets. Hold sensor flat and still.
 * Averages 200 samples (~1 second). Assumes Z accel = +1g.
 */
void mpu6050_calibrate(void);

/**
 * Read raw 14-byte burst (accel, temp, gyro).
 *
 * @param[out] raw  Raw sensor values.
 * @return 0 on success, -1 on failure.
 */
int mpu6050_read_raw(mpu6050_raw_t *raw);

/**
 * Read filtered and calibrated data.
 * Median-of-3 then IIR smoothing, offset corrected.
 *
 * @param[out] data  Scaled and filtered values.
 * @return 0 on success, -1 on failure.
 */
int mpu6050_read(mpu6050_data_t *data);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Mpu6050 {
public:
    int  init()                       { return mpu6050_init(); }
    void calibrate()                  { mpu6050_calibrate(); }
    int  readRaw(mpu6050_raw_t *raw)  { return mpu6050_read_raw(raw); }
    int  read(mpu6050_data_t *data)   { return mpu6050_read(data); }
};

#endif

#endif /* ROVARI_MPU6050_H */
