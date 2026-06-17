/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 *
 * rovari_ahrs.h
 * Roll/Pitch/Yaw orientation from MPU6050 for CH32V003
 *
 * All integer, no libm. Roll and pitch from accelerometer via
 * Q15 atan2 approximation. Yaw from gyro Z integration with
 * adaptive drift zeroing during stationary periods.
 * Angles in centidegrees (e.g. 4523 = 45.23 degrees).
 *
 * Requires rovari_mpu6050 to be initialized and calibrated first.
 *
 * C usage:
 *   mpu6050_init();
 *   mpu6050_calibrate();
 *   ahrs_init();
 *   ahrs_angles_t a;
 *   ahrs_update(&a);   // call at ~100 Hz
 *
 * C++ usage:
 *   Mpu6050 imu;
 *   Ahrs ahrs;
 *   imu.init(); imu.calibrate();
 *   ahrs.init();
 *   ahrs_angles_t a;
 *   ahrs.update(&a);
 */

#ifndef ROVARI_AHRS_H
#define ROVARI_AHRS_H

#include "rovari_defs.h"
#include "rovari_mpu6050.h"

/* Level detection band (centidegrees) */
#ifndef AHRS_LEVEL_BAND
#define AHRS_LEVEL_BAND    50   /* +/- 0.50 degrees */
#endif
#ifndef AHRS_LEVEL_HYST
#define AHRS_LEVEL_HYST    20   /* exit hysteresis 0.20 degrees */
#endif

/* Orientation angles in centidegrees */
typedef struct {
    int32_t roll;    /* centidegrees, +/- 18000 */
    int32_t pitch;   /* centidegrees, +/- 9000 */
    int32_t yaw;     /* centidegrees, +/- 18000 */
} ahrs_angles_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize AHRS state. Call after mpu6050_init() and mpu6050_calibrate().
 */
void ahrs_init(void);

/**
 * Read MPU6050, compute orientation, fill angles.
 * Call at ~100 Hz (every 10 ms) in app_run().
 *
 * @param[out] angles  Roll/pitch/yaw in centidegrees.
 * @return 0 on success, -1 if sensor read failed.
 */
int ahrs_update(ahrs_angles_t *angles);

/**
 * Check if sensor is level (roll and pitch within band).
 * Uses hysteresis to avoid flicker.
 *
 * @return 1 if level, 0 otherwise.
 */
uint8_t ahrs_is_level(void);

/**
 * Reset yaw to zero.
 */
void ahrs_reset_yaw(void);

/**
 * Get total inclination from level in centidegrees.
 * Combines roll and pitch into a single tilt angle.
 * 0 = perfectly flat, increases in any direction.
 *
 * @return Tilt in centidegrees.
 */
uint32_t ahrs_tilt(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Ahrs {
public:
    void    init()                       { ahrs_init(); }
    int     update(ahrs_angles_t *a)     { return ahrs_update(a); }
    uint8_t isLevel()                    { return ahrs_is_level(); }
    void     resetYaw()                   { ahrs_reset_yaw(); }
    uint32_t tilt()                       { return ahrs_tilt(); }
};

#endif

#endif /* ROVARI_AHRS_H */