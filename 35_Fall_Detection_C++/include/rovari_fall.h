/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 *
 * rovari_fall.h
 * Fall detection state machine for CH32V003 using MPU6050
 *
 * Detects falls by watching for a high-g impact followed by
 * an orientation change and post-impact stillness.
 * Call fall_update() at ~100 Hz with filtered MPU6050 data.
 *
 * C usage:
 *   fall_init();
 *   // in loop:
 *   mpu6050_data_t d;
 *   mpu6050_read(&d);
 *   fall_update(&d);
 *   if (fall_detected()) { ... alert ... }
 *
 * C++ usage:
 *   FallDetector fall;
 *   fall.update(&d);
 *   if (fall.detected()) { ... }
 */

#ifndef ROVARI_FALL_H
#define ROVARI_FALL_H

#include "rovari_defs.h"
#include "rovari_mpu6050.h"

/* Tuning (override with #define before including) */
#ifndef FALL_IMPACT_MG
#define FALL_IMPACT_MG       2500   /* impact threshold in mg */
#endif
#ifndef FALL_FREEFALL_MG
#define FALL_FREEFALL_MG     400    /* freefall threshold in mg */
#endif
#ifndef FALL_ORIENT_DOT
#define FALL_ORIENT_DOT      500    /* dot product threshold (0..1000) */
#endif
#ifndef FALL_STILL_MS
#define FALL_STILL_MS        3000   /* post impact stillness window */
#endif
#ifndef FALL_STILL_GYRO
#define FALL_STILL_GYRO      500    /* gyro quiet threshold (mdps) */
#endif

/* State machine states */
#define FALL_ST_WATCH     0
#define FALL_ST_FREEFALL  1
#define FALL_ST_IMPACT    2
#define FALL_ST_VERIFY    3
#define FALL_ST_ALERT     4

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize fall detector state machine.
 */
void fall_init(void);

/**
 * Set detection thresholds. Call after fall_init().
 *
 * @param[in] impact_mg    Impact threshold in mg (default 2500).
 * @param[in] freefall_mg  Freefall threshold in mg (default 400).
 */
void fall_set_thresholds(uint16_t impact_mg, uint16_t freefall_mg);

/**
 * Feed one sample. Call at ~100 Hz with filtered MPU6050 data.
 *
 * @param[in] data  Filtered accelerometer/gyroscope in mg/mdps.
 */
void fall_update(const mpu6050_data_t *data);

/**
 * Check if a fall was detected. Returns 1 once per event,
 * then clears the flag.
 *
 * @return 1 if fall detected, 0 otherwise.
 */
uint8_t fall_detected(void);

/**
 * Get current state machine state.
 *
 * @return One of the FALL_ST_ constants.
 */
uint8_t fall_get_state(void);

/**
 * Get state name string.
 *
 * @param[in] state  State from fall_get_state().
 * @return Constant string.
 */
const char* fall_state_str(uint8_t state);

/**
 * Reset the detector (e.g. after handling an alert).
 */
void fall_reset(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class FallDetector {
public:
    FallDetector()               { fall_init(); }
    void        setThresholds(uint16_t impact, uint16_t ff) { fall_set_thresholds(impact, ff); }
    void        update(const mpu6050_data_t *d) { fall_update(d); }
    uint8_t     detected()       { return fall_detected(); }
    uint8_t     state()          { return fall_get_state(); }
    const char* stateStr()       { return fall_state_str(fall_get_state()); }
    void        reset()          { fall_reset(); }
};

#endif

#endif /* ROVARI_FALL_H */