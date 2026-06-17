/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 *
 * rovari_gps.h
 * NEO-6M GPS receiver for CH32V003
 *
 * Software UART RX at 9600 baud on PD2. Parses RMC sentences
 * and outputs coordinates in microdegrees (integer, no floats).
 * Checksum enforced, RMC only.
 *
 * Wiring:
 *   VCC -> 5V
 *   GND -> GND
 *   TX  -> PD2
 *
 * C usage:
 *   gps_init();
 *   gps_fix_t fix;
 *   if (gps_read(&fix) == 0) { ... fix.lat, fix.lon ... }
 *
 * C++ usage:
 *   Gps gps;
 *   gps_fix_t fix;
 *   if (gps.read(&fix) == 0) { ... }
 */

#ifndef ROVARI_GPS_H
#define ROVARI_GPS_H

#include "rovari_defs.h"

typedef struct {
    int32_t lat;     /* microdegrees (10657432 = 10.657432 degrees) */
    int32_t lon;     /* microdegrees */
    uint8_t valid;   /* 1 = valid fix */
} gps_fix_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize software UART on PD2 at 9600 baud.
 */
void gps_init(void);

/**
 * Read one raw NMEA line (blocking). Returns length.
 */
int gps_read_nmea(char *buf, int maxlen);

/**
 * Read one valid RMC fix. Blocks until a checksummed
 * $GPRMC or $GNRMC sentence with status 'A' is received.
 *
 * @param[out] fix  Parsed coordinates.
 * @return 0 on success, -1 on parse error.
 */
int gps_read(gps_fix_t *fix);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Gps {
public:
    Gps()            { gps_init(); }
    int read(gps_fix_t *f) { return gps_read(f); }
};

#endif

#endif /* ROVARI_GPS_H */