/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 */

/**
 * @file rovari_gps.c
 * @brief NEO-6M GPS receiver for CH32V003.
 * @req REQ-ROVARI-GPS-0010
 */

#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include "ch32v00x.h"
#include "ch32v00x_gpio.h"
#include "ch32v00x_rcc.h"
#include "ch32v00x_tim.h"
#include "debug.h"
#include "rovari_gps.h"

#define SUART_RX_PORT  GPIOD
#define SUART_RX_PIN   GPIO_Pin_2
#define BIT_US         104  /* 1000000 / 9600 */

/* Fast GPIO read */
static inline uint8_t rx_read(void)
{
    return (SUART_RX_PORT->INDR & SUART_RX_PIN) ? 1 : 0;
}

/* Reconfigure TIM2 as free-running 1MHz 16-bit counter for bit timing.
 * This overrides the SDK tick. millis() will not work during GPS use. */
static void timebase_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef t = {0};
    t.TIM_Prescaler     = (SystemCoreClock / 1000000UL) - 1;
    t.TIM_CounterMode   = TIM_CounterMode_Up;
    t.TIM_Period        = 0xFFFF;
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &t);
    TIM_Cmd(TIM2, ENABLE);
}

static inline uint16_t now_t2(void) { return TIM2->CNT; }

static inline void wait_until_t2(uint16_t deadline)
{
    while ((int16_t)(TIM2->CNT - deadline) < 0) {}
}

/* -----------------------------------------------------------------------
 *  Software UART RX (9600 8N1)
 * ----------------------------------------------------------------------- */

void gps_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = SUART_RX_PIN;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(SUART_RX_PORT, &g);
    timebase_init();
}

static uint8_t suart_rx(void)
{
    while (rx_read() == 0) {}  /* wait for idle high */
    while (rx_read() != 0) {}  /* wait for start bit falling edge */

    __disable_irq();
    uint16_t t = now_t2();
    t += (uint16_t)(BIT_US + BIT_US / 2);

    uint8_t v = 0;
    for (uint8_t i = 0; i < 8; i++) {
        wait_until_t2(t);
        if (rx_read()) v |= (1U << i);
        t += BIT_US;
    }
    __enable_irq();
    return v;
}

/* -----------------------------------------------------------------------
 *  NMEA line reader
 * ----------------------------------------------------------------------- */
int gps_read_nmea(char *dst, int maxlen)
{
    uint8_t ch;
    do { ch = suart_rx(); } while (ch != '$');
    int len = 0;
    dst[len++] = '$';
    while (1) {
        ch = suart_rx();
        if (len < maxlen - 1) dst[len++] = (char)ch;
        if (ch == '\n') break;
        if (len >= maxlen - 1) break;
    }
    dst[len] = 0;
    return len;
}

/* -----------------------------------------------------------------------
 *  NMEA checksum
 * ----------------------------------------------------------------------- */
static int checksum_ok(const char *s)
{
    if (!s || s[0] != '$') return 0;
    uint8_t cs = 0;
    const char *p = s + 1;
    while (*p && *p != '*' && *p != '\r' && *p != '\n') cs ^= (uint8_t)(*p++);
    if (*p != '*') return 0;
    char h1 = *(p + 1), h2 = *(p + 2);
    if (!isxdigit((unsigned char)h1) || !isxdigit((unsigned char)h2)) return 0;
    int hi = (h1 <= '9') ? h1 - '0' : (toupper(h1) - 'A' + 10);
    int lo = (h2 <= '9') ? h2 - '0' : (toupper(h2) - 'A' + 10);
    return (cs == (uint8_t)((hi << 4) | lo));
}

/* -----------------------------------------------------------------------
 *  CSV splitter (no strtok)
 * ----------------------------------------------------------------------- */
static int split_csv(char *line, char *out[], int maxf)
{
    int n = 0;
    if (!line || !*line) return 0;
    out[n++] = line;
    for (char *p = line; *p && n < maxf; ++p) {
        if (*p == ',') { *p = 0; out[n++] = p + 1; }
        if (*p == '*') { *p = 0; break; }
    }
    return n;
}

/* -----------------------------------------------------------------------
 *  Coordinate parser (ddmm.mmmm to signed microdegrees)
 * ----------------------------------------------------------------------- */
static int coord_to_udeg(const char *field, int is_lat, int32_t *out)
{
    if (!field || !*field) return 0;

    int dlen = is_lat ? 2 : 3;
    int deg = 0, i = 0;
    for (; i < dlen && isdigit((unsigned char)field[i]); ++i)
        deg = deg * 10 + (field[i] - '0');
    if (i != dlen) return 0;

    if (!isdigit((unsigned char)field[i]) || !isdigit((unsigned char)field[i + 1]))
        return 0;
    int min_int = (field[i] - '0') * 10 + (field[i + 1] - '0');
    i += 2;

    int frac = 0, scale = 1;
    if (field[i] == '.') {
        ++i;
        while (isdigit((unsigned char)field[i]) && scale < 1000000) {
            frac  = frac * 10 + (field[i] - '0');
            scale = scale * 10;
            ++i;
        }
    }

    int64_t micro = (int64_t)deg * 1000000LL;
    micro += ((int64_t)min_int * 1000000LL + 30) / 60;
    micro += ((int64_t)frac * 1000000LL + (int64_t)scale * 30)
             / ((int64_t)scale * 60);

    *out = (int32_t)micro;
    return 1;
}

/* -----------------------------------------------------------------------
 *  Public API
 * ----------------------------------------------------------------------- */

int gps_read(gps_fix_t *fix)
{
    char nmea[128];
    int L = gps_read_nmea(nmea, sizeof(nmea));
    if (L <= 0) return -1;

    if (!checksum_ok(nmea)) return -1;
    if (strncmp(nmea, "$GPRMC", 6) && strncmp(nmea, "$GNRMC", 6)) return -1;

    char buf[128];
    strncpy(buf, nmea, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    char *fld[16] = {0};
    int nf = split_csv(buf, fld, 16);
    if (nf < 7) return -1;
    if (!fld[2] || fld[2][0] != 'A') { fix->valid = 0; return -1; }

    int32_t lat = 0, lon = 0;
    if (!coord_to_udeg(fld[3], 1, &lat)) return -1;
    if (fld[4] && fld[4][0] == 'S') lat = -lat;
    if (!coord_to_udeg(fld[5], 0, &lon)) return -1;
    if (fld[6] && fld[6][0] == 'W') lon = -lon;

    fix->lat   = lat;
    fix->lon   = lon;
    fix->valid = 1;
    return 0;
}