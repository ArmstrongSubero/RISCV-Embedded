/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 51_GPS_Library
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.2
 *                  * Cleaned up code
 *                  * Added additional comments
 *                  * Switched to 64-bit intermediates and cast to 32-bit 
 *                    microdegress
 *                
 * Program Description: This Program allows the CH32V003F4P6 to communicate 
 *                      with a Neo-6M module over GPS, parse the data and 
 *                      output map coordinates in a format that can be interpreted 
 *                      by map software. The library is lean checksum enforced, 
*                       RMC-only, and dosen't use a lot of resources, no floats/strtok/atof.
 * 
 * Hardware Description: A NEO-6M module is connected to the 
 *                       CH32V003F4P6 as follows:
 *           
 *                       VCC -> 5v
 *                       GND -> Gnd
 *                       TX  -> PD2
 *                       
 *                       Additionally a WCH Link-E is used to read the module data and 
 *                       dump it via UART 
 *
 *
 * Created August 18th, 2025, 6:28 PM
 * Updated August 18th, 2025, 6:42 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>

/* ===== Pins & Baud ===== */
#define SUART_BAUDRATE     9600
#define SUART_RX_PORT      GPIOD
#define SUART_RX_PIN       GPIO_Pin_2
/* ======================= */


/*******************************************************************************
 *Timer Configuration 
 ******************************************************************************/
/* ---- TIM2 1 MHz timebase ---- */
static void timebase_init_1mhz(void){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef t = {0};
    t.TIM_Prescaler     = (SystemCoreClock / 1000000UL) - 1; // 1 MHz
    t.TIM_CounterMode   = TIM_CounterMode_Up;
    t.TIM_Period        = 0xFFFF;
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &t);
    TIM_Cmd(TIM2, ENABLE);
}
static inline uint16_t now_t2(void){ return TIM2->CNT; }
static inline void wait_until_t2(uint16_t deadline){
    while ((int16_t)(TIM2->CNT - deadline) < 0) { /* spin */ }
}

/*******************************************************************************
 *Software UART
 ******************************************************************************/
/* ---- GPIO ---- */
static inline uint8_t rx_read_fast(void){
    return (SUART_RX_PORT->INDR & SUART_RX_PIN) ? 1 : 0;
}

/* ---- SUART RX-only init ---- */
#define BIT_TICKS ((uint16_t)(1000000UL / SUART_BAUDRATE))  // 104 @9600
static void SUART_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = SUART_RX_PIN;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Mode  = GPIO_Mode_IPU;         // input, pull-up → idle high
    GPIO_Init(SUART_RX_PORT, &g);
    timebase_init_1mhz();
}

/* ---- RX one byte (falling-edge detect + absolute mid-bit sampling) ---- */
static uint8_t SUART_Receive(void)
{
    while (rx_read_fast() == 0) { /* wait for idle high */ }
    while (rx_read_fast() != 0) { /* wait for start bit falling edge */ }

    __disable_irq();
    uint16_t t = now_t2();
    t += (uint16_t)(BIT_TICKS + BIT_TICKS/2);      // mid bit0

    uint8_t v = 0;
    for (uint8_t i=0;i<8;i++){
        wait_until_t2(t);
        if (rx_read_fast()) v |= (1U<<i);
        t += BIT_TICKS;
    }
    __enable_irq();
    return v;
}

/*******************************************************************************
 *Decode GPS Data
 ******************************************************************************/

/* ---- read one NMEA line starting with '$' and ending at '\n' ---- */
static int SUART_Read_NMEA(char *dst, int maxlen)
{
    uint8_t ch;
    do { ch = SUART_Receive(); } while (ch != '$');
    int len = 0;
    dst[len++] = '$';
    while (1) {
        ch = SUART_Receive();
        if (len < maxlen-1) dst[len++] = (char)ch;
        if (ch == '\n') break;
        if (len >= maxlen-1) break;
    }
    dst[len] = 0;
    return len;
}

/* ---- checksum "$...*HH" (returns 1 if OK) ---- */
static int nmea_checksum_ok(const char *s)
{
    if (!s || s[0] != '$') return 0;
    uint8_t cs = 0;
    const char *p = s + 1;
    while (*p && *p != '*' && *p != '\r' && *p != '\n') cs ^= (uint8_t)(*p++);
    if (*p != '*') return 0;
    char h1 = *(p+1), h2 = *(p+2);
    if (!isxdigit((unsigned char)h1) || !isxdigit((unsigned char)h2)) return 0;
    int hi = (h1 <= '9') ? h1 - '0' : (toupper(h1) - 'A' + 10);
    int lo = (h2 <= '9') ? h2 - '0' : (toupper(h2) - 'A' + 10);
    return (cs == (uint8_t)((hi << 4) | lo));
}

/* ---- small CSV split (no strtok) ---- */
static int split_csv(char *line, char *out[], int maxf)
{
    int n = 0;
    if (!line || !*line) return 0;
    out[n++] = line;
    for (char *p = line; *p && n < maxf; ++p){
        if (*p == ','){ *p = 0; out[n++] = p+1; }
        if (*p == '*'){ *p = 0; break; } // stop before checksum
    }
    return n;
}

/* Convert "ddmm.mmmm" (lat) or "dddmm.mmmm" (lon) to signed micro-degrees.
   Returns 1 on success and writes to *out (e.g., 10657432 == 10.657432°). */
static int coord_to_microdeg(const char *field, int is_lat, int32_t *out)
{
    if (!field || !*field) return 0;

    int dlen = is_lat ? 2 : 3;
    int deg = 0, i = 0;
    for (; i < dlen && isdigit((unsigned char)field[i]); ++i)
        deg = deg*10 + (field[i]-'0');
    if (i != dlen) return 0;

    if (!isdigit((unsigned char)field[i]) || !isdigit((unsigned char)field[i+1]))
        return 0;
    int min_int = (field[i]-'0')*10 + (field[i+1]-'0');
    i += 2;

    int frac = 0, scale = 1;
    if (field[i] == '.') {
        ++i;
        while (isdigit((unsigned char)field[i]) && scale < 1000000) {
            frac  = frac*10 + (field[i]-'0');
            scale = scale*10;
            ++i;
        }
    }

    /* 64-bit accumulation, final value fits 32-bit: ±180,000,000 */
    int64_t micro = (int64_t)deg * 1000000LL;
    micro += ((int64_t)min_int * 1000000LL + 30) / 60;                          // minutes
    micro += ((int64_t)frac    * 1000000LL + (int64_t)scale*30)
             / ((int64_t)scale * 60);                                           // fractional minutes

    *out = (int32_t)micro;
    return 1;
}

/* print signed micro-degrees as D.DDDDDD without using %f */
static void print_microdeg_plain(int32_t udeg)
{
    char sign = '+';
    if (udeg < 0){ sign = '-'; udeg = -udeg; }
    int32_t whole = udeg / 1000000;
    int32_t frac  = udeg % 1000000;
    // print: sign + whole.frac(6)
    printf("%c%ld.%06ld", sign, (long)whole, (long)frac);
}



/*******************************************************************************
 * Function: Main
 *
 * Returns: Nothing
 *
 * Description: Program entry point
 ******************************************************************************/
static char nmea[128];

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);      // hardware printf UART to PC
#endif

    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

    SUART_Init();                   // PD2 = RX from GPS (9600 8N1)
    printf("GPS ready on PD2 (9600 8N1). Waiting for RMC...\r\n");

    while (1)
    {
        int L = SUART_Read_NMEA(nmea, sizeof(nmea));  // blocking
        if (L <= 0) continue;

        // Enforce checksum and sentence type ($GPRMC / $GNRMC only)
        if (!nmea_checksum_ok(nmea)) continue;
        if (strncmp(nmea, "$GPRMC", 6) && strncmp(nmea, "$GNRMC", 6)) continue;

        // Split fields (RMC: 0:$GPRMC 1:time 2:status 3:lat 4:N/S 5:lon 6:E/W ...)
        char buf[128]; strncpy(buf, nmea, sizeof(buf)-1); buf[sizeof(buf)-1]=0;
        char *fld[16] = {0};
        int nf = split_csv(buf, fld, 16);
        if (nf < 7) continue;
        if (!fld[2] || fld[2][0] != 'A') continue; // only valid fixes

        int32_t lat_udeg=0, lon_udeg=0;
        if (!coord_to_microdeg(fld[3], 1, &lat_udeg)) continue;
        if (fld[4] && fld[4][0]=='S') lat_udeg = -lat_udeg;
        if (!coord_to_microdeg(fld[5], 0, &lon_udeg)) continue;
        if (fld[6] && fld[6][0]=='W') lon_udeg = -lon_udeg;

        // print the map coordinates
        printf("Maps coordinates: ");
        print_microdeg_plain(lat_udeg);
        printf(", ");
        print_microdeg_plain(lon_udeg);
        printf("\r\n");
    }
}
