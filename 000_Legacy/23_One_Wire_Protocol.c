/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/ Int OSC @ 48 MHz, 3.3V
 * Program: 23_One_Wire_Protocol
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: In this example we use the CH32V003F4P6 to bit-bang 
 *                      the one-wire protocol 
 *
 * Hardware Description: A DS1820 device is connected to pin PD4 with a 4.7k 
 *                       pullup resistor.
 *
 * Created:  August 23rd, 2025, 3:14 PM
 * Updated:  August 23rd, 2025, 3:14 PM
 */
 
/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"
#include "ch32v00x_rcc.h"
#include "ch32v00x_gpio.h"
#include <stdint.h>
#include <stdio.h>

/* ===================== User Config ===================== */
#define ONEWIRE_PORT        GPIOD
#define ONEWIRE_PIN         GPIO_Pin_4         // PD4 = 1-Wire DQ
#define READ_PERIOD_MS      1000               // print once per second

/* ===================== Low-level 1-Wire line helpers ===================== */
/* We configure the pin as Open-Drain output, and use:
 *   - drive LOW  -> pull line low
 *   - drive HIGH -> release line (external pull-up pulls high)
 * Reads are done from INDR.
 */
static void onewire_gpio_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = ONEWIRE_PIN;
    g.GPIO_Speed = GPIO_Speed_10MHz;
    g.GPIO_Mode  = GPIO_Mode_Out_OD;   // open-drain
    GPIO_Init(ONEWIRE_PORT, &g);

    // release line high (OD "high" = Hi-Z)
    ONEWIRE_PORT->BSHR = ONEWIRE_PIN;
}

static inline void dq_low(void)    { ONEWIRE_PORT->BCR  = ONEWIRE_PIN; }
static inline void dq_release(void){ ONEWIRE_PORT->BSHR = ONEWIRE_PIN; }
static inline uint8_t dq_read(void){ return (ONEWIRE_PORT->INDR & ONEWIRE_PIN) ? 1u : 0u; }

/* ===================== 1-Wire timing primitives ===================== */
/* Standard speed timing (µs):
 * Reset:  pull low >=480us, release; sample presence 60-240us later.
 * Write 1: low 6us, release for rest of 60us slot.
 * Write 0: low 60us.
 * Read:   low 6us, release, sample around +9..15us, full slot ≈60us.
 */

static int onewire_reset_presence(void)
{
    __disable_irq();

    dq_low();            // reset pulse
    Delay_Us(500);
    dq_release();
    Delay_Us(60);        // wait before sampling presence

    int present = (dq_read() == 0);    // presence pulse pulls low

    __enable_irq();

    // finish the slot: presence can last up to ~240us after release
    Delay_Us(420);

    return present;
}

static void onewire_write_bit(uint8_t bit)
{
    __disable_irq();

    if (bit) {
        dq_low();
        Delay_Us(6);     // short low
        dq_release();
        Delay_Us(64);    // rest of slot
    } else {
        dq_low();
        Delay_Us(60);    // hold low whole slot
        dq_release();
        Delay_Us(10);
    }

    __enable_irq();
}

static uint8_t onewire_read_bit(void)
{
    uint8_t v;

    __disable_irq();

    dq_low();
    Delay_Us(6);         // initiate read slot
    dq_release();
    Delay_Us(9);         // wait to sample at ~15us from start (6+9)
    v = dq_read();
    __enable_irq();

    // finish the slot
    Delay_Us(55);

    return v;
}

static void onewire_write_byte(uint8_t b)
{
    for (uint8_t i=0;i<8;i++) {
        onewire_write_bit( (b >> i) & 1u );
    }
}

static uint8_t onewire_read_byte(void)
{
    uint8_t v = 0;
    for (uint8_t i=0;i<8;i++) {
        v |= (onewire_read_bit() << i);
    }
    return v;
}

/* ===================== Dallas/Maxim CRC-8 ===================== */
static uint8_t crc8_dallas(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i=0;i<len;i++){
        uint8_t in = data[i];
        for (uint8_t b=0;b<8;b++){
            uint8_t mix = (crc ^ in) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            in >>= 1;
        }
    }
    return crc;
}

/* ===================== DS18x20 high-level ops ===================== */
/* Commands */
#define CMD_SKIP_ROM        0xCC
#define CMD_CONVERT_T       0x44
#define CMD_READ_SCRATCHPAD 0xBE

/* Start temperature conversion (Skip ROM).
 * For parasite power, some designs "strong pull-up" the line after issuing 0x44.
 * Since we’re open-drain with an external pull-up, we just release the line.
 * Conversion complete can be polled: device outputs '1' when done.
 */
static int ds18x20_start_convert_t(void)
{
    if (!onewire_reset_presence()) return 0;
    onewire_write_byte(CMD_SKIP_ROM);
    onewire_write_byte(CMD_CONVERT_T);

    // For parasite power strong pull-up, you’d hold the line high strongly here.
    // With an external pull-up this is usually fine at 3.3V.
    return 1;
}

/* Poll for conversion done (returns 1 when finished or timeout reached). */
static int ds18x20_wait_convert_done(uint32_t timeout_ms)
{
    while (timeout_ms--) {
        // During conversion, read slots return 0; return 1 when done.
        if (onewire_read_bit()) return 1;
        Delay_Ms(1);
    }
    return 0; // timed out
}

/* Read 9-byte scratchpad into buf[9]. Returns 1 on OK + CRC valid. */
static int ds18x20_read_scratchpad(uint8_t *buf9)
{
    if (!onewire_reset_presence()) return 0;
    onewire_write_byte(CMD_SKIP_ROM);
    onewire_write_byte(CMD_READ_SCRATCHPAD);
    for (int i=0;i<9;i++) buf9[i] = onewire_read_byte();

    uint8_t crc = crc8_dallas(buf9, 8);
    return (crc == buf9[8]);
}

/* Convert raw scratchpad temp to centi-degrees C (°C x100).
 * DS18B20: 12-bit default, raw is in units of 1/16°C => °C*100 = raw*100/16
 * Handles negative properly (two's complement).
 */
static int ds18x20_temp_c_x100_from_scratch(const uint8_t *sp9, int *out_cx100)
{
    int16_t raw = (int16_t)((sp9[1] << 8) | sp9[0]);
    // Some older DS1820 parts use different resolution; this code assumes DS18B20.
    // If you use DS18S20 (9-bit, LSB=0.5C), adapt as needed.

    // scale: *100/16 = *6.25. Do integer rounding toward zero.
    int val = ( (int)raw * 100 + (raw>=0 ? 8 : -8) ) / 16;

    *out_cx100 = val;
    return 1;
}

/* One-shot: start conversion, wait, read scratchpad, compute temp.
 * max_time_ms: up to ~750 ms for 12-bit; we poll early-finish.
 */
static int ds18x20_read_temperature_cx100(int *out_cx100, uint32_t max_time_ms)
{
    if (!ds18x20_start_convert_t()) return 0;
    if (!ds18x20_wait_convert_done(max_time_ms)) return 0;

    uint8_t sp[9];
    if (!ds18x20_read_scratchpad(sp)) return 0;

    return ds18x20_temp_c_x100_from_scratch(sp, out_cx100);
}


/*******************************************************************************
 * Function: Main
 *
 * Returns: Nothing
 *
 * Description: Program entry point
 ******************************************************************************/
int main(void)
{
    SystemCoreClockUpdate();
    Delay_Init();

#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif

    printf("SystemClk:%ld\r\n", SystemCoreClock);
    printf("CH32V003 + DS18x20 (1-Wire) on PD4\r\n");

    onewire_gpio_init();

    // Probe presence once
    if (!onewire_reset_presence()){
        printf("No 1-Wire presence on PD4. Check wiring and pull-up.\r\n");
    } else {
        printf("1-Wire device present.\r\n");
    }

    while (1){
        int tcx100 = 0;
        if (ds18x20_read_temperature_cx100(&tcx100, 800)){
            // print as +/-xxx.yy C
            int sign = (tcx100 < 0) ? -1 : 1;
            int v = tcx100 * sign;
            printf("T = %s%d.%02d C\r\n", (sign<0)?"-":"", v/100, v%100);
        } else {
            printf("Read failed (timeout/CRC). Retrying...\r\n");
        }
        Delay_Ms(READ_PERIOD_MS);
    }
}
