/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 36_RTC
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to read 
 *                      the RTC IC.
 * 
 * Hardware Description: A DS1307 RTC is connected to the 
 *                       CH32V003F4P6 as follows:
 *           
 *                       VCC -> 5v
 *                       GND -> Gnd
 *                       SDA -> SDA (4.7k pupllup)
 *                       SCL -> SCL (4.7k pullup)
 *                      
 *                       - If you’re NOT using a backup cell, tie DS1307 VBAT to GND.
 *                       - DS1307 VCC is 4.5–5.5V. You can safely pull SDA/SCL up to 3.3V.
 *
 * Created August 23rd, 2025, 12:44 PM
 * Updated August 23rd, 2025, 12:44 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/

#include "ch32v00x.h"
#include "ch32v00x_rcc.h"
#include "ch32v00x_gpio.h"
#include "ch32v00x_i2c.h"
#include "debug.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* ================= I2C: PC2=SCL, PC1=SDA ================= */

static uint32_t g_i2c_hz = 400000;

static int wait_event(uint32_t evt, uint32_t ms){
    while (!I2C_CheckEvent(I2C1, evt)) { Delay_Ms(1); if (!ms--) return 0; }
    return 1;
}
static int wait_flag_set(FlagStatus (*fn)(I2C_TypeDef*, uint32_t), I2C_TypeDef *I2Cx, uint32_t flag, uint32_t ms){
    while (fn(I2Cx, flag) == RESET) { Delay_Ms(1); if (!ms--) return 0; }
    return 1;
}
static int wait_flag_clear(FlagStatus (*fn)(I2C_TypeDef*, uint32_t), I2C_TypeDef *I2Cx, uint32_t flag, uint32_t ms){
    while (fn(I2Cx, flag) != RESET) { Delay_Ms(1); if (!ms--) return 0; }
    return 1;
}
static void i2c_bus_soft_recover(void){
    I2C_SoftwareResetCmd(I2C1, ENABLE); Delay_Ms(1);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
}
static void I2C1_Lines_ToGPIO_OD(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Mode  = GPIO_Mode_Out_OD;          // open-drain GPIO
    g.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_1;   // PC2=SCL, PC1=SDA
    GPIO_Init(GPIOC, &g);
    GPIO_SetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_1); // release lines high
}
static void I2C1_Lines_ToAFOD(void){
    GPIO_InitTypeDef g = {0};
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Mode  = GPIO_Mode_AF_OD;           // back to I2C
    g.GPIO_Pin   = GPIO_Pin_2; GPIO_Init(GPIOC, &g); // SCL
    g.GPIO_Pin   = GPIO_Pin_1; GPIO_Init(GPIOC, &g); // SDA
}
static void i2c_bus_full_recover(void){
    I2C_Cmd(I2C1, DISABLE);
    I2C_DeInit(I2C1);
    I2C1_Lines_ToGPIO_OD();

    for (int i=0; i<18; ++i){
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)) break; // SDA released
        GPIO_ResetBits(GPIOC, GPIO_Pin_2); Delay_Us(5);
        GPIO_SetBits(GPIOC, GPIO_Pin_2);  Delay_Us(5);
    }
    // fake STOP
    GPIO_SetBits(GPIOC, GPIO_Pin_2);
    GPIO_ResetBits(GPIOC, GPIO_Pin_1); Delay_Us(5);
    GPIO_SetBits(GPIOC, GPIO_Pin_1);   Delay_Us(5);

    I2C1_Lines_ToAFOD();
    Delay_Us(10);
}
static int i2c_start(void){
    if (!wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 20)) {
        i2c_bus_soft_recover();
        if (!wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 20)) {
            i2c_bus_full_recover();
            if (!wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 20)) return 0;
        }
    }
    I2C_GenerateSTART(I2C1, ENABLE);
    return wait_event(I2C_EVENT_MASTER_MODE_SELECT, 20);
}
static void i2c_stop(void){
    I2C_GenerateSTOP(I2C1, ENABLE);
    (void)wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 10);
}
static int addr_tx(uint8_t ctrl8){
    I2C_Send7bitAddress(I2C1, ctrl8, I2C_Direction_Transmitter);
    return wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, 20);
}
static int addr_rx(uint8_t ctrl8){
    I2C_Send7bitAddress(I2C1, ctrl8, I2C_Direction_Receiver);
    return wait_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, 20);
}
static int send_byte(uint8_t b){
    I2C_SendData(I2C1, b);
    return wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, 20);
}
static void I2C1_Init_CH32(uint32_t clock_hz){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    gpio.GPIO_Pin   = GPIO_Pin_2; GPIO_Init(GPIOC, &gpio); // SCL
    gpio.GPIO_Pin   = GPIO_Pin_1; GPIO_Init(GPIOC, &gpio); // SDA

    I2C_InitTypeDef i2c = {0};
    i2c.I2C_ClockSpeed          = clock_hz;     // 100k or 400k
    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1         = 0x00;
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    g_i2c_hz = clock_hz;
}

/* ================= DS1307 ================= */

#define DS1307_ADDR_7B   0x68
#define DS1307_W         (DS1307_ADDR_7B << 1)
#define DS1307_R         (DS1307_ADDR_7B << 1)

#define DS1307_REG_SECONDS   0x00
#define DS1307_REG_MINUTES   0x01
#define DS1307_REG_HOURS     0x02
#define DS1307_REG_DAY       0x03  /* 1..7 */
#define DS1307_REG_DATE      0x04  /* 1..31 */
#define DS1307_REG_MONTH     0x05  /* 1..12 */
#define DS1307_REG_YEAR      0x06  /* 0..99 */
#define DS1307_REG_CONTROL   0x07

/* BCD helpers */
static inline uint8_t bin_to_bcd(uint8_t v){ return (uint8_t)(((v/10)<<4) | (v%10)); }
static inline uint8_t bcd_to_bin(uint8_t v){ return (uint8_t)(((v>>4)*10) + (v & 0x0F)); }

/* Low-level reg IO */
static int ds1307_write1(uint8_t reg, uint8_t val){
    if (!i2c_start()) return 0;
    if (!addr_tx(DS1307_W)) { i2c_stop(); return 0; }
    if (!send_byte(reg))    { i2c_stop(); return 0; }
    if (!send_byte(val))    { i2c_stop(); return 0; }
    i2c_stop();
    return 1;
}
static int ds1307_writeN(uint8_t reg, const uint8_t *buf, uint8_t len){
    if (!i2c_start()) return 0;
    if (!addr_tx(DS1307_W)) { i2c_stop(); return 0; }
    if (!send_byte(reg))    { i2c_stop(); return 0; }
    for (uint8_t i=0;i<len;i++){
        if (!send_byte(buf[i])) { i2c_stop(); return 0; }
    }
    i2c_stop();
    return 1;
}
static int ds1307_readN(uint8_t reg, uint8_t *buf, uint8_t len){
    if (!i2c_start()) return 0;
    if (!addr_tx(DS1307_W)) { i2c_stop(); return 0; }
    if (!send_byte(reg))    { i2c_stop(); return 0; }

    I2C_GenerateSTART(I2C1, ENABLE);
    if (!wait_event(I2C_EVENT_MASTER_MODE_SELECT, 20)) { i2c_stop(); return 0; }
    if (!addr_rx(DS1307_R)) { i2c_stop(); return 0; }

    for (uint8_t i=0; i<len; i++){
        if (i == len-1) I2C_AcknowledgeConfig(I2C1, DISABLE);
        if (!wait_flag_set(I2C_GetFlagStatus, I2C1, I2C_FLAG_RXNE, 20)){
            i2c_stop(); I2C_AcknowledgeConfig(I2C1, ENABLE); return 0;
        }
        buf[i] = I2C_ReceiveData(I2C1);
    }
    i2c_stop();
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 1;
}

/* 24-hour time structure */
typedef struct {
    uint8_t sec;   // 0..59
    uint8_t min;   // 0..59
    uint8_t hour;  // 0..23
    uint8_t day;   // 1..7  (user-defined; 1=Sun)
    uint8_t date;  // 1..31
    uint8_t month; // 1..12
    uint8_t year;  // 0..99 (20yy)
} rtc_time_t;

/* Read time/calendar (registers 0x00..0x06) */
static int ds1307_read_time(rtc_time_t *t){
    uint8_t b[7];
    if (!ds1307_readN(DS1307_REG_SECONDS, b, 7)) return 0;

    /* Seconds: bit7 = CH (1 = oscillator halted) */
    t->sec   = bcd_to_bin(b[0] & 0x7F);
    t->min   = bcd_to_bin(b[1] & 0x7F);

    /* Hours: handle 24h vs 12h; we'll force 24h */
    if (b[2] & 0x40){ // 12-hour mode
        uint8_t hr = b[2] & 0x1F;
        uint8_t pm = (b[2] & 0x20)?1:0;
        hr = bcd_to_bin(hr);
        if (pm && hr != 12) hr += 12;
        if (!pm && hr == 12) hr = 0;
        t->hour = hr;
    }else{
        t->hour = bcd_to_bin(b[2] & 0x3F);
    }

    t->day   = bcd_to_bin(b[3] & 0x07);
    t->date  = bcd_to_bin(b[4] & 0x3F);
    t->month = bcd_to_bin(b[5] & 0x1F);
    t->year  = bcd_to_bin(b[6]);
    return 1;
}

/* Write time/calendar (forces 24-hour mode, clears CH) */
static int ds1307_write_time(const rtc_time_t *t){
    uint8_t b[7];
    /* Seconds: ensure CH=0 (oscillator run) */
    b[0] = bin_to_bcd(t->sec) & 0x7F;          // CH=0
    b[1] = bin_to_bcd(t->min) & 0x7F;
    b[2] = bin_to_bcd(t->hour) & 0x3F;         // 24h
    b[3] = bin_to_bcd((t->day? t->day:1) & 0x07);
    b[4] = bin_to_bcd(t->date  & 0x3F);
    b[5] = bin_to_bcd(t->month & 0x1F);
    b[6] = bin_to_bcd(t->year);

    return ds1307_writeN(DS1307_REG_SECONDS, b, 7);
}

/* Ensure oscillator runs; if CH=1, clear it, optionally set sane time */
static int ds1307_start_osc_if_needed(void){
    uint8_t sec;
    if (!ds1307_readN(DS1307_REG_SECONDS, &sec, 1)) return 0;
    if (sec & 0x80){
        /* Clear CH and set seconds=0 (keep other fields unchanged if possible) */
        rtc_time_t t;
        if (ds1307_read_time(&t)){
            t.sec = 0;
            return ds1307_write_time(&t);
        }else{
            /* If read failed, write a sane default: Sat 2025-01-01 00:00:00 */
            rtc_time_t d = {0,0,0,6,1,1,25};
            return ds1307_write_time(&d);
        }
    }
    return 1;
}

/* Optional: Control register helper (square wave off by default) */
static int ds1307_set_control(uint8_t ctrl){
    return ds1307_write1(DS1307_REG_CONTROL, ctrl);
}

/* ================= Demo helpers ================= */

static const char* wkday_name(uint8_t d){
    static const char* names[8] = {"?","Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
    return (d>=1 && d<=7) ? names[d] : names[0];
}


/*******************************************************************************
 * Function: Main
 *
 * Returns: Nothing
 *
 * Description: Program entry point
 ******************************************************************************/
 
/* ---- toggle this to do a one-time set on next flash ---- */
#define RTC_ONE_TIME_SET   1   /* set to 0 after you confirm time is correct */

int main(void){
    SystemCoreClockUpdate();
    Delay_Init();

#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif

    printf("SystemClk:%ld\r\n", SystemCoreClock);
    printf("CH32V003 + DS1307 (I2C @ %lu Hz)\r\n", (unsigned long)g_i2c_hz);
    printf("Pins: PC2=SCL, PC1=SDA (pull-ups to 3.3V)\r\n");

    /* Init I2C */
    I2C1_Init_CH32(g_i2c_hz);

    /* Bring DS1307 oscillator up if halted (clears CH if needed) */
    if (!ds1307_start_osc_if_needed()){
        printf("DS1307: start osc failed, trying bus recovery...\r\n");
        i2c_bus_full_recover();
        I2C1_Init_CH32(g_i2c_hz);
        if (!ds1307_start_osc_if_needed()){
            printf("DS1307: FATAL init fail\r\n");
            while(1){ Delay_Ms(500); printf(".\r\n"); }
        }
    }

#if RTC_ONE_TIME_SET
    /* ---- ONE-TIME SET: change these, flash once, then set RTC_ONE_TIME_SET=0 ---- */
    {
        rtc_time_t new_time = {
            .sec   = 0,      // 0..59
            .min   = 44,      // 0..59
            .hour  = 12,     // 0..23 (24h mode)
            .day   = 7,      // 1=Sun .. 7=Sat (choose correct weekday)
            .date  = 23,     // 1..31
            .month = 8,      // 1..12
            .year  = 25      // 00..99 => 20yy (25 = 2025)
        };
        if (ds1307_write_time(&new_time)) {
            printf("RTC set OK\r\n");
            rtc_time_t chk;
            if (ds1307_read_time(&chk)) {
                printf("Confirm: %s %02u-%02u-20%02u %02u:%02u:%02u\r\n",
                       wkday_name(chk.day),
                       chk.date, chk.month, chk.year,
                       chk.hour, chk.min, chk.sec);
            }
        } else {
            printf("RTC set FAILED, trying recovery...\r\n");
            i2c_bus_full_recover();
            I2C1_Init_CH32(g_i2c_hz);
            if (ds1307_write_time(&new_time)) printf("RTC set OK after recovery\r\n");
            else printf("RTC still not responding\r\n");
        }
    }
#endif

    /* Optional: disable SQW (OUT=0, SQWE=0) */
    (void)ds1307_set_control(0x00);

    /* Read once to verify basic comms (optional) */
    rtc_time_t now;
    if (!ds1307_read_time(&now)){
        printf("DS1307 read failed, recovering...\r\n");
        i2c_bus_full_recover();
        I2C1_Init_CH32(g_i2c_hz);
        (void)ds1307_start_osc_if_needed();
    }

    /* Main loop: read/print once per second */
    while (1){
        if (ds1307_read_time(&now)){
            printf("%s %02u-%02u-20%02u  %02u:%02u:%02u\r\n",
                   wkday_name(now.day),
                   now.date, now.month, now.year,
                   now.hour, now.min, now.sec);
        } else {
            /* Recover on read failure */
            i2c_bus_full_recover();
            I2C1_Init_CH32(g_i2c_hz);
            (void)ds1307_start_osc_if_needed();
            printf("I2C/RTC recovered\r\n");
        }
        Delay_Ms(1000);
    }
}

