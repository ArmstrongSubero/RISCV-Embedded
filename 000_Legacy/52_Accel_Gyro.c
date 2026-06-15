/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 52_Accel_Gyro
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to demonstrate 
 *                      use of the acceleometer and gyroscope with the 
 *                      MPU6050 module 
 * 
 * Hardware Description: An MPU6050 module is connected to the I2C bus 
 *                       on the CH32v003F4P6: 
 *
 *                       SCL -> PC2 
 *                       SDA -> PC1 
 *
 *
 * Created August 18th, 2025, 12:02 AM
 * Updated August 18th, 2025, 12:02 AM
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


/*******************************************************************************
 * I2C Helper Stuff PC2 = SCL, PC1 = SDA
 ******************************************************************************/

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
    i2c.I2C_ClockSpeed          = clock_hz;     // 400k recommended with good pull-ups
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


/*******************************************************************************
 * MPU6050 Helpers 
 ******************************************************************************/

#ifndef MPU_ADDR_7B
#define MPU_ADDR_7B        0x68       // set to 0x69 if AD0=HIGH
#endif
#define MPU_ADDR_W         (MPU_ADDR_7B << 1)
#define MPU_ADDR_R         (MPU_ADDR_7B << 1)

#define REG_SMPLRT_DIV     0x19
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_ACCEL_XOUT_H   0x3B
#define REG_PWR_MGMT_1     0x6B
#define REG_WHO_AM_I       0x75

/* INT/USER (for DRDY gating & FIFO disable) */
#define REG_INT_ENABLE     0x38
#define REG_INT_STATUS     0x3A
#define REG_INT_PIN_CFG    0x37
#define REG_USER_CTRL      0x6A
#define REG_FIFO_EN        0x23

/* ---- I2C reg helpers ---- */
static int mpu_write_reg(uint8_t reg, uint8_t val){
    if (!i2c_start()) return 0;
    if (!addr_tx(MPU_ADDR_W)) { i2c_stop(); return 0; }
    if (!send_byte(reg))      { i2c_stop(); return 0; }
    if (!send_byte(val))      { i2c_stop(); return 0; }
    i2c_stop();
    return 1;
}
static int mpu_read_reg(uint8_t reg, uint8_t *val){
    if (!i2c_start()) return 0;
    if (!addr_tx(MPU_ADDR_W)) { i2c_stop(); return 0; }
    if (!send_byte(reg))      { i2c_stop(); return 0; }

    I2C_GenerateSTART(I2C1, ENABLE);
    if (!wait_event(I2C_EVENT_MASTER_MODE_SELECT, 20)) { i2c_stop(); return 0; }
    if (!addr_rx(MPU_ADDR_R)) { i2c_stop(); return 0; }

    I2C_AcknowledgeConfig(I2C1, DISABLE);
    if (!wait_flag_set(I2C_GetFlagStatus, I2C1, I2C_FLAG_RXNE, 20)) { i2c_stop(); I2C_AcknowledgeConfig(I2C1, ENABLE); return 0; }
    *val = I2C_ReceiveData(I2C1);
    i2c_stop();
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 1;
}

/* 14B burst read */
static int mpu_read_multi(uint8_t reg, uint8_t *buf, uint8_t len){
    if (!i2c_start()) return 0;
    if (!addr_tx(MPU_ADDR_W)) { i2c_stop(); return 0; }
    if (!send_byte(reg))      { i2c_stop(); return 0; }

    I2C_GenerateSTART(I2C1, ENABLE);
    if (!wait_event(I2C_EVENT_MASTER_MODE_SELECT, 20)) { i2c_stop(); return 0; }
    if (!addr_rx(MPU_ADDR_R)) { i2c_stop(); return 0; }

    for (uint8_t i = 0; i < len; i++) {
        if (i == len - 1) I2C_AcknowledgeConfig(I2C1, DISABLE);
        if (!wait_flag_set(I2C_GetFlagStatus, I2C1, I2C_FLAG_RXNE, 20)) {
            i2c_stop(); I2C_AcknowledgeConfig(I2C1, ENABLE); return 0;
        }
        buf[i] = I2C_ReceiveData(I2C1);
    }
    i2c_stop();
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 1;
}

/* frame sanity */
static int mpu_frame_sane(const uint8_t *b, uint8_t len){
    uint8_t orv=0, andv=0xFF;
    for (uint8_t i=0;i<len;i++){ orv |= b[i]; andv &= b[i]; }
    if (orv==0x00 || andv==0xFF) return 0;      // all 0x00 or all 0xFF -> bad bus
    int16_t traw = (int16_t)((b[6]<<8)|b[7]);
    if (traw < -15000 || traw > 15000) return 0; // temp range guard
    return 1;
}

/* robust 14B read with recoveries */
static int mpu_read_multi_robust(uint8_t reg, uint8_t *buf, uint8_t len){
    for (int attempt=0; attempt<3; ++attempt){
        if (mpu_read_multi(reg, buf, len) && mpu_frame_sane(buf,len)) return 1;
        i2c_bus_soft_recover();
        Delay_Us(200);
    }
    // Full bus recovery + minimal reconfigure
    i2c_bus_full_recover();
    I2C1_Init_CH32(g_i2c_hz);
    // Reconfigure sensor to a known good state (no FIFO, DRDY enabled)
    (void)mpu_write_reg(REG_USER_CTRL, 0x00);
    (void)mpu_write_reg(REG_FIFO_EN,   0x00);
    (void)mpu_write_reg(REG_INT_ENABLE,0x01);
    Delay_Ms(2);
    return (mpu_read_multi(reg, buf, len) && mpu_frame_sane(buf,len));
}

/* minimal runtime config (non-FIFO) */
static int mpu_set_dlpf(uint8_t cfg){ return mpu_write_reg(REG_CONFIG, (cfg & 7)); }
static int mpu_reconfigure_min(void){
    // Wake, PLL on X gyro
    if (!mpu_write_reg(REG_PWR_MGMT_1, 0x01)) return 0;
    Delay_Ms(2);
    // DLPF ~42 Hz, 100 Hz sample
    if (!mpu_set_dlpf(3)) return 0;
    if (!mpu_write_reg(REG_SMPLRT_DIV, 9)) return 0;     // 1kHz/(1+9)=100Hz
    if (!mpu_write_reg(REG_GYRO_CONFIG,  0x00)) return 0;// ±250 dps
    if (!mpu_write_reg(REG_ACCEL_CONFIG, 0x00)) return 0;// ±2 g
    // Make sure FIFO is OFF and enable DRDY (we poll INT_STATUS)
    (void)mpu_write_reg(REG_USER_CTRL, 0x00);
    (void)mpu_write_reg(REG_FIFO_EN,   0x00);
    (void)mpu_write_reg(REG_INT_PIN_CFG, 0x00);          // push-pull, active high, 50us
    (void)mpu_write_reg(REG_INT_ENABLE,  0x01);          // DATA_RDY_EN
    return 1;
}
static int mpu_init(void){
    uint8_t who=0;
    if (!mpu_read_reg(REG_WHO_AM_I, &who)) return 0;
    if ((who & 0x7E) != 0x68) printf("WHO_AM_I=0x%02X (expect 0x68)\r\n", who);
    if (!mpu_reconfigure_min()) return 0;
    return 1;
}

/* DRDY wait (poll INT_STATUS bit0) */
static int mpu_wait_data_ready(uint32_t ms){
    uint8_t st=0;
    while (ms--){
        if (mpu_read_reg(REG_INT_STATUS, &st) && (st & 0x01)) return 1;
        Delay_Ms(1);
    }
    return 0;
}


/*******************************************************************************
 * Raw Read 
 ******************************************************************************/
typedef struct { int16_t ax, ay, az, temp, gx, gy, gz; } mpu_raw_t;
static int mpu_read_raw(mpu_raw_t *o){
    uint8_t b[14];
    if (!mpu_read_multi_robust(REG_ACCEL_XOUT_H, b, 14)) return 0;
    o->ax = (int16_t)((b[0] << 8) | b[1]);
    o->ay = (int16_t)((b[2] << 8) | b[3]);
    o->az = (int16_t)((b[4] << 8) | b[5]);
    o->temp= (int16_t)((b[6] << 8) | b[7]);
    o->gx = (int16_t)((b[8] << 8) | b[9]);
    o->gy = (int16_t)((b[10] << 8) | b[11]);
    o->gz = (int16_t)((b[12] << 8) | b[13]);
    return 1;
}

/*******************************************************************************
 * Fixed-Point Scaling and Smoothing
 ******************************************************************************/
static inline int32_t raw_to_mg(int16_t r){ return ((int32_t)r * 1000 + (r>=0?8192:-8192)) / 16384; }
static inline int32_t raw_to_mdps(int16_t r){ return ((int32_t)r * 1000 + (r>=0?65:-65)) / 131; }
static inline int32_t temp_to_c_x100(int16_t t){ return ((int32_t)t * 100 + (t>=0?169:-169)) / 340 + 3653; }

static int32_t g_ax_off_mg=0, g_ay_off_mg=0, g_az_off_mg=0;
static int32_t g_gx_off_mdps=0, g_gy_off_mdps=0, g_gz_off_mdps=0;

static inline int32_t median3(int32_t a, int32_t b, int32_t c){
    if (a > b){ int32_t t=a; a=b; b=t; }
    if (b > c){ int32_t t=b; b=c; c=t; }
    if (a > b){ int32_t t=a; a=b; b=t; }
    return b;
}


/*******************************************************************************
 * Calibration
 ******************************************************************************/
#define ACC_SMOOTH_SHIFT   3   // alpha=1/8
#define GYR_SMOOTH_SHIFT   2   // alpha=1/4

static int32_t s_ax=0, s_ay=0, s_az=0;
static int32_t s_gx=0, s_gy=0, s_gz=0;
static int32_t init_done = 0;

static void mpu_calibrate(uint16_t samples, uint16_t ms_between){
    int64_t sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;

    mpu_set_dlpf(6); // ~5 Hz for quieter mean
    Delay_Ms(50);

    for (uint16_t i=0; i<samples; ++i){
        // Gate by DRDY to ensure fresh/consistent samples during calib
        if (!mpu_wait_data_ready(20)) continue;
        mpu_raw_t r;
        if (mpu_read_raw(&r)){
            sax += raw_to_mg(r.ax);
            say += raw_to_mg(r.ay);
            saz += raw_to_mg(r.az);
            sgx += raw_to_mdps(r.gx);
            sgy += raw_to_mdps(r.gy);
            sgz += raw_to_mdps(r.gz);
        }
        Delay_Ms(ms_between);
    }

    int32_t ax_avg = (int32_t)(sax / samples);
    int32_t ay_avg = (int32_t)(say / samples);
    int32_t az_avg = (int32_t)(saz / samples);
    int32_t gx_avg = (int32_t)(sgx / samples);
    int32_t gy_avg = (int32_t)(sgy / samples);
    int32_t gz_avg = (int32_t)(sgz / samples);

    g_ax_off_mg = ax_avg;
    g_ay_off_mg = ay_avg;
    g_az_off_mg = az_avg - 1000; // expect +1g on Z when flat

    g_gx_off_mdps = gx_avg;
    g_gy_off_mdps = gy_avg;
    g_gz_off_mdps = gz_avg;

    printf("Calib Aoff mg: X=%ld Y=%ld Z=%ld | Goff mdps: X=%ld Y=%ld Z=%ld\r\n",
           (long)g_ax_off_mg,(long)g_ay_off_mg,(long)g_az_off_mg,
           (long)g_gx_off_mdps,(long)g_gy_off_mdps,(long)g_gz_off_mdps);

    mpu_set_dlpf(3); // ~42 Hz
    Delay_Ms(10);

    s_ax = 0; s_ay = 0; s_az = 1000;
    s_gx = 0; s_gy = 0; s_gz = 0;
    init_done = 1;
}


static void filter_seed(void){
    mpu_raw_t r;
    for (int i=0;i<3;i++){
        if (!mpu_wait_data_ready(20)) continue;
        if (mpu_read_raw(&r)){
            int32_t ax = raw_to_mg(r.ax) - g_ax_off_mg;
            int32_t ay = raw_to_mg(r.ay) - g_ay_off_mg;
            int32_t az = raw_to_mg(r.az) - g_az_off_mg;
            int32_t gx = raw_to_mdps(r.gx) - g_gx_off_mdps;
            int32_t gy = raw_to_mdps(r.gy) - g_gy_off_mdps;
            int32_t gz = raw_to_mdps(r.gz) - g_gz_off_mdps;

            if (ax < -4000) ax = -4000; if (ax > 4000) ax = 4000;
            if (ay < -4000) ay = -4000; if (ay > 4000) ay = 4000;
            if (az < -4000) az = -4000; if (az > 4000) az = 4000;
            if (gx < -250000) gx = -250000; if (gx > 250000) gx = 250000;
            if (gy < -250000) gy = -250000; if (gy > 250000) gy = 250000;
            if (gz < -250000) gz = -250000; if (gz > 250000) gz = 250000;

            if (i==0){
                s_ax=ax; s_ay=ay; s_az=az;
                s_gx=gx; s_gy=gy; s_gz=gz;
            }else{
                s_ax += (ax - s_ax) >> ACC_SMOOTH_SHIFT;
                s_ay += (ay - s_ay) >> ACC_SMOOTH_SHIFT;
                s_az += (az - s_az) >> ACC_SMOOTH_SHIFT;
                s_gx += (gx - s_gx) >> GYR_SMOOTH_SHIFT;
                s_gy += (gy - s_gy) >> GYR_SMOOTH_SHIFT;
                s_gz += (gz - s_gz) >> GYR_SMOOTH_SHIFT;
            }
        }
        Delay_Ms(5);
    }
    init_done = 1;
}

/*******************************************************************************
 * One Filtered step, median-of-3 then IIR
 ******************************************************************************/
static void filter_step(const mpu_raw_t *r)
{
    static int32_t ax_hist[2]={0,0}, ay_hist[2]={0,0}, az_hist[2]={0,0};
    static int32_t gx_hist[2]={0,0}, gy_hist[2]={0,0}, gz_hist[2]={0,0};

    int32_t ax = raw_to_mg(r->ax) - g_ax_off_mg;
    int32_t ay = raw_to_mg(r->ay) - g_ay_off_mg;
    int32_t az = raw_to_mg(r->az) - g_az_off_mg;
    int32_t gx = raw_to_mdps(r->gx) - g_gx_off_mdps;
    int32_t gy = raw_to_mdps(r->gy) - g_gy_off_mdps;
    int32_t gz = raw_to_mdps(r->gz) - g_gz_off_mdps;

    if (ax < -4000) ax = -4000; if (ax > 4000) ax = 4000;
    if (ay < -4000) ay = -4000; if (ay > 4000) ay = 4000;
    if (az < -4000) az = -4000; if (az > 4000) az = 4000;
    if (gx < -250000) gx = -250000; if (gx > 250000) gx = 250000;
    if (gy < -250000) gy = -250000; if (gy > 250000) gy = 250000;
    if (gz < -250000) gz = -250000; if (gz > 250000) gz = 250000;

    int32_t ax_m = median3(ax_hist[0], ax_hist[1], ax);
    int32_t ay_m = median3(ay_hist[0], ay_hist[1], ay);
    int32_t az_m = median3(az_hist[0], az_hist[1], az);
    int32_t gx_m = median3(gx_hist[0], gx_hist[1], gx);
    int32_t gy_m = median3(gy_hist[0], gy_hist[1], gy);
    int32_t gz_m = median3(gz_hist[0], gz_hist[1], gz);

    ax_hist[1]=ax_hist[0]; ax_hist[0]=ax;
    ay_hist[1]=ay_hist[0]; ay_hist[0]=ay;
    az_hist[1]=az_hist[0]; az_hist[0]=az;
    gx_hist[1]=gx_hist[0]; gx_hist[0]=gx;
    gy_hist[1]=gy_hist[0]; gy_hist[0]=gy;
    gz_hist[1]=gz_hist[0]; gz_hist[0]=gz;

    if (!init_done){
        s_ax=ax_m; s_ay=ay_m; s_az=az_m; s_gx=gx_m; s_gy=gy_m; s_gz=gz_m;
        init_done=1;
    }else{
        s_ax += (ax_m - s_ax) >> ACC_SMOOTH_SHIFT;
        s_ay += (ay_m - s_ay) >> ACC_SMOOTH_SHIFT;
        s_az += (az_m - s_az) >> ACC_SMOOTH_SHIFT;

        s_gx += (gx_m - s_gx) >> GYR_SMOOTH_SHIFT;
        s_gy += (gy_m - s_gy) >> GYR_SMOOTH_SHIFT;
        s_gz += (gz_m - s_gz) >> GYR_SMOOTH_SHIFT;
    }
}


/*******************************************************************************
 * Demo Print 
 ******************************************************************************/
static void print_filtered_line(int32_t temp_c_x100)
{
    printf("ACC mg: X=%ld Y=%ld Z=%ld | GY mdps: X=%ld Y=%ld Z=%ld | T=%ld.%02ldC\r\n",
           (long)s_ax,(long)s_ay,(long)s_az,
           (long)s_gx,(long)s_gy,(long)s_gz,
           (long)(temp_c_x100/100),(long)(temp_c_x100%100));
}


/*******************************************************************************
 * Function: Main
 *
 * Returns: Nothing
 *
 * Description: Program entry point
 ******************************************************************************/
int main(void){
    SystemCoreClockUpdate();
    Delay_Init();

#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif

    printf("SystemClk:%ld\r\n", SystemCoreClock);
    printf("CH32V003 + MPU-6050 (I2C @ 400kHz), calib + smoothing, NO FIFO, DRDY-gated\r\n");
    printf("Pins: PC2=SCL, PC1=SDA (pull-ups to 3.3V)\r\n");

    I2C1_Init_CH32(g_i2c_hz);

    if (!mpu_init()){
        printf("MPU init failed\r\n");
        while(1){ Delay_Ms(500); printf(".\r\n"); }
    }
    printf("MPU init OK\r\n");
    printf("Hold still for calibration...\r\n");
    mpu_calibrate(200, 5);  // ~1s
    printf("Calib done.\r\n");

    filter_seed();

    /* main loop @ ~100 Hz (DRDY gated for coherent reads) */
    while (1){
        if (!mpu_wait_data_ready(20)){
            // recover if DRDY stalls (rare unless bus stuck)
            i2c_bus_full_recover();
            I2C1_Init_CH32(g_i2c_hz);
            (void)mpu_reconfigure_min();
            printf("I2C recovered\r\n");
            continue;
        }
        mpu_raw_t r;
        if (mpu_read_raw(&r)){
            filter_step(&r);
            print_filtered_line(temp_to_c_x100(r.temp));
        } else {
            // extra safety recovery on read failure
            i2c_bus_full_recover();
            I2C1_Init_CH32(g_i2c_hz);
            (void)mpu_reconfigure_min();
            printf("I2C recovered\r\n");
        }
        Delay_Ms(10);
    }
}
