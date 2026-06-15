/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 54_Roll_Pitch_Yaw
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to demonstrate 
 *                      the use of the MPU6050 to generate roll, pitch, yaw 
 *                      values 
 * 
 * Hardware Description: An MPU 6050 module is connected to the 
 *                       CH32V003F4P6 as follows: 
 *
 *                       VDD  -> VDD
 *                       VSS  -> VSS
 *                       SDA  -> PC1 
 *                       SCL  -> PC2 
 *           
 * Created August 25th, 2025, 11:30 PM
 * Updated August 25th, 2025, 11:30 PM
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
 *I2C Helper Stuff 
 ******************************************************************************/
static uint32_t g_i2c_hz = 400000;

static int wait_event(uint32_t evt, uint32_t ms)
{
    while (!I2C_CheckEvent(I2C1, evt)) { Delay_Ms(1); if (!ms--) return 0; }
    return 1;
}

static int wait_flag_set(FlagStatus (*fn)(I2C_TypeDef*, uint32_t), I2C_TypeDef *I2Cx, uint32_t flag, uint32_t ms)
{
    while (fn(I2Cx, flag) == RESET) { Delay_Ms(1); if (!ms--) return 0; }
    return 1;
}

static int wait_flag_clear(FlagStatus (*fn)(I2C_TypeDef*, uint32_t), I2C_TypeDef *I2Cx, uint32_t flag, uint32_t ms)
{
    while (fn(I2Cx, flag) != RESET) { Delay_Ms(1); if (!ms--) return 0; }
    return 1;
}

static void i2c_bus_soft_recover(void)
{
    I2C_SoftwareResetCmd(I2C1, ENABLE); Delay_Ms(1);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
}

static void I2C1_Lines_ToGPIO_OD(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Mode  = GPIO_Mode_Out_OD;          // open-drain GPIO
    g.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_1;   // PC2=SCL, PC1=SDA
    GPIO_Init(GPIOC, &g);
    GPIO_SetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_1); // release lines high
}

static void I2C1_Lines_ToAFOD(void)
{
    GPIO_InitTypeDef g = {0};
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Mode  = GPIO_Mode_AF_OD;           // back to I2C
    g.GPIO_Pin   = GPIO_Pin_2; GPIO_Init(GPIOC, &g); // SCL
    g.GPIO_Pin   = GPIO_Pin_1; GPIO_Init(GPIOC, &g); // SDA
}

static void i2c_bus_full_recover(void)
{
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

static int i2c_start(void)
{
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

static void i2c_stop(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    (void)wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 10);
}

static int addr_tx(uint8_t ctrl8)
{
    I2C_Send7bitAddress(I2C1, ctrl8, I2C_Direction_Transmitter);
    return wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, 20);
}

static int addr_rx(uint8_t ctrl8)
{
    I2C_Send7bitAddress(I2C1, ctrl8, I2C_Direction_Receiver);
    return wait_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, 20);
}

static int send_byte(uint8_t b)
{
    I2C_SendData(I2C1, b);
    return wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, 20);
}

static void I2C1_Init_CH32(uint32_t clock_hz)
{
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
 * MPU-6050 Driver Layer
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

/* ---- raw read ---- */
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

/* ================= Fixed-point scaling & smoothing ================= */

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

#define ACC_SMOOTH_SHIFT   3   // alpha=1/8
#define GYR_SMOOTH_SHIFT   2   // alpha=1/4

static int32_t s_ax=0, s_ay=0, s_az=0;
static int32_t s_gx=0, s_gy=0, s_gz=0;
static int32_t init_done = 0;

/* Calibration: hold still ~1s */
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

/* One filtered step: median-of-3 then IIR */
static void filter_step(const mpu_raw_t *r){
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

/* ================= Demo print ================= */
static void print_filtered_line(int32_t temp_c_x100){
    printf("ACC mg: X=%ld Y=%ld Z=%ld | GY mdps: X=%ld Y=%ld Z=%ld | T=%ld.%02ldC\r\n",
           (long)s_ax,(long)s_ay,(long)s_az,
           (long)s_gx,(long)s_gy,(long)s_gz,
           (long)(temp_c_x100/100),(long)(temp_c_x100%100));
}

/*******************************************************************************
 * Application Roll, Pitch, Yaw
 ******************************************************************************/
/* ---- fixed-point helpers ---- */
static inline int32_t iabs32(int32_t v){ return v < 0 ? -v : v; }

/* integer sqrt (bitwise) */
static uint32_t isqrt_u32(uint32_t x){
    uint32_t op = x, res = 0, one = 1uL << 30;
    while (one > op) one >>= 2;
    while (one){
        if (op >= res + one){ op -= res + one; res = (res >> 1) + one; }
        else res >>= 1;
        one >>= 2;
    }
    return res;
}

/* atan(z) ≈ 45*z + 15.64*z*(1-|z|)  with z in Q15, result in deg*100 */
static int32_t atan_deg_x100_from_q15(int32_t z_q15){
    int32_t az    = z_q15 < 0 ? -z_q15 : z_q15;              /* Q15 */
    int32_t term1 = ( (int64_t)4500 * z_q15 ) >> 15;         /* 45°*100 * z */
    int32_t t     = ( (int64_t)z_q15 * (32768 - az) ) >> 15; /* z*(1-|z|) */
    int32_t term2 = ( (int64_t)1564 * t ) >> 15;             /* 15.64°*100 * t */
    return term1 + term2;                                    /* deg*100 */
}

/* atan2(y,x) in deg*100 (no libm), quadrant-corrected */
static int32_t atan2_deg_x100_i32(int32_t y, int32_t x){
    if (x == 0){
        if      (y > 0)  return  9000;
        else if (y < 0)  return -9000;
        else             return 0;
    }
    int32_t ax = iabs32(x), ay = iabs32(y);
    int32_t a;

    if (ax >= ay){
        /* |y/x| <= 1 */
        int32_t z = (int32_t)(((int64_t)y << 15) / ax);      /* Q15 */
        a = atan_deg_x100_from_q15(z);
    }else{
        /* |y/x| > 1 : atan(r) = sign(r)*90 - atan(1/|r|) */
        int32_t z_abs = (int32_t)(((int64_t)iabs32(x) << 15) / ay); /* |x/y| in Q15 */
        int32_t a1 = atan_deg_x100_from_q15(z_abs);
        int32_t signr = ((x ^ y) < 0) ? -1 : 1;
        a = signr * 9000 - a1;
    }

    /* quadrant fix */
    if (x > 0) return a;
    return a + ((y >= 0) ? 18000 : -18000);
}

/* roll/pitch from accelerometer (deg*100):
   roll  = atan2(Ay, Az)
   pitch = atan2(-Ax, sqrt(Ay^2 + Az^2)) */
static void inclinometer_compute_deg_x100(int32_t ax_mg, int32_t ay_mg, int32_t az_mg,
                                          int32_t *roll_d100, int32_t *pitch_d100){
    int32_t roll = atan2_deg_x100_i32(ay_mg, az_mg);

    uint32_t ay2 = (uint32_t)ay_mg * (uint32_t)ay_mg;
    uint32_t az2 = (uint32_t)az_mg * (uint32_t)az_mg;
    uint32_t den = isqrt_u32(ay2 + az2);
    if (den == 0) den = 1;

    int32_t pitch = atan2_deg_x100_i32(-ax_mg, (int32_t)den);

    *roll_d100  = roll;
    *pitch_d100 = pitch;
}

/* ========= UI: degree text & LED level with hysteresis ========= */

/* use ASCII "deg" for max terminal compatibility. set to 0 for UTF-8 degree */
#define PRINT_DEG_ASCII  1
#if PRINT_DEG_ASCII
  #define DEGSTR "deg"
#else
  #define DEGSTR "\xC2\xB0"
#endif

/* LED on PC4 lights when near level */
#define LEVEL_LED_ENABLE   1
#define LEVEL_BAND_CDEG    50   /* enter band: ±0.50° */
#define LEVEL_HYST_CDEG    20   /* exit hysteresis: +0.20° */

#if LEVEL_LED_ENABLE
static void LED_Init_PC4(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Mode = GPIO_Mode_Out_PP; g.GPIO_Speed = GPIO_Speed_50MHz; g.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOC, &g);
    GPIO_ResetBits(GPIOC, GPIO_Pin_4);
}
static inline void LED_SetLevel(bool on){
    if (on) GPIO_SetBits(GPIOC, GPIO_Pin_4);
    else    GPIO_ResetBits(GPIOC, GPIO_Pin_4);
}
#else
static inline void LED_Init_PC4(void) {}
static inline void LED_SetLevel(bool on){ (void)on; }
#endif

/* pretty print without floats */
static void print_inclinometer_line(int32_t roll_cdeg, int32_t pitch_cdeg,
                                    int32_t ax, int32_t ay, int32_t az, int32_t t_x100){
    int32_t rS = (roll_cdeg  < 0), pS = (pitch_cdeg < 0);
    int32_t rA = rS ? -roll_cdeg  : roll_cdeg;
    int32_t pA = pS ? -pitch_cdeg : pitch_cdeg;

    printf("ROLL=%c%ld.%02ld" DEGSTR "  PITCH=%c%ld.%02ld" DEGSTR
           "  | ACC mg: X=%ld Y=%ld Z=%ld | T=%ld.%02ldC\r\n",
           rS?'-':'+', (long)(rA/100), (long)(rA%100),
           pS?'-':'+', (long)(pA/100), (long)(pA%100),
           (long)ax,(long)ay,(long)az,
           (long)(t_x100/100),(long)(t_x100%100));
}

/* ================= Time-based yaw zeroing (drift cancellation) ================= */

/* if you don’t already have this from the YPR patch */
static int32_t wrap18000(int32_t a){ while(a<=-18000) a+=36000; while(a>18000) a-=36000; return a; }

/* --- Config --- */
#define STATIONARY_ACC_TOL_MG      60     /* |g|-window for “still” (mg) */
#define STATIONARY_GYRO_TOL_MDPS   800    /* per-axis gyro threshold (mdps) */
#define STATIONARY_MIN_SAMPLES     50     /* ~0.5 s @ 100 Hz */
#define ZERO_MIN_MS               2000    /* need at least this long “still” */
#define ZERO_UPDATE_PERIOD_MS      500    /* refresh bias at this cadence */
#define BIAS_CORR_GAIN_NUM           1    /* 1/1 = full, 1/2 = half etc. */
#define BIAS_CORR_GAIN_DEN           2    /* smoother corrections (use 2) */
#define DRIFT_MDPS_CLAMP          5000    /* safety clamp (±5 deg/s) */

/* Bias estimate (mdeg/s) applied to yaw integration */
static int32_t g_bias_gz_mdps = 0;

/* State for stationary detection & zero manager */
typedef struct {
    uint8_t  stationary;         /* 0/1 */
    uint16_t count;              /* consecutive still samples */
    uint32_t enter_ms;           /* when stationary window began */
    int32_t  yaw_start_d100;     /* yaw at window start (deg*100) */
    uint32_t last_corr_ms;       /* last time we applied correction */
} yaw_zero_t;

static yaw_zero_t g_yz = {0};

/* Orientation-invariant “still” check (no sqrt): |a|≈1g and each gyro small */
static inline uint8_t is_still(int32_t ax, int32_t ay, int32_t az,
                               int32_t gx, int32_t gy, int32_t gz)
{
    /* accel magnitude window */
    int64_t a2 = (int64_t)ax*ax + (int64_t)ay*ay + (int64_t)az*az;
    int32_t lo = (1000 - STATIONARY_ACC_TOL_MG);
    int32_t hi = (1000 + STATIONARY_ACC_TOL_MG);
    int64_t lo2 = (int64_t)lo*lo, hi2 = (int64_t)hi*hi;

    if (a2 < lo2 || a2 > hi2) return 0;

    /* gyro quiet per-axis */
    if (gx < -STATIONARY_GYRO_TOL_MDPS || gx > STATIONARY_GYRO_TOL_MDPS) return 0;
    if (gy < -STATIONARY_GYRO_TOL_MDPS || gy > STATIONARY_GYRO_TOL_MDPS) return 0;
    if (gz < -STATIONARY_GYRO_TOL_MDPS || gz > STATIONARY_GYRO_TOL_MDPS) return 0;

    return 1;
}

/* Convert mdps -> deg*100 step for DT_MS loop (same as your ypr code) */
#ifndef DT_MS
#define DT_MS 10
#endif
static inline int32_t mdps_to_deg100_step(int32_t mdps){ return (mdps * DT_MS) / 10000; }

/* Call this AFTER you’ve updated g_yaw_d100 / yaw integration each loop */
static void yaw_zero_manager_step(uint32_t now_ms, int32_t yaw_d100,
                                  int32_t ax, int32_t ay, int32_t az,
                                  int32_t gx, int32_t gy, int32_t gz)
{
    /* 1) Stationary detector with entry/exit hysteresis via counter */
    if (is_still(ax, ay, az, gx, gy, gz)) {
        if (g_yz.count < 0xFFFF) g_yz.count++;
    } else {
        g_yz.count = 0;
        g_yz.stationary = 0;
    }

    if (!g_yz.stationary && g_yz.count >= STATIONARY_MIN_SAMPLES) {
        g_yz.stationary   = 1;
        g_yz.enter_ms     = now_ms;
        g_yz.yaw_start_d100 = yaw_d100;
        g_yz.last_corr_ms = now_ms;
    }

    /* 2) During a long still window, estimate drift slope Δψ/Δt and fold into bias */
    if (g_yz.stationary) {
        uint32_t el_ms = now_ms - g_yz.enter_ms;
        if (el_ms >= ZERO_MIN_MS && (now_ms - g_yz.last_corr_ms) >= ZERO_UPDATE_PERIOD_MS) {

            int32_t dpsi   = wrap18000(yaw_d100 - g_yz.yaw_start_d100); /* deg*100 */
            /* drift rate in mdps: mdps = (Δ(deg*100)/Δt_ms) * 10000 */
            int32_t drift_mdps = (int32_t)(( (int64_t)dpsi * 10000 ) / (int32_t)el_ms);

            /* clamp; then apply a gentle fraction to the bias estimate */
            if (drift_mdps >  DRIFT_MDPS_CLAMP) drift_mdps =  DRIFT_MDPS_CLAMP;
            if (drift_mdps < -DRIFT_MDPS_CLAMP) drift_mdps = -DRIFT_MDPS_CLAMP;

            g_bias_gz_mdps += (drift_mdps * BIAS_CORR_GAIN_NUM) / BIAS_CORR_GAIN_DEN;

            g_yz.last_corr_ms = now_ms;

            /* Optional: rebase the window to avoid growing Δt forever */
            g_yz.enter_ms       = now_ms;
            g_yz.yaw_start_d100 = yaw_d100;

            /* Debug (comment out on production to save UART time)
               printf("bias_z=%ld mdps\r\n", (long)g_bias_gz_mdps); */
        }
    }
}


/* ---- pretty-print: roll/pitch/yaw without floats ---- */
static void print_ypr_line(int32_t roll_cdeg, int32_t pitch_cdeg, int32_t yaw_cdeg,
                           int32_t ax, int32_t ay, int32_t az, int32_t t_x100,
                           int32_t bias_mdps)
{
    int32_t rS = (roll_cdeg  < 0), pS = (pitch_cdeg < 0), yS = (yaw_cdeg < 0);
    int32_t rA = rS ? -roll_cdeg  : roll_cdeg;
    int32_t pA = pS ? -pitch_cdeg : pitch_cdeg;
    int32_t yA = yS ? -yaw_cdeg   : yaw_cdeg;

    printf("ROLL=%c%ld.%02ld"  DEGSTR
           "  PITCH=%c%ld.%02ld" DEGSTR
           "  YAW=%c%ld.%02ld"   DEGSTR
           "  | ACC mg: X=%ld Y=%ld Z=%ld | T=%ld.%02ldC | biasZ=%ld mdps\r\n",
           rS?'-':'+', (long)(rA/100), (long)(rA%100),
           pS?'-':'+', (long)(pA/100), (long)(pA%100),
           yS?'-':'+', (long)(yA/100), (long)(yA%100),
           (long)ax,(long)ay,(long)az,
           (long)(t_x100/100),(long)(t_x100%100),
           (long)bias_mdps);
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
    printf("CH32V003 + MPU-6050 (I2C @ 400kHz) — R/P/Y with time-based yaw zero (no libm)\r\n");
    printf("Pins: PC2=SCL, PC1=SDA (pull-ups to 3.3V)\r\n");

    LED_Init_PC4();
    I2C1_Init_CH32(g_i2c_hz);

    if (!mpu_init()){
        printf("MPU init failed\r\n");
        while(1){ Delay_Ms(500); printf(".\r\n"); }
    }
    printf("MPU init OK\r\n");
    printf("Hold still for calibration...\r\n");
    mpu_calibrate(200, 5);
    printf("Calib done.\r\n");

    filter_seed();

    /* yaw integrator (deg*100) */
    int32_t yaw_d100 = 0;

    /* time base */
#ifndef DT_MS
#define DT_MS 10
#endif
    uint32_t now_ms = 0;

    /* main loop @ ~100 Hz, DRDY-gated */
    uint32_t tick = 0;
    bool level_latched = false;

    while (1){
        if (!mpu_wait_data_ready(20)){
            i2c_bus_full_recover();
            I2C1_Init_CH32(g_i2c_hz);
            (void)mpu_reconfigure_min();
            printf("I2C recovered\r\n");
            continue;
        }

        mpu_raw_t r;
        if (mpu_read_raw(&r)){
            now_ms += DT_MS;

            /* update filtering (s_ax/s_ay/s_az & s_gx/s_gy/s_gz) */
            filter_step(&r);

            /* compute roll/pitch from filtered accel (deg*100) */
            int32_t roll_cdeg=0, pitch_cdeg=0;
            inclinometer_compute_deg_x100(s_ax, s_ay, s_az, &roll_cdeg, &pitch_cdeg);

            /* ---- yaw integrate from filtered Z-gyro with bias correction ----
               s_gz is in mdps; convert to deg*100 per step and wrap to [-18000, +18000] */
            int32_t yaw_step = mdps_to_deg100_step(s_gz - g_bias_gz_mdps);
            yaw_d100 = wrap18000(yaw_d100 + yaw_step);

            /* feed the time-based zero manager AFTER integration */
            yaw_zero_manager_step(now_ms, yaw_d100, s_ax, s_ay, s_az, s_gx, s_gy, s_gz);

            /* hysteretic LEVEL logic uses only R/P */
            const int32_t band_in  = LEVEL_BAND_CDEG;
            const int32_t band_out = LEVEL_BAND_CDEG + LEVEL_HYST_CDEG;

            bool inside  = (iabs32(roll_cdeg)  <= band_in)  && (iabs32(pitch_cdeg) <= band_in);
            bool outside = (iabs32(roll_cdeg)  >  band_out) || (iabs32(pitch_cdeg) >  band_out);

            if (!level_latched && inside)      level_latched = true;
            else if (level_latched && outside) level_latched = false;

            LED_SetLevel(level_latched);

            /* print every 10 samples (~10 Hz) */
            if ((tick % 10) == 0){
                print_ypr_line(roll_cdeg, pitch_cdeg, yaw_d100,
                               s_ax, s_ay, s_az, temp_to_c_x100(r.temp),
                               g_bias_gz_mdps);
                if (level_latched) printf("LEVEL\r\n");
            }
            tick++;
        } else {
            i2c_bus_full_recover();
            I2C1_Init_CH32(g_i2c_hz);
            (void)mpu_reconfigure_min();
            printf("I2C recovered\r\n");
        }

        Delay_Ms(DT_MS);
    }
}
