/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 37_Ultraviolet_Sensor
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to read 
 *                      the GYML8511 UV sensor. 
 * 
 * Hardware Description: A GYML8511 UV sensor is connected to the 
 *                       CH32V003F4P6 as follows:
 *         
 *                       VIN -> NC
 *                       VCC -> 3.3v
 *                       GND -> Gnd
 *                       OUT -> PA1
 *                       EN  -> 3.3v
 *                      
 *                       - The ML8511 output is analog ~1.0V in dark, up to ~2.2V at 10 mW/cm² (typical).
 *                       - We model UV ≈ (Vout - VZERO) / SENS. Tune VZERO & SENS per your unit!
 *                       - Add a tiny RC (e.g., 1k + 100 nF to GND) at OUT if readings are noisy.
 *
 * Created August 23rd, 2025, 1:18 PM
 * Updated August 23rd, 2025, 1:18 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "ch32v00x.h"
#include "ch32v00x_rcc.h"
#include "ch32v00x_gpio.h"
#include "ch32v00x_adc.h"
#include "debug.h"
#include <stdio.h>
#include <stdint.h>

/* ===================== User Config ===================== */

/* ADC pin/channel mapping — change to your actual pin */
#define UV_GPIO_PORT        GPIOA
#define UV_GPIO_PIN         GPIO_Pin_1        /* PA1 */
#define UV_ADC_CHANNEL      ADC_Channel_1     /* channel for PA1 */

/* ADC reference and resolution */
#define VDD_MV              3300              /* ADC reference in millivolts (Vref = VDDA) */

/* If your lib is 12-bit ADC, set 4095; if 10-bit, set 1023 */
#define ADC_FULL_SCALE      1023

/* Averaging for smoother readings */
#define UV_AVG_SAMPLES      32                /* 8..64 is reasonable */

/* ML8511 transfer (tune these for your PCB/unit/environment!) */
#define UV_VZERO_MV         1000              /* output at ~0 mW/cm² (typ. ~1.0V at 3.3V) */
#define UV_SENS_MV_PER_MWCM2 120              /* typ. ~120 mV per mW/cm² */

/* Print period */
#define PRINT_MS            500               /* 0.5s; use 1000 for 1 Hz */

/* ===================== ADC Setup ===================== */

static void adc_init(void)
{
    /* Clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /* Analog input pin */
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin   = UV_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_Init(UV_GPIO_PORT, &gpio);

    /* ADC config (single conversion, software trigger) */
    ADC_DeInit(ADC1);

    ADC_InitTypeDef adc = {0};
    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign          = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &adc);

    /* Calibrate (STM32F1-style API; CH32V003 keeps it) */
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET) { }
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET) { }
}

/* One blocking conversion on a channel; long sample time for stability */
static uint16_t adc_read_once(uint8_t ch)
{
    /* Use a long sample time; pick the one your headers define.
       Common names: ADC_SampleTime_239Cycles5 or ADC_SampleTime_239_5Cycles.
       If your header differs, just swap the enum below. */
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_43Cycles);

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) { }
    return ADC_GetConversionValue(ADC1);
}

static uint16_t adc_read_avg(uint8_t ch, uint16_t samples)
{
    uint32_t acc = 0;
    for (uint16_t i = 0; i < samples; ++i) {
        acc += adc_read_once(ch);
    }
    return (uint16_t)(acc / samples);
}

/* ===================== UV Math (integer) ===================== */

static inline uint32_t adc_to_millivolts(uint16_t adc)
{
    /* mV = adc * Vref / FS */
    return ( (uint32_t)adc * (uint32_t)VDD_MV + (ADC_FULL_SCALE/2) ) / (uint32_t)ADC_FULL_SCALE;
}

static inline int32_t uv_mwcm2_x100_from_mV(uint32_t mv)
{
    /* uv (mW/cm²) = max(0, (Vout - VZERO) / SENS)
       We return x100 for nicer integer prints. */
    if (mv <= UV_VZERO_MV) return 0;
    uint32_t delta = mv - UV_VZERO_MV;
    /* x100 scaling */
    return (int32_t)((delta * 100U + (UV_SENS_MV_PER_MWCM2/2)) / UV_SENS_MV_PER_MWCM2);
}

/* ===================== main ===================== */

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
    printf("CH32V003 + ML8511 (ADC)\r\n");
    printf("ADC FS=%d, Vref=%dmV, pin=PA1, samples=%d\r\n",
           ADC_FULL_SCALE, VDD_MV, UV_AVG_SAMPLES);
    printf("Model: UV≈(Vout-%dmV)/%d mV per mW/cm^2\r\n",
           UV_VZERO_MV, UV_SENS_MV_PER_MWCM2);

    adc_init();

    while (1)
    {
        uint16_t raw = adc_read_avg(UV_ADC_CHANNEL, UV_AVG_SAMPLES);
        uint32_t mv  = adc_to_millivolts(raw);
        int32_t  uvx = uv_mwcm2_x100_from_mV(mv); /* x100 */

        /* Print: raw, mV, mW/cm².x2 */
        printf("RAW=%4u  V=%ld.%03ld V  UV=%ld.%02ld mW/cm^2\r\n",
               raw,
               (long)(mv/1000), (long)(mv%1000),
               (long)(uvx/100), (long)(uvx%100));

        Delay_Ms(PRINT_MS);
    }
}
