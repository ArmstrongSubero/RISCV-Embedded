/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/ Int OSC @ 48 MHz, 3.3V
 * Program: 33_Temp_Sensor
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: In this example we use the CH32V003F4P6 to read a MCP9700
 *                      temperature sensor. 
 *
 * Hardware Description: An MCP9700 temperature sensor is connected to pin C4
 *
 * Created:  August 21st, 2025, 9:19 PM
 * Updated:  August 21st, 2025, 9:19 PM
 */
 
/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"

// ==== ADC pin/channel (PC4 -> CH2) ====
#define ADC_GPIO_PORT   GPIOC
#define ADC_GPIO_PIN    GPIO_Pin_4
#define ADC_CHANNEL     ADC_Channel_2

// ---- ADC init/read (SPL-style) ----
static void ADC1_Init_Simple(void)
{
    GPIO_InitTypeDef gpio = {0};
    ADC_InitTypeDef  adc  = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Pin  = ADC_GPIO_PIN;
    GPIO_Init(ADC_GPIO_PORT, &gpio);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);

    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

static uint16_t ADC1_ReadOnce(uint8_t ch)
{
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_241Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    return ADC_GetConversionValue(ADC1);
}

static uint16_t ADC_ReadAveraged(uint8_t ch, uint8_t samples)
{
    uint32_t acc = 0;
    for(uint8_t i=0;i<samples;i++) acc += ADC1_ReadOnce(ch);
    return (uint16_t)(acc / samples);
}

// ---- Core + UART init ----
static void initMain(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
}

static void initUART(void)
{
#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif
}

// ---- Temperature conversion helpers (fixed-point, no float printf) ----
// Returns temperature in 0.1 °C units
static int16_t mcp9700_read_T_x10(void)
{
    // 16-sample average on CH2 (PC4)
    uint16_t raw = ADC_ReadAveraged(ADC_CHANNEL, 16);

    // Compute millivolts with rounding (Vref=3300 mV, 10-bit ADC)
    // mv = round(raw * 3300 / 1023)
    uint32_t mv = ((uint32_t)raw * 3300u + 511u) / 1023u;

    // MCP9700E: Vout = 500 mV + 10 mV/°C
    // T_x10 (0.1°C) = (mv - 500)  because 1°C = 10 mV
    int32_t T_x10 = (int32_t)mv - 500;   // e.g., mv=825 -> 325 -> 32.5°C
    return (int16_t)T_x10;
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
    initMain();
    initUART();
    printf("\r\n===== CH32V003 + MCP9700E =====\r\n");
    printf("SystemClk:%ld\r\n", SystemCoreClock);

    // ADC
    ADC1_Init_Simple();

    while (1)
    {
        // Temperature in tenths of °C
        int16_t T_x10 = mcp9700_read_T_x10();
        int16_t T_int = T_x10 / 10;
        int16_t T_dec = (T_x10 < 0 ? -T_x10 : T_x10) % 10;

        // Also print raw & mv for visibility
        uint16_t raw = ADC_ReadAveraged(ADC_CHANNEL, 16);
        uint32_t mv  = ((uint32_t)raw * 3300u + 511u) / 1023u; // rounded mV

        printf("ADC:%4u  V:%4lumV  T:%d.%d C\r\n",
               raw, (unsigned long)mv, (int)T_int, (int)T_dec);

        Delay_Ms(250);
    }
}
