/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 15_Pulse_Width_Modulation
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.3.0)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to demonstrate 
 *                      the use of the Pulse Width Modulation module
 *                      
 * Hardware Description: An LED is connected to pin D2 via a 1k resistor.
 *           
 * Created June    19th, 2025, 12:39 PM
 * Updated January 03rd, 2026, 02:03 PM 
 */
 
/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"

// Constants
#define PWM_MODE1   0
#define PWM_MODE2   1

// Select PWM Mode
// You can switch to PWM_MODE2 if desired
#define PWM_MODE PWM_MODE1  

void initPWM_TIM1(uint16_t period, uint16_t prescaler, uint16_t duty);
void initPWM_GPIO();

/*******************************************************************************
 * Function: void initMain()
 *
 * Returns: Nothing
 *
 * Description: Contains initializations for main
 * 
 * Usage: initMain()
 ******************************************************************************/
void initMain()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
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
    uint16_t period = 2399;    // ARR
    uint16_t prescaler = 0;    // PSC

    initMain();                                  // Core system setup
    initPWM_GPIO();                              // Setup PD2 for PWM
    initPWM_TIM1(period, prescaler, 1200);       // PWM: 20kHz freq, 50% duty cycle

    while(1)
    {
        // PWM signal continues running in hardware
    }
}


/*******************************************************************************
 * Function: void initPWM_TIM1(uint16_t period, uint16_t prescaler, uint16_t duty)
 *
 * Returns: Nothing
 *
 * Description: Initialize Timer1 in PWM Output Mode on Channel 1
 * 
 * Usage: initPWM_TIM1(period, prescaler, duty)
 ******************************************************************************/
void initPWM_TIM1(uint16_t period, uint16_t prescaler, uint16_t duty)
{
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

    // Enable clock for TIM1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // Basic timer setup
    TIM_TimeBaseInitStructure.TIM_Period = period;
    TIM_TimeBaseInitStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

#if (PWM_MODE == PWM_MODE1)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
#elif (PWM_MODE == PWM_MODE2)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
#endif

    // PWM configuration
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = duty;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    // Enable PWM output and start timer
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/*******************************************************************************
 * Function: void initPWM_GPIO()
 *
 * Returns: Nothing
 *
 * Description: Sets up GPIO for PWM output on PD2
 * 
 * Usage: initPWM_GPIO()
 ******************************************************************************/
void initPWM_GPIO()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Enable clock for GPIOD
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    // Configure PD2 as Alternate Function Push-Pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}