/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 09_Timer_Interrupt
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.3.0)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to demonstrate 
 *                      use of the timer interrupt. 
 *                      
 * 
 * Hardware Description: An LED is connected to pin C1 via a 1k resistor. 
 *           
 * Created June    15th, 2025, 01:04 PM
 * Updated January 02nd, 2026, 10:26 PM
 */
 
/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
 #include "debug.h"
#include <stdbool.h>

// LED defines
#define LED_GPIO_PIN GPIO_Pin_1
#define LED_GPIO_PORT GPIOC
#define LED_GPIO_RCC RCC_APB2Periph_GPIOC
#define LED_FREQ 2ul     // 2Hz = 500ms
#define COUNTER_MAX 65536ul

volatile bool toggleLED = false;

void initGPIO(void);
void initTimer2(void);

/*******************************************************************************
 * Function: void TIM2_IRQHandler(void)
 *
 * Returns: Nothing
 *
 * Description: Interrupt Handler
 * 
 * Usage: __attribute__((interrupt("WCH-Interrupt-fast")))
 *        void TIM2_IRQHandler(void)
 ******************************************************************************/
__attribute__((interrupt("WCH-Interrupt-fast")))
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        toggleLED = true;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

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
    initMain();
    initGPIO();
    initTimer2();

    BitAction ledState = Bit_RESET;

    while (1)
    {
        if (toggleLED)
        {
            ledState = (ledState == Bit_SET) ? Bit_RESET : Bit_SET;
            GPIO_WriteBit(LED_GPIO_PORT, LED_GPIO_PIN, ledState);
            toggleLED = false;
        }
    }
}

/*******************************************************************************
 * Function: void initGPIO(void)
 *
 * Returns: Nothing
 *
 * Description: Sets up PC1 as output
 * 
 * Usage: initGPIO()
 ******************************************************************************/ 
void initGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(LED_GPIO_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
}


/*******************************************************************************
 * Function: void initTimer2()
 *
 * Returns: Nothing
 *
 * Description: Initialize TIM2 to trigger ever 500ms (2Hz)
 * 
 * Usage: initTimer2()
 ******************************************************************************/
void initTimer2(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);

    uint16_t prescalerValue = 1;
    uint32_t freqRatio = clocks.PCLK1_Frequency / LED_FREQ;

    if (freqRatio > COUNTER_MAX)
    {
        prescalerValue = freqRatio / COUNTER_MAX;
        if (clocks.PCLK1_Frequency % prescalerValue)
        {
            prescalerValue++;
        }
        freqRatio /= prescalerValue;
    }

    prescalerValue--;
    uint16_t autoReloadValue = freqRatio - 1;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef timerInit = {0};
    timerInit.TIM_Period = autoReloadValue;
    timerInit.TIM_Prescaler = prescalerValue;
    timerInit.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInit.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &timerInit);

    NVIC_InitTypeDef nvicInit = {0};
    nvicInit.NVIC_IRQChannel = TIM2_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
    nvicInit.NVIC_IRQChannelSubPriority = 0;
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}