/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 10_Smart_Button
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to demonstrate 
 *                      hw we can create a "smart" button that detects short 
 *                      press, long press and double press. 
 * 
 * Hardware Description: An LED is connected to pin C1 via a 1k resistor and 
 *                       a pushbutton is connected to pin C2. 
 *           
 * Created June 17th, 2025, 10:38 PM
 * Updated June 17th, 2025, 10:38 PM
 */
 
/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"
#include <stdbool.h>

#define LED_PIN         GPIO_Pin_1
#define BUTTON_PIN      GPIO_Pin_2
#define GPIO_PORT       GPIOC
#define GPIO_RCC        RCC_APB2Periph_GPIOC

#define LED_ON()        GPIO_WriteBit(GPIO_PORT, LED_PIN, Bit_RESET)
#define LED_OFF()       GPIO_WriteBit(GPIO_PORT, LED_PIN, Bit_SET)
#define LED_TOGGLE()    GPIO_WriteBit(GPIO_PORT, LED_PIN, !GPIO_ReadOutputDataBit(GPIO_PORT, LED_PIN))

#define TICK_MS             50      // Timer tick every 50ms
#define LONG_PRESS_TICKS    20      // 50ms * 20 = 1 second
#define DOUBLE_CLICK_GAP     8      // 50ms * 8 = 400ms

volatile uint8_t timerTick = 0;

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
        timerTick++;
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
 * Function: void initGPIO(void)
 *
 * Returns: Nothing
 *
 * Description: Sets up PC1 as output and PC2 as input
 * 
 * Usage: initGPIO()
 ******************************************************************************/
void initGPIO()
{
    GPIO_InitTypeDef gpio = {0};
    RCC_APB2PeriphClockCmd(GPIO_RCC, ENABLE);

    gpio.GPIO_Pin = LED_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIO_PORT, &gpio);
    LED_OFF();

    gpio.GPIO_Pin = BUTTON_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIO_PORT, &gpio);
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
void initTimer2()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);

    uint32_t ticks = clocks.PCLK1_Frequency / (1000 / TICK_MS);  // 50ms tick
    uint16_t prescaler = ticks / 65536;
    uint16_t period = ticks / (prescaler + 1);

    TIM_TimeBaseInitTypeDef tim = {0};
    tim.TIM_Prescaler = prescaler;
    tim.TIM_Period = period - 1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim);

    NVIC_InitTypeDef nvic = {0};
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

/*******************************************************************************
 * Function: bool isButtonPressed()
 *
 * Returns: Button pressed status
 *
 * Description: Checkes fof if pushbutton is pressed 
 * 
 * Usage: if (isButtonPressed())
 ******************************************************************************/
bool isButtonPressed()
{
    if (GPIO_ReadInputDataBit(GPIO_PORT, BUTTON_PIN) == 0)
    {
        Delay_Ms(20);
        if (GPIO_ReadInputDataBit(GPIO_PORT, BUTTON_PIN) == 0)
            return true;
    }
    return false;
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

    uint8_t pressTicks = 0;
    uint8_t clickCount = 0;
    uint8_t doubleClickTimer = 0;
    uint8_t blinkMode = 0;
    uint8_t blinkTick = 0;
    bool buttonHeld = false;

    while (1)
    {
        if (isButtonPressed())
        {
            pressTicks = 0;
            buttonHeld = true;

            while (GPIO_ReadInputDataBit(GPIO_PORT, BUTTON_PIN) == 0)
            {
                Delay_Ms(10);
                if (timerTick)
                {
                    timerTick = 0;
                    pressTicks++;
                    if (pressTicks >= LONG_PRESS_TICKS)
                    {
                        blinkMode = 1;
                        buttonHeld = false;
                        break;
                    }
                }
            }

            if (buttonHeld && pressTicks < LONG_PRESS_TICKS)
            {
                clickCount++;
                doubleClickTimer = 0;
                buttonHeld = false;
            }
        }

        if (timerTick)
        {
            timerTick = 0;

            if (blinkMode)
            {
                blinkTick++;
                if (blinkTick % 4 == 0)  // Toggle every 200ms
                    LED_TOGGLE();
            }

            if (clickCount == 1)
            {
                doubleClickTimer++;
                if (doubleClickTimer >= DOUBLE_CLICK_GAP)
                {
                    LED_TOGGLE();  // Single short press → toggle
                    clickCount = 0;
                    doubleClickTimer = 0;
                }
            }
            else if (clickCount == 2)
            {
                LED_ON();
                Delay_Ms(2000);
                LED_OFF();
                clickCount = 0;
                doubleClickTimer = 0;
                blinkMode = 0;
            }
        }
    }
}
