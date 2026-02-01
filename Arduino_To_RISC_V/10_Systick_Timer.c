/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3V
 * Program: 10_SysTick_Timer
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.3.0)
 * Program Version: 1.0
 *                  * Uses SysTick to toggle LED every 1 second
 *
 * Program Description: This program configures SysTick to generate an interrupt at 1 Hz.
 *                      On each interrupt, it toggles an LED connected to PD0.
 *
 * Hardware Description: An LED is connected via a 1k resistor connected to PD0.
 *
 * Created August  10th, 2025, 07:15 PM
 * Updated January 02nd, 2026, 10:41 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"

/*******************************************************************************
 * Function: void initMain()
 *
 * Returns: Nothing
 *
 * Description: Contains initializations for main
 * 
 * Usage: initMain()
 ******************************************************************************/
static void initMain(void)
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
 * Description: Sets up PD0 as LED output
 * 
 * Usage: initGPIO()
 ******************************************************************************/
static void initGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Start with LED off (active-low wiring)
    GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_SET);
}

/*******************************************************************************
 * Function: static void initSysTick_1Hz(void)
 *
 * Returns: Nothing
 *
 * Description: Initialize SysTick to 1 Hz
 * 
 * Usage: initSysTick_1Hz()
 ******************************************************************************/
static void initSysTick_1Hz(void)
{
    NVIC_EnableIRQ(SysTick_IRQn);

    SysTick->SR  &= ~(1U << 0);                 // Clear any pending flag
    SysTick->CMP  = (SystemCoreClock/1) - 1;    // 1 Hz tick (syscoreclock/10) for 10 Hz for example
    SysTick->CNT  = 0;                          // Reset counter
    SysTick->CTLR = 0xF;                        // Enable counter + interrupt
}

/*******************************************************************************
 * Function:  void SysTick_Handler(void)  
 *
 * Returns: Nothing
 *
 * Description: SysTick ISR
 * 
 * Usage: void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
 *        void SysTick_Handler(void)
 ******************************************************************************/
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void)
{
    // Toggle PD0 (active-low LED)
    if (GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_0))
        GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_RESET);
    else
        GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_SET);

    SysTick->SR = 0; // Clear SysTick interrupt flag
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
    initSysTick_1Hz();

    while (1)
    {
        // Idle loop LED toggle happens in SysTick interrupt
    }
}
