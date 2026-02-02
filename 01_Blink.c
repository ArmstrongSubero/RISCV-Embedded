/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 01_Blink
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to blink an 
 *                      LED connected to PIN D0
 * 
 * Hardware Description: An LED is connected via a 1k resistor to pin 
 *                       D0
 *           
 * Created August 10th, 2025, 4:57 PM
 * Updated August 10th, 2025, 4:57 PM
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
void initMain()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
}

/*******************************************************************************
 * Function: void initGPIO()
 *
 * Returns: Nothing
 *
 * Description: Contains initializations for GPIO pins
 * 
 * Usage: initGPIO()
 ******************************************************************************/
void initGPIO()
{
    // initialize structure used to hold GPIO config settings 
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Enable clock for GPIOD, clock needs to be enabled fomr Reset and Clock Control (RCC) unit
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    // select pin 0 of PORTD to configure 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;

    // Set the pin output to push-pull mode
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    // seet outpit speed to 30 MHz
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;

    // Apply the configuration to GPIOD using the settings provided in the init structure 
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*******************************************************************************
 * Function: void blinkLed()
 *
 * Returns: Nothing
 *
 * Description: Blinks LED connected to PIND0
 * 
 * Usage: initGPIO()
 ******************************************************************************/
void blinkLed()
{
    // set PIND0 low
    GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_SET);
    Delay_Ms(250);

    // set PIND0 high
    GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_RESET);
    Delay_Ms(250);
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
    // initialization
    initMain();
    initGPIO();

    while(1)
    {
      // main loop 
      blinkLed();
    }
}
