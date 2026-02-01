/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 5.0v
 * Program: 01_Blink
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.3.0)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allow the CH32V003F4P6 to blink 
 *                      an LED.  
 * 
 * Hardware Description: An LED is connected to the CH32V003F4P6 to 
 *                       pin D0
 *
 * Created December 29th, 2025, 10:58 PM
 * Updated December 29th, 2025, 10:58 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
// Includes 
#include "debug.h"

// Initialization for the Main Program 
void initMain()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
}

// Initialize the GPIO 
void initGPIO()
{
    // initialize structure used to hold GPIO config settings 
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Enable clock for GPIOD, clock needs to be enabled for Reset and Clock Control (RCC) unit
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

// Function to blink the LED connected to PD0
void blinkLed()
{
    // set PIND0 low
    GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_SET);
    Delay_Ms(250);

    // set PIND0 high
    GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_RESET);
    Delay_Ms(250);
}

// Main Program 
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
