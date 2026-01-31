/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 5.0v
 * Program: 02_Dual LEDs
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.3.0)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allow the CH32V003F4P6 to blink 
 *                      dual LEDs using a single pin 
 * 
 * Hardware Description: Two LEDs are connected to PIN D0 one for sourcing 
 *                       and one for sinking
 *
 * Created December 29th, 2025, 10:58 PM
 * Updated January  02nd, 2026, 01:29 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"

typedef enum {
    LED_A_ON,  // Sink current: PD0 = LOW
    LED_B_ON,  // Source current: PD0 = HIGH
    LED_OFF    // Hi-Z: PD0 = input, both OFF
} LED_Mode;

void setLedMode(LED_Mode mode);
void initGPIO(void);

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
    // initialization
    initMain();
    initGPIO();

    while(1)
    {
       setLedMode(LED_A_ON);
       Delay_Ms(1000);

       setLedMode(LED_B_ON);
       Delay_Ms(1000);

       setLedMode(LED_OFF);
       Delay_Ms(1000);
    }
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
void initGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*******************************************************************************
 * Function: void setLedMode(LED_Mode mode)
 *
 * Returns: Nothing
 *
 * Description: 
 * 
 * Usage:  setLedMode(LED_x_ON);
 ******************************************************************************/
void setLedMode(LED_Mode mode)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;

    if (mode == LED_OFF)
    {
        // High impedance mode: turn both LEDs off
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
    }
    else
    {
        // Enable output mode
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
        GPIO_Init(GPIOD, &GPIO_InitStructure);

        // Set output level
        GPIO_WriteBit(GPIOD, GPIO_Pin_0, (mode == LED_B_ON) ? Bit_SET : Bit_RESET);
    }
}