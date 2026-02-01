/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 05_read_Pushbutton 
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.3.0)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to read a
 *                      pushbutton connected to PIN C2
 * 
 * Hardware Description: An LED is connected via a 1k resistor to pin 
 *                       C1 and a pushbutton is connected to pin C2 
 *           
 * Created June    17th, 2025, 5:10 PM
 * Updated January 02nd, 2026, 7:46 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"

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
void initMain(void)
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
    uint8_t buttonState = 0;

    initMain();
    initGPIO();

    while(1)
    {
        // Read button on PC2
        buttonState = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);

        // Active-low button logic (0 when pressed)
        if(buttonState == 0)
        {
            // Turn LED ON (PC1 = LOW if active-low LED)
            GPIO_WriteBit(GPIOC, GPIO_Pin_1, Bit_RESET);
        }
        else
        {
            // Turn LED OFF
            GPIO_WriteBit(GPIOC, GPIO_Pin_1, Bit_SET);
        }

        Delay_Ms(50);  // Debounce delay
    }
}

/*******************************************************************************
 * Function: void initGPIO()
 *
 * Returns: Nothing
 *
 * Description: Contains initializations for GPIO pins PC1 is setup as LED output 
 *              and PC2 is setup as button input
 * 
 * Usage: initGPIO()
 ******************************************************************************/
void initGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Enable GPIOC Clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // --- Configure PC1 as Output Push-Pull ---
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // --- Configure PC2 as Input with Pull-Up ---
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  // Input pull-up
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}
