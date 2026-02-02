/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 07_Double_Click_Detection
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to perform 
 *                      double click detection on pin C2
 * 
 * Hardware Description: A pushbutton is connected via a 10k resistor to pin 
 *                       C2
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
void initMain(void)
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
void initGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // PC1 as output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // PC2 as input with pull-up
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*******************************************************************************
 * Function: uint8_t debounceButton(GPIO_TypeDef* port, uint16_t pin)
 *
 * Returns: Nothing
 *
 * Description: Allows us to perform software debouncing on a pin
 * 
 * Usage: debounceButton(GPIOC, GPIO_Pin_2)
 ******************************************************************************/
uint8_t debounceButton(GPIO_TypeDef* port, uint16_t pin)
{
    if (GPIO_ReadInputDataBit(port, pin) == 0)  // Button pressed (active-low)
    {
        Delay_Ms(20);  // Wait 20ms
        if (GPIO_ReadInputDataBit(port, pin) == 0)  // Still pressed?
        {
            while (GPIO_ReadInputDataBit(port, pin) == 0);  // Wait until released
            Delay_Ms(20);  // Confirm release
            return 1;
        }
    }
    return 0;
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
    uint8_t clickCount = 0;
    uint32_t timer = 0;

    initMain();
    initGPIO();

    while (1)
    {
        if (debounceButton(GPIOC, GPIO_Pin_2))
        {
            if (clickCount == 0)
            {
                clickCount = 1;
                timer = 0; // start timing window
            }
            else if (clickCount == 1 && timer < 400)
            {
                // Double click detected
                // Toggle LED on PC1
                if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_1))
                    GPIO_WriteBit(GPIOC, GPIO_Pin_1, Bit_RESET);
                else
                    GPIO_WriteBit(GPIOC, GPIO_Pin_1, Bit_SET);

                clickCount = 0;
                timer = 0;
            }
        }

        // Timing window for double-click
        if (clickCount == 1)
        {
            Delay_Ms(10);
            timer += 10;
            if (timer >= 400)
            {
                // Too slow → reset
                clickCount = 0;
                timer = 0;
            }
        }
    }
}