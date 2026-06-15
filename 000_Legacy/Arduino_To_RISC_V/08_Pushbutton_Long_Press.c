/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 08_Pushbutton_Long_Press
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.3.0)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to perform 
 *                      long press detection on pin C2 and a LED 
 *                      connected to pin C1. 
 * 
 * Hardware Description: A pushbutton is connected via a 10k resistor to pin 
 *                       C2 and an LED is connected to pin C1 
 *           
 * Created June    17th, 2025, 10:27 PM
 * Updated January 02nd, 2026, 10:11 PM
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
    uint16_t holdTime = 0;
    uint8_t longPressDetected = 0;

    initMain();
    initGPIO();

    while (1)
    {
        // Button is being held down
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == 0)
        {
            Delay_Ms(10); // Small interval
            holdTime += 10;

            // Check for long press threshold
            if (holdTime >= 1000 && !longPressDetected)
            {
                // Toggle LED on long press
                if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_1))
                    GPIO_WriteBit(GPIOC, GPIO_Pin_1, Bit_RESET);
                else
                    GPIO_WriteBit(GPIOC, GPIO_Pin_1, Bit_SET);

                longPressDetected = 1; // Prevent retrigger while holding
            }
        }
        else
        {
            // Button released
            holdTime = 0;
            longPressDetected = 0;
        }
    }
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
