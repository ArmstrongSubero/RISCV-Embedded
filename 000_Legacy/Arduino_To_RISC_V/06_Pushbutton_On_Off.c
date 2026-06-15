/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 06_Pushbutton_On_Off
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.3.0)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to read a
 *                      pushbutton connected to PIN C2 via debounce 
 * 
 * Hardware Description: An LED is connected via a 1k resistor to pin 
 *                       C1 and a pushbutton is connected to pin C2 
 *           
 *           
 * Created August  19th, 2025, 3:23 PM
 * Updated January 02nd, 2026, 8:09 PM
 */
 
 /*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"

void initGPIO(void);
uint8_t debounceButton(GPIO_TypeDef* port, uint16_t pin);

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
    initMain();
    initGPIO();

    while(1)
    {
        if (debounceButton(GPIOC, GPIO_Pin_2))
        {
            // Toggle LED on PC1
            if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_1))
                GPIO_WriteBit(GPIOC, GPIO_Pin_1, Bit_RESET);
            else
                GPIO_WriteBit(GPIOC, GPIO_Pin_1, Bit_SET);
        }
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
 * Returns: Debouce Output
 *
 * Description: Debounce Helper
 * 
 * Usage: debounceButton(Port, Pin)
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

