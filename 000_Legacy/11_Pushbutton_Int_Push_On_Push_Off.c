/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 11_Pushbutton_Int_Push_On_Push_Off
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to demonstrate 
 *                      use of pushbutton interrupt, using push on, push off
 *                      toggle button using an external interrupt. 
 *                      
 * 
 * Hardware Description: An LED is connected to pin C1 via a 1k resistor and 
 *                       a pushbutton is connectecto pin C2. 
 *           
 * Created June 17th, 2025, 10:19 PM
 * Updated June 17th, 2025, 10:19 PM
 */
 
/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"

volatile uint8_t interruptFlag = 0;

// === Initialization ===
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
 * Description: GPIO + EXTI setup, sets up PC1 as output and PC2 as input
 * 
 * Usage: initGPIO()
 ******************************************************************************/ 
void initGPIO_EXTI(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);

    // --- PC1 as LED output ---
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOC, GPIO_Pin_1, Bit_SET);  // LED OFF initially

    // --- PC2 as input with pull-up ---
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // --- Connect EXTI Line2 to PC2 ---
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);

    // --- Configure EXTI Line2 ---
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  // Falling edge (press)
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // --- Configure NVIC for EXTI7_0 (covers EXTI2) ---
    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
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
    initGPIO_EXTI();

    while(1)
    {
        if (interruptFlag)
        {
            interruptFlag = 0;

            // Toggle LED on PC1
            if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_1))
                GPIO_WriteBit(GPIOC, GPIO_Pin_1, Bit_RESET);
            else
                GPIO_WriteBit(GPIOC, GPIO_Pin_1, Bit_SET);
        }
    }
}


/*******************************************************************************
 * Function: ISR Handler
 *
 * Returns: Nothing
 *
 * Description: Handles ISR
 ******************************************************************************/
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) != RESET)
    {
        interruptFlag = 1;
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}
