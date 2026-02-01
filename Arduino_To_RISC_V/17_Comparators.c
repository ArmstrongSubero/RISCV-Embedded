/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 17_Comparator 
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to use the 
 *                      on board comparator module. The LED lights up 
 *                      when V+ > V- which can be swapped.  
 * 
 * Hardware Description: An LED is connected via a 1k resistor to pin 
 *                       D4, PA1 (-ve inverting) is connected to a 1k voltage divider and 
 *                       PA2 (+ve non-inverting) is connected to a potentiometer.
 *                       If you're using the WCH eval board, PA1 and PA2 are tied to 
 *                       the crystal, so you'll need to remove the crystal and 
 *                       load caps. 
 *           
 * Created August 21st, 2025, 7:00 PM
 * Updated August 21st, 2025, 7:00 PM
 */
 
/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"

// 1) Make sure we are not using HSE so PA1/PA2 can be GPIO/analog
static void use_internal_clock_only(void)
{
    // Enable HSI (internal RC) and wait ready
    RCC->CTLR |= RCC_HSION;
    while(!(RCC->CTLR & RCC_HSIRDY));

    // Switch SYSCLK to HSI
    RCC->CFGR0 &= ~RCC_SWS;          // clear status (read-only in many MCUs; fine to ignore)
    RCC->CFGR0 &= ~RCC_SW;           // clear switch bits
    RCC->CFGR0 |= RCC_SW_HSI;        // select HSI as system clock
    while((RCC->CFGR0 & RCC_SWS) != RCC_SWS_HSI);

    // Disable HSE so the pins aren’t reserved by the oscillator block
    RCC->CTLR &= ~RCC_HSEON;
    // (Optional) Disable PLL if it was using HSE
    RCC->CTLR &= ~RCC_PLLON;
}

// 2) Configure PA1/PA2 as analog inputs (high-Z, no digital buffer)
static void config_pa1_pa2_as_analog(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gi = {0};
    gi.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2;
    gi.GPIO_Mode  = GPIO_Mode_AIN;    // analog mode = no pull, no digital Schmitt trigger
    // Speed is ignored for AIN, but set something valid:
    gi.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gi);
}

// 3) Enable OPA as comparator: PA2 = V+, PA1 = V-
static void opa_enable_pa2_vs_pa1(void)
{
    // Ensure EN, and select PA2 (PSEL=0), PA1 (NSEL=0)
    EXTEN->EXTEN_CTR |= EXTEN_OPA_EN;
    EXTEN->EXTEN_CTR &= ~EXTEN_OPA_PSEL;  // 0 -> PA2 as positive
    EXTEN->EXTEN_CTR &= ~EXTEN_OPA_NSEL;  // 0 -> PA1 as negative
}

void init_for_comparator_on_pa1_pa2(void)
{
    use_internal_clock_only();     // stop HSE so PA1/PA2 are free
    config_pa1_pa2_as_analog();    // put pins in analog mode
    opa_enable_pa2_vs_pa1();       // turn on comparator
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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    // LED on PD0 as before...
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin = GPIO_Pin_0;
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOD, &g);

    init_for_comparator_on_pa1_pa2();

    while(1)
    {
        // Easiest: read the OPA output pin PD4 (wire LED/scope to PD4),
        // or mirror it in software if PD4 reads high/low:
    }
}
