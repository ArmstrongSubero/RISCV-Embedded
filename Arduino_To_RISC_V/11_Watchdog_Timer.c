/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 11_Watchdog_Timer
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.3.0)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program is adapted from the WCH example and 
 *                      allows us to see usage of the Independent Watchdog (IWDG) 
 *                      with an LED. Here we have an LED that toggles every 200 ms 
 *                      if FEED_WATCHDOG = 1, we feed the watchdog and there is no 
 *                      reset, if FEED_WATCHDOG = 0, it does not feed the watchdog 
 *                      and the CH32V003F4P6 will reset after ~4s. 
 * 
 * Hardware Description: An LED is connected via a 1k redidtot to PD0
 *           
 * Created June    17th, 2025, 11:27 PM
 * Updated January 02nd, 2026, 10:50 PM
 */


/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"

#define FEED_WATCHDOG  0   // 1 = keep alive; 0 = demonstrate reset after timeout

// ===== LED helpers (active-low) =====
static inline void led_on(void)  { GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_RESET); }
static inline void led_off(void) { GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_SET);   }
static inline void led_toggle(void)
{
    if (GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_0)) led_on(); else led_off();
}

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
 * Function: static void initGPIO(void)
 *
 * Returns: Nothing
 *
 * Description: Sets up PD0 as output
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

    led_off();  // start LED off (active-low)
}

/*******************************************************************************
 * Function: static void boot_blink(void)
 *
 * Returns: Nothing
 *
 * Description: Quick triple-blink on LED so we can see resets 
 * 
 * Usage: boot_blink()
 ******************************************************************************/
static void boot_blink(void)
{
    // Quick triple-blink so you can see resets
    for (int i = 0; i < 3; ++i) { led_on(); Delay_Ms(80); led_off(); Delay_Ms(80); }
}

/*******************************************************************************
 * Function: static void iwdg_init(void)
 *
 * Returns: Nothing
 *
 * Description: Independent watchdog timer init 128 prescaler, 4000 (4s) reload 
 * 
 * Usage: iwdg_init()
 ******************************************************************************/
static void iwdg_init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_128);
    IWDG_SetReload(4000);
    IWDG_ReloadCounter();
    IWDG_Enable();
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
    boot_blink();
    iwdg_init();

    uint32_t ms = 0;

    while (1)
    {
        led_toggle();
        Delay_Ms(200);
        ms += 200;

#if FEED_WATCHDOG
        // Feed often enough (< timeout). Here: every 400 ms.
        if (ms >= 400) {
            IWDG_ReloadCounter();
            ms = 0;
        }
#endif
        // If FEED_WATCHDOG == 0, we never reload → device will reset after ~4 s.
    }
}
