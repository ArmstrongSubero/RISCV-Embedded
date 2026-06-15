 /*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 04_Bidirectional_Knightrider.c
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to have the Knight Rider/ Larson scanner 
 *                      effect
 *
 * Hardware Description: 8 LEDs are connected via a 1k resistors to PORTC (PC0..PC7)
 *           
 * Created August 10th, 2025, 5:30 PM
 * Updated August 10th, 2025, 5:30 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"

// --------- TUNE ME ---------
#define LED_ACTIVE_HIGH  1          // 1 = drive pin HIGH turns LED ON; 0 = drive pin LOW turns LED ON
#define STEP_DELAY_MS    60         // delay between steps

// List the pins you’re using on PORTC, in order left->right on your light bar
static const uint16_t PIN_LIST[] = {
    GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3,
    GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7
};

#define LED_COUNT (sizeof(PIN_LIST)/sizeof(PIN_LIST[0]))


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
 * Description: Initializes pins as push-pull @ 30 MHz can also be used 
 *              active low 
 * 
 * Usage: initGPIO()
 ******************************************************************************/
static void initGPIO(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin   =
        GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
        GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // All OFF to start
#if LED_ACTIVE_HIGH
    GPIO_ResetBits(GPIOC, GPIO_InitStructure.GPIO_Pin);
#else
    GPIO_SetBits(GPIOC, GPIO_InitStructure.GPIO_Pin);
#endif
}


/*******************************************************************************
 * Function: static inline void leds_all_off(void)
 *
 * Returns: Nothing
 *
 * Description: Turns all LEDs off 
 * 
 * Usage: leds_all_off()
 ******************************************************************************/
static inline void leds_all_off(void)
{
#if LED_ACTIVE_HIGH
    GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                          GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
#else
    GPIO_SetBits(GPIOC,   GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                          GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
#endif
}


/*******************************************************************************
 * Function: static inline void led_one_on(size_t i)
 *
 * Returns: Nothing
 *
 * Description: Lights exactly one LED at index i 
 * 
 * Usage: led_one_on((size_t)i)
 ******************************************************************************/
static inline void led_one_on(size_t i)
{
    uint16_t pin = PIN_LIST[i];
    leds_all_off();
#if LED_ACTIVE_HIGH
    GPIO_SetBits(GPIOC, pin);
#else
    GPIO_ResetBits(GPIOC, pin);
#endif
}


/*******************************************************************************
 * Function: static void knight_rider_step(void)
 *
 * Returns: Nothing
 *
 * Description: Knight Rider sweep: 0->N-1 then N-2->1
 * 
 * Usage: knight_rider_step()
 ******************************************************************************/
static void knight_rider_step(void)
{
    // forward
    for (size_t i = 0; i < LED_COUNT; ++i) {
        led_one_on(i);
        Delay_Ms(STEP_DELAY_MS);
    }
    // backward (skip the ends to avoid a double hold)
    for (int i = (int)LED_COUNT - 2; i >= 1; --i) {
        led_one_on((size_t)i);
        Delay_Ms(STEP_DELAY_MS);
    }
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

    while (1) {
        knight_rider_step();
    }
}
