/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 04_Complementary_Drive_Circuit.c
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.3.0)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 control six LEDs with
 *                      3 lines using "Charlieplexing".
 * 
 * Hardware Description: Three LEDs are connected via a 1k resistors to PC1, PC2 and PC3
 *           
 * Created August  10th, 2025, 5:20 PM
 * Updated January 02nd, 2026, 8:04 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"

#define LED_PIN_1 GPIO_Pin_1
#define LED_PIN_2 GPIO_Pin_2
#define LED_PIN_3 GPIO_Pin_3
#define LED_PORT GPIOC

typedef enum {
    LED_1 = 0,
    LED_2,
    LED_3,
    LED_4,
    LED_5,
    LED_6,
    LED_COUNT
} LEDIndex;

typedef struct {
    uint16_t source;
    uint16_t sink;
    uint16_t hiz;
} LEDControlMap;

LEDControlMap ledMap[LED_COUNT] = {
    {LED_PIN_1, LED_PIN_2, LED_PIN_3},  // LED 1
    {LED_PIN_2, LED_PIN_1, LED_PIN_3},  // LED 2
    {LED_PIN_1, LED_PIN_3, LED_PIN_2},  // LED 3
    {LED_PIN_3, LED_PIN_1, LED_PIN_2},  // LED 4
    {LED_PIN_2, LED_PIN_3, LED_PIN_1},  // LED 5
    {LED_PIN_3, LED_PIN_2, LED_PIN_1},  // LED 6
};


void enableLED(LEDIndex led);
void set_pin(GPIO_TypeDef* port, uint16_t pin, GPIOMode_TypeDef mode, BitAction val);


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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
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

    while(1)
    {
       for (LEDIndex i = LED_1; i < LED_COUNT; i++)
        {
            enableLED(i);
            Delay_Ms(1000);
        }
    }
}

/*******************************************************************************
 * Function: void set_pin(GPIO_TypeDef* port, uint16_t pin, 
 *                 GPIOMode_TypeDef mode, BitAction val)
 *
 * Returns: Nothing
 *
 * Description: Contains initializations for main
 * 
 * Usage: set_pin(LED_PORT, config.source, GPIO_Mode_Out_PP, Bit_SET); 
 *        set_pin(LED_PORT, config.sink, GPIO_Mode_Out_PP, Bit_RESET);    
 *        set_pin(LED_PORT, config.hiz, GPIO_Mode_IN_FLOATING, 0); 
 ******************************************************************************/
void set_pin(GPIO_TypeDef* port, uint16_t pin, GPIOMode_TypeDef mode, BitAction val)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_InitStructure.GPIO_Mode = mode;
    GPIO_Init(port, &GPIO_InitStructure);

    if (mode == GPIO_Mode_Out_PP)
    {
        GPIO_WriteBit(port, pin, val);
    }
}


/*******************************************************************************
 * Function: void enableLED(LEDIndex led)
 *
 * Returns: Nothing
 *
 * Description: Contains initializations for main
 * 
 * Usage: enableLED(i)
 ******************************************************************************/
void enableLED(LEDIndex led)
{
    LEDControlMap config = ledMap[led];

    set_pin(LED_PORT, config.source, GPIO_Mode_Out_PP, Bit_SET);     // Source HIGH
    set_pin(LED_PORT, config.sink, GPIO_Mode_Out_PP, Bit_RESET);     // Sink LOW
    set_pin(LED_PORT, config.hiz, GPIO_Mode_IN_FLOATING, 0);         // Hi-Z
}

