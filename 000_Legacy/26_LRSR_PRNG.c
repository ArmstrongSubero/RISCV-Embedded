/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/ Int OSC @ 48 MHz, 3.3V
 * Program: 28_LFSR_PRNG
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: Demonstrates a 16-bit Linear Feedback Shift Register (LFSR) pseudo-random
 *                      number generator on the CH32V003F4P6. The LFSR uses polynomial 0xB400.
 *                      Each iteration updates the LFSR state and toggles an LED on PD4 based on
 *                      the parity of the generated value (odd → LED ON, even → LED OFF).
 *
 * Hardware Description: An LED (with ~1 kΩ series resistor) is connected to pin PD4.
 *
 * Created:  August 21st, 2025, 7:50 PM
 * Updated:  August 21st, 2025, 7:50 PM
 */
 
/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"  

/* =========================
   16-bit LFSR RNG (0xB400)
   ========================= */
static uint16_t lfsr_state = 0;

static inline uint16_t lfsr16_next(uint16_t n) {
    // Same poly as your STM32F0 version
    return (n >> 1) ^ (-(n & 1u) & 0xB400u);
}

static inline void random_init(uint16_t seed) {
    if (seed == 0) seed = 0xACE1;     // avoid zero lock
    lfsr_state = seed;
}

static inline uint16_t random16(void) {
    lfsr_state = lfsr16_next(lfsr_state);
    return lfsr_state;
}

/* =========================
   Board Init
   ========================= */
static void initMain(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
}

static void initGPIO_PD4_LED(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;        // PD4
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  // push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;  // OK on CH32V003
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Start LED off
    GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);
}

/* =========================
   Main
   ========================= */
int main(void)
{
    initMain();
    initGPIO_PD4_LED();

    // Seed with something variable; fallback given
    random_init((uint16_t)SysTick->CNT ^ 0xABCD);

    while (1)
    {
        uint16_t r = random16();

        if (r & 1u) {
            // Odd → LED ON
            GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_SET);
            // (Direct register alternative)
            // GPIOD->BSHR = (1u << 4);
        } else {
            // Even → LED OFF
            GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);
            // GPIOD->BSHR = (1u << (4 + 16));
        }

        Delay_Ms(100);   // adjust blink speed here
    }
}
