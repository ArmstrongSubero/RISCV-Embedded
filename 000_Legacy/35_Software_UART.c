/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 35_Software_UART
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to communicate 
 *                      via software UART since the device only has one 
 *                      UART module by default. 
 * 
 * Hardware Description: A CP2104 module is connected to the 
 *                       CH32V003F4P6 as follows:
 *                       
 *                       GND -> Common
 *                       RX  -> PD2
 *                       TX  -> PD3
 *
 * Created August 23rd, 2025, 1:39 PM
 * Updated August 23rd, 2025, 1:39 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"
#include "ch32v00x_rcc.h"
#include "ch32v00x_gpio.h"
#include "ch32v00x_tim.h"
#include <stdint.h>
#include <stdio.h>

/*******************************************************************************
 * User Config 
 ******************************************************************************/
#define SUART_BAUDRATE     9600
#define SUART_RX_PORT      GPIOD
#define SUART_RX_PIN       GPIO_Pin_2    // PD2 input
#define SUART_TX_PORT      GPIOD
#define SUART_TX_PIN       GPIO_Pin_3    // PD3 output

/*******************************************************************************
 * Set Timebase TIM2 -> 1 MHz 
 ******************************************************************************/
static void timebase_init_1mhz(void){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef t = {0};
    t.TIM_Prescaler     = (SystemCoreClock / 1000000UL) - 1;
    t.TIM_CounterMode   = TIM_CounterMode_Up;
    t.TIM_Period        = 0xFFFF;
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &t);
    TIM_Cmd(TIM2, ENABLE);
}
static inline uint16_t t2_now(void){ return TIM2->CNT; }
static inline void t2_wait_until(uint16_t deadline){
    while ((int16_t)(TIM2->CNT - deadline) < 0) { /* spin */ }
}

/*******************************************************************************
 *Software UART implementation 
 ******************************************************************************/
#define BIT_TICKS ((uint16_t)(1000000UL / SUART_BAUDRATE))

/* fast GPIO */
static inline uint8_t suart_rx_sample(void){
    return (SUART_RX_PORT->INDR & SUART_RX_PIN) ? 1u : 0u;
}
static inline void suart_tx_high(void){ SUART_TX_PORT->BSHR = SUART_TX_PIN; }
static inline void suart_tx_low(void){  SUART_TX_PORT->BCR  = SUART_TX_PIN; }

/* init pins + timebase */
static void SUART_Init(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = SUART_RX_PIN;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Mode  = GPIO_Mode_IPU;           // input pull-up, idle high
    GPIO_Init(SUART_RX_PORT, &g);

    g.GPIO_Pin   = SUART_TX_PIN;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;        // push-pull
    GPIO_Init(SUART_TX_PORT, &g);
    suart_tx_high();                        // idle high

    timebase_init_1mhz();
}

/* RX one byte (blocking, 8N1) */
static uint8_t SUART_ReadByte(void){
    while (suart_rx_sample() == 0u) { }     // wait idle high
    while (suart_rx_sample() != 0u) { }     // wait start edge

    __disable_irq();

    uint16_t t = t2_now();
    t += (uint16_t)(BIT_TICKS + BIT_TICKS/2);   // mid bit0

    uint8_t v = 0;
    for (uint8_t i=0;i<8;i++){
        t2_wait_until(t);
        if (suart_rx_sample()) v |= (1u<<i);
        t += BIT_TICKS;
    }
    t2_wait_until(t);                        // stop bit center (optional)

    __enable_irq();
    return v;
}

/* TX one byte (blocking, 8N1) */
static void SUART_WriteByte(uint8_t b){
    __disable_irq();

    suart_tx_low();                          // start bit
    uint16_t t = t2_now() + BIT_TICKS;

    for (uint8_t i=0;i<8;i++){               // data bits LSB first
        t2_wait_until(t);
        if (b & (1u<<i)) suart_tx_high(); else suart_tx_low();
        t += BIT_TICKS;
    }

    t2_wait_until(t);
    suart_tx_high();                         // stop bit
    t += BIT_TICKS;
    t2_wait_until(t);                        // guard

    __enable_irq();
}

/* helpers */
static void SUART_WriteString(const char *s){
    while (*s) SUART_WriteByte((uint8_t)*s++);
}
static void SUART_WriteCRLF(void){
    SUART_WriteByte('\r'); SUART_WriteByte('\n');
}

/* ===================== Demo main ===================== */
int main(void){
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);               // hardware printf to PC 
#endif

    printf("SystemClk:%ld\r\n", SystemCoreClock);
    printf("SoftUART RX=PD2 TX=PD3 @ %u bps (8N1)\r\n", (unsigned)SUART_BAUDRATE);

    SUART_Init();

    /* banner over the SOFTWARE UART */
    SUART_WriteString("SoftUART online @ ");
    char buf[16];
    snprintf(buf, sizeof(buf), "%u", (unsigned)SUART_BAUDRATE);
    SUART_WriteString(buf);
    SUART_WriteString(" bps\r\nType a character; I will send (char+1) back.\r\n");

    while (1){
        uint8_t c = SUART_ReadByte();

        /* simple demo: add 1 and send back */
        uint8_t out = (uint8_t)(c + 1u);
        SUART_WriteByte(out);

        /* Also mirror to hardware UART for debugging */
        printf("RX 0x%02X ('%c') -> TX 0x%02X ('%c')\r\n",
               c, (c>=32 && c<127)?c:'.', out, (out>=32 && out<127)?out:'.');
    }
}
