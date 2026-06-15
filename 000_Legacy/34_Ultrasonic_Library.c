/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 34_Ultrasonic Library 
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to demonstrate 
 *                      use of an ultrasonic sensor. 
 * 
 * Hardware Description: A HCSR-04 is connected as follows: 
 *                       VCC  -> 5V
 *                       GND  -> GND 
 *                       TRIG -> C1 
 *                       ECHO -> C2
 *
 * Created August 19th, 2025, 10:36 PM
 * Updated August 19th, 2025, 10:36 PM
 */

#include "ch32v00x.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "debug.h"

// -------- Pins --------
#define HCSR04_TRIG_PORT  GPIOC
#define HCSR04_TRIG_PIN   GPIO_Pin_1
#define HCSR04_ECHO_PORT  GPIOC
#define HCSR04_ECHO_PIN   GPIO_Pin_2

// -------- TIM2 @ 1 MHz for micros() --------
static volatile uint32_t tim2_high = 0;

static void micros_timer_init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // Use the real APB1 frequency (fixes the 2× error)
    RCC_ClocksTypeDef c;
    RCC_GetClocksFreq(&c);
    uint32_t pclk = c.PCLK1_Frequency;              // actual TIM2 clock
    uint16_t psc  = (uint16_t)((pclk / 1000000UL) - 1);

    TIM_TimeBaseInitTypeDef tb = {0};
    tb.TIM_Period        = 0xFFFF;
    tb.TIM_Prescaler     = psc;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tb);

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nvic = {0};
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        tim2_high++;
    }
}

static inline uint32_t micros(void) {
    uint32_t hi1 = tim2_high;
    uint16_t lo  = TIM2->CNT;
    uint32_t hi2 = tim2_high;
    if (hi2 != hi1) { lo = TIM2->CNT; hi1 = hi2; }
    return (hi1 << 16) | lo; // 1 tick = 1 µs
}

// -------- GPIO + helpers --------
static inline uint8_t ECHO_READ(void){
    return GPIO_ReadInputDataBit(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) ? 1 : 0;
}

static void hcsr04_gpio_init(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef g = {0};
    // TRIG: push-pull output, low
    g.GPIO_Pin   = HCSR04_TRIG_PIN;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(HCSR04_TRIG_PORT, &g);
    GPIO_ResetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);

    // ECHO: input pull-down (idles low). If jittery, try GPIO_Mode_IN_FLOATING.
    g.GPIO_Pin   = HCSR04_ECHO_PIN;
    g.GPIO_Mode  = GPIO_Mode_IPD;
    g.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(HCSR04_ECHO_PORT, &g);
}

// Measure ECHO pulse width in microseconds (0 on timeout)
static uint32_t hcsr04_pulse_us(uint32_t timeout_ms){
    const uint32_t timeout_us = timeout_ms * 1000u;

    // ensure idle low
    uint32_t t = micros();
    while (ECHO_READ()){
        if (micros() - t > timeout_us) return 0;
    }

    // 10 µs trigger
    GPIO_ResetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);
    Delay_Us(2);
    GPIO_SetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);
    Delay_Us(12);
    GPIO_ResetBits(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN);

    // wait rising
    t = micros();
    while (!ECHO_READ()){
        if (micros() - t > timeout_us) return 0;
    }

    // measure high
    uint32_t t0 = micros();
    while (ECHO_READ()){
        if (micros() - t0 > timeout_us) return 0;
    }
    return micros() - t0;
}

// distance_mm = pulse_us * 343 / 2000  (0.1715 mm/us)
static uint32_t hcsr04_distance_mm(uint32_t timeout_ms){
    uint32_t us = hcsr04_pulse_us(timeout_ms);
    if (!us) return 0;
    return (us * 343u + 999u) / 2000u;   // rounded
}


// ----------------------------------------
// - Main 
// - Reads Ultrasonic Sensor and Prints 
// - distance
// ----------------------------------------
int main(void){
    SystemInit();
    Delay_Init();
    USART_Printf_Init(115200);

    printf("SystemClk:%lu\r\n", SystemCoreClock);
    printf("HC-SR04 (integer print) TRIG=PC1 ECHO=PC2\r\n");

    micros_timer_init();
    hcsr04_gpio_init();

    while (1){
        uint32_t us  = hcsr04_pulse_us(50);
        uint32_t mm  = (us) ? (us * 343u + 999u) / 2000u : 0;
        if (us){
            uint32_t cm100 = mm * 10u;   // hundredths of cm = mm * 0.1 => mm*10
            printf("pulse=%lu us  distance=%lu mm  (%lu.%02lu cm)\r\n",
                   (unsigned long)us,
                   (unsigned long)mm,
                   (unsigned long)(cm100/100),
                   (unsigned long)(cm100%100));
        }else{
            printf("Timeout / no object\r\n");
        }
        Delay_Ms(150);
    }
}
