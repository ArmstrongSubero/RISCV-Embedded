/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 30_DC_Motor
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to interface with 
 *                      a DC motor through L293D motor driver 
 * 
 * Hardware Description: A L293D motor driver is connected to the CH32V003F4P6 
 *                       as follows:
 *           
 *                       Vcc1 -> 5v
 *                       Vcc2 -> 5v 
 *                       GND -> Gnd
 *                       C0  -> 1A
 *                       C1  -> 2A 
 *                       D2  -> 1,2 EN 
 *                      
 * Created August 23rd, 2025, 12:44 PM
 * Updated August 23rd, 2025, 12:44 PM
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

/* -------- Pin map -------- */
#define M1_IN1_PORT  GPIOC   /* -> L293D pin 2 (1A) */
#define M1_IN1_PIN   GPIO_Pin_0
#define M1_IN2_PORT  GPIOC   /* -> L293D pin 7 (2A) */
#define M1_IN2_PIN   GPIO_Pin_1
#define M1_EN_PORT   GPIOD   /* -> L293D pin 1 (1,2EN) = PWM */
#define M1_EN_PIN    GPIO_Pin_2   /* TIM1_CH1 */

/* -------- Select PWM frequency --------
 * 20 kHz: ARR=2399  (quiet)
 *  5 kHz: ARR=9599  (more starting torque per pulse)
 */
#define USE_5KHZ   1
#if USE_5KHZ
  #define PWM_ARR  (9599)
#else
  #define PWM_ARR  (2399)
#endif
#define PWM_PSC  (0)

/* Helpers */
#define PIN_SET(PORT,PIN)  ((PORT)->BSHR = (PIN))
#define PIN_CLR(PORT,PIN)  ((PORT)->BCR  = (PIN))

/* --- GPIO init --- */
static void init_pwm_gpio(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = M1_EN_PIN;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(M1_EN_PORT, &g);
}
static void init_dir_gpio(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = M1_IN1_PIN | M1_IN2_PIN;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(M1_IN1_PORT, &g);
}

/* --- TIM1 CH1 PWM --- */
static void init_tim1_pwm(uint16_t arr, uint16_t psc, uint16_t duty){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef tb = {0};
    tb.TIM_Period        = arr;
    tb.TIM_Prescaler     = psc;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &tb);

    TIM_OCInitTypeDef oc = {0};
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = duty;
    TIM_OC1Init(TIM1, &oc);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/* --- Direction & brake/coast --- */
static inline void motor_fwd(void){  PIN_SET(M1_IN1_PORT, M1_IN1_PIN); PIN_CLR(M1_IN2_PORT, M1_IN2_PIN); }
static inline void motor_rev(void){  PIN_CLR(M1_IN1_PORT, M1_IN1_PIN); PIN_SET(M1_IN2_PORT, M1_IN2_PIN); }
static inline void motor_coast(void){PIN_CLR(M1_IN1_PORT, M1_IN1_PIN); PIN_CLR(M1_IN2_PORT, M1_IN2_PIN); }
static inline void motor_brake(void){PIN_SET(M1_IN1_PORT, M1_IN1_PIN); PIN_SET(M1_IN2_PORT, M1_IN2_PIN); }

/* --- Duty helpers --- */
static inline void pwm_set_raw(uint16_t ccr){ if (ccr > PWM_ARR) ccr = PWM_ARR; TIM_SetCompare1(TIM1, ccr); }
static void pwm_set_percent(uint8_t pct){
    if (pct > 100) pct = 100;
    uint32_t ccr = ((uint32_t)(PWM_ARR + 1) * pct) / 100;
    pwm_set_raw((uint16_t)ccr);
}

/* --- Kick-start + settle --- */
static void motor_go_fwd(uint8_t target_pct){
    /* Brake briefly to stop rotor, then set forward */
    motor_brake(); Delay_Ms(80);
    motor_fwd();

    /* Kick at 100% to overcome static friction */
    pwm_set_percent(100);
    Delay_Ms(120);   // 80–150 ms is typical; tune to your load

    /* Settle to requested target (e.g., 20%) */
    pwm_set_percent(target_pct);
}

/* --- Same as forward Kick-start + settle --- */
static void motor_go_rev(uint8_t target_pct){
    motor_brake(); Delay_Ms(80);
    motor_rev();
    pwm_set_percent(100);
    Delay_Ms(120);
    pwm_set_percent(target_pct);
}

/* --- Optional: tiny ramp (linear) --- */
static void ramp_to(uint8_t target_pct, uint8_t step, uint16_t step_ms){
    /* read current CCR, derive current % */
    uint16_t cur = TIM_GetCapture1(TIM1);
    uint8_t cur_pct = (uint8_t)((cur * 100UL) / (PWM_ARR + 1));
    if (cur_pct < target_pct){
        for (uint8_t p=cur_pct; p<=target_pct; p+=step){ pwm_set_percent(p); Delay_Ms(step_ms); }
    }else{
        for (int p=cur_pct; p>=target_pct; p-=step){ pwm_set_percent((uint8_t)p); Delay_Ms(step_ms); }
    }
}

/*******************************************************************************
 * Function: Main
 *
 * Returns: Nothing
 *
 * Description: Program entry point
 ******************************************************************************/
int main(void){
    SystemCoreClockUpdate();
    Delay_Init();
#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif

    printf("CH32V003 + L293D — HW PWM on PD2 (%s)\r\n", USE_5KHZ ? "5 kHz" : "20 kHz");
    printf("PD2->EN, PC0->1A, PC1->2A. VCC1=5V, VCC2=motor, GNDs common.\r\n");

    init_dir_gpio();
    init_pwm_gpio();
    init_tim1_pwm(PWM_ARR, PWM_PSC, 0);

    /* Demo: forward @50% with kick, then reverse @70% with kick */
    while (1){
        motor_go_fwd(50);     // kick @100% then hold 20%
        Delay_Ms(2000);

        motor_brake(); Delay_Ms(200);
        motor_go_rev(70);     // kick then hold 30%
        Delay_Ms(2000);

        /* slow ramp down to 0 and coast */
        ramp_to(0, 5, 100);
        
        motor_coast(); 
        Delay_Ms(800);
    }
}
