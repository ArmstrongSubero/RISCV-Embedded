/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 31_Servo_Library
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to demonstrate 
 *                      use of a servo library. 
 * 
 * Hardware Description: A servo is connected to pin PD2

 * Created August 18th, 2025, 12:02 AM
 * Updated August 18th, 2025, 12:02 AM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "debug.h"
#include <stdint.h>

#define TIM1_PSC_48MHZ   47                 // 48MHz/(47+1)=1MHz -> 1 µs tick
#define SERVO_MIN_US     500                // safer SG90 min (tune if needed)
#define SERVO_MAX_US     2500               // typical SG90 max (tune if needed)
#define SERVO_CENTER_US  ((SERVO_MIN_US+SERVO_MAX_US)/2)
#define SERVO_SAFE_MIN_DEG  5               // avoid hard end-stops for sweeps
#define SERVO_SAFE_MAX_DEG  175

#define SERVO_SAFE_MIN   0      // clamp angles to avoid overdrive
#define SERVO_SAFE_MAX   180

// ===== S-curve helpers (fixed-point Q15: 1.0 == 32768) =====
#define Q15_ONE  32768u

static inline uint32_t q15_mul(uint32_t a, uint32_t b)
{
    // (a*b)>>15 with rounding; a,b in [0..Q15_ONE]
    return (uint32_t)(((uint64_t)a * (uint64_t)b + (1u<<14)) >> 15);
}

// Quintic smoothstep: s(t) = 6t^5 - 15t^4 + 10t^3,  t in Q15 [0..1]
static inline uint32_t scurve5_q15(uint32_t t)
{
    // All operations in Q15
    uint32_t t2 = q15_mul(t, t);
    uint32_t t3 = q15_mul(t2, t);
    uint32_t t4 = q15_mul(t3, t);
    uint32_t t5 = q15_mul(t4, t);

    // 6*t^5 - 15*t^4 + 10*t^3  (still Q15)
    // Multiply by small ints in integer domain (safe; result still Q15 range)
    int64_t s = (int64_t)6 * (int64_t)t5
              - (int64_t)15 * (int64_t)t4
              + (int64_t)10 * (int64_t)t3;

    // Clamp to [0..Q15_ONE]
    if (s < 0) s = 0;
    if (s > Q15_ONE) s = Q15_ONE;
    return (uint32_t)s;
}


static inline uint16_t deg_to_us(uint16_t deg)
{
    if (deg < SERVO_SAFE_MIN_DEG) deg = SERVO_SAFE_MIN_DEG;
    if (deg > SERVO_SAFE_MAX_DEG) deg = SERVO_SAFE_MAX_DEG;
    uint32_t span = (uint32_t)(SERVO_MAX_US - SERVO_MIN_US);
    return (uint16_t)(SERVO_MIN_US + (span * deg) / 180U);
}

static inline void servo_write_us(uint16_t us)
{
    if (us < SERVO_MIN_US)  us = SERVO_MIN_US;
    if (us > SERVO_MAX_US)  us = SERVO_MAX_US;
    TIM1->CH1CVR = us;                      
}

static void pwm_init_pd2_tim1(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1, ENABLE);

    // PD2 -> TIM1_CH1 (AF push-pull)
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = GPIO_Pin_2;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOD, &g);

    // TIM1 @ 1 µs tick, 20 ms frame (50 Hz)
    TIM_TimeBaseInitTypeDef tb = {0};
    tb.TIM_Prescaler     = TIM1_PSC_48MHZ; // 47 for 48MHz; use 23 for 24MHz cores
    tb.TIM_Period        = 20000 - 1;      // 20 ms
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1, &tb);

    TIM_OCInitTypeDef oc = {0};
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = SERVO_CENTER_US;
    TIM_OC1Init(TIM1, &oc);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/* Set absolute angle in degrees (0..180) */
static inline void servo_set_angle_deg(uint16_t deg)
{
    servo_write_us(deg_to_us(deg));
}

/* Smooth sweep with simple accel/const/decel profile */
static void servo_sweep_deg(uint16_t start_deg, uint16_t end_deg, uint32_t duration_ms)
{
    if (start_deg > 180) start_deg = 180;
    if (end_deg   > 180) end_deg   = 180;
    int32_t diff = (int32_t)end_deg - (int32_t)start_deg;
    uint32_t steps = (uint32_t)(diff >= 0 ? diff : -diff);   // 1° steps
    if (steps == 0) { servo_set_angle_deg(end_deg); return; }

    uint32_t per_step = duration_ms / steps;
    if (per_step == 0) per_step = 1;

    uint32_t accel = steps / 4, mid = steps / 2, decel = steps - (accel + mid);

    // accel
    for (uint32_t i = 0; i < accel; ++i){
        uint16_t a = (diff > 0) ? (start_deg + i) : (start_deg - i);
        servo_set_angle_deg(a);
        Delay_Ms(per_step * 2);
    }
    // constant
    for (uint32_t i = 0; i < mid; ++i){
        uint16_t a = (diff > 0) ? (start_deg + accel + i) : (start_deg - (accel + i));
        servo_set_angle_deg(a);
        Delay_Ms(per_step);
    }
    // decel
    for (uint32_t i = 0; i < decel; ++i){
        uint16_t a = (diff > 0) ? (start_deg + accel + mid + i) : (start_deg - (accel + mid + i));
        servo_set_angle_deg(a);
        Delay_Ms(per_step * 2);
    }
    servo_set_angle_deg(end_deg);
}

/* Smooth S-curve sweep using quintic profile (zero jerk at ends)
   - start_deg, end_deg: 0..180 (will be clamped)
   - duration_ms: total motion time
   - tick_ms: control interval (e.g., 10 ms). Smaller = smoother.
*/
static void servo_sweep_deg_scurve(uint16_t start_deg,
                                   uint16_t end_deg,
                                   uint32_t duration_ms,
                                   uint16_t tick_ms)
{
    if (start_deg > 180) start_deg = 180;
    if (end_deg   > 180) end_deg   = 180;
    if (duration_ms < 5) duration_ms = 5;
    if (tick_ms < 2) tick_ms = 2;

    int32_t diff = (int32_t)end_deg - (int32_t)start_deg;

    // Number of control updates
    uint32_t steps = duration_ms / tick_ms;
    if (steps < 1) steps = 1;

    for (uint32_t k = 0; k <= steps; ++k){
        // progress t in [0..1] as Q15
        uint32_t t_q15 = (uint32_t)((uint64_t)Q15_ONE * k / steps);

        // S-curve easing
        uint32_t s_q15 = scurve5_q15(t_q15);

        // angle = start + diff * s
        int32_t delta = (int32_t)(((int64_t)diff * (int64_t)s_q15 + (Q15_ONE>>1)) >> 15);
        int32_t a = (int32_t)start_deg + delta;

        // safety clamp
        if (a < SERVO_SAFE_MIN) a = SERVO_SAFE_MIN;
        if (a > SERVO_SAFE_MAX) a = SERVO_SAFE_MAX;

        servo_set_angle_deg((uint16_t)a);
        Delay_Ms(tick_ms);
    }

    // ensure exact final position
    servo_set_angle_deg(end_deg);
}

// Set angle with S-curve motion from current to target
static void servo_set_angle_scurve(uint16_t target_deg,
                                   uint32_t duration_ms,
                                   uint16_t tick_ms)
{
    // clamp target
    if (target_deg > 180) target_deg = 180;

    // read current position by reverse-mapping current TIM1->CH1CVR to degrees
    uint16_t current_us = TIM1->CH1CVR;
    int32_t span_us = SERVO_MAX_US - SERVO_MIN_US;
    uint16_t current_deg = (uint16_t)(((current_us - SERVO_MIN_US) * 180U) / span_us);

    int32_t diff = (int32_t)target_deg - (int32_t)current_deg;
    if (diff == 0) return;

    if (duration_ms < 5) duration_ms = 5;
    if (tick_ms < 2) tick_ms = 2;

    uint32_t steps = duration_ms / tick_ms;
    if (steps < 1) steps = 1;

    for (uint32_t k = 0; k <= steps; ++k){
        uint32_t t_q15 = (uint32_t)((uint64_t)Q15_ONE * k / steps);
        uint32_t s_q15 = scurve5_q15(t_q15);
        int32_t delta = (int32_t)(((int64_t)diff * (int64_t)s_q15 + (Q15_ONE>>1)) >> 15);
        int32_t a = (int32_t)current_deg + delta;

        if (a < SERVO_SAFE_MIN) a = SERVO_SAFE_MIN;
        if (a > SERVO_SAFE_MAX) a = SERVO_SAFE_MAX;

        servo_set_angle_deg((uint16_t)a);
        Delay_Ms(tick_ms);
    }
    servo_set_angle_deg(target_deg);
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
    pwm_init_pd2_tim1();

    // park at 0°, pause
    servo_set_angle_deg(0);
    Delay_Ms(1000);

    servo_set_angle_deg(90);
    Delay_Ms(1000);

    servo_set_angle_deg(180);
    Delay_Ms(1000);

    servo_set_angle_scurve(0, 500, 10);     // move to 90° in 0.5s, silky smooth
    Delay_Ms(1000);
    servo_set_angle_scurve(90,  500, 10);   // move to 90° in 0.5s
    Delay_Ms(1000);
    servo_set_angle_scurve(180,  500, 10);  // move to 180° in 0.5s
    Delay_Ms(1000);

    while (1)
    {
        // 0 → 180 in 2000 ms, back to 0 in 2000 ms (silky S-curve)
        servo_sweep_deg_scurve(0,   180, 2000, 10);  Delay_Ms(300);
        servo_sweep_deg_scurve(180,   0, 2000, 10);  Delay_Ms(300);

        // quicker sweeps; still smooth due to S-curve shaping
        servo_sweep_deg_scurve(0,   180, 600,  8);   Delay_Ms(200);
        servo_sweep_deg_scurve(180,   0, 600,  8);   Delay_Ms(200);
    }
}
