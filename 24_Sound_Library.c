/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 21_ADC
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to demonstrate 
 *                      use of a piezeo speaker sound library
 * 
 * Hardware Description: A piezo speaker is connected to pin A1

 * Created August 16th, 2025, 12:02 AM
 * Updated August 16th, 2025, 12:02 AM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "ch32v00x.h"
#include <stdint.h>
#include <stdbool.h>
#include "debug.h" 

/* ===== Pin mapping (default: PA1 -> TIM1_CH2) ===== */
#define BUZZER_PORT      GPIOA
#define BUZZER_PIN       GPIO_Pin_1   // PA1
#define BUZZER_TIMER     TIM1
#define BUZZER_CH_SETCCR(T, val)   (T->CH2CVR = (val))  // CCR2 register

/* Public API */
void Buzzer_Init(void);
void Buzzer_Set(uint32_t frequency_hz, uint8_t volume_pct);  // volume 0..100
void Buzzer_Stop(void);

/* Beep Function */
void Buzzer_Beep(uint32_t frequency_hz, uint8_t volume_pct, uint32_t duration_ms);

/* UI tones */
void Buzzer_Success(void);
void Buzzer_Failure(void);
void Buzzer_Warning(void);
void Buzzer_Alert(void);
void Buzzer_KeyPress(void);
void Buzzer_MenuSelect(void);
void Buzzer_MenuBack(void);
void Buzzer_PowerUp(void);
void Buzzer_PowerDown(void);
void Buzzer_CriticalBattery(void);
void Buzzer_Processing(void);
void Buzzer_Error(void);

/* Effects (no floats) */
void Buzzer_SweepUp(uint32_t start_hz, uint32_t end_hz, uint32_t duration_ms);
void Buzzer_SweepDown(uint32_t start_hz, uint32_t end_hz, uint32_t duration_ms);
void Buzzer_Siren(uint32_t low_hz, uint32_t high_hz, uint32_t duration_ms);

/* Notes (A4=440 Hz family; you can extend as needed) */
#define NOTE_C4   262
#define NOTE_CS4  277
#define NOTE_D4   294
#define NOTE_DS4  311
#define NOTE_E4   330
#define NOTE_F4   349
#define NOTE_FS4  370
#define NOTE_G4   392
#define NOTE_GS4  415
#define NOTE_A4   440
#define NOTE_AS4  466
#define NOTE_B4   494
#define NOTE_C5   523
#define NOTE_CS5  554
#define NOTE_D5   587
#define NOTE_DS5  622
#define NOTE_E5   659
#define NOTE_F5   698
#define NOTE_FS5  740
#define NOTE_G5   784
#define NOTE_GS5  831
#define NOTE_A5   880
#define NOTE_AS5  932
#define NOTE_B5   988
#define NOTE_C6   1047


/*******************************************************************************
 * Function: void Buzzer_Init(void)
 *
 * Returns: Nothing
 *
 * Description: Initialize the buzzer on PA1, TIM1_CH2 
 * 
 * Usage: Buzzer_Init()
 ******************************************************************************/
void Buzzer_Init(void)
{
    /* Clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO, ENABLE);

    /* PA1 = TIM1_CH2 as AF push-pull */
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = GPIO_Pin_1;             // PA1
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &g);

    /* Base timer */
    TIM_TimeBaseInitTypeDef tb = {0};
    tb.TIM_Period        = 1000 - 1;
    tb.TIM_Prescaler     = 0;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &tb);

    /* PWM on CH2 */
    TIM_OCInitTypeDef oc = {0};
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = 0;                 // start silent
    TIM_OC2Init(TIM1, &oc);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    /* This replaces the BDTR.MOE direct write */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);

    /* Ensure off */
    TIM1->CH2CVR = 0;
}


/*******************************************************************************
 * Function: void Buzzer_Set(uint32_t frequency_hz, uint8_t volume_pct)
 *
 * Returns: Nothing
 *
 * Description:  Set frequency & volume (integer math, no floats),
 *               Finds a prescaler so that ARR fits in 16 bits, then sets duty 
 *                from volume. Volume_pct: 0..100. For a passive piezo, 50% duty 
 *                is optimal; we scale 0..50%.
 * 
 * Usage: Buzzer_Set(2000, 40);
 ******************************************************************************/
void Buzzer_Set(uint32_t frequency_hz, uint8_t volume_pct)
{
    if (frequency_hz == 0 || volume_pct == 0){
        Buzzer_Stop();
        return;
    }
    if (volume_pct > 100) volume_pct = 100;

    uint32_t clk = SystemCoreClock ? SystemCoreClock : 8000000;

    // Find prescaler so ARR <= 65535
    uint32_t min_div = (clk + (frequency_hz * 65536u) - 1u) / (frequency_hz * 65536u);
    if (min_div == 0) min_div = 1;
    if (min_div > 65536u) min_div = 65536u;

    uint16_t psc = (uint16_t)(min_div - 1u);
    uint32_t tclk = clk / (psc + 1u);
    uint32_t arr  = (tclk / frequency_hz);
    if (arr == 0) arr = 1;
    arr -= 1u;
    if (arr > 0xFFFFu) arr = 0xFFFFu;

    // === important: use StdPeriph helpers so the reload happens immediately
    TIM_PrescalerConfig(TIM1, psc, TIM_PSCReloadMode_Immediate); // latch PSC now
    TIM_SetAutoreload(TIM1, (uint16_t)arr);                       // sets ATRLR
    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);              // force ARR/PSC load

    // Duty: cap at 50% to be piezo-friendly
    uint32_t period = arr + 1u;
    uint32_t duty   = (period * volume_pct) / 200u;  // 100% vol → 50% duty
    if (duty > arr) duty = arr;

    BUZZER_CH_SETCCR(TIM1, (uint16_t)duty);          // update CCR after ARR/PSC
}

/*******************************************************************************
 * Function: void Buzzer_Stop(void)
 *
 * Returns: Nothing
 *
 * Description:  Stop the buzzer 
 * 
 * Usage: Buzzer_Stop()
 ******************************************************************************/
void Buzzer_Stop(void)
{
    BUZZER_CH_SETCCR(BUZZER_TIMER, 0);
}

/////////////////////////////
// Helpers and patterns  
/////////////////////////////
/* Some common patterns I used in embedded systems over the years */

// Beep
void Buzzer_Beep(uint32_t frequency_hz, uint8_t volume_pct, uint32_t duration_ms)
{
    Buzzer_Set(frequency_hz, volume_pct);
    Delay_Ms(duration_ms);
    Buzzer_Stop();
}

// Success
void Buzzer_Success(void)
{
    Buzzer_Beep(NOTE_C5, 50, 90);  Delay_Ms(40);
    Buzzer_Beep(NOTE_E5, 50, 90);  Delay_Ms(40);
    Buzzer_Beep(NOTE_G5, 60, 160);
}

// Failure 
void Buzzer_Failure(void)
{
    Buzzer_Beep(NOTE_G5, 60, 160); Delay_Ms(40);
    Buzzer_Beep(NOTE_E5, 60, 220);
}

// Warning 
void Buzzer_Warning(void)
{
    for (int i=0;i<3;i++){ Buzzer_Beep(NOTE_A5, 60, 90); Delay_Ms(90); }
}

// Alert
void Buzzer_Alert(void)
{
    for (int i=0;i<5;i++){ Buzzer_Beep(NOTE_A5, 75, 50); Delay_Ms(50); }
}

// Menu 
void Buzzer_KeyPress(void){    Buzzer_Beep(NOTE_C5, 35, 40); }
void Buzzer_MenuSelect(void){  Buzzer_Beep(NOTE_E5, 40, 70); }
void Buzzer_MenuBack(void){    Buzzer_Beep(NOTE_C5, 40, 70); }

// Power Up
void Buzzer_PowerUp(void)
{
    Buzzer_Beep(NOTE_C5, 55, 80); Delay_Ms(40);
    Buzzer_Beep(NOTE_E5, 55, 80); Delay_Ms(40);
    Buzzer_Beep(NOTE_G5, 55, 80); Delay_Ms(40);
    Buzzer_Beep(NOTE_C6, 60, 150);
}

// Power Down 
void Buzzer_PowerDown(void)
{
    Buzzer_Beep(NOTE_C6, 55, 80); Delay_Ms(40);
    Buzzer_Beep(NOTE_G5, 55, 80); Delay_Ms(40);
    Buzzer_Beep(NOTE_E5, 55, 80); Delay_Ms(40);
    Buzzer_Beep(NOTE_C5, 60, 150);
}

// Critical Battery 
void Buzzer_CriticalBattery(void)
{
    for (int i=0;i<2;i++){ Buzzer_Beep(NOTE_G4, 75, 280); Delay_Ms(280); }
}

// Processing 
void Buzzer_Processing(void)
{
    for (int i=0;i<3;i++){ Buzzer_Beep(NOTE_E5, 35, 45); Delay_Ms(90); }
}

// Error 
void Buzzer_Error(void)
{
    for (int i=0;i<2;i++){ Buzzer_Beep(NOTE_B5, 75, 180); Delay_Ms(90); }
    Buzzer_Beep(NOTE_B5, 75, 360);
}

/////////////////////////////
// Effects (no trig) 
/////////////////////////////
/* Some more common embedded systems effects */ 

// Sweep up 
void Buzzer_SweepUp(uint32_t start_hz, uint32_t end_hz, uint32_t duration_ms)
{
    if (end_hz <= start_hz) return;
    uint32_t steps = duration_ms / 2; if (steps < 1) steps = 1;
    for (uint32_t k=0;k<steps;k++){
        uint32_t f = start_hz + (uint32_t)((uint64_t)(end_hz - start_hz) * k / steps);
        Buzzer_Set(f, 70);
        Delay_Ms(2);
    }
    Buzzer_Stop();
}

// Sweep down 
void Buzzer_SweepDown(uint32_t start_hz, uint32_t end_hz, uint32_t duration_ms)
{
    if (start_hz <= end_hz) return;
    uint32_t steps = duration_ms / 2; if (steps < 1) steps = 1;
    for (uint32_t k=0;k<steps;k++){
        uint32_t f = start_hz - (uint32_t)((uint64_t)(start_hz - end_hz) * k / steps);
        Buzzer_Set(f, 70);
        Delay_Ms(2);
    }
    Buzzer_Stop();
}


// Siren, a triangle wave between low_hz and high_hzm 5 ms tick 
void Buzzer_Siren(uint32_t low_hz, uint32_t high_hz, uint32_t duration_ms)
{
    if (high_hz <= low_hz) return;
    const uint32_t tick = 5;
    uint32_t t = 0;
    int dir = 1;              // 1 = up, -1 = down
    uint32_t f = low_hz;

    while (t < duration_ms){
        Buzzer_Set(f, 70);
        Delay_Ms(tick);
        t += tick;

        /* step size: sweep roughly 1 second per up/down by default */
        uint32_t step = (high_hz - low_hz) / (1000 / tick);
        if (step == 0) step = 1;

        if (dir > 0){
            if (f + step >= high_hz){ f = high_hz; dir = -1; }
            else { f += step; }
        } else {
            if (f <= low_hz + step){ f = low_hz; dir = 1; }
            else { f -= step; }
        }
    }
    Buzzer_Stop();
}

/////////////////////////////
// Military Applications 
/////////////////////////////

// Countdown beep effect with accelerating pace
void Buzzer_Countdown(uint32_t duration_ms) 
{
    uint32_t beep_freq = 2200;  // Beep frequency
    uint32_t initial_delay = 1000;  // Start with 1 second between beeps
    uint32_t elapsed = 0;
    
    while(elapsed < duration_ms) {
        // Make beep
        Buzzer_Set(beep_freq, 70);
        Delay_Ms(50);  // Short beep duration
        Buzzer_Stop();
        
        // Calculate next delay (exponentially decreasing)
        initial_delay = (initial_delay * 8) / 10;  // Reduce delay by 20% each time
        if(initial_delay < 50) initial_delay = 50;  // Don't go faster than 50ms
        
        Delay_Ms(initial_delay);
        elapsed += (50 + initial_delay);
    }
    
    // Final long beep
    Buzzer_Set(3000, 100);  // Higher frequency, full volume
    Delay_Ms(500);
    Buzzer_Stop();
}

// More intense countdown with pitch change
void Buzzer_CountdownIntense(uint32_t duration_ms) 
{
    uint32_t start_freq = 2200;
    uint32_t initial_delay = 1000;
    uint32_t elapsed = 0;
    
    while(elapsed < duration_ms) {
        // Make beep with increasing frequency
        Buzzer_Set(start_freq, 70);
        Delay_Ms(50);
        Buzzer_Stop();
        
        // Increase pitch slightly each time
        start_freq += 50;
        
        // Calculate next delay (exponentially decreasing)
        initial_delay = (initial_delay * 7) / 10;  // Reduce delay by 30% each time
        if(initial_delay < 30) initial_delay = 30;  // Don't go faster than 30ms
        
        Delay_Ms(initial_delay);
        elapsed += (50 + initial_delay);
    }
    
}

// Radar/Sonar Ping
void Buzzer_Ping(void) 
{
    Buzzer_SweepDown(4000, 1000, 100);
    Delay_Ms(50);
    Buzzer_SweepUp(1000, 4000, 50);
}



//////////////////////////////////
// Consumer Goods 
//////////////////////////////////

// Scanner/Processing effect (like a barcode reader)
void Buzzer_Scanner(void) 
{
    Buzzer_Set(2500, 70);
    Delay_Ms(80);
    Buzzer_Stop();
    Delay_Ms(20);
    Buzzer_Set(3000, 50);
    Delay_Ms(40);
    Buzzer_Stop();
}


// WiFi/Bluetooth Connected
void Buzzer_Connected(void) 
{
    Buzzer_Set(2000, 50);
    Delay_Ms(50);
    Buzzer_Stop();
    Delay_Ms(50);
    Buzzer_Set(2500, 50);
    Delay_Ms(100);
    Buzzer_Stop();
}

// WiFi/Bluetooth Disconnected
void Buzzer_Disconnected(void) 
{
    Buzzer_Set(2500, 50);
    Delay_Ms(50);
    Buzzer_Stop();
    Delay_Ms(50);
    Buzzer_Set(2000, 50);
    Delay_Ms(100);
    Buzzer_Stop();
}

// Charging Started
void Buzzer_ChargingStart(void) 
{
    Buzzer_SweepUp(1500, 3000, 200);
}

// Charging Complete
void Buzzer_ChargingComplete(void)
{
    for(int i = 0; i < 3; i++) {
        Buzzer_Set(3000, 60);
        Delay_Ms(100);
        Buzzer_Stop();
        Delay_Ms(100);
    }
}

// Reminder (gentler than alarm)
void Buzzer_Reminder(void) 
{
    for(int i = 0; i < 2; i++) {
        Buzzer_Set(2000, 40);
        Delay_Ms(100);
        Buzzer_Stop();
        Delay_Ms(300);
    }
}

// Timer Complete
void Buzzer_TimerComplete(void) 
{
    for(int i = 0; i < 3; i++) {
        Buzzer_Set(4000, 70);
        Delay_Ms(100);
        Buzzer_Stop();
        Delay_Ms(100);
        Buzzer_Set(3000, 70);
        Delay_Ms(100);
        Buzzer_Stop();
        Delay_Ms(300);
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
    SystemCoreClockUpdate();
    Delay_Init();

    // Init buzzer on PA1 (TIM1_CH2)
    Buzzer_Init();

    // Basic checks
    Buzzer_Beep(3000, 60, 200);
    Delay_Ms(200);
    Buzzer_Success();
    Delay_Ms(400);

    // Sweeps & siren
    Buzzer_SweepUp(1000, 6000, 1200);
    Delay_Ms(200);
    Buzzer_Siren(1500, 3000, 3000);

    Buzzer_Connected();
    Buzzer_Disconnected();

    while(1){
        Buzzer_Countdown(8000);
        Buzzer_Warning();    Delay_Ms(800);
    }
}
