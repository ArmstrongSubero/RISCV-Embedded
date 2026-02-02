/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 19_RGB_LED
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to use 
 *                      the RGB LED. 
 *                      
 * Hardware Description: An RGB LED is connected as follows: 
 *                       R -> PD2 
 *                       G -> PA1 
 *                       B -> PC3
 *           
 * Created August 14th, 2025, 9:35 PM
 * Updated August 14th, 2025, 9:35 PM
 */
 
/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "ch32v00x.h"
#include <stdint.h>
#include <stdbool.h>
#include "debug.h"       

#define RGB_R_PORT   GPIOD
#define RGB_R_PIN    GPIO_Pin_2   // PD2 -> TIM1_CH1
#define RGB_G_PORT   GPIOA
#define RGB_G_PIN    GPIO_Pin_1   // PA1 -> TIM1_CH2
#define RGB_B_PORT   GPIOC
#define RGB_B_PIN    GPIO_Pin_3   // PC3 -> TIM1_CH3

typedef struct { uint8_t r,g,b; } RGB_Color;

void RGB_PWM_Init(uint16_t arr /*e.g.,255*/, uint16_t psc /*e.g.,250-1*/);
void RGB_Init(bool common_anode, bool enable_gamma);

void RGB_Set(uint8_t r, uint8_t g, uint8_t b);
void RGB_SetColor(RGB_Color c);
void RGB_Off(void);
void RGB_White(void);

/* Effects */
void RGB_Fade(RGB_Color from, RGB_Color to, uint32_t duration_ms, uint16_t steps);
void RGB_Rainbow(uint32_t duration_ms, uint16_t steps_per_edge);
void RGB_Pulse(RGB_Color color, uint32_t duration_ms, uint16_t steps);
void RGB_Breathe(RGB_Color color, uint32_t duration_ms, uint16_t steps);

/* Handy colors */
extern const RGB_Color RGB_RED, RGB_GREEN, RGB_BLUE, RGB_YELLOW, RGB_CYAN,
                       RGB_MAGENTA, RGB_WHITE_C, RGB_PURPLE, RGB_ORANGE, RGB_PINK;


/* ===== User opts ===== */
static uint8_t g_invert = 0;  // 0=common cathode, 1=common anode (invert duty)
static uint8_t g_gamma  = 0;

/* ===== Small gamma LUT (approx) ===== */
static uint8_t gamma8[256];
static void build_gamma(void){
    for (int i=0;i<256;i++){
        uint32_t x=i;
        uint32_t y = x*x*(255+x);   // quick monotonic curve ~x^2.2
        y = (y + (1u<<15)) >> 16;
        if (y>255) y=255;
        gamma8[i]=(uint8_t)y;
    }
}
static inline uint8_t gfix(uint8_t v){ return g_gamma ? gamma8[v] : v; }
static inline uint8_t inv(uint8_t v){ return g_invert ? (uint8_t)(255 - v) : v; }


/*******************************************************************************
 * Function: static void SPI1_Init_Master(void)
 *
 * Returns: Nothing
 *
 * Description: TIM1 PWM init on PD2/PA1/PC3
 * 
 * Usage: RGB_PWM_Init(arr, psc)
 ******************************************************************************/
void RGB_PWM_Init(uint16_t arr, uint16_t psc){
    GPIO_InitTypeDef g = {0};
    TIM_TimeBaseInitTypeDef tb = {0};
    TIM_OCInitTypeDef oc = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_TIM1  |
                           RCC_APB2Periph_AFIO, ENABLE);

    /* AF push-pull on selected pins */
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_10MHz;

    g.GPIO_Pin = RGB_R_PIN; GPIO_Init(RGB_R_PORT, &g); // CH1
    g.GPIO_Pin = RGB_G_PIN; GPIO_Init(RGB_G_PORT, &g); // CH2
    g.GPIO_Pin = RGB_B_PIN; GPIO_Init(RGB_B_PORT, &g); // CH3

    /* Base: ARR/PSC */
    tb.TIM_Period        = arr;
    tb.TIM_Prescaler     = psc;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1, &tb);

    /* PWM1, enable CH1..CH3 */
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = 0;

    TIM_OC1Init(TIM1, &oc); TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_OC2Init(TIM1, &oc); TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_OC3Init(TIM1, &oc); TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Disable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    /* start OFF */
    TIM_SetCompare1(TIM1, 0);
    TIM_SetCompare2(TIM1, 0);
    TIM_SetCompare3(TIM1, 0);
}


/*******************************************************************************
 * Function: void RGB_Init(bool common_anode, bool enable_gamma)
 *
 * Returns: Nothing
 *
 * Description: Initialize the RGB LED 
 * 
 * Usage: RGB_Init(common_anode, enable_gamma)
 ******************************************************************************/
void RGB_Init(bool common_anode, bool enable_gamma)
{
    g_invert = common_anode ? 1 : 0;
    g_gamma  = enable_gamma ? 1 : 0;
    if (g_gamma) build_gamma();
    RGB_Off();
}


/*******************************************************************************
 * Function: static inline void write_rgb(uint8_t r, uint8_t g, uint8_t b)
 *
 * Returns: Nothing
 *
 * Description: Write RGB values 
 * 
 * Usage: write_rgb(r,g,b)
 ******************************************************************************/
static inline void write_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    TIM_SetCompare1(TIM1, r);
    TIM_SetCompare2(TIM1, g);
    TIM_SetCompare3(TIM1, b);
}

/*******************************************************************************
 * Function: void RGB_Set(uint8_t r, uint8_t g, uint8_t b)
 *
 * Returns: Nothing
 *
 * Description:  Set RGB colors, intended to the the Public API 
 * 
 * Usage: RGB_Set(r,g,b)
 ******************************************************************************/
void RGB_Set(uint8_t r, uint8_t g, uint8_t b)
{
    r = inv(gfix(r));
    g = inv(gfix(g));
    b = inv(gfix(b));
    write_rgb(r,g,b);
}

// Inline functions for RGB applications 
void RGB_SetColor(RGB_Color c){ RGB_Set(c.r,c.g,c.b); }
void RGB_Off(void){ RGB_Set(0,0,0); }
void RGB_White(void){ RGB_Set(255,255,255); }


/*******************************************************************************
 * Function: static inline uint16_t scurve(uint16_t t)
 *
 * Returns: Nothing
 *
 * Description:  Easing (quintic S-curve in Q15)
 * 
 * Usage: q15mul and scurve work together for S-cure easing in Q15 
 ******************************************************************************/
#define Q15_ONE 32768u
static inline uint32_t q15mul(uint32_t a, uint32_t b){ return (uint32_t)(((uint64_t)a*b + (1u<<14))>>15); }
static inline uint16_t scurve(uint16_t t)
{
    uint32_t t2=q15mul(t,t), t3=q15mul(t2,t), t4=q15mul(t3,t), t5=q15mul(t4,t);
    int64_t s = 6*(int64_t)t5 - 15*(int64_t)t4 + 10*(int64_t)t3;
    if (s<0) s=0; if (s>Q15_ONE) s=Q15_ONE; return (uint16_t)s;
}


/*******************************************************************************
 * Function: void RGB_Set(uint8_t r, uint8_t g, uint8_t b)
 *
 * Returns: Nothing
 *
 * Description: Function for RGB effects, fading effects (blocking) 
 * 
 * Usage: RGB_Fade((RGB_Color){255,0,0}, (RGB_Color){0,0,255}, 1500, 60);
 ******************************************************************************/
void RGB_Fade(RGB_Color from, RGB_Color to, uint32_t duration_ms, uint16_t steps)
{
    if (!steps) steps=1;
    uint32_t tick = duration_ms/steps; if (!tick) tick=1;
    int16_t dr = (int16_t)to.r - (int16_t)from.r;
    int16_t dg = (int16_t)to.g - (int16_t)from.g;
    int16_t db = (int16_t)to.b - (int16_t)from.b;

    for (uint16_t k=0;k<=steps;k++){
        uint16_t t = (uint16_t)((uint32_t)Q15_ONE * k / steps);
        uint16_t s = scurve(t);              // smooth; for linear use: uint16_t s=t;
        uint8_t r = (uint8_t)(from.r + ((int32_t)dr * s >> 15));
        uint8_t g = (uint8_t)(from.g + ((int32_t)dg * s >> 15));
        uint8_t b = (uint8_t)(from.b + ((int32_t)db * s >> 15));
        RGB_Set(r,g,b);
        Delay_Ms(tick);
    }
    RGB_Set(to.r,to.g,to.b);
}


/*******************************************************************************
 * Function: static inline void fade_edge(RGB_Color a, RGB_Color b, uint32_t dur,
 *                                        uint16_t steps)
 *
 * Returns: Nothing
 *
 * Description: Fade Edge effect alternative call for RGB_Fade for API calls  
 * 
 * Usage: fade_edge((RGB_Color){255,  0,  0}, (RGB_Color){255,255,  0}, seg,
 *                   steps_per_edge)
 ******************************************************************************/
static inline void fade_edge(RGB_Color a, RGB_Color b, uint32_t dur, uint16_t steps)
{
    RGB_Fade(a,b,dur,steps);
}

/*******************************************************************************
 * Function: void RGB_Rainbow(uint32_t duration_ms, uint16_t steps_per_edge)
 *
 * Returns: Nothing
 *
 * Description:  RGB effect, intended to show rainbow effect 
 * 
 * Usage: RGB_Rainbow(4000, 48)
 ******************************************************************************/
void RGB_Rainbow(uint32_t duration_ms, uint16_t steps_per_edge)
{
    uint32_t seg = duration_ms/6u;
    fade_edge((RGB_Color){255,  0,  0}, (RGB_Color){255,255,  0}, seg, steps_per_edge); // R->Y
    fade_edge((RGB_Color){255,255,  0}, (RGB_Color){  0,255,  0}, seg, steps_per_edge); // Y->G
    fade_edge((RGB_Color){  0,255,  0}, (RGB_Color){  0,255,255}, seg, steps_per_edge); // G->C
    fade_edge((RGB_Color){  0,255,255}, (RGB_Color){  0,  0,255}, seg, steps_per_edge); // C->B
    fade_edge((RGB_Color){  0,  0,255}, (RGB_Color){255,  0,255}, seg, steps_per_edge); // B->M
    fade_edge((RGB_Color){255,  0,255}, (RGB_Color){255,  0,  0}, seg, steps_per_edge); // M->R
}

/*******************************************************************************
 * Function: void RGB_Pulse(RGB_Color c, uint32_t duration_ms, uint16_t steps)
 *
 * Returns: Nothing 
 *
 * Description:  RGB effect, intended to show pulse effect 
 * 
 * Usage: RGB_Pulse(RGB_ORANGE, 1200, 64)
 ******************************************************************************/
void RGB_Pulse(RGB_Color c, uint32_t duration_ms, uint16_t steps)
{
    if (steps<2) steps=2;
    RGB_Fade((RGB_Color){0,0,0}, c, duration_ms/2, steps/2);
    RGB_Fade(c, (RGB_Color){0,0,0}, duration_ms/2, steps/2);
}

/*******************************************************************************
 * Function: void RGB_Breathe(RGB_Color c, uint32_t duration_ms, uint16_t steps)
 *
 * Returns: Nothing 
 *
 * Description: RGB effect, intended to show breathing effect 
 * 
 * Usage: RGB_Breathe(RGB_MAGENTA,  3000, 120)
 ******************************************************************************/
void RGB_Breathe(RGB_Color c, uint32_t duration_ms, uint16_t steps)
{
    if (steps<2) steps=2;
    uint32_t tick = duration_ms/steps; if (!tick) tick=1;
    for (uint16_t k=0;k<steps;k++){
        /* 0..1..0 using mirrored S-curve */
        uint16_t t = (uint16_t)((uint32_t)Q15_ONE * (k <= steps/2 ? (2*k) : (2*(steps-1-k))) / (steps-1));
        uint16_t s = scurve(t);
        uint8_t r = (uint8_t)((c.r * (uint32_t)s) >> 15);
        uint8_t g = (uint8_t)((c.g * (uint32_t)s) >> 15);
        uint8_t b = (uint8_t)((c.b * (uint32_t)s) >> 15);
        RGB_Set(r,g,b);
        Delay_Ms(tick);
    }
}

/* Colors */
const RGB_Color RGB_RED     = {255,  0,  0};
const RGB_Color RGB_GREEN   = {  0,255,  0};
const RGB_Color RGB_BLUE    = {  0,  0,255};
const RGB_Color RGB_YELLOW  = {255,255,  0};
const RGB_Color RGB_CYAN    = {  0,255,255};
const RGB_Color RGB_MAGENTA = {255,  0,255};
const RGB_Color RGB_WHITE_C = {255,255,255};
const RGB_Color RGB_PURPLE  = {128,  0,128};
const RGB_Color RGB_ORANGE  = {255,165,  0};
const RGB_Color RGB_PINK    = {255,192,203};


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

    /* 8 MHz / 250 / 256 = 125 Hz. Use arr=255 for 8-bit duty. */
    RGB_PWM_Init(/*arr=*/255, /*psc=*/250-1);

    /* Set common_anode=true if LED is CA; enable_gamma=true for nice look */
    RGB_Init(/*common_anode=*/false, /*enable_gamma=*/true);

    while(1){
		// Set some colors
        RGB_SetColor(RGB_RED);     Delay_Ms(400);
        RGB_SetColor(RGB_GREEN);   Delay_Ms(400);
        RGB_SetColor(RGB_BLUE);    Delay_Ms(400);
        RGB_White();               Delay_Ms(600);
        RGB_Off();                 Delay_Ms(200);

        // RGB fade effect 
        RGB_Fade((RGB_Color){255,0,0}, (RGB_Color){0,0,255}, 1500, 60);
        Delay_Ms(150);

        // RGB rainbow effect 
        RGB_Rainbow(4000, 48);
		
		// RGB pulse effect
        RGB_Pulse(RGB_ORANGE, 1200, 64);
    
	    // RGB breathe effect 
        RGB_Breathe(RGB_MAGENTA,  3000, 120);
    }
}
