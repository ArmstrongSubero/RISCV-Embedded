// DMA based sinewave with low pass filter
/*******************************************************************************
 * CH32V003F4P6 | TIM1_CH1 (PD2) + DMA | 32-sample SINE via PWM duty
 * - DMA: mem -> TIM1->CH1CVR, halfword, circular, MINC
 * - Trigger: TIM1 Update (UEV); RCR used to decimate update rate
 ******************************************************************************/
#include "debug.h"

/* ===== Config ===== */
#define SINE_SAMPLES 32
#define ARR_VALUE    832     // ~57.6 kHz PWM carrier @48MHz (PSC=0)
#define PSC_VALUE    0
#define RCR_VALUE    8       // Update every (RCR+1)=9 PWM periods -> ~6.4 kHz sample => ~200 Hz sine

/* ===== Base 12-bit sine (0..4095), 32 samples ===== */
static const uint16_t SINE_BASE12[SINE_SAMPLES] = {
    2048,2447,2831,3185,3495,3750,3939,4056,
    4095,4056,3939,3750,3495,3185,2831,2447,
    2048,1649,1265, 911, 601, 346, 157,  40,
       0,  40, 157, 346, 601, 911,1265,1649
};

/* Scaled/centered LUT (10..90% of ARR, avoids rail hits) */
static uint16_t SINE_WAVE[SINE_SAMPLES];

/* Build centered headroom LUT from base (integer, no libm)
   mid = ARR/2, amp = 0.8*(ARR/2)  -> range ≈ 10%..90% of ARR */
static void gen_sine_lut_scaled(uint16_t arr)
{
    uint32_t mid = arr / 2u;
    uint32_t amp = (arr / 2u) * 80u / 100u;      // 0.8 * (arr/2)
    uint32_t two_amp = amp * 2u;
    for (int i = 0; i < SINE_SAMPLES; i++) {
        uint32_t x = SINE_BASE12[i];             // 0..4095
        uint32_t offset = (two_amp * x + 2047u) / 4095u;  // ≈ 2*amp*(x/4095) with rounding
        int32_t  v = (int32_t)(mid - amp) + (int32_t)offset;
        if (v < 0) v = 0;
        if (v > (int32_t)arr) v = (int32_t)arr;
        SINE_WAVE[i] = (uint16_t)v;
    }
}

/* TIM1 CH1 compare register (WCH naming) */
#define TIM1_CH1CVR_ADDR ((uint32_t)&TIM1->CH1CVR)

/* ===== PD2 as TIM1_CH1 ===== */
static void GPIO_PD2_TIM1CH1_Init(void)
{
    GPIO_InitTypeDef io = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    io.GPIO_Pin   = GPIO_Pin_2;
    io.GPIO_Mode  = GPIO_Mode_AF_PP;
    io.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOD, &io);
}

/* ===== TIM1 base + PWM1 on CH1 (with RCR decimation) ===== */
static void TIM1_PWM_Init(uint16_t arr, uint16_t psc, uint8_t rcr)
{
    TIM_TimeBaseInitTypeDef tb = {0};
    TIM_OCInitTypeDef       oc = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    tb.TIM_Period            = arr;
    tb.TIM_Prescaler         = psc;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = rcr;          // decimate UEV -> DMA pacing
    TIM_TimeBaseInit(TIM1, &tb);

    oc.TIM_OCMode      = TIM_OCMode_PWM1;    // duty = CH1CVR / ARR
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 0;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &oc);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); // latch at update
    TIM_ARRPreloadConfig(TIM1, ENABLE);
}

/* ===== DMA: mem -> TIM1->CH1CVR, circular ===== */
static void TIM1_DMA_Sine_Init(void)
{
    DMA_InitTypeDef d = {0};
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel5);                       // TIM1_UP -> CH5 on CH32V003

    d.DMA_PeripheralBaseAddr = TIM1_CH1CVR_ADDR;     // dest
    d.DMA_MemoryBaseAddr     = (uint32_t)SINE_WAVE;  // src (scaled/centered)
    d.DMA_DIR                = DMA_DIR_PeripheralDST;
    d.DMA_BufferSize         = SINE_SAMPLES;
    d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    d.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    d.DMA_Mode               = DMA_Mode_Circular;
    d.DMA_Priority           = DMA_Priority_VeryHigh;
    d.DMA_M2M                = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel5, &d);
    DMA_Cmd(DMA1_Channel5, ENABLE);

    TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);        // one transfer per UEV
}

int main(void)
{
    SystemCoreClockUpdate();   // 48 MHz expected
    Delay_Init();
#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif
    printf("\r\nCH32V003 TIM1+DMA sine on PD2 (centered 10..90%%)\r\n");
    printf("SystemClk:%ld\r\n", SystemCoreClock);

    gen_sine_lut_scaled(ARR_VALUE);     // build headroom LUT

    GPIO_PD2_TIM1CH1_Init();
    TIM1_PWM_Init(ARR_VALUE, PSC_VALUE, RCR_VALUE);
    TIM1_DMA_Sine_Init();

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);   // MOE for advanced timer

    while (1) { /* DMA + TIM do the work */ }
}
