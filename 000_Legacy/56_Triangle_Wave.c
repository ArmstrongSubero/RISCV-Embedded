/*******************************************************************************
 * CH32V003F4P6  |  TIM1_CH1 (PD2) + DMA  |  32-sample TRIANGLE via PWM duty
 * - DMA: memory -> peripheral (TIM1->CH1CVR), halfword, circular, MINC
 * - Trigger: TIM1 Update DMA request (UEV)
 ******************************************************************************/
#include "debug.h"

/* ===== Config ===== */
#define TRI_SAMPLES 32      // table length
#define ARR_VALUE   832     // ~57.6 kHz carrier at 48 MHz, PSC=0
#define PSC_VALUE   0
#define RCR_VALUE   8       // update/DMA every (RCR+1)=9 PWM periods

/* ===== Triangle LUT (centered with headroom, no libm) =====
   - Range: ~10%..90% of ARR (amp = 0.8 * ARR/2), centered at ARR/2
   - Avoids hitting 0 or ARR -> prevents analog clipping at RC output
*/
static uint16_t TRIANGLE_WAVE[TRI_SAMPLES];

static void gen_triangle_lut(uint16_t arr)
{
    uint32_t mid = arr / 2u;
    uint32_t amp = (arr / 2u) * 80u / 100u;   // 0.8 * (arr/2) -> 10..90%

    for (int i = 0; i < TRI_SAMPLES; i++) {
        /* piecewise linear triangle using integers
           frac in [0..1] with 16 steps up and 16 down (no rails) */
        uint32_t frac_num = (i <= 15) ? (uint32_t)i : (uint32_t)(31 - i);
        uint32_t frac_den = 15u;                       // reaches 1.0 at i=15
        uint32_t two_amp  = amp * 2u;
        uint32_t offset   = (two_amp * frac_num + (frac_den/2)) / frac_den; // ~2*amp*frac

        int32_t val = (int32_t)(mid - amp) + (int32_t)offset;
        if (val < 0) val = 0;
        if (val > (int32_t)arr) val = (int32_t)arr;
        TRIANGLE_WAVE[i] = (uint16_t)val;
    }
}

/* TIM1 CH1 compare register (WCH naming) */
#define TIM1_CH1CVR_ADDR ((uint32_t)&TIM1->CH1CVR)

/* ===== PD2 as TIM1_CH1 ===== */
static void GPIO_PD2_TIM1CH1_Init(void)
{
    GPIO_InitTypeDef io = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    io.GPIO_Pin   = GPIO_Pin_2;          // PD2
    io.GPIO_Mode  = GPIO_Mode_AF_PP;     // AF push-pull
    io.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOD, &io);
}

/* ===== TIM1 base + PWM1 on CH1 =====
   Carrier/update: f = 48e6 / ((PSC+1)*(ARR+1))  ≈ 57.6 kHz
   Effective sample rate (DMA/Update): carrier / (RCR+1) ≈ 6.4 kHz
   Triangle freq: sample / 32 ≈ 200 Hz
*/
static void TIM1_PWM_Init(uint16_t arr, uint16_t psc, uint8_t rcr)
{
    TIM_TimeBaseInitTypeDef tb = {0};
    TIM_OCInitTypeDef       oc = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    tb.TIM_Period            = arr;
    tb.TIM_Prescaler         = psc;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = rcr;      // decimate updates to slow the LUT stepping
    TIM_TimeBaseInit(TIM1, &tb);

    oc.TIM_OCMode      = TIM_OCMode_PWM1;          // duty = CH1CVR / ARR
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 0;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &oc);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); // latch at update
    TIM_ARRPreloadConfig(TIM1, ENABLE);
}

/* ===== DMA: mem -> TIM1->CH1CVR, circular ===== */
static void TIM1_DMA_Triangle_Init(void)
{
    DMA_InitTypeDef d = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel5);                       // TIM1_UP -> CH5 on CH32V003

    d.DMA_PeripheralBaseAddr = TIM1_CH1CVR_ADDR;     // dest: TIM1->CH1CVR (16-bit)
    d.DMA_MemoryBaseAddr     = (uint32_t)TRIANGLE_WAVE; // src: triangle LUT
    d.DMA_DIR                = DMA_DIR_PeripheralDST;   // mem -> periph
    d.DMA_BufferSize         = TRI_SAMPLES;             // 32
    d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc          = DMA_MemoryInc_Enable;    // walk LUT
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    d.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    d.DMA_Mode               = DMA_Mode_Circular;       // loop forever
    d.DMA_Priority           = DMA_Priority_VeryHigh;
    d.DMA_M2M                = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel5, &d);
    DMA_Cmd(DMA1_Channel5, ENABLE);

    // One DMA transfer per Update event (after RCR decimation)
    TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);
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
    printf("\r\nCH32V003 TIM1+DMA triangle on PD2 (centered 10..90%%)\r\n");
    printf("SystemClk:%ld\r\n", SystemCoreClock);

    gen_triangle_lut(ARR_VALUE);           // build headroom-centered LUT

    GPIO_PD2_TIM1CH1_Init();
    TIM1_PWM_Init(ARR_VALUE, PSC_VALUE, RCR_VALUE);
    TIM1_DMA_Triangle_Init();

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);      // MOE for advanced timer

    while (1) { /* DMA + TIM do all the work */ }
}
