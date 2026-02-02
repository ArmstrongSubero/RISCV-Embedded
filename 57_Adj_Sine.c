/*******************************************************************************
 * CH32V003F4P6 | TIM1_CH1 (PD2) + DMA | Adjustable 32-sample SINE (PWM + RC)
 * - DMA: mem -> TIM1->CH1CVR (halfword, circular, MINC)
 * - Trigger: TIM1 Update (UEV). Frequency via TIM1 repetition (through SPL).
 ******************************************************************************/
#include "debug.h"

/* ===== User config ===== */
#define SINE_SAMPLES   32
#define ARR_VALUE      479     // ~100 kHz carrier @48MHz with PSC=0
#define PSC_VALUE      0
#define RCR_INIT       8       // sample ≈ 100k/9 ≈ 11.11 kHz -> ~347 Hz sine

/* ===== Base 12-bit sine (0..4095), 32 samples ===== */
static const uint16_t SINE_BASE12[SINE_SAMPLES] = {
    2048,2447,2831,3185,3495,3750,3939,4056,
    4095,4056,3939,3750,3495,3185,2831,2447,
    2048,1649,1265, 911, 601, 346, 157,  40,
       0,  40, 157, 346, 601, 911,1265,1649
};

/* ===== Working buffers (double-buffer for glitch-free swaps) ===== */
static uint16_t SINE_BUF_A[SINE_SAMPLES];
static uint16_t SINE_BUF_B[SINE_SAMPLES];
static uint16_t *volatile active_buf = SINE_BUF_A;
static uint16_t *volatile shadow_buf = SINE_BUF_B;

/* Current shaping params */
static uint16_t g_arr = ARR_VALUE;
static uint16_t g_psc = PSC_VALUE;
static uint8_t  g_amp_pp_pct = 80;   // peak-to-peak % of full-scale (10..90 OK)
static int8_t   g_dc_pct  = 0;       // DC offset as % of half-scale (-40..+40)

/* TIM1 CH1 compare register (WCH naming) */
#define TIM1_CH1CVR_ADDR ((uint32_t)&TIM1->CH1CVR)

/* ================= Helpers: LUT build & DMA swap ================= */
static void build_sine_into(uint16_t *dst, uint16_t arr, uint8_t amp_pp_pct, int8_t dc_pct)
{
    if (amp_pp_pct > 100) amp_pp_pct = 100;
    if (amp_pp_pct <   1) amp_pp_pct = 1;
    int32_t mid = (int32_t)arr / 2;
    int32_t amp = ((int32_t)arr * amp_pp_pct) / 200;     // half of pp%
    mid += ((int32_t)arr * dc_pct) / 200;                // dc shift
    int32_t two_amp = amp * 2;

    for (int i = 0; i < SINE_SAMPLES; i++) {
        int32_t off = (two_amp * (int32_t)SINE_BASE12[i] + 2047) / 4095;
        int32_t v   = (mid - amp) + off;
        if (v < 0) v = 0;
        if (v > (int32_t)arr) v = arr;
        dst[i] = (uint16_t)v;
    }
}

/* Swap DMA source buffer (CH32V003: MADDR/CNTR, not CMAR/CNDTR) */
static void dma_swap_buffer(uint16_t *newbuf)
{
    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA1_Channel5->MADDR = (uint32_t)newbuf;  // Memory address
    DMA1_Channel5->CNTR  = SINE_SAMPLES;      // Reload count
    DMA_Cmd(DMA1_Channel5, ENABLE);

    active_buf = newbuf;
    shadow_buf = (newbuf == SINE_BUF_A) ? SINE_BUF_B : SINE_BUF_A;
}

/* Rebuild shadow from current params, then swap it in */
static void sine_rebuild_and_swap(void)
{
    build_sine_into(shadow_buf, g_arr, g_amp_pp_pct, g_dc_pct);
    dma_swap_buffer(shadow_buf);
}

/* ================= GPIO/TIM/DMA init ================= */
static void GPIO_PD2_TIM1CH1_Init(void)
{
    GPIO_InitTypeDef io = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    io.GPIO_Pin   = GPIO_Pin_2;          // PD2
    io.GPIO_Mode  = GPIO_Mode_AF_PP;
    io.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOD, &io);
}

static void TIM1_PWM_Init(uint16_t arr, uint16_t psc, uint8_t rcr)
{
    TIM_TimeBaseInitTypeDef tb = {0};
    TIM_OCInitTypeDef       oc = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    tb.TIM_Period            = arr;
    tb.TIM_Prescaler         = psc;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = rcr;          // SPL writes correct reg for WCH
    TIM_TimeBaseInit(TIM1, &tb);

    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 0;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &oc);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
}

/* TIM1 Update -> DMA1 Channel 5 on CH32V003 */
static void TIM1_DMA_Sine_Init(uint16_t *buf)
{
    DMA_InitTypeDef d = {0};
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel5);

    d.DMA_PeripheralBaseAddr = TIM1_CH1CVR_ADDR;     // dest: TIM1->CH1CVR
    d.DMA_MemoryBaseAddr     = (uint32_t)buf;        // src: active buffer
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

/* ================= Runtime controls ================= */
static void tim1_apply_rcr(uint8_t rcr)  // change repetition safely via SPL
{
    TIM_TimeBaseInitTypeDef tb = {0};
    tb.TIM_Period            = g_arr;
    tb.TIM_Prescaler         = g_psc;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = rcr;
    TIM_TimeBaseInit(TIM1, &tb);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
}

/* Set sine frequency in Hz (approx). Keeps carrier fixed; adjusts repetition. */
static void sine_set_freq_hz(uint32_t hz)
{
    if (hz == 0) hz = 1;
    uint32_t f_carrier = SystemCoreClock / ((g_psc + 1u) * (g_arr + 1u)); // ~100 kHz
    uint32_t rcr_plus1 = (f_carrier + (hz * SINE_SAMPLES)/2) / (hz * SINE_SAMPLES);
    if (rcr_plus1 < 1)   rcr_plus1 = 1;
    if (rcr_plus1 > 256) rcr_plus1 = 256;   // RCR range via SPL
    tim1_apply_rcr((uint8_t)(rcr_plus1 - 1));
}

/* Set peak-to-peak amplitude as % of full-scale (1..100). */
static void sine_set_amp_pct(uint8_t pp_pct)
{
    if (pp_pct < 1)   pp_pct = 1;
    if (pp_pct > 100) pp_pct = 100;
    g_amp_pp_pct = pp_pct;
    sine_rebuild_and_swap();
}

/* Set DC offset as % of half-scale (-100..+100). */
static void sine_set_dc_pct(int8_t dc_pct)
{
    if (dc_pct < -100) dc_pct = -100;
    if (dc_pct >  100) dc_pct =  100;
    g_dc_pct = dc_pct;
    sine_rebuild_and_swap();
}

/* ================= Main ================= */
int main(void)
{
    SystemCoreClockUpdate();   // 48 MHz
    Delay_Init();
#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif

    printf("\r\nCH32V003 TIM1+DMA adjustable sine on PD2\r\n");
    printf("SystemClk:%ld\r\n", SystemCoreClock);

    g_arr = ARR_VALUE; g_psc = PSC_VALUE;
    build_sine_into(SINE_BUF_A, g_arr, g_amp_pp_pct, g_dc_pct);
    build_sine_into(SINE_BUF_B, g_arr, g_amp_pp_pct, g_dc_pct);
    active_buf = SINE_BUF_A; shadow_buf = SINE_BUF_B;

    GPIO_PD2_TIM1CH1_Init();
    TIM1_PWM_Init(g_arr, g_psc, RCR_INIT);
    TIM1_DMA_Sine_Init(active_buf);

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    /* Demo knobs (adjust as you like) */
    sine_set_freq_hz(1000);    // ~60 Hz - 1 kHz sine
    sine_set_amp_pct(80);     // 10..90% range rec. keep >80
    sine_set_dc_pct(0);       // centered

    while (1) {
        // e.g.:
        // sine_set_freq_hz(1000);
        // sine_set_amp_pct(60);
        // sine_set_dc_pct(+10);
        Delay_Ms(1000);
    }
}
