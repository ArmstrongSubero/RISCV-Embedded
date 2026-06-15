/*******************************************************************************
 * CH32V003F4P6 | TIM1_CH1 (PD2) + DMA | Function Generator (SINE / TRI / SQR)
 * - DMA: mem -> TIM1->CH1CVR (halfword, circular, MINC)
 * - Trigger: TIM1 Update (UEV). Frequency via TIM1 repetition (through SPL).
 * - Adjustable: frequency, amplitude (pp%), DC offset, square duty.
 ******************************************************************************/
#include "debug.h"

/* ====================== User Config ====================== */
#define WF_SAMPLES     32               // table length (all waveforms share this)
#define ARR_VALUE      479              // ~100 kHz PWM carrier @48 MHz (PSC=0)
#define PSC_VALUE      0
#define RCR_INIT       8                // sample ≈ 100k/9 ≈ 11.11 kHz -> ~347 Hz base
                                        // f_out = 48e6 / ((PSC+1)*(ARR+1)*(RCR+1)*WF_SAMPLES)

/* ====================== Waveform Select ====================== */
typedef enum {
    WAVE_SINE = 0,
    WAVE_TRI  = 1,
    WAVE_SQR  = 2,
} wave_t;

static wave_t  g_wave       = WAVE_SINE;
static uint8_t g_sqr_duty   = 50;       // % (1..99)

/* ====================== Shaping Params ====================== */
static uint16_t g_arr       = ARR_VALUE;
static uint16_t g_psc       = PSC_VALUE;
static uint8_t  g_amp_pp_pct= 80;       // peak-to-peak % of full-scale (recommend 10..90)
static int8_t   g_dc_pct    = 0;        // DC offset as % of half-scale (-40..+40 typical)

/* ====================== Buffers (double-buffered) ====================== */
static uint16_t BUF_A[WF_SAMPLES];
static uint16_t BUF_B[WF_SAMPLES];
static uint16_t *volatile active_buf = BUF_A;
static uint16_t *volatile shadow_buf = BUF_B;

/* ====================== Base Sine (12-bit, 32 samples) ====================== */
static const uint16_t SINE_BASE12[WF_SAMPLES] = {
    2048,2447,2831,3185,3495,3750,3939,4056,
    4095,4056,3939,3750,3495,3185,2831,2447,
    2048,1649,1265, 911, 601, 346, 157,  40,
       0,  40, 157, 346, 601, 911,1265,1649
};

/* ====================== TIM/DMA Addresses ====================== */
#define TIM1_CH1CVR_ADDR ((uint32_t)&TIM1->CH1CVR)

/* ====================== Helpers: math & LUT builders (integer) ====================== */

static inline void clamp_u16_inplace(int32_t *v, uint16_t maxv) {
    if (*v < 0) *v = 0;
    if (*v > (int32_t)maxv) *v = (int32_t)maxv;
}

/* Compute mid/amp in timer counts from settings */
static void compute_mid_amp(uint16_t arr, uint8_t pp_pct, int8_t dc_pct, int32_t *mid, int32_t *amp)
{
    if (pp_pct < 1)   pp_pct = 1;
    if (pp_pct > 100) pp_pct = 100;
    *mid = (int32_t)arr / 2;
    *amp = ((int32_t)arr * pp_pct) / 200;    // half of pp%
    *mid += ((int32_t)arr * dc_pct) / 200;   // dc shift (±half-scale%)
}

/* Build SINE into dst (centered & headroom via pp% and dc%) */
static void build_sine(uint16_t *dst, uint16_t arr, uint8_t pp_pct, int8_t dc_pct)
{
    int32_t mid, amp; compute_mid_amp(arr, pp_pct, dc_pct, &mid, &amp);
    int32_t two_amp = 2 * amp;
    for (int i = 0; i < WF_SAMPLES; i++) {
        int32_t off = (two_amp * (int32_t)SINE_BASE12[i] + 2047) / 4095; // scale 0..4095
        int32_t v   = (mid - amp) + off;
        clamp_u16_inplace(&v, arr);
        dst[i] = (uint16_t)v;
    }
}

/* Build TRIANGLE into dst (linear up then down, same headroom model) */
static void build_triangle(uint16_t *dst, uint16_t arr, uint8_t pp_pct, int8_t dc_pct)
{
    int32_t mid, amp; compute_mid_amp(arr, pp_pct, dc_pct, &mid, &amp);
    // Piecewise linear: up over N/2-1 steps, down over N/2 steps (no rail hits)
    const uint32_t up_den = (WF_SAMPLES/2 - 1);  // 15 for 32 samples
    for (int i = 0; i < WF_SAMPLES; i++) {
        uint32_t num = (i <= (WF_SAMPLES/2 - 1)) ? (uint32_t)i : (uint32_t)(WF_SAMPLES - 1 - i);
        // offset ~ 2*amp * (num/up_den)
        int32_t off = (int32_t)((2u*amp * num + (up_den/2)) / up_den);
        int32_t v   = (mid - amp) + off;
        clamp_u16_inplace(&v, arr);
        dst[i] = (uint16_t)v;
    }
}

/* Build SQUARE into dst (two levels around mid; duty=1..99%) */
static void build_square(uint16_t *dst, uint16_t arr, uint8_t pp_pct, int8_t dc_pct, uint8_t duty_pct)
{
    if (duty_pct < 1)  duty_pct = 1;
    if (duty_pct > 99) duty_pct = 99;
    int32_t mid, amp; compute_mid_amp(arr, pp_pct, dc_pct, &mid, &amp);
    int32_t v_lo = mid - amp;
    int32_t v_hi = mid + amp;
    clamp_u16_inplace(&v_lo, arr);
    clamp_u16_inplace(&v_hi, arr);

    uint32_t hi_count = (WF_SAMPLES * (uint32_t)duty_pct + 50) / 100; // rounded
    for (uint32_t i = 0; i < WF_SAMPLES; i++) {
        dst[i] = (i < hi_count) ? (uint16_t)v_hi : (uint16_t)v_lo;
    }
}

/* ====================== DMA buffer swap (CH32 regs) ====================== */
static void dma_swap_buffer(uint16_t *newbuf)
{
    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA1_Channel5->MADDR = (uint32_t)newbuf;   // memory source
    DMA1_Channel5->CNTR  = WF_SAMPLES;         // reload circular count
    DMA_Cmd(DMA1_Channel5, ENABLE);

    active_buf = newbuf;
    shadow_buf = (newbuf == BUF_A) ? BUF_B : BUF_A;
}

/* Rebuild shadow for current waveform+params, then swap into DMA */
static void rebuild_and_swap(void)
{
    switch (g_wave) {
        case WAVE_SINE: build_sine(shadow_buf, g_arr, g_amp_pp_pct, g_dc_pct); break;
        case WAVE_TRI:  build_triangle(shadow_buf, g_arr, g_amp_pp_pct, g_dc_pct); break;
        case WAVE_SQR:  build_square(shadow_buf, g_arr, g_amp_pp_pct, g_dc_pct, g_sqr_duty); break;
        default:        build_sine(shadow_buf, g_arr, g_amp_pp_pct, g_dc_pct); break;
    }
    dma_swap_buffer(shadow_buf);
}

/* ====================== GPIO / TIM1 / DMA init ====================== */
static void GPIO_PD2_TIM1CH1_Init(void)
{
    GPIO_InitTypeDef io = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    io.GPIO_Pin   = GPIO_Pin_2;             // PD2 -> TIM1_CH1
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
    tb.TIM_RepetitionCounter = rcr;      // decimate Update events -> DMA pacing
    TIM_TimeBaseInit(TIM1, &tb);

    oc.TIM_OCMode      = TIM_OCMode_PWM1;    // duty = CH1CVR / ARR
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 0;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &oc);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); // latch on UEV
    TIM_ARRPreloadConfig(TIM1, ENABLE);
}

static void TIM1_DMA_Init(uint16_t *buf)
{
    DMA_InitTypeDef d = {0};
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel5);                      // TIM1 Update -> DMA1 CH5 (CH32V003)

    d.DMA_PeripheralBaseAddr = TIM1_CH1CVR_ADDR;    // dest: TIM1->CH1CVR
    d.DMA_MemoryBaseAddr     = (uint32_t)buf;       // src: LUT buffer
    d.DMA_DIR                = DMA_DIR_PeripheralDST;
    d.DMA_BufferSize         = WF_SAMPLES;
    d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    d.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    d.DMA_Mode               = DMA_Mode_Circular;
    d.DMA_Priority           = DMA_Priority_VeryHigh;
    d.DMA_M2M                = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel5, &d);
    DMA_Cmd(DMA1_Channel5, ENABLE);
    TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);       // DMA on Update (UEV)
}

/* ====================== Runtime controls ====================== */
/* Change repetition safely via SPL to set output frequency */
static void tim1_apply_rcr(uint8_t rcr)
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

/* Set frequency in Hz (approx). Keeps carrier fixed; adjusts repetition. */
static void fg_set_freq_hz(uint32_t hz)
{
    if (hz == 0) hz = 1;
    uint32_t f_carrier = SystemCoreClock / ((g_psc + 1u) * (g_arr + 1u)); // ~100 kHz
    uint32_t rcr_plus1 = (f_carrier + (hz * WF_SAMPLES)/2) / (hz * WF_SAMPLES);
    if (rcr_plus1 < 1)   rcr_plus1 = 1;
    if (rcr_plus1 > 256) rcr_plus1 = 256;   // RCR is 0..255
    tim1_apply_rcr((uint8_t)(rcr_plus1 - 1));
}

/* Set amplitude as % of full-scale (1..100). Rebuilds and swaps LUT. */
static void fg_set_amp_pct(uint8_t pp_pct)
{
    g_amp_pp_pct = (pp_pct < 1) ? 1 : (pp_pct > 100 ? 100 : pp_pct);
    rebuild_and_swap();
}

/* Set DC offset as % of half-scale (-100..+100). Rebuilds and swaps LUT. */
static void fg_set_dc_pct(int8_t dc_pct)
{
    if (dc_pct < -100) dc_pct = -100;
    if (dc_pct >  100) dc_pct =  100;
    g_dc_pct = dc_pct;
    rebuild_and_swap();
}

/* Set square duty (1..99%). Rebuilds and swaps if square selected. */
static void fg_set_square_duty(uint8_t duty_pct)
{
    if (duty_pct < 1) duty_pct = 1;
    if (duty_pct > 99) duty_pct = 99;
    g_sqr_duty = duty_pct;
    if (g_wave == WAVE_SQR) rebuild_and_swap();
}

/* Select waveform; rebuild and swap. */
static void fg_set_waveform(wave_t w)
{
    g_wave = w;
    rebuild_and_swap();
}

/* ====================== Main ====================== */
int main(void)
{
    SystemCoreClockUpdate();   // 48 MHz
    Delay_Init();
#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif
    printf("\r\nCH32V003 TIM1+DMA Function Generator on PD2 (SINE/TRI/SQR)\r\n");
    printf("SystemClk:%ld\r\n", SystemCoreClock);

    /* Build initial buffers (default SINE) */
    build_sine(BUF_A, g_arr, g_amp_pp_pct, g_dc_pct);
    build_sine(BUF_B, g_arr, g_amp_pp_pct, g_dc_pct);

    GPIO_PD2_TIM1CH1_Init();
    TIM1_PWM_Init(g_arr, g_psc, RCR_INIT);
    TIM1_DMA_Init(BUF_A);

    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);  // MOE

    /* Demo: tweak at runtime (replace with your UI/buttons/UART) */
    fg_set_freq_hz(200);          // ~200 Hz output (all waveforms)
    fg_set_amp_pct(80);           // ~10..90% headroom
    fg_set_dc_pct(0);             // centered

    // Example: switch waveforms and settings every second
    while (1) {
        fg_set_waveform(WAVE_SINE);  Delay_Ms(1000);
        fg_set_waveform(WAVE_TRI);   Delay_Ms(1000);
        fg_set_waveform(WAVE_SQR);   fg_set_square_duty(30); Delay_Ms(1000);
        fg_set_square_duty(70);      Delay_Ms(1000);
    }
}
