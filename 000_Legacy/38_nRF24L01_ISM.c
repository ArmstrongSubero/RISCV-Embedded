/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/ Int. OSC @ 48 MHz, 3.3 V
 * Program: 38_nRF24L01_ISM
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.2
 *                  * Hardware SPI1 support (PC5=SCK, PC6=MOSI, PC7=MISO)
 *                  * Combined TX/RX operation
 *
 * Program Description:
 *   Implements bidirectional communication using an nRF24L01+ 2.4 GHz RF module.
 *   - Configures the radio for 1 Mbps, fixed payload (20 bytes), channel 83.
 *   - Starts in RX mode (listening on pipe 0..5).
 *   - Can switch to TX mode and transmit to matching devices.
 *   - Outputs debug info (status, config, FIFO) via UART at 115200 baud.
 *
 * Hardware Description:
 *   CH32V003F4P6 Connections to nRF24L01+:
 *
 *     CE   -> PC0         // Chip Enable (activates RX/TX)
 *     CSN  -> PC3         // SPI Chip Select (active low)
 *     SCK  -> PC5         // SPI1 SCK
 *     MOSI -> PC6         // SPI1 MOSI
 *     MISO -> PC7         // SPI1 MISO
 *     VCC  -> 3.3 V       // Power (use proper decoupling)
 *     GND  -> GND         // Common ground
 *
 * RF Settings:
 *   - Channel: 83 (2.483 GHz)
 *   - Data rate: 1 Mbps
 *   - Payload: 20 bytes fixed length
 *   - TX_ADDR = RX_ADDR_P0 = {0x78,0x78,0x78,0x78,0x78}
 *   - Auto-ACK enabled on pipe 0; retry config: 15 retries, 4000 µs delay.
 *   - RX pipes 0..5 enabled with matching static addresses.
 *
 * Debug Output Example (RX mode):
 *   CH32V003 + nRF24L01+  COMBINED  (boot RX)  CH=83  1Mbps  PL=20
 *   RX: "Hello 123"
 *   ST=0x40 CFG=0x0F CH=83 RF=0x07 FIFO=0x10
 *
 * Debug Output Example (TX mode):
 *   DBG ST=0x0E CFG=0x0E CH=83 RF=0x07 FIFO=0x01 PW0=20 OBS=0xF9
 *   TX OK: "Hello 602"
 *   TX FAIL: "Hello 603"
 *
 * Notes:
 *   - Ensure RX and TX modules use identical addresses, channel, and payload size.
 *   - Keep SPI frequency ≤ 1 MHz for stability during bring-up.
 *   - For long-range or noisy environments, consider lowering data rate (250 kbps).
 *
 * Created  August 23rd, 2025, 10:35 PM
 * Updated  August 23rd, 2025, 10:35 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "ch32v00x.h"
#include "ch32v00x_rcc.h"
#include "ch32v00x_gpio.h"
#include "ch32v00x_spi.h"
#include "debug.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ===== nRF24L01+ registers & instructions ===== */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

/* Bit mnemonics */
#define MASK_RX_DR  0x40
#define MASK_TX_DS  0x20
#define MASK_MAX_RT 0x10
#define EN_CRC      0x08
#define CRCO        0x04
#define PWR_UP      0x02
#define PRIM_RX     0x01
#define ENAA_P5     0x20
#define ENAA_P4     0x10
#define ENAA_P3     0x08
#define ENAA_P2     0x04
#define ENAA_P1     0x02
#define ENAA_P0     0x01
#define ERX_P5      0x20
#define ERX_P4      0x10
#define ERX_P3      0x08
#define ERX_P2      0x04
#define ERX_P1      0x02
#define ERX_P0      0x01
#define AW5         0x03
#define LNA_HCURR   0x01

/* P-variant bits */
#define RF_DR_LOW   0x20
#define RF_DR_HIGH  0x08
#define RF_PWR_LOW  0x02
#define RF_PWR_HIGH 0x06

/* Instructions */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define NOP           0xFF

/* ===== App config ===== */
#define Channel 83
static short dataLength = 20;

/* Start role (1=RX, 0=TX) */
#define START_AS_RX   1

/* Optional: hold a button on PD4 (to GND) to toggle role at runtime */
#define ROLE_BTN_PORT GPIOD
#define ROLE_BTN_PIN  GPIO_Pin_4

/* adr0 LSB-first — used for TX_ADDR and RX_ADDR_P0 */
static const uint8_t adr0[5] = {0x78,0x78,0x78,0x78,0x78};

/* For completeness, RX also configures P1..P5  */
static const uint8_t adr1[5] = {0xf1,0xb4,0xb5,0xb6,0xb3};
static const uint8_t adr2[5] = {0xb3,0xb4,0xb5,0xb6,0xb3};
static const uint8_t adr3[5] = {0x83,0xb4,0xb5,0xb6,0xb3};
static const uint8_t adr4[5] = {0xcf,0xb4,0xb5,0xb6,0xb3};
static const uint8_t adr5[5] = {0x75,0xb4,0xb5,0xb6,0xb3};

/* ===== Pin map ===== */
#define CE_PORT   GPIOC
#define CE_PIN    GPIO_Pin_0
#define CSN_PORT  GPIOC
#define CSN_PIN   GPIO_Pin_3
/* SPI1: PC5=SCK, PC6=MOSI, PC7=MISO */

static inline void CE_H(void){ CE_PORT->BSHR = CE_PIN; }
static inline void CE_L(void){ CE_PORT->BCR  = CE_PIN; }
static inline void CSN_H(void){ CSN_PORT->BSHR = CSN_PIN; }
static inline void CSN_L(void){ CSN_PORT->BCR  = CSN_PIN; }

/* ===== Buffers & globals ===== */
static char Data_In[21];
static char Data_Out[21];
static char stat;
static int  count;
static char txt[24];

static volatile uint8_t role_is_rx = START_AS_RX;

/* ===== SPI1 init & transfer ===== */
static void SPI1_Init_HW(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef g = {0};

    /* CE, CSN outputs */
    g.GPIO_Pin   = CE_PIN | CSN_PIN;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &g);
    CE_L(); CSN_H();

    /* PC5=SCK, PC6=MOSI (AF_PP); PC7=MISO (floating input) */
    g.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &g);

    g.GPIO_Pin   = GPIO_Pin_7;
    g.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &g);

    SPI_InitTypeDef s = {0};
    s.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    s.SPI_Mode              = SPI_Mode_Master;
    s.SPI_DataSize          = SPI_DataSize_8b;
    s.SPI_CPOL              = SPI_CPOL_Low;      /* mode 0 */
    s.SPI_CPHA              = SPI_CPHA_1Edge;    /* mode 0 */
    s.SPI_NSS               = SPI_NSS_Soft;
    s.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; /* ~0.75 MHz */
    s.SPI_FirstBit          = SPI_FirstBit_MSB;
    s.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &s);
    SPI_Cmd(SPI1, ENABLE);

    /* Optional role toggle button PD4 pull-up */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    g.GPIO_Pin   = ROLE_BTN_PIN;
    g.GPIO_Mode  = GPIO_Mode_IPU;  /* internal pull-up */
    g.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(ROLE_BTN_PORT, &g);
}

static inline uint8_t spi1_xfer(uint8_t b)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, b);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}

/* ===== Helpers ===== */
static inline void toggleCSN(void)
{
    CSN_H(); Delay_Us(20);
    CSN_L(); Delay_Us(10);
}
static inline void Clear_Data(char dat[])
{
    for (int i = 0; i < dataLength; i++) dat[i] = ' ';
}
static inline char Get_Status(void)
{
    char s;
    CE_L();
    toggleCSN();
    spi1_xfer(STATUS);
    Delay_Us(10);
    s = spi1_xfer(NOP);
    Delay_Us(10);
    CSN_H();
    return s;
}
static inline char Get_FIFO_Flags(void)
{
    char s;
    CE_L();
    toggleCSN();
    spi1_xfer(FIFO_STATUS);
    Delay_Us(10);
    s = spi1_xfer(NOP);
    Delay_Us(10);
    CSN_H();
    return s;
}

/* ===== Radio init: RX ===== */
static char radio_init_rx(void)
{
    char i;
    CE_L(); Delay_Us(10);

    toggleCSN(); spi1_xfer(CONFIG | W_REGISTER); Delay_Us(10);
    spi1_xfer(PRIM_RX + PWR_UP + CRCO + EN_CRC); Delay_Us(10);

    toggleCSN(); spi1_xfer(EN_AA | W_REGISTER); Delay_Us(10);
    spi1_xfer(ENAA_P0 + ENAA_P1 + ENAA_P2 + ENAA_P3 + ENAA_P4 + ENAA_P5); Delay_Us(10);

    toggleCSN(); spi1_xfer(EN_RXADDR | W_REGISTER); Delay_Us(10);
    spi1_xfer(ERX_P0 + ERX_P1 + ERX_P2 + ERX_P3 + ERX_P4 + ERX_P5); Delay_Us(10);

    toggleCSN(); spi1_xfer(SETUP_AW | W_REGISTER); Delay_Us(10);
    spi1_xfer(AW5); Delay_Us(10);

    toggleCSN(); spi1_xfer(SETUP_RETR | W_REGISTER); Delay_Us(10);
    spi1_xfer(0xAF); Delay_Us(10);

    toggleCSN(); spi1_xfer(RF_CH | W_REGISTER); Delay_Us(10);
    spi1_xfer(Channel); Delay_Us(10);

    toggleCSN(); spi1_xfer(RF_SETUP | W_REGISTER); Delay_Us(10);
    spi1_xfer(RF_PWR_HIGH | LNA_HCURR); Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_ADDR_P0 | W_REGISTER);
    for (i=0;i<5;i++){ Delay_Us(10); spi1_xfer(adr0[i]); }
    Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_ADDR_P1 | W_REGISTER);
    for (i=0;i<5;i++){ Delay_Us(10); spi1_xfer(adr1[i]); }
    Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_ADDR_P2 | W_REGISTER);
    for (i=0;i<5;i++){ Delay_Us(10); spi1_xfer(adr2[i]); }
    Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_ADDR_P3 | W_REGISTER);
    for (i=0;i<5;i++){ Delay_Us(10); spi1_xfer(adr3[i]); }
    Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_ADDR_P4 | W_REGISTER);
    for (i=0;i<5;i++){ Delay_Us(10); spi1_xfer(adr4[i]); }
    Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_ADDR_P5 | W_REGISTER);
    for (i=0;i<5;i++){ Delay_Us(10); spi1_xfer(adr5[i]); }
    Delay_Us(10);

    toggleCSN(); spi1_xfer(TX_ADDR | W_REGISTER);
    for (i=0;i<5;i++){ Delay_Us(10); spi1_xfer(adr0[i]); }
    Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_PW_P1 | W_REGISTER); Delay_Us(10);
    spi1_xfer((uint8_t)dataLength); Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_PW_P2 | W_REGISTER); Delay_Us(10);
    spi1_xfer((uint8_t)dataLength); Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_PW_P3 | W_REGISTER); Delay_Us(10);
    spi1_xfer((uint8_t)dataLength); Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_PW_P4 | W_REGISTER); Delay_Us(10);
    spi1_xfer((uint8_t)dataLength); Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_PW_P5 | W_REGISTER); Delay_Us(10);
    spi1_xfer((uint8_t)dataLength); Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_PW_P0 | W_REGISTER); Delay_Us(10);
    spi1_xfer((uint8_t)dataLength); Delay_Us(10);

    CSN_H();
    return Get_Status();
}

/* ===== Radio init: TX ===== */
static char radio_init_tx(void)
{
    char i;
    CE_L(); Delay_Us(10);

    toggleCSN(); spi1_xfer(CONFIG | W_REGISTER); Delay_Us(10);
    spi1_xfer(PWR_UP + CRCO + EN_CRC); Delay_Us(10);

    toggleCSN(); spi1_xfer(EN_AA | W_REGISTER); Delay_Us(10);
    spi1_xfer(ENAA_P0); Delay_Us(10);

    toggleCSN(); spi1_xfer(EN_RXADDR | W_REGISTER); Delay_Us(10);
    spi1_xfer(ERX_P0); Delay_Us(10);

    toggleCSN(); spi1_xfer(SETUP_AW | W_REGISTER); Delay_Us(10);
    spi1_xfer(AW5); Delay_Us(10);

    toggleCSN(); spi1_xfer(SETUP_RETR | W_REGISTER); Delay_Us(10);
    spi1_xfer(0xAF); Delay_Us(10);

    toggleCSN(); spi1_xfer(RF_CH | W_REGISTER); Delay_Us(10);
    spi1_xfer(Channel); Delay_Us(10);

    toggleCSN(); spi1_xfer(RF_SETUP | W_REGISTER); Delay_Us(10);
    spi1_xfer(RF_PWR_HIGH | LNA_HCURR); Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_ADDR_P0 | W_REGISTER);
    for (i=0;i<5;i++){ Delay_Us(10); spi1_xfer(adr0[i]); }
    Delay_Us(10);

    toggleCSN(); spi1_xfer(TX_ADDR | W_REGISTER);
    for (i=0;i<5;i++){ Delay_Us(10); spi1_xfer(adr0[i]); }
    Delay_Us(10);

    toggleCSN(); spi1_xfer(RX_PW_P0 | W_REGISTER); Delay_Us(10);
    spi1_xfer((uint8_t)dataLength); Delay_Us(10);

    CSN_H();
    return Get_Status();
}

/* ===== RX data path ===== */
static char readBuffer(void)
{
    char i, s;
    CE_L();
    for (i=0;i<dataLength;i++) Data_In[i] = ' ';
    s = Get_FIFO_Flags();
    if ((s & 0x02) != 0) {
        toggleCSN();
        spi1_xfer(R_RX_PAYLOAD);
        for (i=0; i < dataLength; i++){ Delay_Us(10); Data_In[i] = spi1_xfer(0x00); }
        Delay_Us(10);
    }
    CSN_H();
    CE_H();
    return s;
}

/* ===== TX data path ===== */
static void sendBuffer(void)
{
    char i, j;
    do {
        toggleCSN();
        spi1_xfer(STATUS | W_REGISTER); Delay_Us(10);
        spi1_xfer(0xFF);                Delay_Us(10);   /* clear IRQs */

        toggleCSN();
        spi1_xfer(FLUSH_TX);            Delay_Us(10);

        toggleCSN();
        spi1_xfer(W_TX_PAYLOAD);
        for (i = 0; i < dataLength; i++) { Delay_Us(10); spi1_xfer((uint8_t)Data_Out[i]); }
        Delay_Us(10);
        CSN_H();

        CE_H();
        Delay_Ms(10); /* long CE */

        j = (Get_Status() & MASK_TX_DS);
    } while (j == 0);
}

/* ===== Runtime helpers ===== */
static void role_apply(uint8_t rx)
{
    if (rx) {
        stat = radio_init_rx();
        Delay_Ms(10);
        CE_H(); /* listen */
        printf("ROLE=RX  STATUS=0x%02X\r\n", (unsigned)stat);
    } else {
        stat = radio_init_tx();
        Delay_Ms(10);
        CE_L(); /* PTX standby */
        printf("ROLE=TX  STATUS=0x%02X\r\n", (unsigned)stat);
    }
}

static uint8_t btn_pressed(void)
{
    /* simple edge detect with ~10ms debounce */
    static uint8_t last = 1, stable = 1, cnt = 0;
    uint8_t now = (ROLE_BTN_PORT->INDR & ROLE_BTN_PIN) ? 1 : 0; /* pull-up idle=1, pressed=0 */
    if (now != last){ last = now; cnt = 0; }
    else if (cnt < 10){ cnt++; if (cnt==10) stable = now; }
    return (stable == 0); /* pressed when stable low */
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

#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif

    printf("CH32V003 + nRF24L01+  COMBINED  (boot %s)  CH=%d  1Mbps  PL=%d\r\n",
           START_AS_RX ? "RX" : "TX", Channel, dataLength);

    SPI1_Init_HW();
    Delay_Ms(100);
    role_apply(role_is_rx);

    /* clear TX buffer text space */
    for (int i=0;i<dataLength;i++) Data_Out[i] = ' ';

    uint32_t tick = 0;
    while (1)
    {
        /* Optional runtime toggle via PD4 */
        if (! (ROLE_BTN_PORT->INDR & ROLE_BTN_PIN)) {   /* button low = request toggle */
            Delay_Ms(20);
            if (! (ROLE_BTN_PORT->INDR & ROLE_BTN_PIN)) {
                role_is_rx ^= 1;
                role_apply(role_is_rx);
                while (! (ROLE_BTN_PORT->INDR & ROLE_BTN_PIN)) { Delay_Ms(5); } /* wait release */
            }
        }

        if (role_is_rx) {
            char flags = readBuffer();
            if ((flags & 0x02) != 0) {
                Data_In[dataLength-1] = 0;
                printf("RX: \"%s\"\r\n", Data_In);
            }
            Delay_Ms(100);
        } else {
            /* TX: send a line every ~100ms */
            snprintf(txt, sizeof(txt), "Count: %lu", (unsigned long)tick++);
            /* makeMsg(): clear + copy */
            for (int i=0;i<dataLength;i++) Data_Out[i] = ' ';
            strncpy(Data_Out, txt, dataLength);
            sendBuffer();
            printf("TX OK: \"%s\"\r\n", txt);
            Delay_Ms(100);
        }
    }
}
