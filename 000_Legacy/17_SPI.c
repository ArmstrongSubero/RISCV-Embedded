/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 17_SPI
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to demonstrate 
 *                      the use of SPI be interfacing with an MCP41010 digipot
 * 
 * Hardware Description: An MCP41010 digital potentiometer is connected to the 
 *                       CH32V003F4P6 as follows: 
 *
 *                       VDD  -> VDD
 *                       VSS  -> VSS
 *                       SCK  -> PC5
 *                       MOSI -> PC6
 *                       MISO -> PC7
 *                       CS   -> PC3 (software-controlled)
 *           
 * Created August 15th, 2025, 8:57 PM
 * Updated August 15th, 2025, 8:57 PM
 */


/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "ch32v00x.h"
#include "ch32v00x_gpio.h"
#include "ch32v00x_rcc.h"
#include "ch32v00x_spi.h"
#include "debug.h" 

#define MCP41010_CMD_WRITE  0x11
#define MCP_CS_PORT   GPIOC
#define MCP_CS_PIN    GPIO_Pin_3

static void SPI1_Init_Master(void);
static void MCP41010_Init(void);
static void MCP41010_SetWiper(uint8_t value);
static uint8_t spi1_transfer(uint8_t data);

// inline functions for toggling CS pins 
static inline void cs_low(void)  { GPIO_ResetBits(MCP_CS_PORT, MCP_CS_PIN); }
static inline void cs_high(void) { GPIO_SetBits(MCP_CS_PORT, MCP_CS_PIN);  }

/*******************************************************************************
 * Function: Main
 *
 * Returns: Nothing
 *
 * Description: Program entry point
 ******************************************************************************/
int main(void)
{
    /* System clock comes up at 48 MHz by default in most BSPs.
       Update timing and init millisecond delay helper. */
    SystemCoreClockUpdate();
    Delay_Init();

    SPI1_Init_Master();
    MCP41010_Init();

    while (1)
    {
        for (uint8_t i = 0; i < 255; i++) {   // 0..254
            MCP41010_SetWiper(i);
            Delay_Ms(10);
        }
    }
}


/*******************************************************************************
 * Function: static void SPI1_Init_Master(void)
 *
 * Returns: Nothing
 *
 * Description: SPI and GPIO initialization
 * 
 * Usage: PI1_Init_Master()
 ******************************************************************************/
static void SPI1_Init_Master(void)
{
    GPIO_InitTypeDef gpio = {0};
    SPI_InitTypeDef  spi  = {0};

    /* Enable clocks for GPIOC and SPI1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1, ENABLE);

    /* CS (PC3) as push-pull output, high speed, idle high */
    gpio.GPIO_Pin   = MCP_CS_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MCP_CS_PORT, &gpio);
    GPIO_SetBits(MCP_CS_PORT, MCP_CS_PIN);

    /* SPI pins: PC5=SCK (AF_PP), PC6=MOSI (AF_PP), PC7=MISO (floating input) */
    gpio.GPIO_Pin   = GPIO_Pin_5;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_6;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &gpio);

    /* SPI configuration:
       - Master, 8-bit
       - CPOL=0, CPHA=1 (data captured on the second edge) per your original settings (CPHA_2Edge)
       - Software NSS
       - Baud prescaler: start slow (e.g., /256) and increase after validation
       - MSB first
    */
    spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode              = SPI_Mode_Master;
    spi.SPI_DataSize          = SPI_DataSize_8b;
    spi.SPI_CPOL              = SPI_CPOL_Low;
    spi.SPI_CPHA              = SPI_CPHA_1Edge;
    spi.SPI_NSS               = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // safe; MCP41010 allows up to ~10MHz, tune later
    spi.SPI_FirstBit          = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);
}


/*******************************************************************************
 * Function: static void MCP41010_Init(void)
 *
 * Returns: Nothing
 *
 * Description: Initialize the MCP41010 Initialization 
 * 
 * Usage: MCP41010_Init()
 ******************************************************************************/
static void MCP41010_Init(void)
{
    // CS pin already configured in SPI1_Init_Master. Keep CS high.
    GPIO_SetBits(MCP_CS_PORT, MCP_CS_PIN);
}


/*******************************************************************************
 * Function: static uint8_t spi1_transfer(uint8_t data)
 *
 * Returns: Byte recieved on the SPI bus 
 *
 * Description: Transfer a byte of data via SPI 
 * 
 * Usage: (void)spi1_transfer(value)
 ******************************************************************************/
static uint8_t spi1_transfer(uint8_t data)
{
    /* Wait TXE, send byte */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, data);

    /* Wait RXNE, read received byte */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}


/*******************************************************************************
 * Function: static void MCP41010_SetWiper(uint8_t value)
 *
 * Returns: Nothing
 *
 * Description: Set the position of the MCP41010 
 * 
 * Usage: MCP41010_SetWiper(i)
 ******************************************************************************/
static void MCP41010_SetWiper(uint8_t value)
{
    cs_low();
    
    (void)spi1_transfer(MCP41010_CMD_WRITE); // command
    (void)spi1_transfer(value);              // data
   
    cs_high();
}
