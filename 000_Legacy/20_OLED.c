/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: OLED
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to demonstrate 
 *                      interfacing to an SSD1306 OLED
 * 
 * Hardware Description: A SSD1306 OLED is connected to the 
 *                       CH32V003F4P6 as follows: 
 *
 *                       VDD  -> VDD
 *                       VSS  -> VSS
 *                       SDA  -> PC1 
 *                       SCL  -> PC2 

 * Created August 15th, 2025, 11:10 PM
 * Updated August 15th, 2025, 11:10 PM
 */


/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "ch32v00x.h"
#include "ch32v00x_rcc.h"
#include "ch32v00x_gpio.h"
#include "ch32v00x_i2c.h"
#include "debug.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ---- Display geometry ---- */
#ifndef OLED_WIDTH
#define OLED_WIDTH   128
#endif
#ifndef OLED_HEIGHT
#define OLED_HEIGHT  32           /* set 64 for 128x64 modules */
#endif
#define OLED_PAGES   (OLED_HEIGHT/8)

extern const uint8_t OledFont[][8];   /* ASCII 32..127, 8x8 each */
extern const uint8_t BatteryIcon[];   /* at least 8 bytes used */
static uint8_t screen_buffer[OLED_PAGES][OLED_WIDTH];

/* ---- SSD1306 commands ---- */
#define OLED_SETCONTRAST              0x81
#define OLED_DISPLAYALLON_RESUME      0xA4
#define OLED_NORMALDISPLAY            0xA6
#define OLED_DISPLAYOFF               0xAE
#define OLED_DISPLAYON                0xAF
#define OLED_SETDISPLAYOFFSET         0xD3
#define OLED_SETCOMPINS               0xDA
#define OLED_SETVCOMDETECT            0xDB
#define OLED_SETDISPLAYCLOCKDIV       0xD5
#define OLED_SETPRECHARGE             0xD9
#define OLED_SETMULTIPLEX             0xA8
#define OLED_SETSTARTLINE             0x40
#define OLED_MEMORYMODE               0x20
#define OLED_SEGREMAP                 0xA0
#define OLED_COMSCANDEC               0xC8
#define OLED_CHARGEPUMP               0x8D

#define OLED_DEACTIVATE_SCROLL        0x2E
#define OLED_ACTIVATE_SCROLL          0x2F

#define OLED_SEGREMAP            0xA0  // +0 or +1
#define OLED_COMSCANINC          0xC0
#define OLED_COMSCANDEC          0xC8
#define OLED_DEACTIVATE_SCROLL   0x2E

/* ---- OLED Font ---- */
const uint8_t OledFont[][8] =
{
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x5F,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x07,0x00,0x07,0x00,0x00,0x00},
  {0x00,0x14,0x7F,0x14,0x7F,0x14,0x00,0x00},
  {0x00,0x24,0x2A,0x7F,0x2A,0x12,0x00,0x00},
  {0x00,0x23,0x13,0x08,0x64,0x62,0x00,0x00},
  {0x00,0x36,0x49,0x55,0x22,0x50,0x00,0x00},
  {0x00,0x00,0x05,0x03,0x00,0x00,0x00,0x00},
  {0x00,0x1C,0x22,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x41,0x22,0x1C,0x00,0x00,0x00,0x00},
  {0x00,0x08,0x2A,0x1C,0x2A,0x08,0x00,0x00},
  {0x00,0x08,0x08,0x3E,0x08,0x08,0x00,0x00},
  {0x00,0xA0,0x60,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x08,0x08,0x08,0x08,0x08,0x00,0x00},
  {0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x20,0x10,0x08,0x04,0x02,0x00,0x00},
  {0x00,0x3E,0x51,0x49,0x45,0x3E,0x00,0x00},
  {0x00,0x00,0x42,0x7F,0x40,0x00,0x00,0x00},
  {0x00,0x62,0x51,0x49,0x49,0x46,0x00,0x00},
  {0x00,0x22,0x41,0x49,0x49,0x36,0x00,0x00},
  {0x00,0x18,0x14,0x12,0x7F,0x10,0x00,0x00},
  {0x00,0x27,0x45,0x45,0x45,0x39,0x00,0x00},
  {0x00,0x3C,0x4A,0x49,0x49,0x30,0x00,0x00},
  {0x00,0x01,0x71,0x09,0x05,0x03,0x00,0x00},
  {0x00,0x36,0x49,0x49,0x49,0x36,0x00,0x00},
  {0x00,0x06,0x49,0x49,0x29,0x1E,0x00,0x00},
  {0x00,0x00,0x36,0x36,0x00,0x00,0x00,0x00},
  {0x00,0x00,0xAC,0x6C,0x00,0x00,0x00,0x00},
  {0x00,0x08,0x14,0x22,0x41,0x00,0x00,0x00},
  {0x00,0x14,0x14,0x14,0x14,0x14,0x00,0x00},
  {0x00,0x41,0x22,0x14,0x08,0x00,0x00,0x00},
  {0x00,0x02,0x01,0x51,0x09,0x06,0x00,0x00},
  {0x00,0x32,0x49,0x79,0x41,0x3E,0x00,0x00},
  {0x00,0x7E,0x09,0x09,0x09,0x7E,0x00,0x00},
  {0x00,0x7F,0x49,0x49,0x49,0x36,0x00,0x00},
  {0x00,0x3E,0x41,0x41,0x41,0x22,0x00,0x00},
  {0x00,0x7F,0x41,0x41,0x22,0x1C,0x00,0x00},
  {0x00,0x7F,0x49,0x49,0x49,0x41,0x00,0x00},
  {0x00,0x7F,0x09,0x09,0x09,0x01,0x00,0x00},
  {0x00,0x3E,0x41,0x41,0x51,0x72,0x00,0x00},
  {0x00,0x7F,0x08,0x08,0x08,0x7F,0x00,0x00},
  {0x00,0x41,0x7F,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x20,0x40,0x41,0x3F,0x01,0x00,0x00},
  {0x00,0x7F,0x08,0x14,0x22,0x41,0x00,0x00},
  {0x00,0x7F,0x40,0x40,0x40,0x40,0x00,0x00},
  {0x00,0x7F,0x02,0x0C,0x02,0x7F,0x00,0x00},
  {0x00,0x7F,0x04,0x08,0x10,0x7F,0x00,0x00},
  {0x00,0x3E,0x41,0x41,0x41,0x3E,0x00,0x00},
  {0x00,0x7F,0x09,0x09,0x09,0x06,0x00,0x00},
  {0x00,0x3E,0x41,0x51,0x21,0x5E,0x00,0x00},
  {0x00,0x7F,0x09,0x19,0x29,0x46,0x00,0x00},
  {0x00,0x26,0x49,0x49,0x49,0x32,0x00,0x00},
  {0x00,0x01,0x01,0x7F,0x01,0x01,0x00,0x00},
  {0x00,0x3F,0x40,0x40,0x40,0x3F,0x00,0x00},
  {0x00,0x1F,0x20,0x40,0x20,0x1F,0x00,0x00},
  {0x00,0x3F,0x40,0x38,0x40,0x3F,0x00,0x00},
  {0x00,0x63,0x14,0x08,0x14,0x63,0x00,0x00},
  {0x00,0x03,0x04,0x78,0x04,0x03,0x00,0x00},
  {0x00,0x61,0x51,0x49,0x45,0x43,0x00,0x00},
  {0x00,0x7F,0x41,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x02,0x04,0x08,0x10,0x20,0x00,0x00},
  {0x00,0x41,0x41,0x7F,0x00,0x00,0x00,0x00},
  {0x00,0x04,0x02,0x01,0x02,0x04,0x00,0x00},
  {0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00},
  {0x00,0x01,0x02,0x04,0x00,0x00,0x00,0x00},
  {0x00,0x20,0x54,0x54,0x54,0x78,0x00,0x00},
  {0x00,0x7F,0x48,0x44,0x44,0x38,0x00,0x00},
  {0x00,0x38,0x44,0x44,0x28,0x00,0x00,0x00},
  {0x00,0x38,0x44,0x44,0x48,0x7F,0x00,0x00},
  {0x00,0x38,0x54,0x54,0x54,0x18,0x00,0x00},
  {0x00,0x08,0x7E,0x09,0x02,0x00,0x00,0x00},
  {0x00,0x18,0xA4,0xA4,0xA4,0x7C,0x00,0x00},
  {0x00,0x7F,0x08,0x04,0x04,0x78,0x00,0x00},
  {0x00,0x00,0x7D,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x80,0x84,0x7D,0x00,0x00,0x00,0x00},
  {0x00,0x7F,0x10,0x28,0x44,0x00,0x00,0x00},
  {0x00,0x41,0x7F,0x40,0x00,0x00,0x00,0x00},
  {0x00,0x7C,0x04,0x18,0x04,0x78,0x00,0x00},
  {0x00,0x7C,0x08,0x04,0x7C,0x00,0x00,0x00},
  {0x00,0x38,0x44,0x44,0x38,0x00,0x00,0x00},
  {0x00,0xFC,0x24,0x24,0x18,0x00,0x00,0x00},
  {0x00,0x18,0x24,0x24,0xFC,0x00,0x00,0x00},
  {0x00,0x00,0x7C,0x08,0x04,0x00,0x00,0x00},
  {0x00,0x48,0x54,0x54,0x24,0x00,0x00,0x00},
  {0x00,0x04,0x7F,0x44,0x00,0x00,0x00,0x00},
  {0x00,0x3C,0x40,0x40,0x7C,0x00,0x00,0x00},
  {0x00,0x1C,0x20,0x40,0x20,0x1C,0x00,0x00},
  {0x00,0x3C,0x40,0x30,0x40,0x3C,0x00,0x00},
  {0x00,0x44,0x28,0x10,0x28,0x44,0x00,0x00},
  {0x00,0x1C,0xA0,0xA0,0x7C,0x00,0x00,0x00},
  {0x00,0x44,0x64,0x54,0x4C,0x44,0x00,0x00},
  {0x00,0x08,0x36,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x7F,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x41,0x36,0x08,0x00,0x00,0x00,0x00},
  {0x00,0x02,0x01,0x01,0x02,0x01,0x00,0x00},
  {0x00,0x02,0x05,0x05,0x02,0x00,0x00,0x00},
};

/* ---- A battery Icon bitmap array ---- */
const uint8_t BatteryIcon[10] = 
{
    0xFF, // 11111111  Top border of the larger battery
    0x81, // 10000001  Left and right borders
    0xBD, // 10111101  Fill partway for half-filled battery
    0xBD, // 10111101  Continue half-filled representation
    0xBD, // 10111101  Continue half-filled representation
    0x81, // 10000001  Left and right borders
    0xFF, // 11111111  Bottom border of the battery
    0x18, // 00011000  Small rectangle (knob) on the right side
    0x00, // 00000000  Empty space for padding
    0x00  // 00000000  Empty space for padding
};


// SSD1306 function prototypes 
void I2C1_Init_CH32(uint32_t clock_hz);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_YX(uint8_t page, uint8_t col_char);
void OLED_WriteString(const char* s, uint8_t page, uint8_t col_char);
void OLED_Printf_Line(uint8_t line, const char* fmt, ...);
void OLED_DrawBattery(uint8_t percentage, uint8_t x, uint8_t page);
void OLED_UpdatePartialLine(uint8_t page, uint8_t start_col, uint8_t num_cols);


/*******************************************************************************
 * I2C Functions 
 ******************************************************************************/
/* ===== I2C address as 8-bit "control byte" ===== */
#define SSD1306_ADDR8   (0x3Cu<<1)   /* 0x78 write / 0x79 read */

/* ===== I2C init (PC2=SCL, PC1=SDA) ===== */
void I2C1_Init_CH32(uint32_t clock_hz)
{
    GPIO_InitTypeDef gpio = {0};
    I2C_InitTypeDef  i2c  = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;

    gpio.GPIO_Pin = GPIO_Pin_2; GPIO_Init(GPIOC, &gpio);  /* SCL */
    gpio.GPIO_Pin = GPIO_Pin_1; GPIO_Init(GPIOC, &gpio);  /* SDA */

    i2c.I2C_ClockSpeed          = clock_hz;               /* 100k to start */
    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1         = 0x00;
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

/* ===== tiny I2C helpers (event-driven) ===== */
static int wait_event(uint32_t evt, uint32_t ms)
{
    while(!I2C_CheckEvent(I2C1, evt)){ Delay_Ms(1); if(!ms--) return 0; }
    return 1;
}
static int wait_busy_clear(uint32_t ms)
{
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)!=RESET){ Delay_Ms(1); if(!ms--) return 0; }
    return 1;
}
static int i2c_start(void)
{
    if(!wait_busy_clear(20)) return 0;
    I2C_GenerateSTART(I2C1, ENABLE);
    return wait_event(I2C_EVENT_MASTER_MODE_SELECT, 20);  /* EV5 */
}
static void i2c_stop(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    (void)wait_busy_clear(10);
}
static int addr_tx(uint8_t ctrl8)
{
    I2C_Send7bitAddress(I2C1, ctrl8, I2C_Direction_Transmitter);
    return wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, 20); /* EV6 */
}

static int send_byte(uint8_t b)
{
    I2C_SendData(I2C1, b);
    return wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, 20);         /* EV8_2 */
}

/*******************************************************************************
 * Function: static int OLED_Command(uint8_t cmd)
 *
 * Returns: Command status 
 *
 * Description: Send a command to the SSD1306 OLED module 
 * 
 * Usage: OLED_Command(COMMAND)
 ******************************************************************************/
static int OLED_Command(uint8_t cmd)
{
    if(!i2c_start()) return 0;
    if(!addr_tx(SSD1306_ADDR8)) { i2c_stop(); return 0; }
    if(!send_byte(0x00))         { i2c_stop(); return 0; } /* control: command */
    if(!send_byte(cmd))          { i2c_stop(); return 0; }
    i2c_stop();
    return 1;
}

/*******************************************************************************
 * Function: static int OLED_WriteData(const uint8_t* data, uint16_t len)
 *
 * Returns: Command status 
 *
 * Description: Send data to the SSD1306 OLED module 
 * 
 * Usage:  OLED_WriteData(DATA)
 ******************************************************************************/
static int OLED_WriteData(const uint8_t* data, uint16_t len)
{
    if(!i2c_start()) return 0;
    if(!addr_tx(SSD1306_ADDR8)) { i2c_stop(); return 0; }
    if(!send_byte(0x40))         { i2c_stop(); return 0; } /* control: data */
    for(uint16_t i=0;i<len;i++){
        if(!send_byte(data[i])) { i2c_stop(); return 0; }
    }
    i2c_stop();
    return 1;
}

typedef enum {
    OLED_ORIENT_0 = 0,        // current default in your init (A1 + C8)
    OLED_ORIENT_180,          // rotate 180° (A0 + C0)
    OLED_MIRROR_X,            // mirror horizontally only
    OLED_MIRROR_Y             // mirror vertically only
} oled_orient_t;


/*******************************************************************************
 * Function: static void OLED_SetOrientation(oled_orient_t o)
 *
 * Returns: Nothing 
 *
 * Description: Set orientation of the SSD1306 display 
 * 
 * Usage:  OLED_SetOrientation(OLED_ORIENT_180)
 ******************************************************************************/
static void OLED_SetOrientation(oled_orient_t o)
{
    /* Stop any scroll so visual change is immediate */
    OLED_Command(OLED_DEACTIVATE_SCROLL);

    switch (o) {
    default:
    case OLED_ORIENT_0:
        /* Match init: A1 + C8 */
        OLED_Command(OLED_SEGREMAP | 0x01);   // SEG 127..0 (normal for most modules)
        OLED_Command(OLED_COMSCANDEC);        // COM N-1..0
        break;

    case OLED_ORIENT_180:
        /* 180°: invert both axes */
        OLED_Command(OLED_SEGREMAP | 0x00);   // SEG 0..127
        OLED_Command(OLED_COMSCANINC);        // COM 0..N-1
        break;

    case OLED_MIRROR_X:
        /* horizontal mirror only */
        /* keep COM like current (C8), toggle SEG */
        OLED_Command(OLED_SEGREMAP | 0x00);   // flip X
        OLED_Command(OLED_COMSCANDEC);        // keep Y
        break;

    case OLED_MIRROR_Y:
        /* vertical mirror only */
        /* keep SEG like current (A1), toggle COM */
        OLED_Command(OLED_SEGREMAP | 0x01);   // keep X
        OLED_Command(OLED_COMSCANINC);        // flip Y
        break;
    }
}


// Send single data byte to the SSD1306 OLED module 
void OLED_Data(uint8_t d){ (void)OLED_WriteData(&d,1); }

/*******************************************************************************
 * Function: void OLED_YX(uint8_t page, uint8_t col_char)
 *
 * Returns: Nothing
 *
 * Description: Positioning for page addressing 
 * 
 * Usage:  OLED_YX(page, 0)
 ******************************************************************************/
void OLED_YX(uint8_t page, uint8_t col_char /* 0..15 */)
{
    OLED_Command(0xB0 | (page & 0x07));
    OLED_Command(0x00 | ((col_char*8) & 0x0F));
    OLED_Command(0x10 | (((col_char*8)>>4) & 0x0F));
}

/*******************************************************************************
 * Function: void OLED_Init(void)
 *
 * Returns: Nothing
 *
 * Description: Initializes the OLED module  
 * 
 * Usage: void OLED_Init(void)
 ******************************************************************************/
void OLED_Init(void)
{
    Delay_Ms(10);

    OLED_Command(OLED_DISPLAYOFF);          // 0xAE
    OLED_Command(OLED_DEACTIVATE_SCROLL);   // <- stop any auto-scroll left/right

    OLED_Command(OLED_SETDISPLAYCLOCKDIV);  // 0xD5
    OLED_Command(0x80);

    OLED_Command(OLED_SETMULTIPLEX);        // 0xA8
    OLED_Command(0x1F);                     // 32 rows (for 128x32)

    OLED_Command(OLED_SETDISPLAYOFFSET);    // 0xD3
    OLED_Command(0x00);

    OLED_Command(OLED_SETSTARTLINE | 0x00); // 0x40 | 0

    OLED_Command(OLED_CHARGEPUMP);          // 0x8D
    OLED_Command(0x14);                     // internal VCC (use 0x10 for external)

    OLED_Command(OLED_MEMORYMODE);          // 0x20
    OLED_Command(0x02);                     // <- PAGE addressing (matches B0/00/10)

    OLED_Command(OLED_SEGREMAP | 0x01);     // mirror if needed
    OLED_Command(OLED_COMSCANDEC);          // scan COM from N-1 to 0

    OLED_Command(OLED_SETCOMPINS);          // 0xDA
    OLED_Command(0x02);                     // 128x32

    OLED_Command(OLED_SETCONTRAST);         // 0x81
    OLED_Command(0x8F);

    OLED_Command(OLED_SETPRECHARGE);        // 0xD9
    OLED_Command(0xF1);

    OLED_Command(OLED_SETVCOMDETECT);       // 0xDB
    OLED_Command(0x40);

    OLED_Command(OLED_DISPLAYALLON_RESUME); // 0xA4
    OLED_Command(OLED_NORMALDISPLAY);       // 0xA6
    OLED_Command(OLED_DISPLAYON);           // 0xAF

    OLED_Command(OLED_DEACTIVATE_SCROLL);   // <- again, just in case

    /* Clear VRAM on the panel */
    for (uint8_t p = 0; p < OLED_PAGES; p++) {
        memset(screen_buffer[p], 0, OLED_WIDTH);
        OLED_YX(p, 0);
        OLED_WriteData(screen_buffer[p], OLED_WIDTH);
    }
}


/*******************************************************************************
 * Function: void OLED_UpdateLine(uint8_t page)
 *
 * Returns: Nothing
 *
 * Description: Updates a line on the OLED module for drawing / text  
 * 
 * Usage: OLED_UpdateLine(p)
 ******************************************************************************/
void OLED_UpdateLine(uint8_t page)
{
    OLED_YX(page, 0);
    OLED_WriteData(screen_buffer[page], OLED_WIDTH);
}

/*******************************************************************************
 * Function: void OLED_UpdatePartialLine(uint8_t page, uint8_t start_col, 
 *                                  uint8_t num_cols)
 *
 * Returns: Nothing
 *
 * Description: Give a partial line update to the OLED 
 * 
 * Usage: OLED_UpdatePartialLine(page, x, 8);
 ******************************************************************************/
void OLED_UpdatePartialLine(uint8_t page, uint8_t start_col, uint8_t num_cols)
{
    if(page>=OLED_PAGES || start_col>=OLED_WIDTH) return;
    if(start_col+num_cols > OLED_WIDTH) num_cols = OLED_WIDTH - start_col;

    OLED_Command(0xB0 | (page & 0x07));
    OLED_Command(0x00 | (start_col & 0x0F));
    OLED_Command(0x10 | ((start_col>>4) & 0x0F));
    OLED_WriteData(&screen_buffer[page][start_col], num_cols);
}

/*******************************************************************************
 * Function: void OLED_Clear(void)
 *
 * Returns: Nothing
 *
 * Description: Clears the OLED module 
 * 
 * Usage: OLED_Clear()
 ******************************************************************************/
void OLED_Clear(void)
{
    for(uint8_t p=0;p<OLED_PAGES;p++){
        memset(screen_buffer[p], 0, OLED_WIDTH);
        OLED_UpdateLine(p);
    }
}


/*******************************************************************************
 * Function: void OLED_PutChar(char ch)
 *
 * Returns: Nothing
 *
 * Description: Writes a character to the OLED module 
 * 
 * Usage: OLED_PutChar(char) 
 ******************************************************************************/
void OLED_PutChar(char ch)
{
    if(ch < 32 || ch > 127) ch = ' ';
    OLED_WriteData((const uint8_t*)&OledFont[ch-32][0], 8);
}


/*******************************************************************************
 * Function: void OLED_WriteString(const char* s, uint8_t page, uint8_t col_char)
 *
 * Returns: Nothing
 *
 * Description: Writes a string to the OLED module 
 * 
 * Usage: OLED_WriteString(string) 
 ******************************************************************************/
void OLED_WriteString(const char* s, uint8_t page, uint8_t col_char)
{
    OLED_YX(page, col_char);
    while(*s){
        char ch = *s++;
        if(ch < 32 || ch > 127) ch = ' ';
        OLED_WriteData((const uint8_t*)&OledFont[ch-32][0], 8);
    }
}


/*******************************************************************************
 * Function: void OLED_Printf_Line(uint8_t line, const char* fmt, ...)
 *
 * Returns: Nothing
 *
 * Description: Function to make printing to the OLED easier and simpler
 * 
 * Usage:  OLED_Printf_Line(2, "Hello, world!")
 ******************************************************************************/
void OLED_Printf_Line(uint8_t line, const char* fmt, ...)
{
    static char last_text[8][32];  /* up to 8 lines */
    char buf[32];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if(strcmp(buf, last_text[line])==0) return;
    strcpy(last_text[line], buf);

    uint8_t clear_w = (line==0 ? (OLED_WIDTH-8) : OLED_WIDTH);
    memset(&screen_buffer[line][0], 0, clear_w);

    uint8_t col=0;
    for(const char* p=buf; *p && (col+8)<=clear_w; ++p){
        char ch = *p;
        if(ch < 32 || ch > 127) ch = ' ';
        memcpy(&screen_buffer[line][col], &OledFont[ch-32][0], 8);
        col += 8;
    }
    OLED_UpdateLine(line);
}


/*******************************************************************************
 * Function: void OLED_DrawBattery(uint8_t percentage, uint8_t x, uint8_t page)
 *
 * Returns: Nothing
 *
 * Description: Draw battery icon into screen buffer, we can also push with a 
 *              partial line update 
 * 
 * Usage:  OLED_DrawBattery(battery, 120, 0)
 ******************************************************************************/
void OLED_DrawBattery(uint8_t percentage, uint8_t x, uint8_t page)
{
    if(percentage>100) percentage=100;
    if(x+8 > OLED_WIDTH || page>=OLED_PAGES) return;

    for(uint8_t i=0;i<8;i++){
        screen_buffer[page][x+i] = BatteryIcon[i];
    }
    uint8_t fill_w = (percentage * 6) / 100;
    for(uint8_t i=1;i<=fill_w && i<=6; i++){
        screen_buffer[page][x+i] |= 0x7E;   /* middle 6 bits */
    }
    /* push if you want immediate update: */
    /* OLED_UpdatePartialLine(page, x, 8); */
}

/* ---- Convenience single-command wrapper ---- */
void OLED_Command_Wrapper(uint8_t temp){ (void)OLED_Command(temp); }  /* alias if needed */


/*******************************************************************************
 * Function: Main
 *
 * Returns: Nothing
 *
 * Description: Program entry point
 ******************************************************************************/
int main(void)
{
    /* Clock/Delay/UART init (per WCH examples) */
    SystemCoreClockUpdate();
    Delay_Init();
#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif

    printf("SystemClk:%ld\r\n", SystemCoreClock);
    printf("CH32V003 + SSD1306 demo\r\n");

    /* I2C + OLED init */
    I2C1_Init_CH32(100000);   /* start at 100 kHz; you can try 400 kHz later */
    Delay_Ms(10);
    OLED_Init();
    Delay_Ms(10);
    OLED_SetOrientation(OLED_ORIENT_180);  // rotate 180°
    Delay_Ms(10);
    OLED_Clear();

    /* Demo variables */
    uint32_t counter = 0;
    uint8_t battery  = 100;

    while (1)
    {
        /* Text lines (line 0 reserves last 8px for battery in your driver) */
        OLED_Printf_Line(0, "CH32V003 OLED");
        OLED_Printf_Line(1, "Count: %lu", (unsigned long)counter);
        OLED_Printf_Line(2, "Hello, world!");

        /* Battery icon at top-right (x=120..127), page 0 */
        OLED_DrawBattery(battery, 120, 0);
        OLED_UpdatePartialLine(0, 120, 8);  /* push only the battery region */

        /* Simple animation */
        counter++;
        if (battery > 0) battery--; else battery = 100;

        Delay_Ms(500);
    }
}