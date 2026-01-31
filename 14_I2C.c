/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 14_I2C
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to demonstrate 
 *                      the use of I2C be interfacing with a 24LC16B EEPROM
 * 
 * Hardware Description: A 24LC16B EEPROM is connected to the 
 *                       CH32V003F4P6 as follows: 
 *
 *                       VDD  -> VDD
 *                       VSS  -> VSS
 *                       SDA  -> PC1 (4.7k Pullup)
 *                       SCL  -> PC2 (4.7k Pullup)
 *                       WP   -> GND
 *                       A0   -> GND
 *                       A1   -> GND
 *                       A2   -> GND
 *           
 * Created August  15th, 2025, 10:41 PM
 * Updated January 03rd, 2026, 01:10 PM
 */

/*******************************************************************************
 *Includes and defines
 ******************************************************************************/
#include "ch32v00x.h"
#include "ch32v00x_rcc.h"
#include "ch32v00x_gpio.h"
#include "ch32v00x_i2c.h"
#include "debug.h"
#include <stdio.h>

static inline uint8_t eeprom_ctrl(uint16_t word_addr);
static int wait_event(uint32_t evt, uint32_t ms);
static int wait_flag_set(FlagStatus (*fn)(I2C_TypeDef*, uint32_t),
                         I2C_TypeDef *I2Cx, uint32_t flag, uint32_t ms);
static int wait_flag_clear(FlagStatus (*fn)(I2C_TypeDef*, uint32_t),
                           I2C_TypeDef *I2Cx, uint32_t flag, uint32_t ms);
static void i2c_bus_soft_recover(void);
static void I2C1_Init_CH32(uint32_t clock_hz);
static int i2c_start(void);                                     
static int i2c_restart(void);
static void i2c_stop(void);
static int addr_tx(uint8_t ctrl8);
static int addr_rx(uint8_t ctrl8);
static int send_byte(uint8_t b);
static int eeprom_write_byte(uint16_t word_addr, uint8_t data);
static int eeprom_read_byte(uint16_t word_addr, uint8_t *out);

/*******************************************************************************
 * Function: void initMain()
 *
 * Returns: Nothing
 *
 * Description: Contains initializations for main
 * 
 * Usage: initMain()
 ******************************************************************************/
void initmain()
{
    SystemCoreClockUpdate();
    Delay_Init();
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
    initmain();
 
#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif

    printf("SystemClk:%ld\r\n", SystemCoreClock);
    printf("CH32V003 I2C + 24LC16B (event-driven, 8-bit ctrl + proper RESTART)\r\n");

    I2C1_Init_CH32(400000); // start at 100 kHz

    uint16_t addr = 0;
    uint8_t  wr   = 8;

    while (1)
    {
        printf("Write addr=%u data=%u\r\n", addr, wr);
        if (!eeprom_write_byte(addr, wr)) {
            printf("Write timeout @%u\r\n", addr);
        }

        uint8_t rd = 0xFF;
        if (!eeprom_read_byte(addr, &rd)) {
            printf("Read  timeout @%u\r\n\r\n", addr);
        } else {
            printf("Read  addr=%u data=%u\r\n\r\n", addr, rd);
        }

        wr += 2;
        addr = (addr + 1) & 0x07FF; // 0..2047
        Delay_Ms(1000);
    }
}



/*******************************************************************************
 * Function: static inline uint8_t eeprom_ctrl(uint16_t word_addr) 
 *
 * Returns: Nothing
 *
 * Description: block = A10..A8 (0..7), base = 0xA0
 *              For reads we still pass the same ctrl byte to I2C_Send7bitAddress(),
 *              but with I2C_Direction_Receiver (the lib sets R/W for us)
 * 
 * Usage:  uint8_t ctrl8 = eeprom_ctrl(word_addr);
 ******************************************************************************/
static inline uint8_t eeprom_ctrl(uint16_t word_addr) 
{
    uint8_t block = (word_addr >> 8) & 0x07;       // A10..A8
    return (uint8_t)(0xA0 | (block << 1));         // 0xA0,0xA2,...,0xAE
}


/*******************************************************************************
 * Function: static int wait_event(uint32_t evt, uint32_t ms) 
 *
 * Returns: Status of wait event 
 *
 * Description: Timeout for a wait event 
 * 
 * Usage: PI1_Init_Master()
 ******************************************************************************/
static int wait_event(uint32_t evt, uint32_t ms)
{
    while (!I2C_CheckEvent(I2C1, evt)) { Delay_Ms(1); if (!ms--) return 0; }
    return 1;
}


/*******************************************************************************
 * Function: static int wait_flag_set(FlagStatus (*fn)(I2C_TypeDef*, uint32_t),
 *               I2C_TypeDef *I2Cx, uint32_t flag, uint32_t ms)
 *
 * Returns: Status of the wait flag 
 *
 * Description: Wait flag event 
 * 
 * Usage: if (!wait_flag_set(I2C_GetFlagStatus, I2C1, I2C_FLAG_RXNE, 20))
 ******************************************************************************/
static int wait_flag_set(FlagStatus (*fn)(I2C_TypeDef*, uint32_t),
                         I2C_TypeDef *I2Cx, uint32_t flag, uint32_t ms)
{
    while (fn(I2Cx, flag) == RESET) { Delay_Ms(1); if (!ms--) return 0; }
    return 1;
}


/*******************************************************************************
 * Function: static int wait_flag_clear(FlagStatus (*fn)(I2C_TypeDef*, uint32_t),
 *                      I2C_TypeDef *I2Cx, uint32_t flag, uint32_t ms)
 *
 * Returns: Status of the wait flag 
 *
 * Description: Wait flag clear status 
 * 
 * Usage: if (!wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 20))
 ******************************************************************************/
static int wait_flag_clear(FlagStatus (*fn)(I2C_TypeDef*, uint32_t),
                           I2C_TypeDef *I2Cx, uint32_t flag, uint32_t ms)
{
    while (fn(I2Cx, flag) != RESET) { Delay_Ms(1); if (!ms--) return 0; }
    return 1;
}


/*******************************************************************************
 * Function: static void i2c_bus_soft_recover(void)
 *
 * Returns: Nothing 
 *
 * Description: Fucntion to help the I2C bus recover  
 * 
 * Usage: i2c_bus_soft_recover()
 ******************************************************************************/
static void i2c_bus_soft_recover(void)
{
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    Delay_Ms(1);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
}


/*******************************************************************************
 * Function: static void I2C1_Init_CH32(uint32_t clock_hz)
 *
 * Returns: Nothing 
 *
 * Description: Function to initialize the I2C bus  
 * 
 * Usage: I2C1_Init_CH32(100000)
 ******************************************************************************/
static void I2C1_Init_CH32(uint32_t clock_hz)
{
    GPIO_InitTypeDef gpio = {0};
    I2C_InitTypeDef  i2c  = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /* PC2=SCL, PC1=SDA, AF-OD, 50 MHz */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;

    gpio.GPIO_Pin = GPIO_Pin_2; GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_1; GPIO_Init(GPIOC, &gpio);

    i2c.I2C_ClockSpeed          = clock_hz;                // e.g. 100k
    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1         = 0x00;                    // don't care
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);

    /* Explicit ACK enable, same as Pallav’s code */
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}


/*******************************************************************************
 * Function: static int i2c_start(void)
 *
 * Returns: Wait event status 
 *
 * Description: START/RESTART/STOP  
 * 
 * Usage: i2c_start()
 ******************************************************************************/
static int i2c_start(void)
{
    if (!wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 20)) {
        i2c_bus_soft_recover();
        if (!wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 20))
            return 0;
    }
    I2C_GenerateSTART(I2C1, ENABLE);
    return wait_event(I2C_EVENT_MASTER_MODE_SELECT, 20);  // EV5
}

/*******************************************************************************
 * Function: static int i2c_restart(void)
 *
 * Returns: Wait event status 
 *
 * Description: Proper repeated start, don't wait for BUSY to clear  
 * 
 * Usage: i2c_start()
 ******************************************************************************/
static int i2c_restart(void)
{
    I2C_GenerateSTART(I2C1, ENABLE);
    return wait_event(I2C_EVENT_MASTER_MODE_SELECT, 20);  // EV5
}

/*******************************************************************************
 * Function: static int i2c_restart(void)
 *
 * Returns: Nothing 
 *
 * Description: Stops the I2C bus 
 * 
 * Usage: i2c_stop()
 ******************************************************************************/
static void i2c_stop(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    (void)wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 10);
}

/*******************************************************************************
 * Function: static int addr_tx(uint8_t ctrl8)
 *
 * Returns: Wait event status  
 *
 * Description: Address pahse using 8-bit control byte vis Send7bitAddress
 * 
 * Usage: if (!addr_tx(ctrl8)) { i2c_stop(); return 0; }
 ******************************************************************************/
static int addr_tx(uint8_t ctrl8)
{
    I2C_Send7bitAddress(I2C1, ctrl8, I2C_Direction_Transmitter);
    return wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, 20); // EV6
}


/*******************************************************************************
 * Function: static int addr_rx(uint8_t ctrl8)
 *
 * Returns: Wait event status 
 *
 * Description: Address rx
 * 
 * Usage: if (!addr_rx(ctrl8)) { i2c_stop(); return 0; }
 ******************************************************************************/
static int addr_rx(uint8_t ctrl8)
{
    I2C_Send7bitAddress(I2C1, ctrl8, I2C_Direction_Receiver);
    return wait_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, 20);    // EV6
}


/*******************************************************************************
 * Function: static int send_byte(uint8_t b)
 *
 * Returns: Wait event status 
 *
 * Description: Event-based daat byte helpers 
 * 
 * Usage:   if (!send_byte(data)){ i2c_stop(); return 0; }
 ******************************************************************************/
static int send_byte(uint8_t b)
{
    I2C_SendData(I2C1, b);
    return wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, 20);          // EV8_2
}

/*******************************************************************************
 * Function: static int send_byte(uint8_t b)
 *
 * Returns: Status 
 *
 * Description: 24LC16B single-byte write simple tWR relay, no polling 
 * 
 * Usage: if (!eeprom_write_byte(addr, wr))
 ******************************************************************************/
static int eeprom_write_byte(uint16_t word_addr, uint8_t data)
{
    uint8_t ctrl8 = eeprom_ctrl(word_addr);
    uint8_t mem   = (uint8_t)(word_addr & 0xFF);

    /* START + ctrlW + mem + data + STOP */
    if (!i2c_start()) return 0;
    if (!addr_tx(ctrl8)) { i2c_stop(); return 0; }
    if (!send_byte(mem)) { i2c_stop(); return 0; }
    if (!send_byte(data)){ i2c_stop(); return 0; }
    i2c_stop();

    /* 24LC16B write-cycle time ~5ms (tWR). Wait slightly over and report success. */
    Delay_Ms(6);
    return 1;
}

/*******************************************************************************
 * Function: static int eeprom_read_byte(uint16_t word_addr, uint8_t *out)
 *
 * Returns: Status 
 *
 * Description: 24LC16B single-byte random read 
 * 
 * Usage: if (!eeprom_read_byte(addr, &rd))
 ******************************************************************************/
static int eeprom_read_byte(uint16_t word_addr, uint8_t *out)
{
    uint8_t ctrl8 = eeprom_ctrl(word_addr);
    uint8_t mem   = (uint8_t)(word_addr & 0xFF);

    /* Send memory address */
    if (!i2c_start()) return 0;
    if (!addr_tx(ctrl8)) { i2c_stop(); return 0; }
    if (!send_byte(mem)) { i2c_stop(); return 0; }

    /* Repeated START + same ctrl (Receiver dir) — use i2c_restart() */
    if (!i2c_restart()) { i2c_stop(); return 0; }
    if (!addr_rx(ctrl8)) { i2c_stop(); return 0; }

    /* Read one byte: NACK then STOP */
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    if (!wait_flag_set(I2C_GetFlagStatus, I2C1, I2C_FLAG_RXNE, 20)) {
        I2C_AcknowledgeConfig(I2C1, ENABLE);
        i2c_stop();
        return 0;
    }
    *out = I2C_ReceiveData(I2C1);
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 1;
}


