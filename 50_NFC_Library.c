/*
 * File: Main.c
 * Author: Armstrong Subero
 * MCU: CH32V003F4P6 w/Int OSC @ 48 MHz, 3.3v
 * Program: 50_NFC_Library
 * Compiler: WCH Toolchain (GCC8, MounRiver Studio v2.20)
 * Program Version: 1.1
 *                  * Cleaned up code
 *                  * Added additional comments
 *                
 * Program Description: This Program allows the CH32V003F4P6 to read NFC 
 *                      cards with the PN532 NFC communications module
 *                      the device is intended to read NTAG/Ultralight Tags
 *                      this program read the card blocks and extract the data
 * 
 * Hardware Description: An Elechouse V3 NFC module is connected to the 
 *                       CH32V003F4P6 as follows:
 *           
 *                       VCC -> 5v
 *                       GND -> Gnd
 *                       SDA -> SDA
 *                       SCL -> SCL
 *
 *                       Additionally a WCH Link-E is used to read the card data and 
 *                       dump it via UART 
 *
 *
 * Created August 16th, 2025, 4:00 PM
 * Updated August 18th, 2025, 2:24 PM
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


/*******************************************************************************
 *I2C Constants
 ******************************************************************************/
#define PN532_I2C_ADDRESS                   (0x48 >> 1)  // 7-bit address
#define PN532_I2C_READBIT                   (0x01)
#define PN532_I2C_BUSY                      (0x00)
#define PN532_I2C_READY                     (0x01)
#define PN532_I2C_READYTIMEOUT              (20)


/*******************************************************************************
 *PN532 Constants
 ******************************************************************************/
// PN532 Protocol Constants
#define PN532_PREAMBLE                      (0x00)
#define PN532_STARTCODE1                    (0x00)
#define PN532_STARTCODE2                    (0xFF)
#define PN532_POSTAMBLE                     (0x00)

#define PN532_HOSTTOPN532                   (0xD4)
#define PN532_PN532TOHOST                   (0xD5)

// PN532 Commands
#define PN532_COMMAND_DIAGNOSE              (0x00)
#define PN532_COMMAND_GETFIRMWAREVERSION    (0x02)
#define PN532_COMMAND_GETGENERALSTATUS      (0x04)
#define PN532_COMMAND_READREGISTER          (0x06)
#define PN532_COMMAND_WRITEREGISTER         (0x08)
#define PN532_COMMAND_READGPIO              (0x0C)
#define PN532_COMMAND_WRITEGPIO             (0x0E)
#define PN532_COMMAND_SETSERIALBAUDRATE     (0x10)
#define PN532_COMMAND_SETPARAMETERS         (0x12)
#define PN532_COMMAND_SAMCONFIGURATION      (0x14)
#define PN532_COMMAND_POWERDOWN             (0x16)
#define PN532_COMMAND_RFCONFIGURATION       (0x32)
#define PN532_COMMAND_RFREGULATIONTEST      (0x58)
#define PN532_COMMAND_INJUMPFORDEP          (0x56)
#define PN532_COMMAND_INJUMPFORPSL          (0x46)
#define PN532_COMMAND_INLISTPASSIVETARGET   (0x4A)
#define PN532_COMMAND_INATR                 (0x50)
#define PN532_COMMAND_INPSL                 (0x4E)
#define PN532_COMMAND_INDATAEXCHANGE        (0x40)
#define PN532_COMMAND_INCOMMUNICATETHRU     (0x42)
#define PN532_COMMAND_INDESELECT            (0x44)
#define PN532_COMMAND_INRELEASE             (0x52)
#define PN532_COMMAND_INSELECT              (0x54)
#define PN532_COMMAND_INAUTOPOLL            (0x60)
#define PN532_COMMAND_TGINITASTARGET        (0x8C)
#define PN532_COMMAND_TGSETGENERALBYTES     (0x92)
#define PN532_COMMAND_TGGETDATA             (0x86)
#define PN532_COMMAND_TGSETDATA             (0x8E)
#define PN532_COMMAND_TGSETMETADATA         (0x94)
#define PN532_COMMAND_TGGETINITIATORCOMMAND (0x88)
#define PN532_COMMAND_TGRESPONSETOINITIATOR (0x90)
#define PN532_COMMAND_TGGETTARGETSTATUS     (0x8A)

#define PN532_RESPONSE_INDATAEXCHANGE       (0x41)
#define PN532_RESPONSE_INLISTPASSIVETARGET  (0x4B)
#define PN532_MIFARE_ISO14443A              (0x00)

// Mifare Commands
#define MIFARE_CMD_AUTH_A                   (0x60)
#define MIFARE_CMD_AUTH_B                   (0x61)
#define MIFARE_CMD_READ                     (0x30)
#define MIFARE_CMD_WRITE                    (0xA0)
#define MIFARE_CMD_TRANSFER                 (0xB0)
#define MIFARE_CMD_DECREMENT                (0xC0)
#define MIFARE_CMD_INCREMENT                (0xC1)
#define MIFARE_CMD_STORE                    (0xC2)
#define MIFARE_ULTRALIGHT_CMD_WRITE         (0xA2)

#define MIFARE_UID_MAX_LENGTH               (10)
#define MIFARE_UID_SINGLE_LENGTH            (4)
#define MIFARE_UID_DOUBLE_LENGTH            (7)
#define MIFARE_UID_TRIPLE_LENGTH            (10)
#define MIFARE_KEY_LENGTH                   (6)
#define MIFARE_BLOCK_LENGTH                 (16)

// NTAG2xx Commands
#define NTAG2XX_BLOCK_LENGTH                (4)

// Error Codes
#define PN532_ERROR_NONE                    (0x00)
#define PN532_ERROR_TIMEOUT                 (0x01)
#define PN532_ERROR_CRC                     (0x02)
#define PN532_ERROR_PARITY                  (0x03)
#define PN532_ERROR_COLLISION_BITCOUNT      (0x04)
#define PN532_ERROR_MIFARE_FRAMING          (0x05)
#define PN532_ERROR_COLLISION_BITCOLLISION  (0x06)
#define PN532_ERROR_NOBUFS                  (0x07)
#define PN532_ERROR_RFNOBUFS                (0x09)
#define PN532_ERROR_ACTIVE_TOOSLOW          (0x0a)
#define PN532_ERROR_RFPROTO                 (0x0b)
#define PN532_ERROR_TOOHOT                  (0x0d)
#define PN532_ERROR_INTERNAL_NOBUFS         (0x0e)
#define PN532_ERROR_INVAL                   (0x10)
#define PN532_ERROR_DEP_INVALID_COMMAND     (0x12)
#define PN532_ERROR_DEP_BADDATA             (0x13)
#define PN532_ERROR_MIFARE_AUTH             (0x14)
#define PN532_ERROR_NOSECURE                (0x18)
#define PN532_ERROR_I2CBUSY                 (0x19)
#define PN532_ERROR_UIDCHECKSUM             (0x23)
#define PN532_ERROR_DEPSTATE                (0x25)
#define PN532_ERROR_HCIINVAL                (0x26)
#define PN532_ERROR_CONTEXT                 (0x27)
#define PN532_ERROR_RELEASED                (0x29)
#define PN532_ERROR_CARDSWAPPED             (0x2a)
#define PN532_ERROR_NOCARD                  (0x2b)
#define PN532_ERROR_MISMATCH                (0x2c)
#define PN532_ERROR_OVERCURRENT             (0x2d)
#define PN532_ERROR_NONAD                   (0x2e)

#define PN532_STATUS_ERROR                  (-1)
#define PN532_STATUS_OK                     (0)

// PN532 structure
typedef struct _PN532 {
    int (*reset)(void);
    int (*read_data)(uint8_t* data, uint16_t count);
    int (*write_data)(uint8_t *data, uint16_t count);
    bool (*wait_ready)(uint32_t timeout);
    int (*wakeup)(void);
    void (*log)(const char* log);
} PN532;

// No additional pins needed for I2C communication

#define PN532_FRAME_MAX_LENGTH              255
#define PN532_DEFAULT_TIMEOUT               1000

// Global buffers and PN532 instance
static PN532 pn532;
static uint8_t pn532_packetbuffer[PN532_FRAME_MAX_LENGTH + 7];
static const uint8_t PN532_ACK[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};


/*******************************************************************************
 * I2C Helper Functions
 ******************************************************************************/

static int wait_event(uint32_t evt, uint32_t ms)
{
    while (!I2C_CheckEvent(I2C1, evt)) { 
        Delay_Ms(1); 
        if (!ms--) return 0; 
    }
    return 1;
}

static int wait_flag_set(FlagStatus (*fn)(I2C_TypeDef*, uint32_t),
                         I2C_TypeDef *I2Cx, uint32_t flag, uint32_t ms)
{
    while (fn(I2Cx, flag) == RESET) { 
        Delay_Ms(1); 
        if (!ms--) return 0; 
    }
    return 1;
}

static int wait_flag_clear(FlagStatus (*fn)(I2C_TypeDef*, uint32_t),
                           I2C_TypeDef *I2Cx, uint32_t flag, uint32_t ms)
{
    while (fn(I2Cx, flag) != RESET) { 
        Delay_Ms(1); 
        if (!ms--) return 0; 
    }
    return 1;
}

static void i2c_bus_soft_recover(void)
{
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    Delay_Ms(1);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
}

static int i2c_start(void)
{
    if (!wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 20)) {
        i2c_bus_soft_recover();
        if (!wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 20))
            return 0;
    }
    I2C_GenerateSTART(I2C1, ENABLE);
    return wait_event(I2C_EVENT_MASTER_MODE_SELECT, 20);
}

static int i2c_restart(void)
{
    I2C_GenerateSTART(I2C1, ENABLE);
    return wait_event(I2C_EVENT_MASTER_MODE_SELECT, 20);
}

static void i2c_stop(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    (void)wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 10);
}

static int addr_tx(uint8_t ctrl8)
{
    I2C_Send7bitAddress(I2C1, ctrl8, I2C_Direction_Transmitter);
    return wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, 20);
}

static int addr_rx(uint8_t ctrl8)
{
    I2C_Send7bitAddress(I2C1, ctrl8, I2C_Direction_Receiver);
    return wait_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, 20);
}

static int send_byte(uint8_t b)
{
    I2C_SendData(I2C1, b);
    return wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, 20);
}


/*******************************************************************************
 * PN532 Hardware Interface Functions 
 ******************************************************************************/

static int PN532_I2C_WriteData(uint8_t *data, uint16_t count)
{
    if (!i2c_start()) return PN532_STATUS_ERROR;
    if (!addr_tx(PN532_I2C_ADDRESS << 1)) { i2c_stop(); return PN532_STATUS_ERROR; }
    
    for (uint16_t i = 0; i < count; i++) {
        if (!send_byte(data[i])) { i2c_stop(); return PN532_STATUS_ERROR; }
    }
    
    i2c_stop();
    return PN532_STATUS_OK;
}

static int PN532_I2C_ReadData(uint8_t *data, uint16_t count)
{
    uint8_t status = 0;
    
    if (!i2c_start()) return PN532_STATUS_ERROR;
    if (!addr_rx(PN532_I2C_ADDRESS << 1)) { i2c_stop(); return PN532_STATUS_ERROR; }
    
    // Read status byte first
    if (!wait_flag_set(I2C_GetFlagStatus, I2C1, I2C_FLAG_RXNE, 20)) {
        i2c_stop(); return PN532_STATUS_ERROR;
    }
    status = I2C_ReceiveData(I2C1);
    
    if (status != PN532_I2C_READY) {
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        i2c_stop();
        I2C_AcknowledgeConfig(I2C1, ENABLE);
        return PN532_STATUS_ERROR;
    }
    
    // Read data bytes
    for (uint16_t i = 0; i < count; i++) {
        if (i == count - 1) I2C_AcknowledgeConfig(I2C1, DISABLE);
        
        if (!wait_flag_set(I2C_GetFlagStatus, I2C1, I2C_FLAG_RXNE, 20)) {
            I2C_AcknowledgeConfig(I2C1, ENABLE);
            i2c_stop();
            return PN532_STATUS_ERROR;
        }
        data[i] = I2C_ReceiveData(I2C1);
    }
    
    i2c_stop();
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return PN532_STATUS_OK;
}

static bool PN532_I2C_WaitReady(uint32_t timeout_ms)
{
    uint8_t status = 0;
    uint32_t attempts = timeout_ms / 10;
    
    for (uint32_t i = 0; i < attempts; i++) {
        if (!i2c_start()) { i2c_stop(); Delay_Ms(5); continue; }
        if (!addr_rx(PN532_I2C_ADDRESS << 1)) { i2c_stop(); Delay_Ms(5); continue; }
        
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        if (wait_flag_set(I2C_GetFlagStatus, I2C1, I2C_FLAG_RXNE, 20)) {
            status = I2C_ReceiveData(I2C1);
        }
        I2C_GenerateSTOP(I2C1, ENABLE);
        I2C_AcknowledgeConfig(I2C1, ENABLE);
        
        if (status == PN532_I2C_READY) return true;
        Delay_Ms(10);
    }
    return false;
}

static int PN532_I2C_Wakeup(void)
{
    // No wakeup needed for I2C mode
    return PN532_STATUS_OK;
}

static void PN532_I2C_Log(const char* log)
{
    printf("%s\r\n", log);
}

static void PN532_I2C_Init_CH32(PN532* pn532, uint32_t clock_hz)
{
    // Set up function pointers like Arduino version
    pn532->reset = NULL;  // No reset in I2C mode
    pn532->read_data = PN532_I2C_ReadData;
    pn532->write_data = PN532_I2C_WriteData;
    pn532->wait_ready = PN532_I2C_WaitReady;
    pn532->wakeup = PN532_I2C_Wakeup;
    pn532->log = PN532_I2C_Log;
    
    // Initialize I2C hardware
    GPIO_InitTypeDef gpio = {0};
    I2C_InitTypeDef  i2c  = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // PC2=SCL, PC1=SDA, AF-OD, 50 MHz
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;

    gpio.GPIO_Pin = GPIO_Pin_2; GPIO_Init(GPIOC, &gpio); // SCL
    gpio.GPIO_Pin = GPIO_Pin_1; GPIO_Init(GPIOC, &gpio); // SDA

    i2c.I2C_ClockSpeed          = clock_hz;
    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1         = 0x00;
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);

    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

static int PN532_WriteData(uint8_t *data, uint16_t count)
{
    if (!i2c_start()) return PN532_STATUS_ERROR;
    if (!addr_tx(PN532_I2C_ADDRESS << 1)) { i2c_stop(); return PN532_STATUS_ERROR; }
    
    for (uint16_t i = 0; i < count; i++) {
        if (!send_byte(data[i])) { i2c_stop(); return PN532_STATUS_ERROR; }
    }
    
    i2c_stop();
    return PN532_STATUS_OK;
}

static int PN532_ReadData(uint8_t *data, uint16_t count)
{
    uint8_t status = 0;
    
    if (!i2c_start()) return PN532_STATUS_ERROR;
    if (!addr_rx(PN532_I2C_ADDRESS << 1)) { i2c_stop(); return PN532_STATUS_ERROR; }
    
    // Read status byte first
    if (!wait_flag_set(I2C_GetFlagStatus, I2C1, I2C_FLAG_RXNE, 20)) {
        i2c_stop(); return PN532_STATUS_ERROR;
    }
    status = I2C_ReceiveData(I2C1);
    
    if (status != PN532_I2C_READY) {
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        i2c_stop();
        I2C_AcknowledgeConfig(I2C1, ENABLE);
        return PN532_STATUS_ERROR;
    }
    
    // Read data bytes
    for (uint16_t i = 0; i < count; i++) {
        if (i == count - 1) I2C_AcknowledgeConfig(I2C1, DISABLE);
        
        if (!wait_flag_set(I2C_GetFlagStatus, I2C1, I2C_FLAG_RXNE, 20)) {
            I2C_AcknowledgeConfig(I2C1, ENABLE);
            i2c_stop();
            return PN532_STATUS_ERROR;
        }
        data[i] = I2C_ReceiveData(I2C1);
    }
    
    i2c_stop();
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return PN532_STATUS_OK;
}

static bool PN532_WaitReady(uint32_t timeout_ms)
{
    uint8_t status = 0;
    uint32_t attempts = timeout_ms / 10;
    
    for (uint32_t i = 0; i < attempts; i++) {
        if (!i2c_start()) { i2c_stop(); Delay_Ms(5); continue; }
        if (!addr_rx(PN532_I2C_ADDRESS << 1)) { i2c_stop(); Delay_Ms(5); continue; }
        
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        if (wait_flag_set(I2C_GetFlagStatus, I2C1, I2C_FLAG_RXNE, 20)) {
            status = I2C_ReceiveData(I2C1);
        }
        I2C_GenerateSTOP(I2C1, ENABLE);
        I2C_AcknowledgeConfig(I2C1, ENABLE);
        
        if (status == PN532_I2C_READY) return true;
        Delay_Ms(10);
    }
    return false;
}

static void PN532_Log(const char* log)
{
    printf("%s\r\n", log);
}

/*******************************************************************************
 * PN532 Protocol Functions
 ******************************************************************************/

static int PN532_WriteFrame(uint8_t* data, uint16_t length)
{
    if (length > PN532_FRAME_MAX_LENGTH || length < 1) {
        return PN532_STATUS_ERROR;
    }
    
    uint8_t frame[PN532_FRAME_MAX_LENGTH + 7];
    uint8_t checksum = 0;
    
    frame[0] = PN532_PREAMBLE;
    frame[1] = PN532_STARTCODE1;
    frame[2] = PN532_STARTCODE2;
    
    for (uint8_t i = 0; i < 3; i++) {
        checksum += frame[i];
    }
    
    frame[3] = length & 0xFF;
    frame[4] = (~length + 1) & 0xFF;
    
    for (uint8_t i = 0; i < length; i++) {
        frame[5 + i] = data[i];
        checksum += data[i];
    }
    
    frame[length + 5] = ~checksum & 0xFF;
    frame[length + 6] = PN532_POSTAMBLE;
    
    if (pn532.write_data(frame, length + 7) != PN532_STATUS_OK) {
        return PN532_STATUS_ERROR;
    }
    
    return PN532_STATUS_OK;
}

static int PN532_ReadFrame(uint8_t* response, uint16_t length)
{
    uint8_t checksum = 0;
    
    // Read frame with expected length of data
    if (pn532.read_data(pn532_packetbuffer, length + 7) != PN532_STATUS_OK) {
        return PN532_STATUS_ERROR;
    }
    
    // Swallow all the 0x00 values that precede 0xFF
    uint8_t offset = 0;
    while (pn532_packetbuffer[offset] == 0x00) {
        offset += 1;
        if (offset >= length + 8) {
            PN532_Log("Response frame preamble does not contain 0x00FF!");
            return PN532_STATUS_ERROR;
        }
    }
    
    if (pn532_packetbuffer[offset] != 0xFF) {
        PN532_Log("Response frame preamble does not contain 0x00FF!");
        return PN532_STATUS_ERROR;
    }
    offset += 1;
    
    if (offset >= length + 8) {
        PN532_Log("Response contains no data!");
        return PN532_STATUS_ERROR;
    }
    
    // Check length & length checksum match
    uint8_t frame_len = pn532_packetbuffer[offset];
    if (((frame_len + pn532_packetbuffer[offset+1]) & 0xFF) != 0) {
        PN532_Log("Response length checksum did not match length!");
        return PN532_STATUS_ERROR;
    }
    
    // Check frame checksum value matches bytes
    for (uint8_t i = 0; i < frame_len + 1; i++) {
        checksum += pn532_packetbuffer[offset + 2 + i];
    }
    checksum &= 0xFF;
    
    if (checksum != 0) {
        PN532_Log("Response checksum did not match expected checksum");
        return PN532_STATUS_ERROR;
    }
    
    // Return frame data
    for (uint8_t i = 0; i < frame_len; i++) {
        response[i] = pn532_packetbuffer[offset + 2 + i];
    }
    
    return frame_len;
}

static int PN532_CallFunction(uint8_t command, uint8_t* response, uint16_t response_length,
                             uint8_t* params, uint16_t params_length, uint32_t timeout)
{
    // Build frame data with command and parameters using shared buffer
    pn532_packetbuffer[0] = PN532_HOSTTOPN532;
    pn532_packetbuffer[1] = command & 0xFF;
    
    for (uint8_t i = 0; i < params_length; i++) {
        pn532_packetbuffer[2 + i] = params[i];
    }
    
    // Send frame and wait for response
    if (PN532_WriteFrame(pn532_packetbuffer, params_length + 2) != PN532_STATUS_OK) {
        PN532_Log("Failed to write frame");
        return PN532_STATUS_ERROR;
    }
    
    if (!pn532.wait_ready(timeout)) {
        return PN532_STATUS_ERROR;
    }
    
    // Verify ACK response and wait to be ready for function response
    if (pn532.read_data(pn532_packetbuffer, sizeof(PN532_ACK)) != PN532_STATUS_OK) {
        return PN532_STATUS_ERROR;
    }
    
    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++) {
        if (PN532_ACK[i] != pn532_packetbuffer[i]) {
            pn532.log("Did not receive expected ACK from PN532!");
            return PN532_STATUS_ERROR;
        }
    }
    
    if (!pn532.wait_ready(timeout)) {
        return PN532_STATUS_ERROR;
    }
    
    // Read response bytes
    int frame_len = PN532_ReadFrame(pn532_packetbuffer, response_length + 2);
    if (frame_len < 0) {
        return PN532_STATUS_ERROR;
    }
    
    // Check that response is for the called function
    if (!((pn532_packetbuffer[0] == PN532_PN532TOHOST) && (pn532_packetbuffer[1] == (command+1)))) {
        pn532.log("Received unexpected command response!");
        return PN532_STATUS_ERROR;
    }
    
    // Return response data
    for (uint8_t i = 0; i < response_length && i < (frame_len - 2); i++) {
        response[i] = pn532_packetbuffer[i + 2];
    }
    
    return frame_len - 2;
}

/*******************************************************************************
 * PN532 High-Level Functions
 ******************************************************************************/
int PN532_GetFirmwareVersion(uint8_t* version)
{
    if (PN532_CallFunction(PN532_COMMAND_GETFIRMWAREVERSION, version, 4, 
                          NULL, 0, 500) == PN532_STATUS_ERROR) {
        pn532.log("Failed to detect the PN532");
        return PN532_STATUS_ERROR;
    }
    return PN532_STATUS_OK;
}

int PN532_SamConfiguration(void)
{
    uint8_t params[] = {0x01, 0x14, 0x01};
    if (PN532_CallFunction(PN532_COMMAND_SAMCONFIGURATION, NULL, 0, 
                          params, sizeof(params), PN532_DEFAULT_TIMEOUT) < 0) {
        return PN532_STATUS_ERROR;
    }
    return PN532_STATUS_OK;
}

int PN532_ReadPassiveTarget(uint8_t* response, uint8_t card_baud, uint32_t timeout)
{
    uint8_t params[] = {0x01, card_baud};
    uint8_t buff[19];
    
    int length = PN532_CallFunction(PN532_COMMAND_INLISTPASSIVETARGET, buff, sizeof(buff),
                                   params, sizeof(params), timeout);
    if (length < 0) {
        return PN532_STATUS_ERROR;
    }
    
    // Check only 1 card with up to a 7 byte UID is present
    if (buff[0] != 0x01) {
        pn532.log("More than one card detected!");
        return PN532_STATUS_ERROR;
    }
    
    if (buff[5] > 7) {
        pn532.log("Found card with unexpectedly long UID!");
        return PN532_STATUS_ERROR;
    }
    
    for (uint8_t i = 0; i < buff[5]; i++) {
        response[i] = buff[6 + i];
    }
    
    return buff[5];
}

int PN532_MifareClassicAuthenticateBlock(uint8_t* uid, uint8_t uid_length, 
                                        uint16_t block_number, uint16_t key_number, uint8_t* key)
{
    uint8_t response[1] = {0xFF};
    uint8_t params[3 + MIFARE_UID_MAX_LENGTH + MIFARE_KEY_LENGTH];
    
    params[0] = 0x01;
    params[1] = key_number & 0xFF;
    params[2] = block_number & 0xFF;
    
    // Copy key
    for (uint8_t i = 0; i < MIFARE_KEY_LENGTH; i++) {
        params[3 + i] = key[i];
    }
    
    // Copy UID
    for (uint8_t i = 0; i < uid_length; i++) {
        params[3 + MIFARE_KEY_LENGTH + i] = uid[i];
    }
    
    PN532_CallFunction(PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response),
                      params, 3 + MIFARE_KEY_LENGTH + uid_length, PN532_DEFAULT_TIMEOUT);
    return response[0];
}

int PN532_MifareClassicReadBlock(uint8_t* response, uint16_t block_number)
{
    uint8_t params[] = {0x01, MIFARE_CMD_READ, block_number & 0xFF};
    uint8_t buff[MIFARE_BLOCK_LENGTH + 1];
    
    PN532_CallFunction(PN532_COMMAND_INDATAEXCHANGE, buff, sizeof(buff),
                      params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    
    if (buff[0] != PN532_ERROR_NONE) {
        return buff[0];
    }
    
    for (uint8_t i = 0; i < MIFARE_BLOCK_LENGTH; i++) {
        response[i] = buff[i + 1];
    }
    
    return buff[0];
}

int PN532_MifareClassicWriteBlock(uint8_t* data, uint16_t block_number)
{
    uint8_t params[MIFARE_BLOCK_LENGTH + 3];
    uint8_t response[1];
    
    params[0] = 0x01;
    params[1] = MIFARE_CMD_WRITE;
    params[2] = block_number & 0xFF;
    
    for (uint8_t i = 0; i < MIFARE_BLOCK_LENGTH; i++) {
        params[3 + i] = data[i];
    }
    
    PN532_CallFunction(PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response),
                      params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    return response[0];
}

int PN532_Ntag2xxReadBlock(uint8_t* response, uint16_t block_number)
{
    uint8_t params[] = {0x01, MIFARE_CMD_READ, block_number & 0xFF};
    uint8_t buff[MIFARE_BLOCK_LENGTH + 1];
    
    PN532_CallFunction(PN532_COMMAND_INDATAEXCHANGE, buff, sizeof(buff),
                      params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    
    if (buff[0] != PN532_ERROR_NONE) {
        return buff[0];
    }
    
    // NTAG2xx only returns first 4 bytes
    for (uint8_t i = 0; i < NTAG2XX_BLOCK_LENGTH; i++) {
        response[i] = buff[i + 1];
    }
    
    return buff[0];
}

int PN532_Ntag2xxWriteBlock(uint8_t* data, uint16_t block_number)
{
    uint8_t params[NTAG2XX_BLOCK_LENGTH + 3];
    uint8_t response[1];
    
    params[0] = 0x01;
    params[1] = MIFARE_ULTRALIGHT_CMD_WRITE;
    params[2] = block_number & 0xFF;
    
    for (uint8_t i = 0; i < NTAG2XX_BLOCK_LENGTH; i++) {
        params[3 + i] = data[i];
    }
    
    PN532_CallFunction(PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response),
                      params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    return response[0];
}

int PN532_ReadGpio(uint8_t* pins_state)
{
    return PN532_CallFunction(PN532_COMMAND_READGPIO, pins_state, 3,
                             NULL, 0, PN532_DEFAULT_TIMEOUT);
}

bool PN532_ReadGpioP(uint8_t pin_number)
{
    uint8_t pins_state[3];
    if (PN532_CallFunction(PN532_COMMAND_READGPIO, pins_state, sizeof(pins_state),
                          NULL, 0, PN532_DEFAULT_TIMEOUT) < 0) {
        return false;
    }
    
    if ((pin_number >= 30) && (pin_number <= 37)) {
        return (pins_state[0] >> (pin_number - 30)) & 1 ? true : false;
    }
    if ((pin_number >= 70) && (pin_number <= 77)) {
        return (pins_state[1] >> (pin_number - 70)) & 1 ? true : false;
    }
    return false;
}

bool PN532_ReadGpioI(uint8_t pin_number)
{
    uint8_t pins_state[3];
    if (PN532_CallFunction(PN532_COMMAND_READGPIO, pins_state, sizeof(pins_state),
                          NULL, 0, PN532_DEFAULT_TIMEOUT) < 0) {
        return false;
    }
    
    if (pin_number <= 7) {
        return (pins_state[2] >> pin_number) & 1 ? true : false;
    }
    return false;
}

int PN532_WriteGpio(uint8_t* pins_state)
{
    uint8_t params[2];
    params[0] = 0x80 | pins_state[0];
    params[1] = 0x80 | pins_state[1];
    return PN532_CallFunction(PN532_COMMAND_WRITEGPIO, NULL, 0,
                             params, sizeof(params), PN532_DEFAULT_TIMEOUT);
}

int PN532_WriteGpioP(uint8_t pin_number, bool pin_state)
{
    uint8_t pins_state[3];
    uint8_t params[2];
    
    if (PN532_ReadGpio(pins_state) == PN532_STATUS_ERROR) {
        return PN532_STATUS_ERROR;
    }
    
    if ((pin_number >= 30) && (pin_number <= 37)) {
        if (pin_state) {
            params[0] = 0x80 | pins_state[0] | (1 << (pin_number - 30));
        } else {
            params[0] = (0x80 | pins_state[0]) & ~(1 << (pin_number - 30));
        }
        params[1] = 0x00;   // leave p7 unchanged
    } else if ((pin_number >= 70) && (pin_number <= 77)) {
        if (pin_state) {
            params[1] = 0x80 | pins_state[1] | (1 << (pin_number - 70));
        } else {
            params[1] = (0x80 | pins_state[1]) & ~(1 << (pin_number - 70));
        }
        params[0] = 0x00;   // leave p3 unchanged
    } else {
        return PN532_STATUS_ERROR;
    }
    
    return PN532_CallFunction(PN532_COMMAND_WRITEGPIO, NULL, 0,
                             params, sizeof(params), PN532_DEFAULT_TIMEOUT);
}


int PN532_Init(void)
{
    // Initialize PN532 structure and I2C - exactly like Arduino
    PN532_I2C_Init_CH32(&pn532, 100000);  // 100kHz I2C
    return PN532_STATUS_OK;
}



/*******************************************************************************
 * PN532 Application Functions 
 ******************************************************************************/

void print_hex_array(uint8_t* data, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++) {
        if (data[i] <= 0xF) printf("0");
        printf("%02X", data[i]);
        if (i < length - 1) printf(" ");
    }
}

void print_ascii_representation(uint8_t* data, uint8_t length)
{
    printf(" | ");
    for (uint8_t i = 0; i < length; i++) {
        if (data[i] >= 32 && data[i] <= 126) {
            printf("%c", (char)data[i]);
        } else {
            printf(".");
        }
    }
}

// Example function to read NTAG card
void read_ntag_card(uint8_t* uid, uint8_t uid_len)
{
    printf("\n=== Reading as NTAG/Ultralight ===\r\n");
    
    bool success = false;
    uint8_t blocks_read = 0;
    uint8_t buffer[NTAG2XX_BLOCK_LENGTH];
    
    // Try to read blocks 0-15 (typical NTAG range)
    for (uint8_t block = 0; block < 16; block++) {
        int error = PN532_Ntag2xxReadBlock(buffer, block);
        
        if (error == PN532_ERROR_NONE) {
            success = true;
            blocks_read++;
            
            printf("Block %2d: ", block);
            print_hex_array(buffer, NTAG2XX_BLOCK_LENGTH);
            print_ascii_representation(buffer, NTAG2XX_BLOCK_LENGTH);
            
            // Add block descriptions
            if (block == 0) {
                printf(" [UID/MFG]");
            } else if (block == 1) {
                printf(" [UID cont]");
            } else if (block == 2) {
                printf(" [Internal]");
            } else if (block == 3) {
                printf(" [CC/Locks]");
            } else if (block >= 4 && block <= 10) {
                printf(" [Data]");
            }
            
            printf("\r\n");
        } else {
            printf("Block %d failed: 0x%02X\r\n", block, error);
            
            // Stop on first error (likely reached end of card)
            if (block > 4) {
                printf("Reached end of readable memory\r\n");
                break;
            }
        }
    }
    
    if (success) {
        printf("\nSUCCESS! Read %d blocks as NTAG/Ultralight\r\n", blocks_read);
        printf("Your card is NTAG, not MIFARE Classic!\r\n");
        printf("No authentication keys needed.\r\n");
    } else {
        printf("FAILED: Card doesn't respond to NTAG commands\r\n");
        printf("This might be a different card type entirely\r\n");
    }
}

// Example function to read MIFARE Classic card
void read_mifare_classic_card(uint8_t* uid, uint8_t uid_len)
{
    printf("\n=== Reading as MIFARE Classic ===\r\n");
    
    uint8_t key_a[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t buffer[MIFARE_BLOCK_LENGTH];
    
    // Try to authenticate and read sector 0
    for (uint8_t block = 0; block < 4; block++) {
        // Authenticate block (only needed once per sector)
        if (block == 0 || (block % 4) == 0) {
            int auth_result = PN532_MifareClassicAuthenticateBlock(uid, uid_len, block, 
                                                                  MIFARE_CMD_AUTH_A, key_a);
            if (auth_result != PN532_ERROR_NONE) {
                printf("Authentication failed for block %d: 0x%02X\r\n", block, auth_result);
                continue;
            }
        }
        
        // Read block
        int error = PN532_MifareClassicReadBlock(buffer, block);
        if (error == PN532_ERROR_NONE) {
            printf("Block %2d: ", block);
            print_hex_array(buffer, MIFARE_BLOCK_LENGTH);
            print_ascii_representation(buffer, MIFARE_BLOCK_LENGTH);
            
            if (block == 0) {
                printf(" [Manufacturer]");
            } else if ((block + 1) % 4 == 0) {
                printf(" [Sector Trailer]");
            } else {
                printf(" [Data]");
            }
            
            printf("\r\n");
        } else {
            printf("Read failed for block %d: 0x%02X\r\n", block, error);
        }
    }
}

/*******************************************************************************
 *  PN532 Functions for Handling Text and Data Extraction 
 ******************************************************************************/

#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif

#define NDEF_TEXT_MAX_LEN 128

// --- Globals to hold last dump for variable extraction ---
static uint8_t g_last_dump[64];
static size_t  g_last_dump_len = 0;

// -----------------------------
// TLV finder + NDEF text parser
// -----------------------------

// Find NDEF TLV (type 0x03) starting at page 4 (offset 16) and return pointer + length.
static int tlv_find_ndef(const uint8_t *buf, size_t len, const uint8_t **ndef, size_t *ndef_len) {
    if (!buf || len < 18) return -1;

    // TLVs live in user memory, which begins at page 4 => byte offset 16
    size_t i = 16;  // start at page 4
    while (i < len) {
        uint8_t t = buf[i++];

        if (t == 0x00) {                 // NULL TLV padding
            continue;
        } else if (t == 0xFE) {          // Terminator TLV
            break;
        }

        // Need at least a length byte
        if (i >= len) return -1;

        size_t L = 0;
        if (buf[i] == 0xFF) {            // 3-byte length format
            if (i + 2 >= len) return -1;
            L = ((size_t)buf[i+1] << 8) | buf[i+2];
            i += 3;
        } else {                          // 1-byte length
            L = buf[i++];
        }

        if (i + L > len) return -1;       // bounds check

        if (t == 0x03) {                  // NDEF Message TLV
            *ndef = &buf[i];
            *ndef_len = L;
            return 0;
        }

        // Skip unknown TLV payload and continue
        i += L;
    }
    return -1;
}

// -----------------------------
// Extract text from "T" Record
// -----------------------------

static int ndef_get_text(const uint8_t *ndef, size_t ndef_len, char *out, size_t out_size) {
    if (!ndef || ndef_len < 3 || !out || out_size == 0) return -1;

    uint8_t hdr = ndef[0];      // Expect usually 0xD1: MB=1, ME=1, SR=1, TNF=0x01
    uint8_t type_len = ndef[1];

    size_t idx = 2;
    size_t payload_len = 0;

    // Short Record?
    if (hdr & 0x10) { // SR bit
        if (idx >= ndef_len) return -1;
        payload_len = ndef[idx++];
    } else {
        if (idx + 3 >= ndef_len) return -1;
        payload_len = ((size_t)ndef[idx] << 24) | ((size_t)ndef[idx+1] << 16) |
                      ((size_t)ndef[idx+2] << 8) | ndef[idx+3];
        idx += 4;
    }

    // ID Length present?
    if (hdr & 0x08) { // IL bit
        if (idx >= ndef_len) return -1;
        uint8_t id_len = ndef[idx++];
        if (idx + id_len > ndef_len) return -1;
        idx += id_len;
    }

    // Type
    if (idx + type_len > ndef_len) return -1;
    const uint8_t *type = &ndef[idx];
    idx += type_len;

    // Payload
    if (idx + payload_len > ndef_len) return -1;
    const uint8_t *payload = &ndef[idx];

    // Must be Well-known Text: type "T"
    if (!((hdr & 0x07) == 0x01 && type_len == 1 && type[0] == 'T')) return -1;
    if (payload_len == 0) return -1;

    uint8_t status   = payload[0];
    uint8_t lang_len = status & 0x3F; // lower 6 bits = language code length
    if ((size_t)1 + lang_len > payload_len) return -1;

    const uint8_t *text = &payload[1 + lang_len];
    size_t text_len = payload_len - 1 - lang_len;

    size_t copy_len = MIN(text_len, out_size - 1);
    memcpy(out, text, copy_len);
    out[copy_len] = '\0';
    return 0;
}

// -----------------------------
// "Pretty Printer"
// -----------------------------
static void print_ndef_text_from_dump(const uint8_t *dump, size_t dump_len) {
    const uint8_t *ndef = NULL;
    size_t ndef_len = 0;
    if (tlv_find_ndef(dump, dump_len, &ndef, &ndef_len) == 0) {
        char text[NDEF_TEXT_MAX_LEN];
        if (ndef_get_text(ndef, ndef_len, text, sizeof(text)) == 0) {
            printf("NDEF Text (lang=en?): %s\r\n", text);
        } else {
            printf("NDEF present, but not a Text record (or parse error)\r\n");
        }
    } else {
        printf("No NDEF TLV found\r\n");
    }
}

// -----------------------------
// Return Text into Caller Buf
// -----------------------------

static int get_last_card_text(char *out, size_t out_sz) {
    if (!out || out_sz == 0 || g_last_dump_len == 0) return 0;

    const uint8_t *ndef = NULL;
    size_t ndef_len = 0;
    if (tlv_find_ndef(g_last_dump, g_last_dump_len, &ndef, &ndef_len) != 0) return 0;
    return (ndef_get_text(ndef, ndef_len, out, out_sz) == 0) ? 1 : 0;
}

// ----------------------------------------
// - Save dump to globals for later retrieval
// - Call pretty printer
// ----------------------------------------
void read_ntag_card_extract(uint8_t* uid, uint8_t uid_len)
{
    printf("\n=== Reading as NTAG/Ultralight ===\r\n");

    bool success = false;
    uint8_t blocks_read = 0;
    uint8_t buffer[NTAG2XX_BLOCK_LENGTH];

    // Collect a 64-byte dump of pages 0..15 as we go
    uint8_t dump[16 * NTAG2XX_BLOCK_LENGTH] = {0};

    // Try to read blocks 0-15 (typical NTAG range)
    for (uint8_t block = 0; block < 16; block++) {
        int error = PN532_Ntag2xxReadBlock(buffer, block);

        if (error == PN532_ERROR_NONE) {
            success = true;
            blocks_read++;

            // Copy 4-byte page into dump
            dump[block * NTAG2XX_BLOCK_LENGTH + 0] = buffer[0];
            dump[block * NTAG2XX_BLOCK_LENGTH + 1] = buffer[1];
            dump[block * NTAG2XX_BLOCK_LENGTH + 2] = buffer[2];
            dump[block * NTAG2XX_BLOCK_LENGTH + 3] = buffer[3];

            printf("Block %2d: ", block);
            print_hex_array(buffer, NTAG2XX_BLOCK_LENGTH);
            print_ascii_representation(buffer, NTAG2XX_BLOCK_LENGTH);

            // Add block descriptions
            if (block == 0) {
                printf(" [UID/MFG]");
            } else if (block == 1) {
                printf(" [UID cont]");
            } else if (block == 2) {
                printf(" [Internal]");
            } else if (block == 3) {
                printf(" [CC/Locks]");
            } else if (block >= 4 && block <= 10) {
                printf(" [Data]");
            }

            printf("\r\n");
        } else {
            printf("Block %d failed: 0x%02X\r\n", block, error);

            // Stop on first error (likely reached end of card)
            if (block > 4) {
                printf("Reached end of readable memory\r\n");
                break;
            }
        }
    }

    if (success) {
        printf("\nSUCCESS! Read %d blocks as NTAG/Ultralight\r\n", blocks_read);
        printf("Your card is NTAG, not MIFARE Classic!\r\n");
        printf("No authentication keys needed.\r\n");

        // Save last dump for variable extraction
        memcpy(g_last_dump, dump, sizeof(dump));
        g_last_dump_len = sizeof(dump);

        // Print decoded text (keeps your console output)
        print_ndef_text_from_dump(dump, sizeof(dump));

    } else {
        printf("FAILED: Card doesn't respond to NTAG commands\r\n");
        printf("This might be a different card type entirely\r\n");
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

#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif

    printf("SystemClk:%ld\r\n", SystemCoreClock);
    printf("CH32V003 PN532 NFC Reader\r\n");
    printf("=========================\r\n");

    // Initialize PN532
    if (PN532_Init() != PN532_STATUS_OK) {
        printf("Failed to initialize PN532!\r\n");
        while(1);
    }

    // Get firmware version
    uint8_t version[4];
    if (PN532_GetFirmwareVersion(version) == PN532_STATUS_OK) {
        printf("Found PN532 firmware: %d.%d\r\n", version[1], version[2]);
    } else {
        printf("Failed to get PN532 firmware version!\r\n");
        while(1);
    }

    // Configure SAM
    if (PN532_SamConfiguration() != PN532_STATUS_OK) {
        printf("Failed to configure PN532 SAM!\r\n");
        while(1);
    }

    printf("Place your card (BC 4E 3B 06) on reader...\r\n");

    uint8_t uid[MIFARE_UID_MAX_LENGTH];
    int32_t uid_len = 0;

    // Initial card detection loop
    while (1) {
        uid_len = PN532_ReadPassiveTarget(uid, PN532_MIFARE_ISO14443A, 1000);
        if (uid_len > 0) 
        {
            printf("\nCard UID: ");
            print_hex_array(uid, uid_len);
            printf("\r\n");

            // Read + dump blocks, and decode NDEF to console
            read_ntag_card_extract(uid, uid_len);

            // Also get the text as a variable you can reuse
            char card_text[NDEF_TEXT_MAX_LEN] = {0};
            if (get_last_card_text(card_text, sizeof(card_text))) 
            {
                printf("Extracted (variable): %s\r\n", card_text);
                // Clear stored dump after use so next read starts fresh
                memset(g_last_dump, 0, sizeof(g_last_dump));
                g_last_dump_len = 0;
            } 
            else
            {
                printf("No NDEF text available for variable\r\n");
            }
            break;
        } else {
            printf(".");
        }
    }

    // Loop for re-present
    while (1) {
        Delay_Ms(1000);

        // Check if card is still present
        uid_len = PN532_ReadPassiveTarget(uid, PN532_MIFARE_ISO14443A, 100);
        if (uid_len <= 0) {
            printf("\nCard removed. Place card to read again...\r\n");
            while (uid_len <= 0) {
                uid_len = PN532_ReadPassiveTarget(uid, PN532_MIFARE_ISO14443A, 1000);
                if (uid_len <= 0) {
                    printf(".");
                }
            }
            printf("\nNew card detected!\r\n");

            // Read + dump again
            read_ntag_card_extract(uid, uid_len);

            // Refresh the variable with latest card’s text
            char card_text[NDEF_TEXT_MAX_LEN] = {0};
            if (get_last_card_text(card_text, sizeof(card_text))) {
                printf("Extracted (variable): %s\r\n", card_text);
                // Clear stored dump after use
                memset(g_last_dump, 0, sizeof(g_last_dump));
                g_last_dump_len = 0;
            } else {
                printf("No NDEF text available for variable\r\n");
            }
        }
    }
}
