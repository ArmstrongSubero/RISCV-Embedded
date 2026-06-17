/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 */

/**
 * @file rovari_pn532.c
 * @brief PN532 NFC reader for CH32V003.
 * @req REQ-ROVARI-PN532-0010
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ch32v00x.h"
#include "ch32v00x_i2c.h"
#include "ch32v00x_gpio.h"
#include "ch32v00x_rcc.h"
#include "debug.h"
#include "rovari_pn532.h"

/* PN532 protocol constants */
#define PN532_PREAMBLE           0x00
#define PN532_STARTCODE1         0x00
#define PN532_STARTCODE2         0xFF
#define PN532_POSTAMBLE          0x00
#define PN532_HOSTTOPN532        0xD4
#define PN532_PN532TOHOST        0xD5
#define PN532_I2C_READY          0x01
#define PN532_DEFAULT_TIMEOUT    1000
#define PN532_FRAME_MAX          64

/* Commands */
#define CMD_GETFIRMWAREVERSION   0x02
#define CMD_SAMCONFIGURATION     0x14
#define CMD_INLISTPASSIVETARGET  0x4A
#define CMD_INDATAEXCHANGE       0x40
#define MIFARE_CMD_READ          0x30
#define MIFARE_CMD_WRITE         0xA0
#define MIFARE_UL_CMD_WRITE      0xA2

#define ADDR_W  (PN532_I2C_ADDRESS << 1)

static uint8_t pn532_buf[PN532_FRAME_MAX + 7];
static const uint8_t PN532_ACK[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

/* -----------------------------------------------------------------------
 *  I2C HAL helpers (PN532 needs raw I2C, not register-addressed)
 * ----------------------------------------------------------------------- */
static int wait_event(uint32_t evt, uint32_t ms)
{
    while (!I2C_CheckEvent(I2C1, evt)) {
        Delay_Ms(1);
        if (!ms--) return 0;
    }
    return 1;
}

static int wait_flag_set(FlagStatus (*fn)(I2C_TypeDef*, uint32_t),
                         I2C_TypeDef *i2c, uint32_t flag, uint32_t ms)
{
    while (fn(i2c, flag) == RESET) {
        Delay_Ms(1);
        if (!ms--) return 0;
    }
    return 1;
}

static int wait_flag_clear(FlagStatus (*fn)(I2C_TypeDef*, uint32_t),
                           I2C_TypeDef *i2c, uint32_t flag, uint32_t ms)
{
    while (fn(i2c, flag) != RESET) {
        Delay_Ms(1);
        if (!ms--) return 0;
    }
    return 1;
}

static void soft_recover(void)
{
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    Delay_Ms(1);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
}

static int i2c_start(void)
{
    if (!wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 20)) {
        soft_recover();
        if (!wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 20))
            return 0;
    }
    I2C_GenerateSTART(I2C1, ENABLE);
    return wait_event(I2C_EVENT_MASTER_MODE_SELECT, 20);
}

static void i2c_stop(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    (void)wait_flag_clear(I2C_GetFlagStatus, I2C1, I2C_FLAG_BUSY, 10);
}

static int addr_tx(void)
{
    I2C_Send7bitAddress(I2C1, ADDR_W, I2C_Direction_Transmitter);
    return wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, 20);
}

static int addr_rx(void)
{
    I2C_Send7bitAddress(I2C1, ADDR_W, I2C_Direction_Receiver);
    return wait_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, 20);
}

static int send_byte(uint8_t b)
{
    I2C_SendData(I2C1, b);
    return wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED, 20);
}

/* -----------------------------------------------------------------------
 *  PN532 I2C transport
 * ----------------------------------------------------------------------- */
static int pn532_write(uint8_t *data, uint16_t count)
{
    if (!i2c_start()) return PN532_STATUS_ERROR;
    if (!addr_tx()) { i2c_stop(); return PN532_STATUS_ERROR; }
    for (uint16_t i = 0; i < count; i++) {
        if (!send_byte(data[i])) { i2c_stop(); return PN532_STATUS_ERROR; }
    }
    i2c_stop();
    return PN532_STATUS_OK;
}

static int pn532_read(uint8_t *data, uint16_t count)
{
    uint8_t status = 0;
    if (!i2c_start()) return PN532_STATUS_ERROR;
    if (!addr_rx()) { i2c_stop(); return PN532_STATUS_ERROR; }

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

static bool pn532_wait_ready(uint32_t timeout_ms)
{
    uint8_t status = 0;
    uint32_t attempts = timeout_ms / 10;
    for (uint32_t i = 0; i < attempts; i++) {
        if (!i2c_start()) { i2c_stop(); Delay_Ms(5); continue; }
        if (!addr_rx()) { i2c_stop(); Delay_Ms(5); continue; }
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

/* -----------------------------------------------------------------------
 *  PN532 protocol
 * ----------------------------------------------------------------------- */
static int write_frame(uint8_t *data, uint16_t length)
{
    if (length > PN532_FRAME_MAX || length < 1) return PN532_STATUS_ERROR;

    uint8_t frame[PN532_FRAME_MAX + 7];
    uint8_t checksum = 0;

    frame[0] = PN532_PREAMBLE;
    frame[1] = PN532_STARTCODE1;
    frame[2] = PN532_STARTCODE2;

    for (uint8_t i = 0; i < 3; i++)
        checksum += frame[i];

    frame[3] = length & 0xFF;
    frame[4] = (~length + 1) & 0xFF;

    for (uint8_t i = 0; i < length; i++) {
        frame[5 + i] = data[i];
        checksum += data[i];
    }
    frame[length + 5] = ~checksum & 0xFF;
    frame[length + 6] = PN532_POSTAMBLE;

    return pn532_write(frame, length + 7);
}

static int read_frame(uint8_t *response, uint16_t length)
{
    uint8_t checksum = 0;

    if (pn532_read(pn532_buf, length + 7) != PN532_STATUS_OK)
        return PN532_STATUS_ERROR;

    uint8_t offset = 0;
    while (pn532_buf[offset] == 0x00) {
        offset++;
        if (offset >= length + 8) return PN532_STATUS_ERROR;
    }
    if (pn532_buf[offset] != 0xFF) return PN532_STATUS_ERROR;
    offset++;
    if (offset >= length + 8) return PN532_STATUS_ERROR;

    uint8_t frame_len = pn532_buf[offset];
    if (((frame_len + pn532_buf[offset + 1]) & 0xFF) != 0)
        return PN532_STATUS_ERROR;

    for (uint8_t i = 0; i < frame_len + 1; i++)
        checksum += pn532_buf[offset + 2 + i];
    if ((checksum & 0xFF) != 0) return PN532_STATUS_ERROR;

    for (uint8_t i = 0; i < frame_len; i++)
        response[i] = pn532_buf[offset + 2 + i];

    return frame_len;
}

static int call_function(uint8_t command, uint8_t *response, uint16_t resp_len,
                         uint8_t *params, uint16_t params_len, uint32_t timeout)
{
    pn532_buf[0] = PN532_HOSTTOPN532;
    pn532_buf[1] = command;
    for (uint8_t i = 0; i < params_len; i++)
        pn532_buf[2 + i] = params[i];

    if (write_frame(pn532_buf, params_len + 2) != PN532_STATUS_OK)
        return PN532_STATUS_ERROR;

    if (!pn532_wait_ready(timeout)) return PN532_STATUS_ERROR;

    if (pn532_read(pn532_buf, sizeof(PN532_ACK)) != PN532_STATUS_OK)
        return PN532_STATUS_ERROR;

    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++) {
        if (PN532_ACK[i] != pn532_buf[i]) return PN532_STATUS_ERROR;
    }

    if (!pn532_wait_ready(timeout)) return PN532_STATUS_ERROR;

    int frame_len = read_frame(pn532_buf, resp_len + 2);
    if (frame_len < 0) return PN532_STATUS_ERROR;

    if (!(pn532_buf[0] == PN532_PN532TOHOST && pn532_buf[1] == (command + 1)))
        return PN532_STATUS_ERROR;

    for (uint8_t i = 0; i < resp_len && i < (frame_len - 2); i++)
        response[i] = pn532_buf[i + 2];

    return frame_len - 2;
}

/* -----------------------------------------------------------------------
 *  I2C hardware init
 * ----------------------------------------------------------------------- */
static void i2c_hw_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    gpio.GPIO_Pin   = GPIO_Pin_2; GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin   = GPIO_Pin_1; GPIO_Init(GPIOC, &gpio);

    I2C_InitTypeDef i2c = {0};
    i2c.I2C_ClockSpeed          = 100000;
    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1         = 0x00;
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

/* -----------------------------------------------------------------------
 *  Public API
 * ----------------------------------------------------------------------- */

int pn532_init(void)
{
    i2c_hw_init();
    return PN532_STATUS_OK;
}

int pn532_get_firmware(uint8_t *version)
{
    if (call_function(CMD_GETFIRMWAREVERSION, version, 4, NULL, 0, 500) == PN532_STATUS_ERROR)
        return PN532_STATUS_ERROR;
    return PN532_STATUS_OK;
}

int pn532_sam_config(void)
{
    uint8_t params[] = {0x01, 0x14, 0x01};
    if (call_function(CMD_SAMCONFIGURATION, NULL, 0, params, 3, PN532_DEFAULT_TIMEOUT) < 0)
        return PN532_STATUS_ERROR;
    return PN532_STATUS_OK;
}

int pn532_read_passive(uint8_t *uid, uint32_t timeout_ms)
{
    uint8_t params[] = {0x01, PN532_MIFARE_ISO14443A};
    uint8_t buff[19];

    int length = call_function(CMD_INLISTPASSIVETARGET, buff, sizeof(buff),
                               params, 2, timeout_ms);
    if (length < 0) return PN532_STATUS_ERROR;
    if (buff[0] != 0x01) return PN532_STATUS_ERROR;
    if (buff[5] > 7) return PN532_STATUS_ERROR;

    for (uint8_t i = 0; i < buff[5]; i++)
        uid[i] = buff[6 + i];

    return buff[5];
}

int pn532_ntag_read_block(uint8_t *response, uint16_t block)
{
    uint8_t params[] = {0x01, MIFARE_CMD_READ, block & 0xFF};
    uint8_t buff[MIFARE_BLOCK_LENGTH + 1];

    call_function(CMD_INDATAEXCHANGE, buff, sizeof(buff), params, 3, PN532_DEFAULT_TIMEOUT);
    if (buff[0] != PN532_ERROR_NONE) return buff[0];

    for (uint8_t i = 0; i < NTAG2XX_BLOCK_LENGTH; i++)
        response[i] = buff[i + 1];
    return buff[0];
}

int pn532_ntag_write_block(uint8_t *data, uint16_t block)
{
    uint8_t params[NTAG2XX_BLOCK_LENGTH + 3];
    uint8_t response[1];
    params[0] = 0x01;
    params[1] = MIFARE_UL_CMD_WRITE;
    params[2] = block & 0xFF;
    for (uint8_t i = 0; i < NTAG2XX_BLOCK_LENGTH; i++)
        params[3 + i] = data[i];
    call_function(CMD_INDATAEXCHANGE, response, 1, params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    return response[0];
}

int pn532_mifare_auth(uint8_t *uid, uint8_t uid_len,
                      uint16_t block, uint16_t key_num, uint8_t *key)
{
    uint8_t response[1] = {0xFF};
    uint8_t params[3 + MIFARE_UID_MAX_LENGTH + MIFARE_KEY_LENGTH];
    params[0] = 0x01;
    params[1] = key_num & 0xFF;
    params[2] = block & 0xFF;
    for (uint8_t i = 0; i < MIFARE_KEY_LENGTH; i++)
        params[3 + i] = key[i];
    for (uint8_t i = 0; i < uid_len; i++)
        params[3 + MIFARE_KEY_LENGTH + i] = uid[i];
    call_function(CMD_INDATAEXCHANGE, response, 1,
                  params, 3 + MIFARE_KEY_LENGTH + uid_len, PN532_DEFAULT_TIMEOUT);
    return response[0];
}

int pn532_mifare_read_block(uint8_t *response, uint16_t block)
{
    uint8_t params[] = {0x01, MIFARE_CMD_READ, block & 0xFF};
    uint8_t buff[MIFARE_BLOCK_LENGTH + 1];
    call_function(CMD_INDATAEXCHANGE, buff, sizeof(buff), params, 3, PN532_DEFAULT_TIMEOUT);
    if (buff[0] != PN532_ERROR_NONE) return buff[0];
    for (uint8_t i = 0; i < MIFARE_BLOCK_LENGTH; i++)
        response[i] = buff[i + 1];
    return buff[0];
}

int pn532_mifare_write_block(uint8_t *data, uint16_t block)
{
    uint8_t params[MIFARE_BLOCK_LENGTH + 3];
    uint8_t response[1];
    params[0] = 0x01;
    params[1] = MIFARE_CMD_WRITE;
    params[2] = block & 0xFF;
    for (uint8_t i = 0; i < MIFARE_BLOCK_LENGTH; i++)
        params[3 + i] = data[i];
    call_function(CMD_INDATAEXCHANGE, response, 1, params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    return response[0];
}

int pn532_read_ntag_dump(uint8_t *uid, uint8_t uid_len,
                         uint8_t *dump, uint8_t *blocks_read)
{
    (void)uid; (void)uid_len;
    uint8_t buffer[NTAG2XX_BLOCK_LENGTH];
    *blocks_read = 0;

    for (uint8_t block = 0; block < 16; block++) {
        int err = pn532_ntag_read_block(buffer, block);
        if (err != PN532_ERROR_NONE) {
            if (block > 4) break;
            continue;
        }
        dump[block * 4 + 0] = buffer[0];
        dump[block * 4 + 1] = buffer[1];
        dump[block * 4 + 2] = buffer[2];
        dump[block * 4 + 3] = buffer[3];
        (*blocks_read)++;
    }
    return (*blocks_read > 0) ? 0 : -1;
}

/* -----------------------------------------------------------------------
 *  NDEF text extraction
 * ----------------------------------------------------------------------- */
#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif

int pn532_extract_ndef_text(const uint8_t *dump, uint8_t dump_len,
                            char *out, uint8_t out_size)
{
    if (!dump || dump_len < 18 || !out || out_size == 0) return -1;

    /* Find NDEF TLV (type 0x03) starting at page 4 (offset 16) */
    const uint8_t *ndef = NULL;
    uint8_t ndef_len = 0;
    uint8_t i = 16;

    while (i < dump_len) {
        uint8_t t = dump[i++];
        if (t == 0x00) continue;
        if (t == 0xFE) break;
        if (i >= dump_len) return -1;

        uint8_t L = dump[i++];
        if (i + L > dump_len) return -1;

        if (t == 0x03) {
            ndef = &dump[i];
            ndef_len = L;
            break;
        }
        i += L;
    }
    if (!ndef) return -1;

    /* Parse NDEF text record */
    if (ndef_len < 3) return -1;
    uint8_t hdr = ndef[0];
    uint8_t type_len = ndef[1];
    uint8_t idx = 2;
    uint8_t payload_len = 0;

    if (hdr & 0x10) {
        if (idx >= ndef_len) return -1;
        payload_len = ndef[idx++];
    } else {
        return -1; /* long form not supported in 64-byte dump */
    }

    if (hdr & 0x08) {
        if (idx >= ndef_len) return -1;
        idx += ndef[idx] + 1;
    }

    if (idx + type_len > ndef_len) return -1;
    const uint8_t *type = &ndef[idx];
    idx += type_len;

    if (!((hdr & 0x07) == 0x01 && type_len == 1 && type[0] == 'T'))
        return -1;
    if (payload_len == 0 || idx + payload_len > ndef_len) return -1;

    const uint8_t *payload = &ndef[idx];
    uint8_t lang_len = payload[0] & 0x3F;
    if (1 + lang_len > payload_len) return -1;

    const uint8_t *text = &payload[1 + lang_len];
    uint8_t text_len = payload_len - 1 - lang_len;
    uint8_t copy_len = MIN(text_len, out_size - 1);

    memcpy(out, text, copy_len);
    out[copy_len] = '\0';
    return 0;
}