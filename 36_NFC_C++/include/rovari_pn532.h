/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Rovari - rvembedded.com
 *
 * rovari_pn532.h
 * PN532 NFC reader for CH32V003 (Elechouse V3 module, I2C)
 *
 * Supports NTAG/Ultralight and MIFARE Classic cards.
 * NDEF text record extraction included.
 * I2C at 100 kHz on PC2/PC1.
 *
 * Wiring:
 *   VCC -> 5V
 *   GND -> GND
 *   SDA -> PC1
 *   SCL -> PC2
 *
 * C usage:
 *   pn532_init();
 *   uint8_t uid[10];
 *   int len = pn532_read_passive(uid, 1000);
 *   if (len > 0) { ... }
 *
 * C++ usage:
 *   Pn532 nfc;
 *   nfc.init();
 *   uint8_t uid[10];
 *   int len = nfc.readPassive(uid, 1000);
 */

#ifndef ROVARI_PN532_H
#define ROVARI_PN532_H

#include "rovari_defs.h"
#include <stdbool.h>

#define PN532_I2C_ADDRESS          (0x48 >> 1)
#define PN532_MIFARE_ISO14443A     (0x00)
#define PN532_ERROR_NONE           (0x00)
#define PN532_STATUS_ERROR         (-1)
#define PN532_STATUS_OK            (0)
#define NTAG2XX_BLOCK_LENGTH       (4)
#define MIFARE_BLOCK_LENGTH        (16)
#define MIFARE_KEY_LENGTH          (6)
#define MIFARE_UID_MAX_LENGTH      (10)
#define NDEF_TEXT_MAX_LEN          128

#ifdef __cplusplus
extern "C" {
#endif

int  pn532_init(void);
int  pn532_get_firmware(uint8_t *version);
int  pn532_sam_config(void);
int  pn532_read_passive(uint8_t *uid, uint32_t timeout_ms);

int  pn532_ntag_read_block(uint8_t *response, uint16_t block);
int  pn532_ntag_write_block(uint8_t *data, uint16_t block);

int  pn532_mifare_auth(uint8_t *uid, uint8_t uid_len,
                       uint16_t block, uint16_t key_num, uint8_t *key);
int  pn532_mifare_read_block(uint8_t *response, uint16_t block);
int  pn532_mifare_write_block(uint8_t *data, uint16_t block);

int  pn532_read_ntag_dump(uint8_t *uid, uint8_t uid_len,
                          uint8_t *dump, uint8_t *blocks_read);
int  pn532_extract_ndef_text(const uint8_t *dump, uint8_t dump_len,
                             char *out, uint8_t out_size);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Pn532 {
public:
    int     init()                              { return pn532_init(); }
    int     getFirmware(uint8_t *v)             { return pn532_get_firmware(v); }
    int     samConfig()                         { return pn532_sam_config(); }
    int     readPassive(uint8_t *uid, uint32_t t) { return pn532_read_passive(uid, t); }
    int     ntagRead(uint8_t *r, uint16_t b)    { return pn532_ntag_read_block(r, b); }
    int     ntagWrite(uint8_t *d, uint16_t b)   { return pn532_ntag_write_block(d, b); }
    int     mifareAuth(uint8_t *uid, uint8_t len,
                       uint16_t b, uint16_t k, uint8_t *key)
                                                { return pn532_mifare_auth(uid, len, b, k, key); }
    int     mifareRead(uint8_t *r, uint16_t b)  { return pn532_mifare_read_block(r, b); }
    int     mifareWrite(uint8_t *d, uint16_t b) { return pn532_mifare_write_block(d, b); }
};

#endif

#endif /* ROVARI_PN532_H */
