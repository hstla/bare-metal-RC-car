#ifndef __04_RFID_RC522_H
#define __04_RFID_RC522_H

#include "main.h"

#define MFRC522_DUMMY   0x00
#define MAX_LEN         16

// MFRC522 명령어
#define PCD_IDLE       0x00
#define PCD_AUTHENT    0x0E
#define PCD_RECEIVE    0x08
#define PCD_TRANSMIT   0x04
#define PCD_TRANSCEIVE 0x0C
#define PCD_RESETPHASE 0x0F
#define PCD_CALCCRC    0x03

// Mifare_One 카드 명령어
#define PICC_REQIDL    0x26
#define PICC_ANTICOLL  0x93

// 함수 선언
void MFRC522_Init(void);
HAL_StatusTypeDef MFRC522_Check(uint8_t* id);
uint8_t MFRC522_ReadReg(uint8_t addr);
void MFRC522_WriteReg(uint8_t addr, uint8_t val);

#endif
