#include "04_rfid_rc522.h"

extern SPI_HandleTypeDef hspi2;

// --- 하위 레벨 제어 ---
void MFRC522_CS_Select(void) { HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_RESET); }
void MFRC522_CS_Unselect(void) { HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_SET); }

uint8_t SPI2_WriteRead(uint8_t data) {
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi2, &data, &rx_data, 1, 10);
    return rx_data;
}

void MFRC522_WriteReg(uint8_t addr, uint8_t val) {
    MFRC522_CS_Select();
    SPI2_WriteRead((addr << 1) & 0x7E);
    SPI2_WriteRead(val);
    MFRC522_CS_Unselect();
}

uint8_t MFRC522_ReadReg(uint8_t addr) {
    uint8_t val;
    MFRC522_CS_Select();
    SPI2_WriteRead(((addr << 1) & 0x7E) | 0x80);
    val = SPI2_WriteRead(MFRC522_DUMMY);
    MFRC522_CS_Unselect();
    return val;
}

// --- 블로그의 핵심 로직: 카드 통신 함수 ---
HAL_StatusTypeDef MFRC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint32_t *backLen) {
    uint8_t status = HAL_ERROR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;

    if (command == PCD_AUTHENT) { irqEn = 0x12; waitIRq = 0x10; }
    if (command == PCD_TRANSCEIVE) { irqEn = 0x77; waitIRq = 0x30; }

    MFRC522_WriteReg(0x02, irqEn | 0x80);
    MFRC522_WriteReg(0x04, 0x7F);        // Interrupt 지우기
    MFRC522_WriteReg(0x0A, 0x80);        // FIFO Flush
    MFRC522_WriteReg(0x01, PCD_IDLE);    // NO action

    for (int i = 0; i < sendLen; i++) MFRC522_WriteReg(0x09, sendData[i]);

    MFRC522_WriteReg(0x01, command);
    if (command == PCD_TRANSCEIVE) MFRC522_WriteReg(0x0D, MFRC522_ReadReg(0x0D) | 0x80);

    uint32_t i = 2000; // 타임아웃
    uint8_t n;
    do {
        n = MFRC522_ReadReg(0x04);
        i--;
    } while (i != 0 && !(n & 0x01) && !(n & waitIRq));

    MFRC522_WriteReg(0x0D, MFRC522_ReadReg(0x0D) & (~0x80));

    if (i != 0) {
        if (!(MFRC522_ReadReg(0x06) & 0x1B)) {
            status = HAL_OK;
            if (backData && backLen) {
                n = MFRC522_ReadReg(0x0A);
                *backLen = n * 8; // 비트 단위로 변환
                for (int j = 0; j < n; j++) backData[j] = MFRC522_ReadReg(0x09);
            }
        }
    }
    return status;
}

// --- 초기화 및 체크 ---
void MFRC522_Init(void) {
    HAL_GPIO_WritePin(SPI2_RST_GPIO_Port, SPI2_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(SPI2_RST_GPIO_Port, SPI2_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    MFRC522_WriteReg(0x01, PCD_RESETPHASE);
    HAL_Delay(10);
    MFRC522_WriteReg(0x2A, 0x8D);
    MFRC522_WriteReg(0x2B, 0x3E);
    MFRC522_WriteReg(0x2D, 30);
    MFRC522_WriteReg(0x2C, 0);
    MFRC522_WriteReg(0x15, 0x40);
    MFRC522_WriteReg(0x11, 0x3D);
    MFRC522_WriteReg(0x26, 0x70); // 감도 최대

    uint8_t temp = MFRC522_ReadReg(0x14);
    MFRC522_WriteReg(0x14, temp | 0x03); // 안테나 ON
}

HAL_StatusTypeDef MFRC522_Check(uint8_t* id) {
    uint8_t str[MAX_LEN];
    uint32_t len;

    // 1. Request
    MFRC522_WriteReg(0x0D, 0x07);
    uint8_t tagType = PICC_REQIDL;
    if (MFRC522_ToCard(PCD_TRANSCEIVE, &tagType, 1, str, &len) != HAL_OK) return HAL_ERROR;

    // 2. Anticollision (ID 가져오기)
    uint8_t searchCode[2] = {PICC_ANTICOLL, 0x20};
    if (MFRC522_ToCard(PCD_TRANSCEIVE, searchCode, 2, str, &len) == HAL_OK) {
        for (int i = 0; i < 4; i++) id[i] = str[i];
        return HAL_OK;
    }
    return HAL_ERROR;
}
