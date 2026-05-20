#ifndef NRF24_H
#define NRF24_H

#include "main.h" // STM32 HAL සහ Pin definitions සඳහා

// main.c හි ඇති hspi1 භාවිතා කිරීමට
extern SPI_HandleTypeDef hspi1;

// NRF24L01+ Register Addresses
#define NRF_CONFIG      0x00
#define NRF_EN_AA       0x01
#define NRF_EN_RXADDR   0x02
#define NRF_SETUP_AW    0x03
#define NRF_SETUP_RETR  0x04
#define NRF_RF_CH       0x05
#define NRF_RF_SETUP    0x06
#define NRF_STATUS      0x07
#define NRF_RX_ADDR_P0  0x0A
#define NRF_RX_PW_P0    0x11
#define NRF_FIFO_STATUS 0x17

// NRF24L01+ Commands
#define NRF_R_RX_PAYLOAD 0x61
#define NRF_W_TX_PAYLOAD 0xA0
#define NRF_FLUSH_TX    0xE1
#define NRF_FLUSH_RX    0xE2

// NRF Data Package Structure (Direct NRF24)
#pragma pack(push, 1)
struct Data_Package {
  uint16_t throttle; // 1000 to 2000
  int16_t yaw;       // -500 to 500
  int16_t pitch;     // -500 to 500
  int16_t roll;      // -500 to 500
};
#pragma pack(pop)

// NRF Function Prototypes
void NRF_Write_Reg(uint8_t reg, uint8_t data);
uint8_t NRF_Read_Reg(uint8_t reg);
void NRF_Write_Buf(uint8_t reg, uint8_t *data, uint8_t len);
void NRF_Init_RX(void);
uint8_t NRF_DataAvailable(void);
void NRF_ReadPayload(void *buf, uint8_t len);

#endif /* NRF24_H */
