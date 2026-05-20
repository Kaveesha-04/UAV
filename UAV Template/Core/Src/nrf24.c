#include "nrf24.h"

// CSN Pin Control
static void CSN_Select(void) {
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
}
static void CSN_Unselect(void) {
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

// CE Pin Control
static void CE_Enable(void) {
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
}
static void CE_Disable(void) {
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
}

// Write to an NRF Register
void NRF_Write_Reg(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg | 0x20; // 0x20 is the Write command bit
    buf[1] = data;
    CSN_Select();
    HAL_SPI_Transmit(&hspi1, buf, 2, 10);
    CSN_Unselect();
}

// Read an NRF Register
uint8_t NRF_Read_Reg(uint8_t reg) {
    uint8_t tx = reg; // 0x00 is Read command mask
    uint8_t rx = 0;
    CSN_Select();
    HAL_SPI_Transmit(&hspi1, &tx, 1, 10);
    HAL_SPI_Receive(&hspi1, &rx, 1, 10);
    CSN_Unselect();
    return rx;
}

// Write multiple bytes to NRF
void NRF_Write_Buf(uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t cmd = reg | 0x20;
    CSN_Select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    HAL_SPI_Transmit(&hspi1, data, len, 10);
    CSN_Unselect();
}

// Initialize NRF24L01 as a Receiver
void NRF_Init_RX(void) {
    CE_Disable();
    HAL_Delay(50);
    
    // Clear all status flags (RX_DR, TX_DS, MAX_RT) to prevent hanging
    NRF_Write_Reg(NRF_STATUS, 0x70);

    // Explicitly set Address Width to 5 bytes
    NRF_Write_Reg(0x03, 0x03); 

    // Set RF Channel to 115
    NRF_Write_Reg(NRF_RF_CH, 115);
    
    // Set Data Rate 1Mbps, Power 0dBm
    NRF_Write_Reg(NRF_RF_SETUP, 0x06);
    
    // Set Payload Size for Pipe 0 (8 Bytes)
    NRF_Write_Reg(NRF_RX_PW_P0, sizeof(struct Data_Package));
    
    // Enable Auto-Acknowledgment on Pipe 0
    NRF_Write_Reg(NRF_EN_AA, 0x01);
    
    // Enable RX Address on Pipe 0
    NRF_Write_Reg(NRF_EN_RXADDR, 0x01);

    // Set RX Address to "EEEEE" (Mirrored to prevent Endianness bugs!)
    uint8_t rx_addr[5] = {'E', 'E', 'E', 'E', 'E'};
    NRF_Write_Buf(0x0A, rx_addr, 5); // 0x0A is NRF_RX_ADDR_P0
    
    // Config: CRC 2-byte, Power UP, PRX (Primary Receiver)
    NRF_Write_Reg(NRF_CONFIG, 0x0F);
    
    // Start Listening
    CE_Enable();
    
    // Flush Buffers
    uint8_t cmd;
    CSN_Select();
    cmd = NRF_FLUSH_RX;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    CSN_Unselect();
}

// Check if Data is Available
uint8_t NRF_DataAvailable(void) {
    uint8_t status = NRF_Read_Reg(NRF_STATUS);
    if ((status & 0x40) == 0x40) { // RX_DR bit is set
        // Clear the status bit by writing 1 to it
        NRF_Write_Reg(NRF_STATUS, 0x40);
        return 1;
    }
    return 0;
}

// Read the Payload Data
void NRF_ReadPayload(void *buf, uint8_t len) {
    uint8_t cmd = NRF_R_RX_PAYLOAD;
    CSN_Select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    HAL_SPI_Receive(&hspi1, (uint8_t *)buf, len, 10);
    CSN_Unselect();
}
