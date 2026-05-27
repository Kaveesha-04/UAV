#include "qmc5883l.h"
#include <math.h>

#define QMC5883L_ADDR (0x0D << 1)

// mag_x, mag_y, mag_z are now global externs from main.h

void QMC5883L_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;
    
    // Reset
    data = 0x80;
    HAL_I2C_Mem_Write(hi2c, QMC5883L_ADDR, 0x0A, 1, &data, 1, 100);
    HAL_Delay(10);
    
    // Set/Reset Period (recommended 0x01)
    data = 0x01;
    HAL_I2C_Mem_Write(hi2c, QMC5883L_ADDR, 0x0B, 1, &data, 1, 100);
    
    // Control Register 1: OSR=512, RNG=8G, ODR=200Hz, MODE=Continuous
    data = 0x1D; 
    HAL_I2C_Mem_Write(hi2c, QMC5883L_ADDR, 0x09, 1, &data, 1, 100);
}

void QMC5883L_Read(I2C_HandleTypeDef *hi2c) {
    uint8_t status = 0;
    if (HAL_I2C_Mem_Read(hi2c, QMC5883L_ADDR, 0x06, 1, &status, 1, 100) != HAL_OK) {
        return; // I2C Error
    }
    
    if (status & 0x01) { // Data Ready
        uint8_t raw[6];
        if (HAL_I2C_Mem_Read(hi2c, QMC5883L_ADDR, 0x00, 1, raw, 6, 100) != HAL_OK) {
            return;
        }
        
        mag_x = (int16_t)(raw[1] << 8 | raw[0]);
        mag_y = (int16_t)(raw[3] << 8 | raw[2]);
        mag_z = (int16_t)(raw[5] << 8 | raw[4]);
        
        // Heading is now computed centrally in main.c!
    }
}

#define QMC5883P_ADDR (0x2C << 1)

void QMC5883P_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;
    
    // Soft Reset (Control 2 = 0x80)
    data = 0x80;
    HAL_I2C_Mem_Write(hi2c, QMC5883P_ADDR, 0x0B, 1, &data, 1, 100);
    HAL_Delay(50);
    
    // Control Register 1: Mode=Normal(0x01) | ODR=50Hz(0x04) | OSR=4(0x10) | DSR=2(0x40) -> 0x55
    data = 0x55; 
    HAL_I2C_Mem_Write(hi2c, QMC5883P_ADDR, 0x0A, 1, &data, 1, 100);
    
    // Control Register 2: Set/Reset=On(0x00) | Range=8G(0x08) -> 0x08
    data = 0x08;
    HAL_I2C_Mem_Write(hi2c, QMC5883P_ADDR, 0x0B, 1, &data, 1, 100);
}

void QMC5883P_Read(I2C_HandleTypeDef *hi2c) {
    uint8_t status = 0;
    if (HAL_I2C_Mem_Read(hi2c, QMC5883P_ADDR, 0x09, 1, &status, 1, 100) != HAL_OK) {
        return; // I2C Error
    }
    
    if (status & 0x01) { // Data Ready
        uint8_t raw[6];
        // Read 6 bytes starting from X LSB (0x01)
        if (HAL_I2C_Mem_Read(hi2c, QMC5883P_ADDR, 0x01, 1, raw, 6, 100) != HAL_OK) {
            return;
        }
        
        // QMC5883P is LSB first (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
        mag_x = (int16_t)(raw[1] << 8 | raw[0]);
        mag_y = (int16_t)(raw[3] << 8 | raw[2]);
        mag_z = (int16_t)(raw[5] << 8 | raw[4]);
    }
}
