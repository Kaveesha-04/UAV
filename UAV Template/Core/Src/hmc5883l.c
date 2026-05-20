#include "hmc5883l.h" // Ensure your header file name matches
#include <math.h>

// HMC5883L factory I2C address is 0x1E. 
// Shifted left by 1 for STM32 HAL, this becomes 0x3C.
#define HMC5883L_ADDR (0x1E << 1)

// mag_x, mag_y, mag_z are now global externs from main.h

void HMC5883L_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;
    
    // Configuration Register A (0x00): 8-sample average, 75Hz data rate, normal measurement
    data = 0x78; 
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x00, 1, &data, 1, 100);
    
    // Configuration Register B (0x01): Gain = 1090 LSb/Gauss (default range)
    data = 0x20; 
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x01, 1, &data, 1, 100);
    
    // Mode Register (0x02): Continuous measurement mode
    data = 0x00; 
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x02, 1, &data, 1, 100);
}

void HMC5883L_Read(I2C_HandleTypeDef *hi2c) {
    uint8_t status = 0;
    
    // Read the Status Register (0x09)
    if (HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, 0x09, 1, &status, 1, 100) != HAL_OK) {
        return; // I2C Error
    }
    
    // Check if the Data Ready (RDY) bit (Bit 0) is set
    if (status & 0x01) { 
        uint8_t raw[6];
        
        // Data registers start at 0x03. Read all 6 bytes.
        if (HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, 0x03, 1, raw, 6, 100) != HAL_OK) {
            return;
        }
        
        // HMC5883L outputs MSB first. 
        // The hardware axis ordering in the registers is X, Z, Y.
        mag_x = (int16_t)((raw[0] << 8) | raw[1]);
        mag_z = (int16_t)((raw[2] << 8) | raw[3]); // Z is in the middle
        mag_y = (int16_t)((raw[4] << 8) | raw[5]); // Y is at the end
        
        // Heading is now computed centrally in main.c!
    }
}