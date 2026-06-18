#include "hmc5883l.h" 
#include <math.h>

#define HMC5883L_ADDR (0x1E << 1)

extern uint8_t i2c_error_count;

void HMC5883L_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;
    
    // Give the sensor time to power up
    HAL_Delay(50);
    
    // Configuration Register A (0x00): 8-sample average, 75Hz data rate, normal measurement
    data = 0x78; 
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x00, 1, &data, 1, 100);
    
    // Configuration Register B (0x01): Gain = 1090 LSb/Gauss (default range)
    data = 0x20; 
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x01, 1, &data, 1, 100);
    
    // Mode Register (0x02): Continuous measurement mode
    data = 0x00; 
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x02, 1, &data, 1, 100);
    
    HAL_Delay(10);
}

void HMC5883L_Read(I2C_HandleTypeDef *hi2c) {
    uint8_t raw[6];
    
    // We completely bypass checking the Status Register (0x09). 
    // Many cheap GY-271 HMC5883L clones (like DA5883) have a broken Status Register 
    // that never sets the DRDY bit to 1, causing the driver to permanently ignore the data.
    // Since we poll at 50Hz and the chip runs at 75Hz continuous, data is always ready.
    
    if (HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, 0x03, 1, raw, 6, 100) != HAL_OK) {
        i2c_error_count++;
        return; // I2C hardware error (bus stuck or wire disconnected)
    }
    
    // Reset error count on successful I2C transaction
    i2c_error_count = 0;
    
    // HMC5883L outputs MSB first. 
    // The hardware axis ordering in the registers is X, Z, Y.
    mag_x = (int16_t)((raw[0] << 8) | raw[1]);
    mag_z = (int16_t)((raw[2] << 8) | raw[3]); 
    mag_y = (int16_t)((raw[4] << 8) | raw[5]); 
}