#include "hmc5883l.h" 
#include <math.h>

#define HMC5883L_ADDR (0x1E << 1)

extern uint8_t i2c_error_count;
static uint8_t is_disguised_qmc = 0;

void HMC5883L_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;
    uint8_t id[3];
    
    // Check if it's a genuine Honeywell chip by reading ID registers 10, 11, 12
    HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, 0x0A, 1, id, 3, 100);
    
    if (id[0] == 'H' && id[1] == '4' && id[2] == '3') {
        is_disguised_qmc = 0; // Genuine
        data = 0x78; 
        HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x00, 1, &data, 1, 100);
        data = 0x20; 
        HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x01, 1, &data, 1, 100);
        data = 0x00; 
        HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x02, 1, &data, 1, 100);
    } else {
        is_disguised_qmc = 1; // It's a QST QMC5883L responding to the HMC address
        data = 0x80;
        HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x0A, 1, &data, 1, 100); // Soft Reset
        HAL_Delay(10);
        data = 0x01;
        HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x0B, 1, &data, 1, 100); // Set/Reset Period
        data = 0x1D; 
        HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, 0x09, 1, &data, 1, 100); // Continuous, 200Hz, 8G
    }
}

void HMC5883L_Read(I2C_HandleTypeDef *hi2c) {
    uint8_t status = 0;
    uint8_t raw[6];
    
    if (is_disguised_qmc) {
        // --- Disguised QMC5883L Read Logic ---
        if (HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, 0x06, 1, &status, 1, 100) != HAL_OK) {
            i2c_error_count++; return;
        }
        if (status & 0x01) { 
            if (HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, 0x00, 1, raw, 6, 100) != HAL_OK) {
                i2c_error_count++; return;
            }
            i2c_error_count = 0;
            mag_x = (int16_t)(raw[1] << 8 | raw[0]);
            mag_y = (int16_t)(raw[3] << 8 | raw[2]);
            mag_z = (int16_t)(raw[5] << 8 | raw[4]);
        }
    } else {
        // --- Genuine HMC5883L Read Logic ---
        if (HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, 0x09, 1, &status, 1, 100) != HAL_OK) {
            i2c_error_count++; return;
        }
        // Genuine HMC5883L sometimes fails to set DRDY bit properly if timing is tight.
        // We will unconditionally read it since we poll at 50Hz and it updates at 75Hz.
        if (HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, 0x03, 1, raw, 6, 100) != HAL_OK) {
            i2c_error_count++; return;
        }
        i2c_error_count = 0;
        mag_x = (int16_t)((raw[0] << 8) | raw[1]);
        mag_z = (int16_t)((raw[2] << 8) | raw[3]); 
        mag_y = (int16_t)((raw[4] << 8) | raw[5]); 
    }
}