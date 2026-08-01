#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include "main.h"

// Magic number to check if Flash has been initialized before
#define EEPROM_MAGIC 0xDEADBEE1

// Sector 7, Bank 2 on STM32H743x
#define FLASH_USER_START_ADDR   0x081E0000 

// Data Structure to save
typedef struct {
    uint32_t magic;
    
    // Roll PID
    float r_p; float r_i; float r_d; float r_f;
    // Pitch PID
    float p_p; float p_i; float p_d; float p_f;
    // Yaw PID
    float y_p; float y_i; float y_d; float y_f;
    
    // Magnetometer Hard Iron Offsets
    float mag_offset_x;
    float mag_offset_y;
    float mag_offset_z;
    
} Flash_Data;

void Flash_Save(Flash_Data* data);
uint8_t Flash_Load(Flash_Data* data);

#endif
