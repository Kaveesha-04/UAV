#include "flash_store.h"
#include <string.h>

void Flash_Save(Flash_Data* data) {
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;
    
    HAL_FLASH_Unlock();
    
    // H7 Flash Erase Structure
    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Banks         = FLASH_BANK_2;
    EraseInitStruct.Sector        = FLASH_SECTOR_7;
    EraseInitStruct.NbSectors     = 1;
    
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }
    
    // The STM32H7 programs flash in 256-bit (32-byte) Flash Words.
    // We must pad our struct to the next 32-byte boundary.
    // Explicit 32-byte alignment prevents AXI bus HardFaults during Flash writes.
    #define FLASH_WORD_SIZE 32
    #define PADDED_SIZE (((sizeof(Flash_Data) + FLASH_WORD_SIZE - 1) / FLASH_WORD_SIZE) * FLASH_WORD_SIZE)
    __attribute__((aligned(32))) uint8_t buffer[PADDED_SIZE] = {0};
    data->magic = EEPROM_MAGIC;
    memcpy(buffer, data, sizeof(Flash_Data));
    
    uint32_t address = FLASH_USER_START_ADDR;
    
    // Write Flash Words (32 bytes each)
    for (int i = 0; i < (int)(PADDED_SIZE / FLASH_WORD_SIZE); i++) {
        uint32_t flashWordAddr = (uint32_t)(buffer + (i * 32));
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, flashWordAddr);
        address += 32;
    }
    
    HAL_FLASH_Lock();
}

uint8_t Flash_Load(Flash_Data* data) {
    Flash_Data* saved_data = (Flash_Data*)FLASH_USER_START_ADDR;
    
    if (saved_data->magic == EEPROM_MAGIC) {
        memcpy(data, saved_data, sizeof(Flash_Data));
        return 1; // Success
    }
    return 0; // Not initialized
}
