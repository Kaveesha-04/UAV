#ifndef MPU6500_H
#define MPU6500_H

#include "stm32h7xx_hal.h"

// Function Prototypes
void MPU6500_Init(SPI_HandleTypeDef *hspi);
void MPU6500_Read_Angles(SPI_HandleTypeDef *hspi, float dt);

float MPU6500_GetRoll(void);
float MPU6500_GetPitch(void);
float MPU6500_GetYaw(void);

void MPU6500_Calibrate(SPI_HandleTypeDef *hspi);
#endif // MPU6500_H
