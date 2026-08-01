#ifndef MPU6500_H
#define MPU6500_H

#include "stm32h7xx_hal.h"

// IMU Physical Offset from Center of Gravity (in meters)
// Update these once measured!
#define IMU_OFFSET_X  0.0f  // Forward(+)/Backward(-)
#define IMU_OFFSET_Y  0.0f  // Right(+)/Left(-)
#define IMU_OFFSET_Z  0.0f  // Up(+)/Down(-)

// Function Prototypes
void MPU6500_Init(SPI_HandleTypeDef *hspi);
void MPU6500_Read_Angles(SPI_HandleTypeDef *hspi, float dt);

float MPU6500_GetRoll(void);
float MPU6500_GetPitch(void);
float MPU6500_GetYaw(void);

void MPU6500_Calibrate(SPI_HandleTypeDef *hspi);
#endif // MPU6500_H
