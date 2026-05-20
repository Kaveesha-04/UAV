#ifndef BMP280_H
#define BMP280_H

#include "stm32h7xx_hal.h"

// Function Prototypes
void BMP280_Init(SPI_HandleTypeDef *hspi);
void BMP280_Read_Data(SPI_HandleTypeDef *hspi);

float BMP280_GetTemp(void);       // උෂ්ණත්වය (Celsius)
float BMP280_GetPressure(void);   // පීඩනය (hPa)
float BMP280_GetAltitude(void);   // උස (Meters)
void BMP280_Calibrate_Altitude(SPI_HandleTypeDef *hspi);

#endif // BMP280_H
