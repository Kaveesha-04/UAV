#ifndef HMC5883L_H
#define HMC5883L_H

#include "main.h"

void HMC5883L_Init(I2C_HandleTypeDef *hi2c);
void HMC5883L_Read(I2C_HandleTypeDef *hi2c);

#endif
