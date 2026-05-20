#ifndef QMC5883L_H
#define QMC5883L_H

#include "main.h"

void QMC5883L_Init(I2C_HandleTypeDef *hi2c);
void QMC5883L_Read(I2C_HandleTypeDef *hi2c);

#endif
