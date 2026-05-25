/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern char esp_buffer[100];
extern volatile uint8_t esp_string_ready;

// Raw Magnetometer Data
extern int16_t mag_x;
extern int16_t mag_y;
extern int16_t mag_z;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_K1_Pin GPIO_PIN_3
#define BTN_K1_GPIO_Port GPIOE
#define BTN_K2_Pin GPIO_PIN_4
#define BTN_K2_GPIO_Port GPIOE
#define LED_STATUS_Pin GPIO_PIN_1
#define LED_STATUS_GPIO_Port GPIOA
#define ESP_TX_Pin GPIO_PIN_2
#define ESP_TX_GPIO_Port GPIOA
#define ESP_RX_Pin GPIO_PIN_3
#define ESP_RX_GPIO_Port GPIOA
#define NRF_CSN_Pin GPIO_PIN_4
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_CSK_Pin GPIO_PIN_5
#define NRF_CSK_GPIO_Port GPIOA
#define NRF_MISO_Pin GPIO_PIN_6
#define NRF_MISO_GPIO_Port GPIOA
#define NRF_MOSI_Pin GPIO_PIN_7
#define NRF_MOSI_GPIO_Port GPIOA
#define NRF_CE_Pin GPIO_PIN_0
#define NRF_CE_GPIO_Port GPIOB
#define CH4_M_FR_Pin GPIO_PIN_1
#define CH4_M_FR_GPIO_Port GPIOB
#define BMP_CS_Pin GPIO_PIN_11
#define BMP_CS_GPIO_Port GPIOB
#define MPU_CS_Pin GPIO_PIN_12
#define MPU_CS_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_8
#define GPS_TX_GPIO_Port GPIOD
#define GPS_RX_Pin GPIO_PIN_9
#define GPS_RX_GPIO_Port GPIOD
#define CH1_M_BL_Pin GPIO_PIN_6
#define CH1_M_BL_GPIO_Port GPIOC
#define CH2_M_BR_Pin GPIO_PIN_7
#define CH2_M_BR_GPIO_Port GPIOC
#define CH3_M_FL_Pin GPIO_PIN_8
#define CH3_M_FL_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
