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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AGX_TX_Pin GPIO_PIN_2
#define AGX_TX_GPIO_Port GPIOA
#define AGX_RX_Pin GPIO_PIN_3
#define AGX_RX_GPIO_Port GPIOA
#define MS5837_TX_Pin GPIO_PIN_7
#define MS5837_TX_GPIO_Port GPIOE
#define MS5837_RX_Pin GPIO_PIN_8
#define MS5837_RX_GPIO_Port GPIOE
#define USBL_TX_Pin GPIO_PIN_10
#define USBL_TX_GPIO_Port GPIOB
#define USBL_RX_Pin GPIO_PIN_11
#define USBL_RX_GPIO_Port GPIOB
#define INS_RX_Pin GPIO_PIN_14
#define INS_RX_GPIO_Port GPIOB
#define INS_TX_Pin GPIO_PIN_15
#define INS_TX_GPIO_Port GPIOB
#define VIT6_TX_Pin GPIO_PIN_6
#define VIT6_TX_GPIO_Port GPIOC
#define VIT6_RX_Pin GPIO_PIN_7
#define VIT6_RX_GPIO_Port GPIOC
#define MS5837_RXA11_Pin GPIO_PIN_11
#define MS5837_RXA11_GPIO_Port GPIOA
#define MS5837_TXA12_Pin GPIO_PIN_12
#define MS5837_TXA12_GPIO_Port GPIOA
#define MS5837_SDA_Pin GPIO_PIN_7
#define MS5837_SDA_GPIO_Port GPIOB
#define MS5837_SCL_Pin GPIO_PIN_8
#define MS5837_SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
