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
#include "stm32f1xx_hal.h"

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
#define ID0_Pin GPIO_PIN_0
#define ID0_GPIO_Port GPIOA
#define ID1_Pin GPIO_PIN_1
#define ID1_GPIO_Port GPIOA
#define ID2_Pin GPIO_PIN_2
#define ID2_GPIO_Port GPIOA
#define ID3_Pin GPIO_PIN_3
#define ID3_GPIO_Port GPIOA
#define NSS_C_Pin GPIO_PIN_10
#define NSS_C_GPIO_Port GPIOB
#define NSS_D_Pin GPIO_PIN_11
#define NSS_D_GPIO_Port GPIOB
#define NSS_E_Pin GPIO_PIN_12
#define NSS_E_GPIO_Port GPIOB
#define NSS_F_Pin GPIO_PIN_13
#define NSS_F_GPIO_Port GPIOB
#define NSS_G_Pin GPIO_PIN_14
#define NSS_G_GPIO_Port GPIOB
#define NSS_H_Pin GPIO_PIN_15
#define NSS_H_GPIO_Port GPIOB
#define LS_EN_Pin GPIO_PIN_8
#define LS_EN_GPIO_Port GPIOA
#define DRDY_Pin GPIO_PIN_11
#define DRDY_GPIO_Port GPIOA
#define RS485_TX_EN_Pin GPIO_PIN_12
#define RS485_TX_EN_GPIO_Port GPIOA
#define DEMUX_A_Pin GPIO_PIN_4
#define DEMUX_A_GPIO_Port GPIOB
#define DEMUX_B_Pin GPIO_PIN_5
#define DEMUX_B_GPIO_Port GPIOB
#define NSS_A_Pin GPIO_PIN_8
#define NSS_A_GPIO_Port GPIOB
#define NSS_B_Pin GPIO_PIN_9
#define NSS_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
