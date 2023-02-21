/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define RELAY_HS_Pin GPIO_PIN_3
#define RELAY_HS_GPIO_Port GPIOE
#define RELAY_LS_Pin GPIO_PIN_2
#define RELAY_LS_GPIO_Port GPIOF
#define RELAY_PRECHARGE_Pin GPIO_PIN_12
#define RELAY_PRECHARGE_GPIO_Port GPIOF
#define RELAY_DISCHARGE_Pin GPIO_PIN_13
#define RELAY_DISCHARGE_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOE
#define LED0_Pin GPIO_PIN_15
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOB
#define PSM_CS_0_Pin GPIO_PIN_12
#define PSM_CS_0_GPIO_Port GPIOB
#define PSM_LVDS_EN_Pin GPIO_PIN_10
#define PSM_LVDS_EN_GPIO_Port GPIOD
#define PSM_CS_1_Pin GPIO_PIN_11
#define PSM_CS_1_GPIO_Port GPIOD
#define PSM_CS_3_Pin GPIO_PIN_14
#define PSM_CS_3_GPIO_Port GPIOD
#define PSM_CS_2_Pin GPIO_PIN_15
#define PSM_CS_2_GPIO_Port GPIOD
#define PPT_12V_EN_Pin GPIO_PIN_5
#define PPT_12V_EN_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
