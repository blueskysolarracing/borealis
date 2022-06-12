/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define BMS_WKUP_Pin GPIO_PIN_0
#define BMS_WKUP_GPIO_Port GPIOA
#define BBMB_INT_Pin GPIO_PIN_1
#define BBMB_INT_GPIO_Port GPIOA
#define PSENSE_ALERT_Pin GPIO_PIN_2
#define PSENSE_ALERT_GPIO_Port GPIOA
#define EN_PWR_Pin GPIO_PIN_0
#define EN_PWR_GPIO_Port GPIOB
#define EN_BLN_PWR_Pin GPIO_PIN_1
#define EN_BLN_PWR_GPIO_Port GPIOB
#define MCU_LED_Pin GPIO_PIN_11
#define MCU_LED_GPIO_Port GPIOB
#define LTC6810_CS_Pin GPIO_PIN_9
#define LTC6810_CS_GPIO_Port GPIOC
#define RS485_EN_Pin GPIO_PIN_5
#define RS485_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define TCP_ID BMS_ID
#define MY_ID 1
#define NUM_CELLS 5
#define NUM_TEMP_SENSORS 3
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
