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
#include "buart.h"
#include "btcp.h"
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
#define AFE_WDT_Pin GPIO_PIN_13
#define AFE_WDT_GPIO_Port GPIOC
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
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOB
#define LTC6810_CS_Pin GPIO_PIN_9
#define LTC6810_CS_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

//BMS monitors the module to operate withing these limits. If one is violated, it issues a BMS fault, which opens all the power
//relays, switches the car to supplemental battery supply and overall puts the car into safe state
//Constants from LG MJ1 datasheet: https://www.nkon.nl/sk/k/Specification%20INR18650MJ1%2022.08.2014.pdf
#define OV_threshold 4.2 //Cell overvoltage trip point
#define UV_threshold 2.5 //Cell undervoltage trip point
#define OT_threshold 60 //Overtemperature trip point
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
