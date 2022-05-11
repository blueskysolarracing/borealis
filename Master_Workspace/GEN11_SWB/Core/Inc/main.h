/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define ACC8_Pin GPIO_PIN_13
#define ACC8_GPIO_Port GPIOC
#define R_SIGNAL_Pin GPIO_PIN_0
#define R_SIGNAL_GPIO_Port GPIOC
#define L_SIGNAL_Pin GPIO_PIN_1
#define L_SIGNAL_GPIO_Port GPIOC
#define RAD_SIGNAL_Pin GPIO_PIN_2
#define RAD_SIGNAL_GPIO_Port GPIOC
#define HORN_SIGNAL_Pin GPIO_PIN_3
#define HORN_SIGNAL_GPIO_Port GPIOC
#define UP_Pin GPIO_PIN_11
#define UP_GPIO_Port GPIOB
#define LEFT_Pin GPIO_PIN_12
#define LEFT_GPIO_Port GPIOB
#define DOWN_Pin GPIO_PIN_13
#define DOWN_GPIO_Port GPIOB
#define RIGHT_Pin GPIO_PIN_14
#define RIGHT_GPIO_Port GPIOB
#define SELECT_Pin GPIO_PIN_15
#define SELECT_GPIO_Port GPIOB
#define ACC1_Pin GPIO_PIN_6
#define ACC1_GPIO_Port GPIOC
#define ACC2_Pin GPIO_PIN_7
#define ACC2_GPIO_Port GPIOC
#define ACC3_Pin GPIO_PIN_8
#define ACC3_GPIO_Port GPIOC
#define ACC4_Pin GPIO_PIN_9
#define ACC4_GPIO_Port GPIOC
#define ACC5_Pin GPIO_PIN_10
#define ACC5_GPIO_Port GPIOC
#define ACC6_Pin GPIO_PIN_11
#define ACC6_GPIO_Port GPIOC
#define ACC7_Pin GPIO_PIN_12
#define ACC7_GPIO_Port GPIOC
#define TEMP_SCL_Pin GPIO_PIN_6
#define TEMP_SCL_GPIO_Port GPIOB
#define TEMP_SDA_Pin GPIO_PIN_7
#define TEMP_SDA_GPIO_Port GPIOB
#define CRUISE_SIGNAL_Pin GPIO_PIN_9
#define CRUISE_SIGNAL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
