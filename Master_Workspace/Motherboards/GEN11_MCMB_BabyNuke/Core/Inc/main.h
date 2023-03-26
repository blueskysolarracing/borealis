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
#define GPIO_OUT0_Pin GPIO_PIN_3
#define GPIO_OUT0_GPIO_Port GPIOE
#define GPIO_OUT1_Pin GPIO_PIN_2
#define GPIO_OUT1_GPIO_Port GPIOF
#define GPIO_OUT2_Pin GPIO_PIN_12
#define GPIO_OUT2_GPIO_Port GPIOF
#define GPIO_OUT3_Pin GPIO_PIN_13
#define GPIO_OUT3_GPIO_Port GPIOF
#define GPIO_IN0_Pin GPIO_PIN_0
#define GPIO_IN0_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOE
#define LED0_Pin GPIO_PIN_15
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOB
#define GPIO_OUT4_Pin GPIO_PIN_10
#define GPIO_OUT4_GPIO_Port GPIOD
#define GPIO_OUT5_Pin GPIO_PIN_11
#define GPIO_OUT5_GPIO_Port GPIOD
#define GPIO_OUT8_Pin GPIO_PIN_2
#define GPIO_OUT8_GPIO_Port GPIOG
#define BBMB_PSM_CS_0_Pin GPIO_PIN_8
#define BBMB_PSM_CS_0_GPIO_Port GPIOC
#define GPIO_OUT12_Pin GPIO_PIN_9
#define GPIO_OUT12_GPIO_Port GPIOC
#define GPIO_OUT13_Pin GPIO_PIN_12
#define GPIO_OUT13_GPIO_Port GPIOC
#define GPIO_OUT16_Pin GPIO_PIN_2
#define GPIO_OUT16_GPIO_Port GPIOD
#define GPIO_OUT17_Pin GPIO_PIN_3
#define GPIO_OUT17_GPIO_Port GPIOD
#define PSM_CS_2_Pin GPIO_PIN_6
#define PSM_CS_2_GPIO_Port GPIOD
#define PSM_CS_3_Pin GPIO_PIN_9
#define PSM_CS_3_GPIO_Port GPIOG
#define GPIO_OUT23_Pin GPIO_PIN_11
#define GPIO_OUT23_GPIO_Port GPIOG
#define GPIO_OUT24_Pin GPIO_PIN_12
#define GPIO_OUT24_GPIO_Port GPIOG
#define GPIO_OUT25_Pin GPIO_PIN_13
#define GPIO_OUT25_GPIO_Port GPIOG
#define GPIO_OUT26_Pin GPIO_PIN_14
#define GPIO_OUT26_GPIO_Port GPIOG
#define GPIO_OUT27_Pin GPIO_PIN_15
#define GPIO_OUT27_GPIO_Port GPIOG
#define GPIO_OUT28_Pin GPIO_PIN_4
#define GPIO_OUT28_GPIO_Port GPIOB
#define GPIO_OUT30_Pin GPIO_PIN_9
#define GPIO_OUT30_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
