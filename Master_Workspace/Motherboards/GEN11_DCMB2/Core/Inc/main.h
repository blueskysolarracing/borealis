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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t pToggle;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DISP_CS_1_Pin GPIO_PIN_3
#define DISP_CS_1_GPIO_Port GPIOE
#define BACKUP_CAMERA_CTRL_Pin GPIO_PIN_14
#define BACKUP_CAMERA_CTRL_GPIO_Port GPIOI
#define brakeDetect_Pin GPIO_PIN_4
#define brakeDetect_GPIO_Port GPIOC
#define BACKUP_SCREEN_CTRL_Pin GPIO_PIN_15
#define BACKUP_SCREEN_CTRL_GPIO_Port GPIOI
#define GPIO_IN0_Pin GPIO_PIN_0
#define GPIO_IN0_GPIO_Port GPIOJ
#define GPIO_IN0_EXTI_IRQn EXTI0_IRQn
#define FAN_CTRL_Pin GPIO_PIN_0
#define FAN_CTRL_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOE
#define LED0_Pin GPIO_PIN_6
#define LED0_GPIO_Port GPIOH
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOH
#define GPIO_IN10_Pin GPIO_PIN_10
#define GPIO_IN10_GPIO_Port GPIOD
#define GPIO_IN10_EXTI_IRQn EXTI15_10_IRQn
#define DISP_CS_0_Pin GPIO_PIN_0
#define DISP_CS_0_GPIO_Port GPIOI
#define DISP_A0_Pin GPIO_PIN_12
#define DISP_A0_GPIO_Port GPIOJ
#define DISP_LED_CTRL_Pin GPIO_PIN_13
#define DISP_LED_CTRL_GPIO_Port GPIOJ
#define DISP_RST_2_Pin GPIO_PIN_14
#define DISP_RST_2_GPIO_Port GPIOJ
#define DISP_RST_1_Pin GPIO_PIN_15
#define DISP_RST_1_GPIO_Port GPIOJ
/* USER CODE BEGIN Private defines */
#define TCP_ID 0x04
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
