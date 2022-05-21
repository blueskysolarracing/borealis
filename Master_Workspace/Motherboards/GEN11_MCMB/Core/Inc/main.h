/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_IN0_Pin GPIO_PIN_0
#define GPIO_IN0_GPIO_Port GPIOJ
#define GPIO_IN0_EXTI_IRQn EXTI0_IRQn
#define BBMB_PSM_CS_0_Pin GPIO_PIN_13
#define BBMB_PSM_CS_0_GPIO_Port GPIOB
#define GPIO_IN10_Pin GPIO_PIN_10
#define GPIO_IN10_GPIO_Port GPIOD
#define GPIO_IN10_EXTI_IRQn EXTI15_10_IRQn
#define PSM_CS_0_Pin GPIO_PIN_0
#define PSM_CS_0_GPIO_Port GPIOI
#define PSM_LVDS_EN_Pin GPIO_PIN_12
#define PSM_LVDS_EN_GPIO_Port GPIOJ
#define PSM_CS_1_Pin GPIO_PIN_13
#define PSM_CS_1_GPIO_Port GPIOJ
#define PSM_CS_2_Pin GPIO_PIN_14
#define PSM_CS_2_GPIO_Port GPIOJ
#define PSM_CS_3_Pin GPIO_PIN_15
#define PSM_CS_3_GPIO_Port GPIOJ
#define PSM_DReady_Pin GPIO_PIN_3
#define PSM_DReady_GPIO_Port GPIOK
/* USER CODE BEGIN Private defines */
#define TCP_ID 0x03
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
