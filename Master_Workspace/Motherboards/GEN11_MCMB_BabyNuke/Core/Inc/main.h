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
#define MotorMT0_Pin GPIO_PIN_3
#define MotorMT0_GPIO_Port GPIOE
#define MotorMT1_Pin GPIO_PIN_2
#define MotorMT1_GPIO_Port GPIOF
#define MotorMT2_Pin GPIO_PIN_12
#define MotorMT2_GPIO_Port GPIOF
#define MotorMT3_Pin GPIO_PIN_13
#define MotorMT3_GPIO_Port GPIOF
#define GPIO_IN0_Pin GPIO_PIN_0
#define GPIO_IN0_GPIO_Port GPIOG
#define GPIO_IN0_EXTI_IRQn EXTI0_IRQn
#define GPIO_IN10_Pin GPIO_PIN_10
#define GPIO_IN10_GPIO_Port GPIOE
#define GPIO_IN10_EXTI_IRQn EXTI15_10_IRQn
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOE
#define LED0_Pin GPIO_PIN_15
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOB
#define PSM_CS_0_Pin GPIO_PIN_12
#define PSM_CS_0_GPIO_Port GPIOB
#define MotorVfmReset_Pin GPIO_PIN_10
#define MotorVfmReset_GPIO_Port GPIOD
#define MotorVfmDown_Pin GPIO_PIN_11
#define MotorVfmDown_GPIO_Port GPIOD
#define MotorVfmUp_Pin GPIO_PIN_14
#define MotorVfmUp_GPIO_Port GPIOD
#define MotorEco_Pin GPIO_PIN_15
#define MotorEco_GPIO_Port GPIOD
#define MotorFwdRev_Pin GPIO_PIN_2
#define MotorFwdRev_GPIO_Port GPIOG
#define MotorMain_Pin GPIO_PIN_5
#define MotorMain_GPIO_Port GPIOG
#define BBMB_PSM_CS_0_Pin GPIO_PIN_8
#define BBMB_PSM_CS_0_GPIO_Port GPIOC
#define MotorCSAccel_Pin GPIO_PIN_9
#define MotorCSAccel_GPIO_Port GPIOC
#define MotorCSRegen_Pin GPIO_PIN_12
#define MotorCSRegen_GPIO_Port GPIOC
#define PSM_LVDS_EN_Pin GPIO_PIN_4
#define PSM_LVDS_EN_GPIO_Port GPIOD
#define PSM_CS_1_Pin GPIO_PIN_5
#define PSM_CS_1_GPIO_Port GPIOD
#define PSM_CS_2_Pin GPIO_PIN_6
#define PSM_CS_2_GPIO_Port GPIOD
#define PSM_CS_3_Pin GPIO_PIN_9
#define PSM_CS_3_GPIO_Port GPIOG
#define PSM_DReady_Pin GPIO_PIN_10
#define PSM_DReady_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
