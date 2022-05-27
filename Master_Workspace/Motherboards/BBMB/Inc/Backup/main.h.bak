/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#define ON_SIG_Pin GPIO_PIN_3
#define ON_SIG_GPIO_Port GPIOE
#define GND_SIG_Pin GPIO_PIN_9
#define GND_SIG_GPIO_Port GPIOI
#define PRE_SIG_Pin GPIO_PIN_2
#define PRE_SIG_GPIO_Port GPIOF
#define BSD_Pin GPIO_PIN_13
#define BSD_GPIO_Port GPIOI
#define GPIO_IN0_Pin GPIO_PIN_0
#define GPIO_IN0_GPIO_Port GPIOJ
#define GPIO_IN0_EXTI_IRQn EXTI0_IRQn
#define BMS_INT_Pin GPIO_PIN_4
#define BMS_INT_GPIO_Port GPIOJ
#define BMS_INT_EXTI_IRQn EXTI4_IRQn
#define BMS_NO_FLT_Pin GPIO_PIN_12
#define BMS_NO_FLT_GPIO_Port GPIOE
#define ON_AUX_Pin GPIO_PIN_13
#define ON_AUX_GPIO_Port GPIOE
#define ON_AUX_EXTI_IRQn EXTI15_10_IRQn
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOE
#define LED0_Pin GPIO_PIN_6
#define LED0_GPIO_Port GPIOH
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOH
#define PSM_LVDS_EN_Pin GPIO_PIN_13
#define PSM_LVDS_EN_GPIO_Port GPIOB
#define GPIO_IN10_Pin GPIO_PIN_10
#define GPIO_IN10_GPIO_Port GPIOD
#define GPIO_IN10_EXTI_IRQn EXTI15_10_IRQn
#define TMC5160_DIAG1_Pin GPIO_PIN_6
#define TMC5160_DIAG1_GPIO_Port GPIOJ
#define TMC5160_DIAG0_Pin GPIO_PIN_7
#define TMC5160_DIAG0_GPIO_Port GPIOJ
#define TMC5160_CS0_Pin GPIO_PIN_1
#define TMC5160_CS0_GPIO_Port GPIOK
#define PSM_DReady_Pin GPIO_PIN_2
#define PSM_DReady_GPIO_Port GPIOK
#define PSM_CS_1_Pin GPIO_PIN_2
#define PSM_CS_1_GPIO_Port GPIOG
#define PSM_CS_2_Pin GPIO_PIN_3
#define PSM_CS_2_GPIO_Port GPIOG
#define PSM_CS_3_Pin GPIO_PIN_4
#define PSM_CS_3_GPIO_Port GPIOG
#define PSM_CS_0_Pin GPIO_PIN_0
#define PSM_CS_0_GPIO_Port GPIOI
#define HORN_Pin GPIO_PIN_3
#define HORN_GPIO_Port GPIOK
#define TMC5160_CS1_Pin GPIO_PIN_4
#define TMC5160_CS1_GPIO_Port GPIOK
#define Light_ctrl_PWR_EN_Pin GPIO_PIN_5
#define Light_ctrl_PWR_EN_GPIO_Port GPIOK
/* USER CODE BEGIN Private defines */
#define TCP_ID 0x01

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
