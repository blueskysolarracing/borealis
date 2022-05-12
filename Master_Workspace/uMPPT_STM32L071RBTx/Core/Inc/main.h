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
#include "stm32l0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM1_Pin GPIO_PIN_0
#define PWM1_GPIO_Port GPIOA
#define PV2_ADC_Pin GPIO_PIN_1
#define PV2_ADC_GPIO_Port GPIOA
#define PV1_ADC_Pin GPIO_PIN_4
#define PV1_ADC_GPIO_Port GPIOA
#define PV3_ADC_Pin GPIO_PIN_5
#define PV3_ADC_GPIO_Port GPIOA
#define PV4_ADC_Pin GPIO_PIN_6
#define PV4_ADC_GPIO_Port GPIOA
#define PV5_ADC_Pin GPIO_PIN_7
#define PV5_ADC_GPIO_Port GPIOA
#define SHUNT_ADC_Pin GPIO_PIN_0
#define SHUNT_ADC_GPIO_Port GPIOB
#define Vout_ADC_Pin GPIO_PIN_1
#define Vout_ADC_GPIO_Port GPIOB
#define GPIO_LED_Pin GPIO_PIN_2
#define GPIO_LED_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_10
#define PWM4_GPIO_Port GPIOB
#define EN3_Pin GPIO_PIN_11
#define EN3_GPIO_Port GPIOB
#define EN5_Pin GPIO_PIN_12
#define EN5_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_13
#define PWM2_GPIO_Port GPIOB
#define EN4_Pin GPIO_PIN_15
#define EN4_GPIO_Port GPIOB
#define PWM5_Pin GPIO_PIN_8
#define PWM5_GPIO_Port GPIOC
#define PWM3_Pin GPIO_PIN_9
#define PWM3_GPIO_Port GPIOC
#define EN1_Pin GPIO_PIN_8
#define EN1_GPIO_Port GPIOA
#define EN2_Pin GPIO_PIN_9
#define EN2_GPIO_Port GPIOA
#define FLT5_Pin GPIO_PIN_11
#define FLT5_GPIO_Port GPIOA
#define FLT5_EXTI_IRQn EXTI4_15_IRQn
#define MCU_OK_LED_Pin GPIO_PIN_12
#define MCU_OK_LED_GPIO_Port GPIOA
#define FLT4_Pin GPIO_PIN_5
#define FLT4_GPIO_Port GPIOB
#define FLT4_EXTI_IRQn EXTI4_15_IRQn
#define FLT3_Pin GPIO_PIN_7
#define FLT3_GPIO_Port GPIOB
#define FLT3_EXTI_IRQn EXTI4_15_IRQn
#define FLT2_Pin GPIO_PIN_8
#define FLT2_GPIO_Port GPIOB
#define FLT2_EXTI_IRQn EXTI4_15_IRQn
#define FLT1_Pin GPIO_PIN_9
#define FLT1_GPIO_Port GPIOB
#define FLT1_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
