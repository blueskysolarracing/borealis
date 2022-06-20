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
#define RELAY_HS_Pin GPIO_PIN_3
#define RELAY_HS_GPIO_Port GPIOE
#define RELAY_LS_Pin GPIO_PIN_9
#define RELAY_LS_GPIO_Port GPIOI
#define RELAY_PRECHARGE_Pin GPIO_PIN_2
#define RELAY_PRECHARGE_GPIO_Port GPIOF
#define ESD_DETECT_Pin GPIO_PIN_12
#define ESD_DETECT_GPIO_Port GPIOI
#define RELAY_DISCHARGE_Pin GPIO_PIN_13
#define RELAY_DISCHARGE_GPIO_Port GPIOI
#define BMS_FLT_Pin GPIO_PIN_4
#define BMS_FLT_GPIO_Port GPIOJ
#define BMS_NO_FLT_Pin GPIO_PIN_12
#define BMS_NO_FLT_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOE
#define BMS_WKUP_Pin GPIO_PIN_5
#define BMS_WKUP_GPIO_Port GPIOJ
#define LED0_Pin GPIO_PIN_6
#define LED0_GPIO_Port GPIOH
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOH
#define PSM_LVDS_EN_Pin GPIO_PIN_13
#define PSM_LVDS_EN_GPIO_Port GPIOB
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
#define PSM_CS_3_Pin GPIO_PIN_3
#define PSM_CS_3_GPIO_Port GPIOG
#define PSM_CS_2_Pin GPIO_PIN_4
#define PSM_CS_2_GPIO_Port GPIOG
#define PSM_CS_0_Pin GPIO_PIN_0
#define PSM_CS_0_GPIO_Port GPIOI
#define HORN_EN_Pin GPIO_PIN_3
#define HORN_EN_GPIO_Port GPIOK
#define TMC5160_CS1_Pin GPIO_PIN_4
#define TMC5160_CS1_GPIO_Port GPIOK
#define Light_ctrl_PWR_EN_Pin GPIO_PIN_5
#define Light_ctrl_PWR_EN_GPIO_Port GPIOK
/* USER CODE BEGIN Private defines */
#define TCP_ID BBMB_ID

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
