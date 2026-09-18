/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define OLED_DC_Pin GPIO_PIN_4
#define OLED_DC_GPIO_Port GPIOE
#define OLED_RES_Pin GPIO_PIN_13
#define OLED_RES_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_14
#define OLED_SDA_GPIO_Port GPIOC
#define OLED_SCL_Pin GPIO_PIN_15
#define OLED_SCL_GPIO_Port GPIOC
#define VersionBit0_Pin GPIO_PIN_12
#define VersionBit0_GPIO_Port GPIOC
#define UserBuzzer_Pin GPIO_PIN_4
#define UserBuzzer_GPIO_Port GPIOD
#define VersionBit1_Pin GPIO_PIN_6
#define VersionBit1_GPIO_Port GPIOD
#define VersionBit2_Pin GPIO_PIN_7
#define VersionBit2_GPIO_Port GPIOD
#define UserKey_Pin GPIO_PIN_3
#define UserKey_GPIO_Port GPIOD
#define EscSoftUartRx_Pin GPIO_PIN_15
#define EscSoftUartRx_GPIO_Port GPIOD
#define HallA_Pin GPIO_PIN_13
#define HallA_GPIO_Port GPIOE
#define HallB_Pin GPIO_PIN_14
#define HallB_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
//全局调试使用
typedef struct{
	uint8_t powerlost;
	uint8_t chargingfull;
	uint8_t chargingON;
}DebugType_t;

//extern DebugType_t g_sys_debug;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
