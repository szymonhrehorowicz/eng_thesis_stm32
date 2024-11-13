/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define COIL_PWM_A_Pin GPIO_PIN_2
#define COIL_PWM_A_GPIO_Port GPIOA
#define COIL_PWM_B_Pin GPIO_PIN_3
#define COIL_PWM_B_GPIO_Port GPIOA
#define TEMP1_Pin GPIO_PIN_0
#define TEMP1_GPIO_Port GPIOB
#define TEMP2_Pin GPIO_PIN_1
#define TEMP2_GPIO_Port GPIOB
#define FAN_ON_Pin GPIO_PIN_15
#define FAN_ON_GPIO_Port GPIOB
#define FAN_TACH_Pin GPIO_PIN_6
#define FAN_TACH_GPIO_Port GPIOC
#define FAN_PWM_Pin GPIO_PIN_8
#define FAN_PWM_GPIO_Port GPIOA
#define LED_POWER_Pin GPIO_PIN_11
#define LED_POWER_GPIO_Port GPIOC
#define LED_ERROR_Pin GPIO_PIN_12
#define LED_ERROR_GPIO_Port GPIOC
#define LED_USB_Pin GPIO_PIN_2
#define LED_USB_GPIO_Port GPIOD
#define LED_COIL_Pin GPIO_PIN_3
#define LED_COIL_GPIO_Port GPIOB
#define LED_FAN_Pin GPIO_PIN_4
#define LED_FAN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
