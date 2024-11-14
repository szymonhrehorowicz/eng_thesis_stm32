/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdint.h"
/*
    TIM1 - FAN CTRL
    TIM3 - FAN TACH
    TIM5 - COIL CTRL
*/
/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim5;

/* USER CODE BEGIN Private defines */
typedef struct
{
  TIM_HandleTypeDef *handle;
  uint8_t channel;
} PWMController_t;

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
void PWM_setPulse(PWMController_t *this, uint16_t pulse);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */