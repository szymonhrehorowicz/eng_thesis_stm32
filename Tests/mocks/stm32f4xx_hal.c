#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "tim.h"

TIM_HandleTypeDef htim1 = {};
TIM_HandleTypeDef htim5 = {};

int HAL_GPIO_WritePin(int, int, int)
{
    return 0;
}

void PWM_setPulse(PWMController_t *, uint16_t)
{
    return;
}