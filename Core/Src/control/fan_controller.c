/*
 * fan_controller.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/fan_controller.h"
#include "stm32f4xx_hal_gpio.h"

FanController_t fanController;

void FanController_init(FanController_t *this)
{
    // Operation
    this->mode = OFF;

    // Filter
    this->filter.sample_time = SAMPLE_TIME_MS; // ms
    IIR_setCutoffFreq(&this->filter, 100);

    // BB controller
    BBController_reset(&this->BB_controller);
    this->BB_controller.u_max = 160 - 1;
    this->BB_controller.u_min = 0;
    BBController_setParams(&this->BB_controller, 2000, 200, 0);

    // PID controller
    PID_reset(&this->PID_controller);
    this->PID_controller.sample_time = SAMPLE_TIME_MS;
    this->PID_controller.Kp = 10;
    this->PID_controller.Ki = 1;
    this->PID_controller.Kd = 1;
    this->PID_controller.Kaw = 1;
    this->PID_controller.max = 160-1;
    this->PID_controller.min = 0;

    // PWM controller
    this->PWM.channel = TIM_CHANNEL_1;
    this->PWM.handle  = &htim1;
    PWM_setPulse(&this->PWM, 0);
}

void FanController_update(FanController_t *this)
{
    // Filter the raw speed measurement
    IIR_update(&this->filter, this->speed);

    if (this->mode == ON)
    {
        HAL_GPIO_WritePin(FAN_ON_GPIO_Port, FAN_ON_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_FAN_GPIO_Port, LED_FAN_Pin, GPIO_PIN_SET);

        if (this->used_controller == PID)
        {
            PID_update(&this->PID_controller, this->filter.value);
            PWM_setPulse(&this->PWM, this->PID_controller.u_saturated);
        } else
        {
            BBController_update(&this->BB_controller, this->filter.value);
            if(this->BB_controller.command == BB_ON)
            {
                PWM_setPulse(&this->PWM, this->BB_controller.u_max);
            }else
            {
                PWM_setPulse(&this->PWM, this->BB_controller.u_min);
            }
        }
    } else
    {
        PWM_setPulse(&this->PWM, 0);
        HAL_GPIO_WritePin(FAN_ON_GPIO_Port, FAN_ON_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_FAN_GPIO_Port, LED_FAN_Pin, GPIO_PIN_RESET);
    }
}

void FanController_setFilters(FanController_t *this, uint16_t cutoff_freq)
{
    IIR_setCutoffFreq(&this->filter, cutoff_freq);
}

void FanController_setController(FanController_t *this,
        UsedController_t controller)
{
    this->used_controller = controller;
}

void FanController_setRefValue(FanController_t *this, uint16_t set_value)
{
    BBController_reset(&this->BB_controller);
    PID_reset(&this->PID_controller);

    uint16_t hysteresis = this->BB_controller.threshold_top - this->BB_controller.threshold_bottom;
    BBController_setParams(&this->BB_controller, set_value, hysteresis, 0);
    this->PID_controller.set_value = set_value;
}

void FanController_setMode(FanController_t *this, OperationMode_t mode)
{
    this->mode = mode;
}
