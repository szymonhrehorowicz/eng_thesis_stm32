/*
 * fan_controller.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/fan_controller.h"
#include "stm32f4xx_hal_gpio.h"
#include "math.h"

FanController_t fanController;

void FanController_init(FanController_t *this)
{
    // Operation
    this->mode = OFF;

    // Filter
    this->filter.sample_time = SAMPLE_TIME_MS; // ms
    IIR_setCutoffFreq(&this->filter, 1);

    // BB controller
    BBController_reset(&this->BB_controller);
    BBController_setParams(&this->BB_controller, 0.0f);

    // PID controller
    PID_reset(&this->PID_controller);
    this->PID_controller.sample_time = SAMPLE_TIME_MS;
    this->error.sample_time = SAMPLE_TIME_MS;
    IIR_setCutoffFreq(&this->error, 1);
    this->PID_controller.Kp = 0;
    this->PID_controller.Ki = 0;
    this->PID_controller.Kd = 0;
    this->PID_controller.Kaw = 0;
    this->u_max = FAN_U_MAX;
    this->u_min = FAN_U_MIN;

    // PWM controller
    this->PWM.channel = TIM_CHANNEL_1;
    this->PWM.handle  = &htim1;
    PWM_setPulse(&this->PWM, 0);
}

void FanController_update(FanController_t *this)
{
    // Filter the raw speed measurement
    IIR_update(&this->filter, this->speed);
    ControlReference_update(&(this->control_reference));
    IIR_update(&this->error, this->control_reference.ref_value - this->filter.value);

    if (this->mode == ON)
    {
        HAL_GPIO_WritePin(LED_COIL_GPIO_Port, LED_COIL_Pin, GPIO_PIN_SET);

        if (this->used_controller == PID)
        {
            this->u = PID_update(&this->PID_controller, this->error.value, this->u_saturated - this->u);
        } else
        {
            this->u = this->u_max * BBController_update(&this->BB_controller, this->error.value);
        }

        if (this->u > this->u_max) {
            this->u_saturated = this->u_max;
        }
        else if (this->u < this->u_min) {
            this->u_saturated = this->u_min;
        } else {
            this->u_saturated = this->u;
        }

        PWM_setPulse(&(this->PWM), this->u_saturated);
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

    float hysteresis = this->BB_controller.threshold_top - this->BB_controller.threshold_bottom;
    BBController_setParams(&this->BB_controller, hysteresis);
}

void FanController_setMode(FanController_t *this, OperationMode_t mode)
{
    this->mode = mode;
}

void FanController_reset(FanController_t *this)
{
    this->u = 0;
    this->u_saturated = 0;
    BBController_reset(&this->BB_controller);
    PID_reset(&this->PID_controller);
    IIR_reset(&this->error);
}
