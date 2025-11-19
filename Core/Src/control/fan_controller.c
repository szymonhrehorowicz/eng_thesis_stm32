/*
 * fan_controller.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/fan_controller.h"
#include "math.h"
#include "stm32f4xx_hal_gpio.h"


FanController_t fanController;

void FanController_init(FanController_t *self)
{
    // Operation
    self->mode = OFF;

    // Filter
    // self->filter.sample_time = SAMPLE_TIME_MS; // ms
    // IIR_setCutoffFreq(&self->filter, 1);

    // BB controller
    BBController_reset(&self->BB_controller);
    BBController_setParams(&self->BB_controller, 0.0f);

    // PID controller
    PID_reset(&self->PID_controller);
    self->PID_controller.sample_time = SAMPLE_TIME_MS;
    self->error = 0;
    self->PID_controller.Kp = 0;
    self->PID_controller.Ki = 0;
    self->PID_controller.Kd = 0;
    self->PID_controller.Kaw = 0;
    self->u_max = FAN_U_MAX;
    self->u_min = FAN_U_MIN;
    // self->PID_controller.u_d.sample_time = SAMPLE_TIME_MS;
    // IIR_setCutoffFreq(&(self->PID_controller.u_d), 5);

    // PWM controller
    self->PWM.channel = TIM_CHANNEL_1;
    self->PWM.handle = &htim1;
    PWM_setPulse(&self->PWM, 0);
}

void FanController_update(FanController_t *self)
{
    // Filter the raw speed measurement
    IIR_update(&self->filter, self->speed);
    ControlReference_update(&(self->control_reference));

    if (self->mode == COMBINED)
    {
        return;
    }

    if (self->mode == ON)
    {
        HAL_GPIO_WritePin(LED_FAN_GPIO_Port, LED_FAN_Pin, GPIO_PIN_SET);
        self->error = self->control_reference.ref_value - self->filter.value;
        uint8_t skip_saturation = 0;

        if (self->used_controller == PID)
        {
            // PID
            self->u = PID_update(&self->PID_controller, self->error, self->u_saturated - self->u);
        }
        else if (self->used_controller == BANG_BANG)
        {
            // BANG BANG
            self->u = self->u_max * BBController_update(&self->BB_controller, self->error);
        }
        else
        {
            // FORCED
            self->u = self->u_max;
            skip_saturation = 1;
        }

        if (skip_saturation == 0)
        {
            if (self->u > self->u_max)
            {
                self->u_saturated = self->u_max;
            }
            else if (self->u < self->u_min)
            {
                self->u_saturated = self->u_min;
            }
            else
            {
                self->u_saturated = self->u;
            }
        }
        else
        {
            self->u_saturated = self->u;
        }

        PWM_setPulse(&(self->PWM), self->u_saturated);
    }
    else
    {
        self->error = 0;
        PWM_setPulse(&self->PWM, 0);
        HAL_GPIO_WritePin(FAN_ON_GPIO_Port, FAN_ON_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_FAN_GPIO_Port, LED_FAN_Pin, GPIO_PIN_RESET);
    }
}

void FanController_setFilters(FanController_t *self, uint16_t cutoff_freq)
{
    IIR_setCutoffFreq(&self->filter, cutoff_freq);
}

void FanController_setController(FanController_t *self, UsedController_t controller)
{
    self->used_controller = controller;
}

void FanController_setRefValue(FanController_t *self, uint16_t set_value)
{
    BBController_reset(&self->BB_controller);
    PID_reset(&self->PID_controller);

    float hysteresis = self->BB_controller.threshold_top - self->BB_controller.threshold_bottom;
    BBController_setParams(&self->BB_controller, hysteresis);
}

void FanController_setMode(FanController_t *self, OperationMode_t mode)
{
    self->mode = mode;
}

void FanController_reset(FanController_t *self)
{
    self->u = 0;
    self->u_saturated = 0;
    BBController_reset(&self->BB_controller);
    PID_reset(&self->PID_controller);
    self->error = 0;
}
