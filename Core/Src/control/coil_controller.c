/*
 * coil_controller.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/coil_controller.h"
#include "control/controller_enums.h"
#include "measurements/temperature.h"

CoilController_t coilController;

void CoilController_init(CoilController_t *self)
{
    // Operation
    self->mode = OFF;
    self->ref_temp = TEMP_TOP;
    self->ref_coil = COIL_A;

    // TEMP
    self->used_controller = BANG_BANG;

    // Filters
    // for(uint8_t filter_id = 0; filter_id < NUMBER_OF_THERMISTORS; ++filter_id)
    // {
    //     self->filters[filter_id].sample_time = SAMPLE_TIME_MS;
    //     self->filters[filter_id].p_value = 22.0f;
    //     self->filters[filter_id].pp_value = 22.0f;
    //     self->filters[filter_id].p_r_value = 22.0f;
    //     self->filters[filter_id].pp_r_value = 22.0f;
    //     self->filters[filter_id].value = 22.0f;
    //     IIR_setCutoffFreq(&(self->filters[filter_id]), 1);
    // }

    for (int i = 0; i <= FILTER_ORDER; ++i)
    {
        self->filters[TEMP_TOP].x[i] = 0.0;
        self->filters[TEMP_TOP].y[i] = 0.0;
        self->filters[TEMP_BOTTOM].x[i] = 0.0;
        self->filters[TEMP_BOTTOM].y[i] = 0.0;
    }
    self->filters[TEMP_TOP].value = 0.0;
    self->filters[TEMP_BOTTOM].value = 0.0;

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
    self->u_max = HEATER_U_MAX;
    self->u_min = HEATER_U_MIN;
    self->u = 0;
    self->u_saturated = 0;
    // self->PID_controller.error_difference.sample_time = SAMPLE_TIME_MS;
    // IIR_setCutoffFreq(&(self->PID_controller.error_difference), 5);

    // PWM controllers
    self->PWM[COIL_A].channel = TIM_CHANNEL_3;
    self->PWM[COIL_B].channel = TIM_CHANNEL_4;
    self->PWM[COIL_A].handle = &htim5;
    self->PWM[COIL_B].handle = &htim5;
    PWM_setPulse(&(self->PWM[COIL_A]), 0);
    PWM_setPulse(&(self->PWM[COIL_B]), 0);
}

void CoilController_update(CoilController_t *self)
{
    // Calculate temperatures
    for (RefTemperature_t therm_id = 0; therm_id < NUMBER_OF_THERMISTORS; ++therm_id)
    {
        self->temperatures[therm_id] = NTC_ADC2Temperature(self->raw_voltages[therm_id]);
        IIR_update(&self->filters[therm_id], self->temperatures[therm_id]);
    }
    ControlReference_update(&(self->control_reference));

    // Overheat protection
    if ((self->temperatures[TEMP_TOP] >= MAX_TEMPERATURE) || (self->temperatures[TEMP_BOTTOM] >= MAX_TEMPERATURE))
    {
        PWM_setPulse(&(self->PWM[COIL_A]), 0);
        PWM_setPulse(&(self->PWM[COIL_B]), 0);
        HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
        return;
    }
    else
    {
        HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
    }

    if (self->mode == COMBINED)
    {
        return;
    }

    if (self->mode == ON)
    {
        HAL_GPIO_WritePin(LED_COIL_GPIO_Port, LED_COIL_Pin, GPIO_PIN_SET);
        self->error = self->control_reference.ref_value - self->filters[self->ref_temp].value;

        if (self->used_controller == PID)
        {
            self->u = PID_update(&self->PID_controller, self->error, self->u_saturated - self->u);
        }
        else if (self->used_controller == BANG_BANG)
        {
            self->u = self->u_max * BBController_update(&self->BB_controller, self->error);
        }
        else
        {
            self->u = self->u_max;
        }

        uint16_t u_transformed = self->u / 12.0f * 1000.0f;

        if (u_transformed > self->u_max)
        {
            self->u_saturated = (float)self->u_max * 12.0f / 1000.0f;
            u_transformed = self->u_max;
        }
        else if (u_transformed < self->u_min)
        {
            self->u_saturated = (float)self->u_min * 12.0f / 1000.0f;
            u_transformed = self->u_min;
        }
        else
        {
            self->u_saturated = self->u;
        }

        PWM_setPulse(&(self->PWM[self->ref_coil]), u_transformed);
    }
    else
    {
        self->error = 0;
        PWM_setPulse(&(self->PWM[COIL_A]), 0);
        PWM_setPulse(&(self->PWM[COIL_B]), 0);
        HAL_GPIO_WritePin(LED_COIL_GPIO_Port, LED_COIL_Pin, GPIO_PIN_RESET);
    }
}

void CoilController_setFilters(CoilController_t *self, uint16_t cutoff_freq)
{
    for (RefTemperature_t therm_id = 0; therm_id < NUMBER_OF_THERMISTORS; ++therm_id)
    {
        IIR_setCutoffFreq(&(self->filters[therm_id]), cutoff_freq);
    }
}

void CoilController_setController(CoilController_t *self, UsedController_t controller)
{
    // BBController_reset(&self->BB_controller);
    // PID_reset(&self->PID_controller);

    self->used_controller = controller;
}

void CoilController_setRefTemp(CoilController_t *self, RefTemperature_t ref_temp)
{
    PWM_setPulse(&(self->PWM[TEMP_TOP]), 0);
    PWM_setPulse(&(self->PWM[TEMP_BOTTOM]), 0);
    self->ref_temp = ref_temp;
}

void CoilController_setRefValue(CoilController_t *self, uint16_t set_value)
{
    // BBController_reset(&self->BB_controller);
    // PID_reset(&self->PID_controller);

    float hysteresis = self->BB_controller.threshold_top - self->BB_controller.threshold_bottom;
    BBController_setParams(&self->BB_controller, hysteresis);
}

void CoilController_setRefCoil(CoilController_t *self, RefCoil_t coil)
{
    self->ref_coil = coil;
}

void CoilController_setMode(CoilController_t *self, OperationMode_t mode)
{
    self->mode = mode;
}

void CoilController_reset(CoilController_t *self)
{
    self->u = 0;
    self->u_saturated = 0;
    BBController_reset(&self->BB_controller);
    PID_reset(&self->PID_controller);
    self->error = 0;
}
