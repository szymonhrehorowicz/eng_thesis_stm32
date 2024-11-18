/*
 * coil_controller.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/coil_controller.h"
#include "measurements/temperature.h"

CoilController_t coilController;

void CoilController_init(CoilController_t *this)
{
    // Operation
    this->mode = ON;
    this->ref_temp = TEMP_TOP;

    // TEMP
    this->used_controller = BANG_BANG;

    // Filters
    for(uint8_t filter_id = 0; filter_id < NUMBER_OF_THERMISTORS; ++filter_id)
    {
        this->filters[filter_id].sample_time = SAMPLE_TIME_MS;
        IIR_setCutoffFreq(&(this->filters[filter_id]), 100);
    }
    
    // BB controller
    BBController_reset(&this->BB_controller);
    this->BB_controller.u_max = 1000-1;
    this->BB_controller.u_min = 0;
    BBController_setParams(&this->BB_controller, 70, 10, 0);

    // PID controller
    PID_reset(&this->PID_controller);
    this->PID_controller.sample_time = SAMPLE_TIME_MS;
    this->PID_controller.Kp = 2;
    this->PID_controller.Ki = 1;
    this->PID_controller.Kd = 1;
    this->PID_controller.Kaw = 1;
    this->PID_controller.max = 1000-1;
    this->PID_controller.min = 0;

    // PWM controllers
    this->PWM[TEMP_TOP].channel = TIM_CHANNEL_3;
    this->PWM[TEMP_BOTTOM].channel = TIM_CHANNEL_4;
    this->PWM[TEMP_TOP].handle = &htim5;
    this->PWM[TEMP_BOTTOM].handle = &htim5;
    PWM_setPulse(&(this->PWM[TEMP_TOP]), 0);
    PWM_setPulse(&(this->PWM[TEMP_BOTTOM]), 0);
}

void CoilController_update(CoilController_t *this)
{
    // Calculate temperatures
    for (RefTemperature_t therm_id = 0; therm_id < NUMBER_OF_THERMISTORS;
            ++therm_id)
    {
        this->temperatures[therm_id] = NTC_ADC2Temperature(
                this->raw_voltages[therm_id]);
        IIR_update(&this->filters[therm_id], this->temperatures[therm_id]);
    }

    if (this->mode == ON)
    {
        HAL_GPIO_WritePin(LED_COIL_GPIO_Port, LED_COIL_Pin, GPIO_PIN_SET);

        if (this->used_controller == PID)
        {
            PID_update(&this->PID_controller,
                    this->filters[this->ref_temp].value);
            PWM_setPulse(&(this->PWM[this->ref_temp]),
                    this->PID_controller.u_saturated);
        } else
        {
            BBController_update(&this->BB_controller,
                    this->filters[this->ref_temp].value);
            if (this->BB_controller.command == BB_ON)
            {
                PWM_setPulse(&this->PWM[this->ref_temp].handle,
                        this->BB_controller.u_max);
            } else
            {
                PWM_setPulse(&this->PWM[this->ref_temp].handle,
                        this->BB_controller.u_min);
            }
        }
    } else
    {
        PWM_setPulse(&(this->PWM[TEMP_TOP]), 0);
        PWM_setPulse(&(this->PWM[TEMP_BOTTOM]), 0);
        HAL_GPIO_WritePin(LED_COIL_GPIO_Port, LED_COIL_Pin, GPIO_PIN_RESET);
    }

}

void CoilController_setFilters(CoilController_t *this, uint16_t cutoff_freq)
{
    for (RefTemperature_t therm_id = 0; therm_id < NUMBER_OF_THERMISTORS;
            ++therm_id)
    {
        IIR_setCutoffFreq(&(this->filters[therm_id]), cutoff_freq);
    }
}

void CoilController_setController(CoilController_t *this,
        UsedController_t controller)
{
    this->used_controller = controller;
}

void CoilController_setRefTemp(CoilController_t *this,
        RefTemperature_t ref_temp)
{
    PWM_setPulse(&(this->PWM[TEMP_TOP]), 0);
    PWM_setPulse(&(this->PWM[TEMP_BOTTOM]), 0);
    this->ref_temp = ref_temp;
}

void CoilController_setRefValue(CoilController_t *this, uint16_t set_value)
{
    this->BB_controller.set_value = set_value;
    this->PID_controller.set_value = set_value;
    BBController_reset(&this->BB_controller);
    PID_reset(&this->PID_controller);
}

void CoilController_setMode(CoilController_t *this, OperationMode_t mode)
{
    CoilController_init(this);
    this->mode = mode;
}
