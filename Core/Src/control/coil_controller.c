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
    this->mode = OFF;
    this->ref_temp = TEMP_TOP;
    this->ref_coil = COIL_A;

    // TEMP
    this->used_controller = BANG_BANG;

    // Filters
    // for(uint8_t filter_id = 0; filter_id < NUMBER_OF_THERMISTORS; ++filter_id)
    // {
    //     this->filters[filter_id].sample_time = SAMPLE_TIME_MS;
    //     this->filters[filter_id].p_value = 22.0f;
    //     this->filters[filter_id].pp_value = 22.0f;
    //     this->filters[filter_id].p_r_value = 22.0f;
    //     this->filters[filter_id].pp_r_value = 22.0f;
    //     this->filters[filter_id].value = 22.0f;
    //     IIR_setCutoffFreq(&(this->filters[filter_id]), 1);
    // }
    
    // BB controller
    BBController_reset(&this->BB_controller);
    BBController_setParams(&this->BB_controller, 0.0f);

    // PID controller
    PID_reset(&this->PID_controller);
    this->PID_controller.sample_time = SAMPLE_TIME_MS;
    this->error = 0;
    this->PID_controller.Kp = 0;
    this->PID_controller.Ki = 0;
    this->PID_controller.Kd = 0;
    this->PID_controller.Kaw = 0;
    this->u_max = HEATER_U_MAX;
    this->u_min = HEATER_U_MIN;
    //this->PID_controller.error_difference.sample_time = SAMPLE_TIME_MS;
    //IIR_setCutoffFreq(&(this->PID_controller.error_difference), 5);

    // PWM controllers
    this->PWM[COIL_A].channel = TIM_CHANNEL_3;
    this->PWM[COIL_B].channel = TIM_CHANNEL_4;
    this->PWM[COIL_A].handle = &htim5;
    this->PWM[COIL_B].handle = &htim5;
    PWM_setPulse(&(this->PWM[COIL_A]), 0);
    PWM_setPulse(&(this->PWM[COIL_B]), 0);
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
    ControlReference_update(&(this->control_reference));

    // Overheat protection
    if((this->temperatures[TEMP_TOP] >= MAX_TEMPERATURE) || (this->temperatures[TEMP_BOTTOM] >= MAX_TEMPERATURE))
    {
        PWM_setPulse(&(this->PWM[COIL_A]), 0);
        PWM_setPulse(&(this->PWM[COIL_B]), 0);
        HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
        return;
    } else
    {
        HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
    }

    if(this->mode == COMBINED)
    {
        return;
    }

    if (this->mode == ON)
    {
        HAL_GPIO_WritePin(LED_COIL_GPIO_Port, LED_COIL_Pin, GPIO_PIN_SET);
        this->error = this->control_reference.ref_value - this->filters[this->ref_temp].value;

        if (this->used_controller == PID)
        {
            this->u = PID_update(&this->PID_controller, this->error, (this->u_saturated * 12.0f / 1000.0f) - this->u);
        } else if(this->used_controller == BANG_BANG)
        {
            this->u = this->u_max * BBController_update(&this->BB_controller, this->error);
        } else
        {
            this->u = this->u_max;
        }

        uint16_t u_transformed = this->u / 12.0f * 1000.0f;

        if (u_transformed > this->u_max) {
            this->u_saturated = this->u_max;
        } else if (u_transformed < this->u_min) {
            this->u_saturated = this->u_min;
        } else {
            this->u_saturated = u_transformed;
        }

        PWM_setPulse(&(this->PWM[this->ref_coil]), this->u_saturated);
    } else
    {
        this->error = 0;
        PWM_setPulse(&(this->PWM[COIL_A]), 0);
        PWM_setPulse(&(this->PWM[COIL_B]), 0);
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
    // BBController_reset(&this->BB_controller);
    // PID_reset(&this->PID_controller);

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
    // BBController_reset(&this->BB_controller);
    // PID_reset(&this->PID_controller);

    float hysteresis = this->BB_controller.threshold_top - this->BB_controller.threshold_bottom;
    BBController_setParams(&this->BB_controller, hysteresis);
}

void CoilController_setRefCoil(CoilController_t *this, RefCoil_t coil)
{
    this->ref_coil = coil;
}

void CoilController_setMode(CoilController_t *this, OperationMode_t mode)
{
    this->mode = mode;
}

void CoilController_reset(CoilController_t *this)
{
    this->u = 0;
    this->u_saturated = 0;
    BBController_reset(&this->BB_controller);
    PID_reset(&this->PID_controller);
    this->error = 0;
}
