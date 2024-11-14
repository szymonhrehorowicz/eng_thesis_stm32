/*
 * coil_controller.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/coil_controller.h"
#include "measurements/temperature.h"

void CoilController_reset(CoilController_t *this)
{
    // Reset controllers
    BBController_reset(&this->BB_controller);
    PID_reset(&this->PID_controller);
    // Reset filters & PWM outputs
    for(RefTemperature_t therm_id = 0; therm_id < NUMBER_OF_THERMISTORS; ++therm_id)
    {
        PWM_setPulse(&this->PWM[therm_id].handle, 0);
        IIR_reset(&this->filters[therm_id]);
    }
    
}

void CoilController_update(CoilController_t *this)
{
    // Calculate temperatures
    for(RefTemperature_t therm_id = 0; therm_id < NUMBER_OF_THERMISTORS; ++therm_id)
    {
        this->temperatures[therm_id] = NTC_ADC2Temperature(this->raw_voltages[therm_id]);
        IIR_update(&this->filters[therm_id], this->temperatures[therm_id]);
    }

    if(this->mode == ON)
    {
        // Update controllers and set PWM
        if(this->used_controller == PID)
        {
            PID_update(&this->PID_controller, this->filters[this->ref_temp].value);
            PWM_setPulse(&this->PWM[this->ref_temp].handle, this->PID_controller.u_saturated);
        }else {
            BBController_update(&this->BB_controller, this->filters[this->ref_temp].value);
            if(this->BB_controller.command == BB_ON)
            {
                PWM_setPulse(&this->PWM[this->ref_temp].handle, this->BB_controller.u_max);
            }else
            {
                PWM_setPulse(&this->PWM[this->ref_temp].handle, this->BB_controller.u_min);        
            }
        }
    }else
    {
        CoilController_reset(this);
    }

}

void CoilController_configureFilters(CoilController_t *this, uint16_t cutoff_freq)
{
    for(RefTemperature_t therm_id = 0; therm_id < NUMBER_OF_THERMISTORS; ++therm_id)
    {
        IIR_setCutoffFreq(&this->filters[therm_id], cutoff_freq);
    }
}

void CoilController_setController(CoilController_t *this, UsedController_t controller)
{
    this->used_controller = controller;
    // Reset the other one?
}

void CoilController_setRefTemp(CoilController_t *this, RefTemperature_t ref_temp)
{
    this->ref_temp = ref_temp;
}

void CoilController_setRefValue(CoilController_t *this, uint16_t set_value)
{
    this->BB_controller.set_value = set_value;
    this->PID_controller.set_value = set_value;
    BBController_reset(&this->BB_controller);
    PID_reset(&this->PID_controller);
}
