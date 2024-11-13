/*
 * coil_controller.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "coil_controller.h"
#include "control/coil_controller.h"
#include "temperature.h"
#include "measurements/temperature.h"

void CoilController_reset(CoilController_t *this)
{
    // Reset controllers
    // Reset filters
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
        }else {
            BBController_update(&this->BB_controller, this->filters[this->ref_temp].value);
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

void CoilController_useController(CoilController_t *this, UsedController_t controller)
{
    this->used_controller = controller;
    // Reset the other one?
}