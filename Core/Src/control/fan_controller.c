/*
 * fan_controller.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/fan_controller.h"

void FanController_reset(FanController_t *this)
{
    
}

void FanController_update(FanController_t *this)
{
    IIR_update(&this->filter, this->speed);

    if(this->mode == ON)
    {
        if(this->used_controller == PID)
        {
            PID_update(&this->PID_controller, this->filter.value);
        }else
        {
            BBController_update(&this->BB_controller, this->filter.value);
        }
    } else
    {
        FanController_reset(this);
    }
}

void FanController_configureFilters(FanController_t *this, uint16_t cutoff_freq)
{
    IIR_setCutoffFreq(&this->filter, cutoff_freq);
}

void FanController_useController(FanController_t *this, UsedController_t controller)
{
    this->used_controller = controller;
}