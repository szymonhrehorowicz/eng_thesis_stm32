/*
 * control_reference.c
 *
 *  Created on: Dec 12, 2024
 *      Author: Szymon
 */

#include "control/control_reference.h"
#include "control/controller_enums.h"
#include "cmath"

volatile ControlReference_t fanPIDreference;
volatile ControlReference_t coilPIDreference;

RampReference_t fanRampReference;
RampReference_t coilRampReference;
SinewaveReference_t fanSineReference;
SinewaveReference_t coilSineReference;

void ControlReference_init(ControlReference_t *fan_ref, ControlReference_t *coil_ref)
{
    fan_ref->time = 0;
    fan_ref->sample_time = SAMPLE_TIME_MS;
    fan_ref->set_value = 0;
    fan_ref->ref_value = 0;
    fan_ref->ramp = &fanRampReference;
    fan_ref->sinewave = &fanSineReference;

    fanRampReference.slope = 0;
    fanSineReference.amplitude = 0;
    fanSineReference.omega = 0;

    coil_ref->time = 0;
    coil_ref->sample_time = SAMPLE_TIME_MS;
    coil_ref->set_value = 0;
    coil_ref->ref_value = 0;
    coil_ref->ramp = &coilRampReference;
    coil_ref->sinewave = &coilSineReference;

    coilRampReference.slope = 0;
    coilSineReference.amplitude = 0;
    coilSineReference.omega = 0;
}

void ControlReference_update(ControlReference_t* this)
{
    this->time += this->sample_time;

    switch(this->type)
    {
        case STEP:
            this->ref_value = this->set_value;
            break;
        case RAMP:
            uint16_t new_reference = this->ref_value + this->ramp->slope;
            if(new_reference < this->set_value)
            {
                this->ref_value = new_reference;
            }
            break;
        case SINEWAVE:
            this->ref_value = this->set_value + (this->sinewave->amplitude * sin((float)this->sinewave->omega * this->time));
            break;
        default:
            break;
    }
}

void set_rampReference(ControlReference_t *this, uint16_t set_value, uint16_t slope /* unit/s */)
{
    this->type = RAMP;
    this->set_value = set_value;
    
    this->ramp->slope = slope * ((float)this->sample_time / 1000.0f);

    this->time = 0;
    this->ref_value = 0;
}

void set_sineReference(ControlReference_t *this, uint16_t set_value, uint16_t amplitude, uint16_t omega)
{
    this->type = SINEWAVE;
    this->set_value = set_value;
    
    this->sinewave->amplitude = amplitude;
    this->sinewave->omega = omega;

    this->time = 0;
    this->ref_value = 0;
}