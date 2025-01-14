/*
 * control_reference.c
 *
 *  Created on: Dec 12, 2024
 *      Author: Szymon
 */

#include "control/control_reference.h"
#include "control/controller_enums.h"
#include "math.h"

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
    fan_ref->type = STEP;

    fanRampReference.slope = 0;
    fanSineReference.amplitude = 0;
    fanSineReference.omega = 0;

    coil_ref->time = 0;
    coil_ref->sample_time = SAMPLE_TIME_MS;
    coil_ref->set_value = 0;
    coil_ref->ref_value = 0;
    coil_ref->ramp = &coilRampReference;
    coil_ref->sinewave = &coilSineReference;
    coil_ref->type = STEP;

    coilRampReference.slope = 0;
    coilSineReference.amplitude = 0;
    coilSineReference.omega = 0;
}

void ControlReference_update(ControlReference_t* this)
{
    this->time += (float)this->sample_time/1000.0f;

    switch(this->type)
    {
        case STEP:
            this->ref_value = this->set_value;
            break;
        case RAMP:
            float new_ramp_reference = this->ref_value + this->ramp->slope;
            if(new_ramp_reference < this->set_value)
            {
                this->ref_value = new_ramp_reference;
            }
            break;
        case SINEWAVE:
            int new_sine_reference = (int)this->set_value + (int)(((double)this->sinewave->amplitude * sin(this->sinewave->omega * this->time)));
            this->ref_value = new_sine_reference > 0.0f ? (uint16_t)new_sine_reference : 0;
            break;
        default:
            break;
    }
}

void ControlReference_setStepReference(ControlReference_t *this, uint16_t set_value)
{
    this->type = STEP;
    this->set_value = set_value;

    this->time = 0;
    this->ref_value = set_value;
}

void ControlReference_setRampReference(ControlReference_t *this, uint16_t set_value, uint16_t slope /* unit/s */)
{
    this->type = RAMP;
    this->set_value = set_value;
    
    this->ramp->slope = (float)slope * ((float)this->sample_time / 1000.0f);

    this->time = 0;
    this->ref_value = 0;
}

void ControlReference_setSineReference(ControlReference_t *this, uint16_t set_value, uint16_t amplitude, float omega)
{
    this->type = SINEWAVE;
    this->set_value = set_value;
    
    this->sinewave->amplitude = set_value != 0 ? amplitude : 0;
    this->sinewave->omega = set_value != 0 ? omega : 0.0f;

    this->time = 0;
    this->ref_value = 0;
}
