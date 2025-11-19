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

void ControlReference_update(ControlReference_t *self)
{
    self->time += (float)self->sample_time / 1000.0f;

    switch (self->type)
    {
    case STEP:
        self->ref_value = self->set_value;
        break;
    case RAMP:
        float new_ramp_reference = self->ref_value + self->ramp->slope;
        if (new_ramp_reference < self->set_value)
        {
            self->ref_value = new_ramp_reference;
        }
        break;
    case SINEWAVE:
        float new_sine_reference =
            (float)self->set_value +
            (float)(((double)self->sinewave->amplitude * sin(self->sinewave->omega * self->time)));
        self->ref_value = new_sine_reference > 0.0f ? new_sine_reference : 0;
        break;
    default:
        break;
    }
}

void ControlReference_setStepReference(ControlReference_t *self, uint16_t set_value)
{
    self->type = STEP;
    self->set_value = set_value;

    self->time = 0;
    self->ref_value = set_value;
}

void ControlReference_setRampReference(ControlReference_t *self, int16_t start_value, uint16_t set_value,
                                       uint16_t slope /* unit/s */)
{
    self->type = RAMP;
    self->set_value = set_value;

    self->ramp->slope = (float)slope * ((float)self->sample_time / 1000.0f);

    self->time = 0;
    self->ref_value = start_value;
}

void ControlReference_setSineReference(ControlReference_t *self, int16_t start_value, uint16_t set_value,
                                       uint16_t amplitude, float omega)
{
    self->type = SINEWAVE;
    self->set_value = set_value;

    self->sinewave->amplitude = set_value != 0 ? amplitude : 0;
    self->sinewave->omega = set_value != 0 ? omega : 0.0f;

    self->time = 0;
    self->ref_value = start_value;
}
