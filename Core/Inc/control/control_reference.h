/*
 * control_reference.h
 *
 *  Created on: Dec 12, 2024
 *      Author: Szymon
 */

#ifndef INC_CONTROL_CONTROL_REFERENCE_H_
#define INC_CONTROL_CONTROL_REFERENCE_H_

#include "stdint.h"

typedef enum
{
    STEP,
    RAMP,
    SINEWAVE,
} ControlReferenceType_t;

typedef struct
{
    float slope;
} RampReference_t;

typedef struct
{
    float amplitude;
    float omega;
} SinewaveReference_t;

typedef struct ControlReference
{
    uint16_t               sample_time;
    float                  time;
    ControlReferenceType_t type;
    float                  set_value;
    float                  ref_value;
    RampReference_t        *ramp;
    SinewaveReference_t    *sinewave;
} ControlReference_t;

void ControlReference_init(ControlReference_t *fan_ref, ControlReference_t *coil_ref);
void ControlReference_update(ControlReference_t* this);

void ControlReference_setStepReference(ControlReference_t *this, uint16_t set_value);
void ControlReference_setRampReference(ControlReference_t *this, uint16_t set_value, uint16_t slope /* unit/s */);
void ControlReference_setSineReference(ControlReference_t *this, uint16_t set_value, uint16_t amplitude, float omega);

#endif /* INC_CONTROL_CONTROL_REFERENCE_H_ */
