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
    uint16_t slope;
} RampReference_t;

typedef struct
{
    uint16_t amplitude;
    uint16_t omega;
} SinewaveReference_t;

typedef struct ControlReference
{
    uint16_t               sample_time;
    uint32_t               time;
    ControlReferenceType_t type;
    uint16_t               set_value;
    uint16_t               ref_value;
    RampReference_t        *ramp;
    SinewaveReference_t    *sinewave;
} ControlReference_t;

void ControlReference_update(ControlReference_t* this);

extern volatile ControlReference_t fanPIDreference;
extern volatile ControlReference_t coilPIDreference;

#endif /* INC_CONTROL_CONTROL_REFERENCE_H_ */
