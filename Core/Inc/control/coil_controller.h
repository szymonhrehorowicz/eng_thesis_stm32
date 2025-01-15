/*
 * coil_controller.h
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#ifndef INC_CONTROL_COIL_CONTROLLER_H_
#define INC_CONTROL_COIL_CONTROLLER_H_

#include "stdint.h"
#include "utils/iir_filter.h"
#include "control/bang_bang.h"
#include "control/pid.h"
#include "control/controller_enums.h"
#include "control/control_reference.h"
#include "tim.h"

#define NUMBER_OF_THERMISTORS 2u

typedef struct
{
    IIRfilter_t error;
    IIRfilter_t filters[NUMBER_OF_THERMISTORS];
    float temperatures[NUMBER_OF_THERMISTORS];
    uint16_t raw_voltages[NUMBER_OF_THERMISTORS];
    RefTemperature_t ref_temp;
    RefCoil_t ref_coil;
    PID_t PID_controller;
    BBController_t BB_controller;
    UsedController_t used_controller;
    PWMController_t PWM[NUMBER_OF_THERMISTORS];
    OperationMode_t mode;
    ControlReference_t control_reference;
    int16_t u;
    uint16_t u_saturated;
    uint16_t u_max;
    uint16_t u_min;
} CoilController_t;

void CoilController_init(CoilController_t *this);
void CoilController_update(CoilController_t *this);
void CoilController_setFilters(CoilController_t *this, uint16_t cutoff_freq);
void CoilController_setController(CoilController_t *this,
        UsedController_t controller);
void CoilController_setRefTemp(CoilController_t *this,
        RefTemperature_t ref_temp);
void CoilController_setRefValue(CoilController_t *this, uint16_t set_value);
void CoilController_setRefCoil(CoilController_t *this, RefCoil_t coil);
void CoilController_setMode(CoilController_t *this, OperationMode_t mode);
void CoilController_reset(CoilController_t *this);

extern CoilController_t coilController;

#endif /* INC_CONTROL_COIL_CONTROLLER_H_ */
