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

#define NUMBER_OF_THERMISTORS 2u

typedef enum
{
    TEMP_TOP,
    TEMP_BOTTOM,
} RefTemperature_t;

typedef enum
{
    PID,
    BANG_BANG,
} UsedController_t;

typedef enum
{
    OFF,
    ON,
} OperationMode_t;

typedef struct
{
    IIRfilter_t filters[NUMBER_OF_THERMISTORS];
    uint16_t temperatures[NUMBER_OF_THERMISTORS];
    uint16_t raw_voltages[NUMBER_OF_THERMISTORS];
    RefTemperature_t ref_temp;
    PID_t PID_controller;
    BBController_t BB_controller;
    UsedController_t used_controller;
    // PWM control handler
    OperationMode_t mode;
} CoilController_t;

void CoilController_reset(CoilController_t *this);
void CoilController_update(CoilController_t *this);
void CoilController_configureFilters(CoilController_t *this, uint16_t cutoff_freq);
void CoilController_useController(CoilController_t *this, UsedController_t controller);

#endif /* INC_CONTROL_COIL_CONTROLLER_H_ */
