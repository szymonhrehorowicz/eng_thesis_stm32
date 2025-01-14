/*
 * fan_controller.h
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#ifndef INC_CONTROL_FAN_CONTROLLER_H_
#define INC_CONTROL_FAN_CONTROLLER_H_

#include "stdint.h"
#include "utils/iir_filter.h"
#include "control/bang_bang.h"
#include "control/pid.h"
#include "control/controller_enums.h"
#include "control/control_reference.h"
#include "tim.h"

typedef struct
{
    IIRfilter_t error;
    IIRfilter_t filter;
    uint16_t speed;
    PID_t PID_controller;
    BBController_t BB_controller;
    UsedController_t used_controller;
    PWMController_t PWM;
    OperationMode_t mode;
    ControlReference_t control_reference;
    uint16_t u;
    uint16_t u_saturated;
    uint16_t u_max;
    uint16_t u_min;
} FanController_t;

void FanController_init(FanController_t *this);
void FanController_update(FanController_t *this);
void FanController_setFilters(FanController_t *this, uint16_t cutoff_freq);
void FanController_setController(FanController_t *this,
        UsedController_t controller);
void FanController_setRefValue(FanController_t *this, uint16_t set_value);
void FanController_setMode(FanController_t *this, OperationMode_t mode);
void FanController_reset(FanController_t *this);

extern FanController_t fanController;

#endif /* INC_CONTROL_FAN_CONTROLLER_H_ */
