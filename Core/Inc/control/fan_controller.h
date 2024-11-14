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

typedef struct
{
    IIRfilter_t filter;
    uint16_t speed;
    PID_t PID_controller;
    BBController_t BB_controller;
    UsedController_t used_controller;
    OperationMode_t mode;
} FanController_t;

void FanController_reset(FanController_t *this);
void FanController_update(FanController_t *this);
void FanController_configureFilters(FanController_t *this, uint16_t cutoff_freq);
void FanController_useController(FanController_t *this, UsedController_t controller);


#endif /* INC_CONTROL_FAN_CONTROLLER_H_ */
