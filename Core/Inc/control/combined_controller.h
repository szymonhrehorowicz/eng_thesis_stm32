/*
 * combined_controller.h
 *
 *  Created on: Jan 14, 2025
 *      Author: Szymon
 */

#ifndef INC_CONTROL_COMBINED_CONTROLLER_H_
#define INC_CONTROL_COMBINED_CONTROLLER_H_

#include "control/coil_controller.h"
#include "control/controller_enums.h"
#include "control/fan_controller.h"
#include "stdint.h"


typedef struct
{
    FanController_t *p_fan;
    CoilController_t *p_coil;
} CombinedController_t;

void CombinedController_init(CombinedController_t *self, FanController_t *p_fan, CoilController_t *p_coil);
void CombinedController_update(CombinedController_t *self);

extern CombinedController_t combinedController;

#endif /* INC_CONTROL_COMBINED_CONTROLLER_H_ */
