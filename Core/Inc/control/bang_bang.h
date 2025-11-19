/*
 * bang_bang.h
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#ifndef INC_CONTROL_BANG_BANG_H_
#define INC_CONTROL_BANG_BANG_H_

#include "stdint.h"

typedef enum
{
    BB_OFF,
    BB_ON
} BBCommand_t;

typedef struct
{
    float threshold_top;
    float threshold_bottom;
    BBCommand_t command;
} BBController_t;

BBCommand_t BBController_update(BBController_t *self, float error);
void BBController_setParams(BBController_t *self, float hysteresis);
void BBController_reset(BBController_t *self);

#endif /* INC_CONTROL_BANG_BANG_H_ */
