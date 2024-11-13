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
    uint16_t set_value;
    uint16_t threshold_top;
    uint16_t threshold_bottom;
    uint16_t u_max;
    uint16_t u_min;
    BBCommand_t command;
} BBController_t;

void BBController_update(BBController_t *this, uint16_t current_value);
void BBController_setParams(BBController_t *this, uint16_t set_value, uint16_t hysteresis, int16_t hysteresis_shift);
void BBController_reset(BBController_t *this);

#endif /* INC_CONTROL_BANG_BANG_H_ */
