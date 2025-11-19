/*
 * bang_bang.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/bang_bang.h"

BBCommand_t BBController_update(BBController_t *self, float error)
{
    if ((self->command == BB_OFF) && (error >= self->threshold_top))
    {
        self->command = BB_ON;
    }

    if ((self->command == BB_ON) && (error <= self->threshold_bottom))
    {
        self->command = BB_OFF;
    }

    return self->command;
}

void BBController_reset(BBController_t *self)
{
    self->command = BB_OFF;
}

void BBController_setParams(BBController_t *self, float hysteresis)
{
    self->threshold_top = hysteresis / 2.0f;
    self->threshold_bottom = -hysteresis / 2.0f;
}
