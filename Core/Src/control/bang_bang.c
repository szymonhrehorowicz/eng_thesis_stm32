/*
 * bang_bang.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/bang_bang.h"

BBCommand_t BBController_update(BBController_t *this, float error)
{
    if((this->command == BB_OFF) && (error >= this->threshold_top))
    {
        this->command = BB_ON;
    }

    if((this->command == BB_ON) && (error <= this->threshold_bottom))
    {
        this->command = BB_OFF;
    }

    return this->command;
}

void BBController_reset(BBController_t *this)
{
    this->command = BB_OFF;
}

void BBController_setParams(BBController_t *this, float hysteresis)
{
    this->threshold_top = hysteresis / 2.0f;
    this->threshold_bottom = -hysteresis / 2.0f;
}
