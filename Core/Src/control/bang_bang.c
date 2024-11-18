/*
 * bang_bang.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/bang_bang.h"

void BBController_update(BBController_t *this, uint16_t current_value)
{
    if (current_value >= this->threshold_top)
    {
        this->command = BB_OFF;
    } else if (current_value <= this->threshold_bottom)
    {
        this->command = BB_ON;
    }
}

void BBController_reset(BBController_t *this)
{
    this->command = BB_OFF;
}

void BBController_setParams(BBController_t *this, uint16_t set_value,
        uint16_t hysteresis, int16_t hysteresis_shift)
{
    this->set_value = set_value;
    this->threshold_top = set_value + (hysteresis / 2) + hysteresis_shift;
    this->threshold_bottom = set_value - (hysteresis / 2) + hysteresis_shift;
}
