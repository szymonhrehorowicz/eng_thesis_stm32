/*
 * pid.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/pid.h"

void PID_update(PID_t *this, uint16_t current_value)
{
    this->errror = (int16_t)this->set_value - (int16_t)current_value;

    // Calculate parts of control signal
    this->u_p = (float)this->Kp * this->errror;
    this->u_i = ((float)this->Ki * this->errror * this->sample_time) +
                  (float)this->Kaw * (this->u_saturated - this->u) * this->sample_time;
    this->u_d = (float)this->Kd * (this->errror - this->prev_error) * this->sample_time;

    // Calculate control signal
    this->u = this->u_p * this->u_i * this->u_d;

    // If out of bound, saturate control signal
    if(this->u > this->max)
    {
        this->u_saturated = this->max;
    }else if(this->u < this->min)
    {
        this->u_saturated = this->min;
    }else
    {
        this->u_saturated = this->u;
    }

    this->prev_error = this->errror;
}

void PID_reset(PID_t *this)
{
    this->errror = 0;
    this->prev_error = 0;
    this->u = 0;
    this->u_saturated = 0;
    this->u_p = 0;
    this->u_i = 0;
    this->u_d = 0;
}
