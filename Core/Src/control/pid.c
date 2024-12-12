/*
 * pid.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/pid.h"

void PID_update(PID_t *this, uint16_t current_value)
{
    this->error = (float)this->set_value - (float)current_value;

    // Calculate proportional part
    this->u_p = (float) this->Kp * (float) this->error;

    // Calculate integral part
    this->integral_sum += this->error * this->sample_time / 1000.0f;
    this->aw_integral_sum += (this->u_saturated - this->u) * this->sample_time / 1000.0f;
    this->u_i = (this->Ki * this->integral_sum) + (this->Kaw * this->aw_integral_sum);

    // Calculate derivative part
    this->u_d = (float) this->Kd * (this->error - this->prev_error)
            * this->sample_time / 1000.0f;

    // Calculate control signal
    this->u = this->u_p + this->u_i + this->u_d;

    if (this->u > this->max) {
        this->u_saturated = this->max;
    }
    else if (this->u < this->min) {
        this->u_saturated = this->min;
    } else {
        this->u_saturated = this->u;
    }

    this->prev_error = this->error;
}

void PID_reset(PID_t *this)
{
    this->error = 0;
    this->prev_error = 0;
    this->integral_sum = 0;
    this->aw_integral_sum = 0;
    this->u = 0;
    this->u_saturated = 0;
    this->u_p = 0;
    this->u_i = 0;
    this->u_d = 0;
}
