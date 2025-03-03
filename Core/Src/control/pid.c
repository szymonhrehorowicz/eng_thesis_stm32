/*
 * pid.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/pid.h"

float PID_update(PID_t *this, float error, int16_t control_difference)
{
    // Calculate proportional part
    this->u_p = (float) this->Kp * (float) error;

    // Calculate integral part
    this->integral_sum += error * this->sample_time / 1000.0f;
    this->aw_integral_sum += (float)control_difference * (float)this->sample_time / 1000.0f;
    this->u_i = (this->Ki * this->integral_sum) + (this->Kaw * this->aw_integral_sum);

    // Calculate derivative part
    IIR_update(&this->error_difference, error - this->prev_error);
    this->u_d = (float) this->Kd * (error - this->prev_error)
            / ((float)this->sample_time / 1000.0f);

    // Calculate control signal
    this->prev_error = error;

    return this->u_p + this->u_i + this->u_d;
}

void PID_reset(PID_t *this)
{
    this->prev_error = 0;
    this->integral_sum = 0;
    this->aw_integral_sum = 0;
    this->u_p = 0;
    this->u_i = 0;
    this->u_d = 0;
}
