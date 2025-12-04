/*
 * pid.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "control/pid.h"

float PID_update(PID_t *self, float error, float control_difference)
{
    // Calculate proportional part
    self->u_p = (float)self->Kp * (float)error;

    // Calculate integral part
    self->integral_sum += error * self->sample_time / 1000.0f;
    self->aw_integral_sum += (float)control_difference * (float)self->sample_time / 1000.0f;
    self->u_i = (self->Ki * self->integral_sum) + (self->Kaw * self->aw_integral_sum);

    // Calculate derivative part
    IIR_update(&self->error_difference, error - self->prev_error);
    self->u_d = (float)self->Kd * (error - self->prev_error) / ((float)self->sample_time / 1000.0f);

    // Calculate control signal
    self->prev_error = error;

    return self->u_p + self->u_i + self->u_d;
}

void PID_reset(PID_t *self)
{
    self->prev_error = 0;
    self->integral_sum = 0;
    self->aw_integral_sum = 0;
    self->u_p = 0;
    self->u_i = 0;
    self->u_d = 0;
}
