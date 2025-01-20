/*
 * pid.h
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#ifndef INC_CONTROL_PID_H_
#define INC_CONTROL_PID_H_

#include "stdint.h"
#include "utils/iir_filter.h"

typedef struct
{
    uint16_t sample_time; // ms
    float prev_error;
    float integral_sum;
    float aw_integral_sum;
    float Kp;
    float Ki;
    float Kd;
    float Kaw; // anti-windup constant
    float u_p;
    float u_i;
    float u_d;
    IIRfilter_t error_difference;
} PID_t;

int16_t PID_update(PID_t *this, float error, int16_t control_difference);
void PID_update_with_error_injection(PID_t *this, float error);
void PID_reset(PID_t *this);

#endif /* INC_CONTROL_PID_H_ */
