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
    float errror;
    float prev_error;
    float integral_sum;
    float aw_integral_sum;
    uint16_t set_value;
    float Kp;
    float Ki;
    float Kd;
    float Kaw; // anti-windup constant
    uint16_t u;
    uint16_t u_saturated;
    float u_p;
    float u_i;
    float u_d;
    uint16_t max;
    uint16_t min;
} PID_t;

void PID_update(PID_t *this, uint16_t current_value);
void PID_reset(PID_t *this);

#endif /* INC_CONTROL_PID_H_ */
