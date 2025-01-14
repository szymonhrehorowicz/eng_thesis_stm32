/*
 * iir_filter.h
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#ifndef INC_UTILS_IIR_FILTER_H_
#define INC_UTILS_IIR_FILTER_H_

#include "stdint.h"

typedef struct
{
    float value;      // y[n]
    float p_value;    // y[n-1]
    float pp_value;   // y[n-2]
    float p_r_value;  // x[n-1]
    float pp_r_value; // x[n-2]

    float sample_time; // ms
    // Coefficients
    double b0;
    double b1;
    double b2;
    double a1;
    double a2;
} IIRfilter_t;

void IIR_update(IIRfilter_t *this, float current_value);
void IIR_setCutoffFreq(IIRfilter_t *this, float freq);
void IIR_reset(IIRfilter_t *this);

#endif /* INC_UTILS_IIR_FILTER_H_ */
