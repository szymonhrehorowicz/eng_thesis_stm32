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
    float value;
    float prev_value;
    float b;   // coefficient
    float sample_time; // ms
} IIRfilter_t;

void IIR_update(IIRfilter_t *this, float current_value);
void IIR_setCutoffFreq(IIRfilter_t *this, float freq);
void IIR_reset(IIRfilter_t *this);

#endif /* INC_UTILS_IIR_FILTER_H_ */
