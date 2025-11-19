/*
 * iir_filter.h
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#ifndef INC_UTILS_IIR_FILTER_H_
#define INC_UTILS_IIR_FILTER_H_

#include "stdint.h"
#define FILTER_ORDER 7

typedef struct
{
    float value;
    double x[FILTER_ORDER + 1];
    double y[FILTER_ORDER + 1];
} IIRfilter_t;

void IIR_update(IIRfilter_t *self, float current_value);
void IIR_setCutoffFreq(IIRfilter_t *self, double freq);
void IIR_reset(IIRfilter_t *self);

#endif /* INC_UTILS_IIR_FILTER_H_ */
