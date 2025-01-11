/*
 * iir_filter.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "utils/iir_filter.h"
#include "math.h"

void IIR_update(IIRfilter_t *this, float current_value)
{
    this->value = (this->b * this->prev_value)
            + ((1 - this->b) * current_value);
    this->prev_value = this->value;
}

void IIR_setCutoffFreq(IIRfilter_t *this, float freq)
{
    float tau = 1.0f / (2.0f * M_PI * freq);
    this->b = exp((-1.0f * (this->sample_time / 1000.0f)) / tau);
}

void IIR_reset(IIRfilter_t *this)
{
    this->prev_value = 0;
    this->value = 0;
}
