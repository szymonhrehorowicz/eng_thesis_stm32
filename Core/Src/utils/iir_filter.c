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
    // y(n)  =  b0.x(n) + b1.x(n-1) + b2.x(n-2) + a1.y(n-1) + a2.y(n-2)
    this->value = (this->b0 * current_value) +
                  (this->b1 * this->p_r_value) +
                  (this->b2 * this->pp_r_value) +
                  (this->a1 * this->p_value) +
                  (this->a2 * this->pp_value);

    this->pp_value = this->p_value;
    this->p_value = this->value;
    this->pp_r_value = this->p_r_value;
    this->p_r_value = current_value;
}

void IIR_setCutoffFreq(IIRfilter_t *this, float freq)
{
    const double ff  = freq / (1.0f / (this->sample_time / 1000.0f));
    const double ita = 1.0f / tan(M_PI * ff);
    const double q   = sqrt(2.0f);
    this->b0 = 1.0f / (1.0f + (q * ita) + (ita * ita));
    this->b1 = 2.0f * this->b0;
    this->b2 = this->b0;
    this->a1 = 2.0f * ((ita * ita ) - 1.0f) * this->b0;
    this->a2 = -(1.0f - (q * ita) + (ita * ita)) * this->b0;
}

void IIR_reset(IIRfilter_t *this)
{
    this->value = 0;   
    this->p_value = 0;   
    this->pp_value = 0;  
    this->p_r_value = 0; 
    this->pp_r_value = 0;
}
