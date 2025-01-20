/*
 * iir_filter.c
 *
 *  Created on: Nov 13, 2024
 *      Author: Szymon
 */

#include "utils/iir_filter.h"
#include "math.h"

// Numerator coefficients (b): [2.94123952e-09 2.05886766e-08 6.17660299e-08 1.02943383e-07
//  1.02943383e-07 6.17660299e-08 2.05886766e-08 2.94123952e-09]
// Denominator coefficients (a): [  1.          -6.43532476  17.76978865 -27.29100833  25.17605177
//  -13.94983801   4.2985741   -0.56824305]

static double b[FILTER_ORDER + 1] = {2.94123952e-09, 2.05886766e-08, 6.17660299e-08, 1.02943383e-07,
 1.02943383e-07, 6.17660299e-08, 2.05886766e-08, 2.94123952e-09};
static double a[FILTER_ORDER + 1] = {  1.0,         -6.43532476,  17.76978865, -27.29100833,  25.17605177,
 -13.94983801,   4.2985741,   -0.56824305};

void IIR_update(IIRfilter_t *this, float current_value)
{
    for (int i = FILTER_ORDER; i > 0; i--) {
        this->x[i] = this->x[i - 1];
        this->y[i] = this->y[i - 1];
    }

    this->x[0] = current_value;

    this->y[0] = 0.0;
    for (int i = 0; i <= FILTER_ORDER; i++) {
        this->y[0] += b[i] * this->x[i];
        if (i > 0) {
            this->y[0] -= a[i] * this->y[i];
        }
    }

    this->value = this->y[0];
}

void IIR_setCutoffFreq(IIRfilter_t *this, double freq)
{
//    const double ff  = freq / (1.0f / (this->sample_time / 1000.0f));
//    const double ita = 1.0f / tan(M_PI * ff);
//    const double q   = sqrt(2.0f);
//    this->b0 = 1.0f / (1.0f + (q * ita) + (ita * ita));
//    this->b1 = 2.0f * this->b0;
//    this->b2 = this->b0;
//    this->a1 = 2.0f * ((ita * ita ) - 1.0f) * this->b0;
//    this->a2 = -(1.0f - (q * ita) + (ita * ita)) * this->b0;
}

void IIR_reset(IIRfilter_t *this)
{
    this->value = 0;   
//    this->p_value = 0;
//    this->pp_value = 0;
//    this->p_r_value = 0;
//    this->pp_r_value = 0;
}
