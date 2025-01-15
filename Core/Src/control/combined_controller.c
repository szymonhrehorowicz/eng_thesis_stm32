/*
 * combined_controller.c
 *
 *  Created on: Jan 14, 2025
 *      Author: Szymon
 */

#include "control/combined_controller.h"
#include "control/bang_bang.h"
#include "control/pid.h"
#include "utils/iir_filter.h"

CombinedController_t combinedController;

void CombinedController_init(CombinedController_t *this, FanController_t *p_fan, CoilController_t *p_coil)
{
    this->p_fan = p_fan;
    this->p_coil = p_coil;
}

void CombinedController_update(CombinedController_t* this)
{
    IIR_update(&(this->p_coil->error), this->p_coil->control_reference.ref_value - this->p_coil->temperatures[this->p_coil->ref_temp]);
    IIR_update(&(this->p_fan->error), -(this->p_coil->control_reference.ref_value - this->p_coil->temperatures[this->p_coil->ref_temp]));

    HAL_GPIO_WritePin(LED_FAN_GPIO_Port, LED_FAN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_COIL_GPIO_Port, LED_COIL_Pin, GPIO_PIN_SET);

    // Update set coil controller
    if(this->p_coil->used_controller == PID)
    {
        this->p_coil->u = PID_update(&(this->p_coil->PID_controller),
            this->p_coil->error.value,
            this->p_coil->u_saturated - this->p_coil->u);
    }else 
    {
        this->p_coil->u = this->p_coil->u_max * BBController_update(&(this->p_coil->BB_controller),
            this->p_coil->error.value);
    }

    // Update set fan controller
    if(this->p_fan->used_controller == PID)
    {
        this->p_fan->u = PID_update(&(this->p_fan->PID_controller),
            this->p_fan->error.value,
            this->p_fan->u_saturated - this->p_fan->u);
    }else 
    {
        this->p_fan->u = this->p_fan->u_max * BBController_update(&(this->p_fan->BB_controller),
            this->p_fan->error.value);
    }

    // Saturate coil control signal
    if (this->p_coil->u > this->p_coil->u_max) {
        this->p_coil->u_saturated = this->p_coil->u_max;
    }
    else if (this->p_coil->u < this->p_coil->u_min) {
        this->p_coil->u_saturated = this->p_coil->u_min;
    } else {
        this->p_coil->u_saturated = this->p_coil->u;
    }

    // Saturate fan control signal
    if (this->p_fan->u > this->p_fan->u_max) {
        this->p_fan->u_saturated = this->p_fan->u_max;
    }
    else if (this->p_fan->u < this->p_fan->u_min) {
        this->p_fan->u_saturated = this->p_fan->u_min;
    } else {
        this->p_fan->u_saturated = this->p_fan->u;
    }

    PWM_setPulse(&(this->p_coil->PWM[this->p_coil->ref_coil]), this->p_coil->u_saturated);
    PWM_setPulse(&(this->p_fan->PWM), this->p_fan->u_saturated);
}
