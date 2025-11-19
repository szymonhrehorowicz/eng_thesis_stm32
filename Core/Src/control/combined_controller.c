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

void CombinedController_init(CombinedController_t *self, FanController_t *p_fan, CoilController_t *p_coil)
{
    self->p_fan = p_fan;
    self->p_coil = p_coil;
}

void CombinedController_update(CombinedController_t *self)
{
    self->p_coil->error =
        self->p_coil->control_reference.ref_value - self->p_coil->filters[self->p_coil->ref_temp].value;
    self->p_fan->error = self->p_coil->error;

    HAL_GPIO_WritePin(LED_FAN_GPIO_Port, LED_FAN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_COIL_GPIO_Port, LED_COIL_Pin, GPIO_PIN_SET);

    // Update set coil controller
    if (self->p_coil->used_controller == PID)
    {
        self->p_coil->u = PID_update(&(self->p_coil->PID_controller), self->p_coil->error,
                                     self->p_coil->u_saturated - self->p_coil->u);
    }
    else
    {
        self->p_coil->u =
            self->p_coil->u_max * BBController_update(&(self->p_coil->BB_controller), self->p_coil->error);
    }

    // Update set fan controller
    if (self->p_fan->used_controller == PID)
    {
        self->p_fan->u =
            PID_update(&(self->p_fan->PID_controller), self->p_fan->error, self->p_fan->u_saturated - self->p_fan->u);
    }
    else
    {
        self->p_fan->u = self->p_fan->u_max * BBController_update(&(self->p_fan->BB_controller), self->p_fan->error);
    }

    // Saturate coil control signal
    if (self->p_coil->u > self->p_coil->u_max)
    {
        self->p_coil->u_saturated = self->p_coil->u_max;
    }
    else if (self->p_coil->u < self->p_coil->u_min)
    {
        self->p_coil->u_saturated = self->p_coil->u_min;
    }
    else
    {
        self->p_coil->u_saturated = self->p_coil->u;
    }

    // Saturate fan control signal
    if (self->p_fan->u > self->p_fan->u_max)
    {
        self->p_fan->u_saturated = self->p_fan->u_max;
    }
    else if (self->p_fan->u < self->p_fan->u_min)
    {
        self->p_fan->u_saturated = self->p_fan->u_min;
    }
    else
    {
        self->p_fan->u_saturated = self->p_fan->u;
    }

    PWM_setPulse(&(self->p_coil->PWM[self->p_coil->ref_coil]), self->p_coil->u_saturated);
    PWM_setPulse(&(self->p_fan->PWM), self->p_fan->u_saturated);
}
