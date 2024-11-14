/*
 * controller_enums.h
 *
 *  Created on: Nov 14, 2024
 *      Author: Szymon
 */

#ifndef INC_CONTROL_CONTROLLER_ENUMS_H_
#define INC_CONTROL_CONTROLLER_ENUMS_H_

typedef enum
{
    TEMP_TOP,
    TEMP_BOTTOM,
} RefTemperature_t;

typedef enum
{
    PID,
    BANG_BANG,
} UsedController_t;

typedef enum
{
    OFF,
    ON,
} OperationMode_t;

#endif /* INC_CONTROL_CONTROLLER_ENUMS_H_ */
