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
    TEMP_TOP, TEMP_BOTTOM, // top - outter, bottom - inner
} RefTemperature_t;

typedef enum
{
    COIL_A,
    COIL_B,
} RefCoil_t;

typedef enum
{
    PID, BANG_BANG, FORCED
} UsedController_t;

typedef enum
{
    OFF, ON, COMBINED
} OperationMode_t;

#define SAMPLE_TIME_MS 10.0f // miliseconds
#define MAX_TEMPERATURE 100  // deg C
#define MIN_TEMPERATURE 0    // deg C
#define HEATER_U_MAX (1000-1)
#define HEATER_U_MIN 0
#define FAN_U_MAX (1680-1)
#define FAN_U_MIN (168-1)

#endif /* INC_CONTROL_CONTROLLER_ENUMS_H_ */
