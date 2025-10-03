/*
 * communication.c
 *
 *  Created on: Nov 18, 2024
 *      Author: Szymon
 */

#include "communication.h"
#include "control/bang_bang.h"
#include "control/coil_controller.h"
#include "control/controller_enums.h"
#include "control/fan_controller.h"
#include "control/pid.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "tim.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "utils/char_translator.h"


#define MAX_SILIENCE_TIME_MS 15000

volatile uint8_t isConnected = 0;

void COM_checkConnection(void)
{
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
    {
        HAL_GPIO_WritePin(LED_USB_GPIO_Port, LED_USB_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED_USB_GPIO_Port, LED_USB_Pin, GPIO_PIN_RESET);
        FanController_setMode(&fanController, OFF);
        CoilController_setMode(&coilController, OFF);
        isConnected = 0;
    }
}

volatile uint32_t lastMsgTimestamp = MAX_SILIENCE_TIME_MS;
volatile uint8_t wasAlreadyReset = 0;

#define MSG_TYPE 0u
#define MSG_VAR 1u
#define MSG_BODY 3u

typedef enum
{
    CONTROLLER_TYPE = 0,             // 0/1
    BB_SET_VAL = 1,                  // 0 - 6000
    BB_HYSTERESIS = BB_SET_VAL + 4,  // 0 - 6000
    PID_SET_VAL = BB_HYSTERESIS + 4, // 0-6000
    PID_KP = PID_SET_VAL + 4,        // 0 - 100.00
    PID_KI = PID_KP + 6,             // 0 - 100.00
    PID_KD = PID_KI + 6,             // 0 - 100.00
    PID_KAW = PID_KD + 6,            // 0 - 100.00
    ON_OFF = PID_KAW + 6,            // 0/1
    CTRL_REF_TYPE = ON_OFF + 1,
    CTRL_REF_SLOPE = CTRL_REF_TYPE + 1,
    CTRL_REF_AMPL = CTRL_REF_SLOPE + 4,
    CTRL_REF_OMEGA = CTRL_REF_AMPL + 4,
    POWER = CTRL_REF_OMEGA + 6, // 0-100
    COIL_TYPE = POWER + 3,      // 0/1
    LEFT_RIGHT = COIL_TYPE + 1, // 0/1
} ConfMsgBody_t;

typedef enum
{
    APP_CON_REQ,
    FAN_CFG,
    COIL_CFG,
} IncommingMessages_t;

/*
0/1
Byte 0   : 0: FAN/COIL | 1: ALL/FAST | 2: ON/OFF | 3: TEMP_REF_TOP/TEP_REF_BOTTOM | 4: COIL_REF_TOP/COIL_REF_BOTTOM | 5:
NA| 6: NA| 7: NA|

Byte 1-4 : timestamp

if [FAN] #
 Byte 5-6: speed: u16
if [COIL]#
 Byte 5-8: temp_top: u16 | temp_bottom: u16

Rest of Bytes:
[ALL]
Bang-Bang: set_value: u16 | threshold_top: u16 | threshold_bottom: u16 | u_max: u16 | u_min: u16
PID      : set_value: u16 | error: f32 | int_sum: f32 | aw_int_sum: f32 | Kp: f32 | Ki: f32 | Kd: f32 | Kaw: f32 | u:
u16 | u_sat: f32 | u_p: f32 | u_i: f32 | u_d: f32 | max: u16 | min: u16

[FAST]
Bang-Bang: NA
PID      : error: f32 | int_sum: f32 | aw_int_sum: f32 | u: u16 | u_sat: u16 | u_p: f32 | u_i: f32 | u_d: f32
*/

typedef enum
{
    OUT_ALL_FAST,
    OUT_FAN_ON_OFF,
    OUT_COIL_ON_OFF,
    OUT_TEMP_REF,
    OUT_COIL_REF,
} Out0_Byte_t;

#define DEV_TYPE_FAN 0u
#define DEV_TYPE_COIL 1u
#define OUT_ALL 0u
#define OUT_FAST 1u

#define MSG_TYPE 0u

static uint8_t msg[200];
static uint8_t msg_idx = 0;

void COM_sendAllData(void)
{
    msg_idx = 0;
    memset(msg, 0, sizeof(msg));

    // Byte 0 : 0: ALL/FAST | 1: FAN OFF/ON | 2: COIL OFF/ON | 3: TEMP_REF_TOP/TEMP_REF_BOTTOM | 4:
    // COIL_REF_TOP/COIL_REF_BOTTOM | 5: NA| 6: NA| 7: NA|
    msg[msg_idx] |= OUT_ALL << OUT_ALL_FAST;
    msg[msg_idx] |= fanController.mode << OUT_FAN_ON_OFF;
    msg[msg_idx] |= coilController.mode << OUT_COIL_ON_OFF;
    msg[msg_idx] |= coilController.ref_temp << OUT_TEMP_REF;
    msg[msg_idx] |= coilController.ref_coil << OUT_COIL_REF;
    msg_idx++;
    // Byte 1-4 : timestamp
    uint32_t timestamp = HAL_GetTick();
    msg[msg_idx] = (uint8_t)(timestamp >> 24);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(timestamp >> 16);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(timestamp >> 8);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(timestamp & 0x000000ff);
    msg_idx++;

    /*
        FAN DATA
    */
    // Reference value
    float_to_bytes(msg, &msg_idx, &(fanController.control_reference.ref_value));
    // Error
    float_to_bytes(msg, &msg_idx, &(fanController.error));

    // Bang Bang
    // cmd
    msg[msg_idx] = (uint8_t)fanController.BB_controller.command;
    msg_idx++;
    // Threshold top
    float_to_bytes(msg, &msg_idx, &(fanController.BB_controller.threshold_top));
    // Threshold bottom
    float_to_bytes(msg, &msg_idx, &(fanController.BB_controller.threshold_bottom));
    // u_max
    msg[msg_idx] = (uint8_t)(fanController.u_max >> 8);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.u_max & 0x00ff);
    msg_idx++;
    // u_min
    msg[msg_idx] = (uint8_t)(fanController.u_min >> 8);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.u_min & 0x00ff);
    msg_idx++;

    // PID
    // Integral sum
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.integral_sum));
    // Ant-windup integral sum
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.aw_integral_sum));
    // Kp
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.Kp));
    // Ki
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.Ki));
    // Kd
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.Kd));
    // Kaw
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.Kaw));
    // u
    float_to_bytes(msg, &msg_idx, &(fanController.u));
    // u_sat
    float_to_bytes(msg, &msg_idx, &(fanController.u_saturated));
    // u_p
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.u_p));
    // u_i
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.u_i));
    // u_d
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.u_d));
    // max
    msg[msg_idx] = (uint8_t)(fanController.u_max >> 8);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.u_max & 0x00ff);
    msg_idx++;
    // min
    msg[msg_idx] = (uint8_t)(fanController.u_min >> 8);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.u_min & 0x00ff);
    msg_idx++;

    /*
        COIL DATA
    */
    // Reference value
    float_to_bytes(msg, &msg_idx, &(coilController.control_reference.ref_value));
    // Error
    float_to_bytes(msg, &msg_idx, &(coilController.error));

    // Bang Bang
    // cmd
    msg[msg_idx] = (uint8_t)coilController.BB_controller.command;
    msg_idx++;

    // Threshold top
    float_to_bytes(msg, &msg_idx, &(coilController.BB_controller.threshold_top));
    // Threshold bottom
    float_to_bytes(msg, &msg_idx, &(coilController.BB_controller.threshold_bottom));
    // u_max
    msg[msg_idx] = (uint8_t)(coilController.u_max >> 8);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.u_max & 0x00ff);
    msg_idx++;
    // u_min
    msg[msg_idx] = (uint8_t)(coilController.u_min >> 8);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.u_min & 0x00ff);
    msg_idx++;

    // PID
    // Integral sum
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.integral_sum));
    // Ant-windup integral sum
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.aw_integral_sum));
    // Kp
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.Kp));
    // Ki
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.Ki));
    // Kd
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.Kd));
    // Kaw
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.Kaw));
    // u
    float_to_bytes(msg, &msg_idx, &(coilController.u));
    // u_sat
    float_to_bytes(msg, &msg_idx, &(coilController.u_saturated));
    // u_p
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.u_p));
    // u_i
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.u_i));
    // u_d
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.u_d));
    // max
    msg[msg_idx] = (uint8_t)(coilController.u_max >> 8);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.u_max & 0x00ff);
    msg_idx++;
    // min
    msg[msg_idx] = (uint8_t)(coilController.u_min >> 8);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.u_min & 0x00ff);
    msg_idx++;

    /*
        Byte 5-10: speed: u16 | temp_top: u16 | temp_bottom: u16
    */
    float_to_bytes(msg, &msg_idx, &(fanController.filter.value));
    float_to_bytes(msg, &msg_idx, &(coilController.filters[TEMP_TOP].value));
    float_to_bytes(msg, &msg_idx, &(coilController.filters[TEMP_BOTTOM].value));

    msg[msg_idx] = '\n';
    CDC_Transmit_FS((uint8_t *)msg, msg_idx);
}

void COM_sendFastData(void)
{
    msg_idx = 0;
    memset(msg, 0, sizeof(msg));

    // Byte 0 : 0: ALL/FAST | 1: FAN OFF/ON | 2: COIL OFF/ON | 3: TEMP_REF_TOP/TEMP_REF_BOTTOM | 4:
    // COIL_REF_TOP/COIL_REF_BOTTOM | 5: NA| 6: NA| 7: NA|
    msg[msg_idx] |= OUT_FAST << OUT_ALL_FAST;
    msg[msg_idx] |= fanController.mode << OUT_FAN_ON_OFF;
    msg[msg_idx] |= coilController.mode << OUT_COIL_ON_OFF;
    msg[msg_idx] |= coilController.ref_temp << OUT_TEMP_REF;
    msg[msg_idx] |= coilController.ref_coil << OUT_COIL_REF;
    msg_idx++;
    // Byte 1-4 : timestamp
    uint32_t timestamp = HAL_GetTick();
    msg[msg_idx] = (uint8_t)(timestamp >> 24);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(timestamp >> 16);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(timestamp >> 8);
    msg_idx++;
    msg[msg_idx] = (uint8_t)(timestamp & 0x000000ff);
    msg_idx++;

    /*
        FAN DATA
    */
    // Bang Bang
    // cmd
    msg[msg_idx] = (uint8_t)fanController.BB_controller.command;
    msg_idx++;

    // PID
    // Error
    float_to_bytes(msg, &msg_idx, &(fanController.error));
    // Integral sum
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.integral_sum));
    // Ant-windup integral sum
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.aw_integral_sum));
    // u
    float_to_bytes(msg, &msg_idx, &(fanController.u));
    // u_sat
    float_to_bytes(msg, &msg_idx, &(fanController.u_saturated));
    // u_p
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.u_p));
    // u_i
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.u_i));
    // u_d
    float_to_bytes(msg, &msg_idx, &(fanController.PID_controller.u_d));

    /*
        COIL DATA
    */
    // Bang Bang
    // cmd
    msg[msg_idx] = (uint8_t)coilController.BB_controller.command;
    msg_idx++;

    // PID
    // Error
    float_to_bytes(msg, &msg_idx, &(coilController.error));
    // Integral sum
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.integral_sum));
    // Ant-windup integral sum
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.aw_integral_sum));
    // u
    float_to_bytes(msg, &msg_idx, &(coilController.u));
    // u_sat
    float_to_bytes(msg, &msg_idx, &(coilController.u_saturated));
    // u_p
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.u_p));
    // u_i
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.u_i));
    // u_d
    float_to_bytes(msg, &msg_idx, &(coilController.PID_controller.u_d));

    /*
        Byte 5-10: speed: u16 | temp_top: u16 | temp_bottom: u16
    */
    float_to_bytes(msg, &msg_idx, &(fanController.filter.value));
    float_to_bytes(msg, &msg_idx, &(coilController.filters[TEMP_TOP].value));
    float_to_bytes(msg, &msg_idx, &(coilController.filters[TEMP_BOTTOM].value));

    msg[msg_idx] = '\n';
    CDC_Transmit_FS((uint8_t *)msg, msg_idx);
}

void COM_translateMsg(uint8_t *msg, uint16_t len)
{
    static const char con_tx_str[] = {1, '\n', '\0'};
    uint8_t msg_type = c2u8(msg[MSG_TYPE]);

    switch (msg_type)
    {
    case APP_CON_REQ:
        lastMsgTimestamp = HAL_GetTick() + MAX_SILIENCE_TIME_MS;
        isConnected = 1;
        wasAlreadyReset = 0;
        CDC_Transmit_FS((uint8_t *)con_tx_str, strlen(con_tx_str));
        HAL_GPIO_TogglePin(LED_USB_GPIO_Port, LED_USB_Pin);
        break;
    case FAN_CFG:
        // CONTROL REFERENCE
        uint16_t fan_set_value = cccc2u16(&msg[MSG_BODY + PID_SET_VAL]);
        uint8_t fan_mode = c2u8(msg[MSG_BODY + ON_OFF]);

        fanController.control_reference.type = c2u8(msg[MSG_BODY + CTRL_REF_TYPE]);

        switch (fanController.control_reference.type)
        {
        case STEP:
            ControlReference_setStepReference(&fanController.control_reference, fan_mode == ON ? fan_set_value : 0);
            break;
        case RAMP:
            ControlReference_setRampReference(
                &fanController.control_reference, fan_mode == ON ? fanController.filter.value : 0,
                fan_mode == ON ? fan_set_value : 0, cccc2u16(&msg[MSG_BODY + CTRL_REF_SLOPE]));
            break;
        case SINEWAVE:
            ControlReference_setSineReference(
                &fanController.control_reference, fan_mode == ON ? fanController.filter.value : 0,
                fan_mode == ON ? fan_set_value : 0, cccc2u16(&msg[MSG_BODY + CTRL_REF_AMPL]),
                ccc_cc2f(&msg[MSG_BODY + CTRL_REF_OMEGA]));
            break;
        default:
            break;
        }
        FanController_reset(&fanController);
        // BANG BANG
        BBController_setParams(&(fanController.BB_controller), (float)cccc2u16(&msg[MSG_BODY + BB_HYSTERESIS]));
        // PID
        fanController.PID_controller.Kp = ccc_cc2f(&msg[MSG_BODY + PID_KP]);
        fanController.PID_controller.Ki = ccc_cc2f(&msg[MSG_BODY + PID_KI]);
        fanController.PID_controller.Kd = ccc_cc2f(&msg[MSG_BODY + PID_KD]);
        fanController.PID_controller.Kaw = ccc_cc2f(&msg[MSG_BODY + PID_KAW]);
        // COMMON
        if (c2u8(msg[MSG_BODY + CONTROLLER_TYPE]) != FORCED)
        {
            fanController.u_max = FAN_U_MAX;
        }
        else
        {
            fanController.u_max =
                (uint16_t)(((float)FAN_U_MAX - (float)FAN_U_MIN) * ((float)ccc2u8(&msg[MSG_BODY + POWER]) / 100.0f) +
                           (float)FAN_U_MIN);
        }
        FanController_setController(&fanController, c2u8(msg[MSG_BODY + CONTROLLER_TYPE]));
        FanController_setMode(&fanController, fan_mode);
        break;
    case COIL_CFG:
        // CONTROL REFERENCE
        uint16_t coil_set_value = cccc2u16(&msg[MSG_BODY + PID_SET_VAL]);
        uint8_t coil_mode = c2u8(msg[MSG_BODY + ON_OFF]);

        coilController.control_reference.type = c2u8(msg[MSG_BODY + CTRL_REF_TYPE]);

        switch (coilController.control_reference.type)
        {
        case STEP:
            ControlReference_setStepReference(&coilController.control_reference,
                                              coil_mode == ON || coil_mode == COMBINED ? coil_set_value : 0);
            break;
        case RAMP:
            ControlReference_setRampReference(
                &coilController.control_reference,
                coil_mode == ON || coil_mode == COMBINED ? coilController.filters[coilController.ref_temp].value : 0,
                coil_mode == ON || coil_mode == COMBINED ? coil_set_value : 0,
                cccc2u16(&msg[MSG_BODY + CTRL_REF_SLOPE]));
            break;
        case SINEWAVE:
            ControlReference_setSineReference(
                &coilController.control_reference,
                coil_mode == ON || coil_mode == COMBINED ? coilController.filters[coilController.ref_temp].value : 0,
                coil_mode == ON || coil_mode == COMBINED ? coil_set_value : 0, cccc2u16(&msg[MSG_BODY + CTRL_REF_AMPL]),
                ccc_cc2f(&msg[MSG_BODY + CTRL_REF_OMEGA]));
            break;
        default:
            break;
        }
        CoilController_reset(&coilController);
        // BANG BANG
        BBController_setParams(&(coilController.BB_controller), (float)cccc2u16(&msg[MSG_BODY + BB_HYSTERESIS]));
        // PID
        coilController.PID_controller.Kp = ccc_cc2f(&msg[MSG_BODY + PID_KP]);
        coilController.PID_controller.Ki = ccc_cc2f(&msg[MSG_BODY + PID_KI]);
        coilController.PID_controller.Kd = ccc_cc2f(&msg[MSG_BODY + PID_KD]);
        coilController.PID_controller.Kaw = ccc_cc2f(&msg[MSG_BODY + PID_KAW]);
        // COMMON
        CoilController_setRefTemp(&coilController, c2u8(msg[MSG_BODY + LEFT_RIGHT]));
        CoilController_setRefCoil(&coilController, c2u8(msg[MSG_BODY + COIL_TYPE]));
        CoilController_setController(&coilController, c2u8(msg[MSG_BODY + CONTROLLER_TYPE]));
        CoilController_setMode(&coilController, coil_mode);

        coilController.u_max = (uint16_t)((float)HEATER_U_MAX * ((float)ccc2u8(&msg[MSG_BODY + POWER]) / 120.0f));

        if (coil_mode == FORCED)
        {
            FanController_reset(&fanController);
            BBController_setParams(&(fanController.BB_controller), (float)cccc2u16(&msg[MSG_BODY + BB_HYSTERESIS]));
            FanController_setMode(&fanController, COMBINED);
        }
        else if (fanController.mode == COMBINED)
        {
            FanController_setMode(&fanController, OFF);
        }
        break;
    default:
        break;
    }
}
