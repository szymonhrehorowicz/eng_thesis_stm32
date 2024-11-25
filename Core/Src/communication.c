/*
 * communication.c
 *
 *  Created on: Nov 18, 2024
 *      Author: Szymon
 */

#include "communication.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "stdio.h"
#include "control/coil_controller.h"
#include "control/fan_controller.h"
#include "control/bang_bang.h"
#include "control/pid.h"
#include "control/controller_enums.h"
#include "tim.h"

// UserRxBufferFS[APP_RX_DATA_SIZE];
// UserTxBufferFS[APP_TX_DATA_SIZE];

uint8_t COM_checkConnection(void)
{
    uint8_t ret_val = 0;

    if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
    {
        HAL_GPIO_WritePin(LED_USB_GPIO_Port, LED_USB_Pin, GPIO_PIN_SET);
        ret_val = 1;
    }else
    {
        HAL_GPIO_WritePin(LED_USB_GPIO_Port, LED_USB_Pin, GPIO_PIN_RESET);
    }

    return ret_val;
}

static const char * con_tx_str = "DEV_CON_ACK";

typedef enum
{
    APP_CON_REQ,
    FAN_CONF_MSG,
    COIL_CONF_MSG,
    FAN_DATA_MSG,
    COIL_DATA_MSG,
} MsgType_t;

typedef enum
{
    // Config messages
    SET_CONTROLLER,
    SET_REF_VALUE,
    SET_MODE,
    SET_REF_TEMP,
    SET_REF_COIL,
    SET_KP,
    SET_KI,
    SET_KD,
    SET_HYST,
    SET_HYST_SHIFT,
    // Data messages
    GET_BB_SET_VALUE,
    GET_BB_THRESHOLD_TOP,
    GET_BB_THRESHOLD_BOTTOM,
    GET_BB_U_MAX,
    GET_BB_U_MIN,
    GET_BB_CMD,
    GET_PID_SET_VALUE,
    GET_PID_ERROR,
    GET_PID_INTEGRAL_ERROR,
    GET_PID_AW_INTEGRAL_ERROR,
    GET_PID_KP,
    GET_PID_KI,
    GET_PID_KD,
    GET_PID_KAW,
    GET_PID_U,
    GET_PID_U_SATURATED,
    GET_PID_U_P,
    GET_PID_U_I,
    GET_PID_U_D,
    GET_PID_MAX,
    GET_PID_MIN,
    GET_COIL_TEMPERATURES,
    GET_COIL_MODE,
    GET_FAN_SPEED,
    GET_FAN_MODE,
    //
    GET_ALL_FAN_DATA,
    GET_FAST_FAN_DATA,
    GET_ALL_COIL_DATA,
    GET_FAST_COIL_DATA,
    //
    SET_FAN_CONF,
    SET_HEATER_CONF,
} ControlMsg_t;

#define MSG_TYPE 0u
#define MSG_VAR  1u
#define MSG_BODY 3u

typedef enum
{
    CONTROLLER_TYPE = 0, // 0/1
    BB_SET_VAL      = 1, // 0 - 6000
    BB_HYSTERESIS   = BB_SET_VAL + 4, // 0 - 6000
    PID_SET_VAL     = BB_HYSTERESIS + 4, // 0-6000
    PID_KP          = PID_SET_VAL + 4, // 0 - 100.00
    PID_KI          = PID_KP  + 6, // 0 - 100.00
    PID_KD          = PID_KI + 6, // 0 - 100.00
    PID_KAW         = PID_KD + 6, // 0 - 100.00
    ON_OFF          = PID_KAW + 6, // 0/1
    COIL_TYPE       = ON_OFF + 1, // 0/1
    LEFT_RIGHT      = COIL_TYPE + 1, // 0/1
    COIL_POWER      = LEFT_RIGHT + 1, // 0-100
} ConfMsgBody_t;

static inline uint8_t c2u8(uint8_t c)
{
    return (uint8_t)(c - '0');
}

static inline uint8_t cc2u8(uint8_t *c)
{
    return ((uint8_t)(c[0] - '0') * 10) + ((uint8_t)(c[1]) - '0');
}

static inline uint8_t ccc2u8(uint8_t *c)
{
    return ((uint8_t)(c[0] - '0') * 100) + ((uint8_t)(c[1] - '0') * 10) + ((uint8_t)(c[2]) - '0');
}

static inline uint16_t cccc2u8(uint8_t *c)
{
    uint16_t thousands, hundreds, tens, ones;
    thousands = c2u8(c[0]) * 1000;
    hundreds = c2u8(c[1]) * 100;
    tens = c2u8(c[2]) * 10;
    ones = c2u8(c[3]);
    return thousands + hundreds + tens + ones;
}

static inline float ccc_cc2f(uint8_t *c)
{
    float khundreds = c2u8(c[0]) * 100.0f;
    float ktens, kones, kdtens, kdhundreds;
    ktens = c2u8(c[1]) * 10;
    kones = c2u8(c[2]);
    kdtens = c2u8(c[4]) / 10.0f;
    kdhundreds = c2u8(c[5]) / 100.0f;
    return khundreds + ktens + kones + kdtens + kdhundreds;
}

void COM_translateMsg(uint8_t *msg, uint16_t len)
{
    uint8_t msg_var = cc2u8(&msg[MSG_VAR]);

    switch(c2u8(msg[MSG_TYPE]))
    {
        case APP_CON_REQ:
            CDC_Transmit_FS((uint8_t *)con_tx_str, strlen(con_tx_str));
            HAL_GPIO_TogglePin(LED_USB_GPIO_Port, LED_USB_Pin);
            break;
        case FAN_CONF_MSG:
            switch(msg_var)
            {
                case SET_FAN_CONF:
                    // BANG BANG
                    BBController_setParams(&(fanController.BB_controller), cccc2u8(&msg[MSG_BODY + BB_SET_VAL]), cccc2u8(&msg[MSG_BODY + BB_HYSTERESIS]), 0);
                    BBController_reset(&(fanController.BB_controller));
                    // PID
                    fanController.PID_controller.set_value = cccc2u8(&msg[MSG_BODY + PID_SET_VAL]);
                    fanController.PID_controller.Kp = ccc_cc2f(&msg[MSG_BODY + PID_KP]);
                    fanController.PID_controller.Ki = ccc_cc2f(&msg[MSG_BODY + PID_KI]);
                    fanController.PID_controller.Kd = ccc_cc2f(&msg[MSG_BODY + PID_KD]);
                    fanController.PID_controller.Kaw = ccc_cc2f(&msg[MSG_BODY + PID_KAW]);
                    PID_reset(&(fanController.PID_controller));
                    // COMMON
                    FanController_setController(&fanController, c2u8(msg[MSG_BODY + CONTROLLER_TYPE]));
                    FanController_setMode(&fanController, c2u8(msg[MSG_BODY + ON_OFF]));
                    break;
                case SET_CONTROLLER:
                    // BANG_BANG / PID
                    uint8_t controller = c2u8(msg[MSG_BODY]);
                    if((controller == BANG_BANG) || (controller == PID))
                    {
                        FanController_setController(&fanController, controller);
                    }
                    break;
                case SET_MODE:
                    // ON/OFF
                    uint8_t mode = c2u8(msg[MSG_BODY]);
                    if((mode == ON) || (mode == OFF))
                    {
                        FanController_setMode(&fanController, mode);
                    }
                    break;
                case SET_KP:
                case SET_KI:
                case SET_KD:
                    // 000.00
                    float khundreds = c2u8(msg[MSG_BODY]) * 100.0f;
                    float ktens, kones, kdtens, kdhundreds;
                    ktens = c2u8(msg[MSG_BODY + 1]) * 10;
                    kones = c2u8(msg[MSG_BODY + 2]);
                    kdtens = c2u8(msg[MSG_BODY + 4]) / 10.0f;
                    kdhundreds = c2u8(msg[MSG_BODY + 5]) / 100.0f;
                    float Kx = khundreds + ktens + kones + kdtens + kdhundreds;
                    if((Kx >= 0.0f) && (Kx <= 1000.0f))
                    {
                        switch(msg_var)
                        {
                            case SET_KP:
                                fanController.PID_controller.Kp = Kx;
                                break;
                            case SET_KI:
                                fanController.PID_controller.Ki = Kx;
                                break;
                            case SET_KD:
                                fanController.PID_controller.Kd = Kx;
                                break;
                            default:
                                break;
                        }
                    }
                    break;
                case SET_HYST:
                case SET_HYST_SHIFT:
                case SET_REF_VALUE:
                    // Expected 0 - 6000
                    uint16_t thousands, hundreds, tens, ones;
                    thousands = c2u8(msg[MSG_BODY]) * 1000;
                    hundreds = c2u8(msg[MSG_BODY + 1]) * 100;
                    tens = c2u8(msg[MSG_BODY + 2]) * 10;
                    ones = c2u8(msg[MSG_BODY + 3]);
                    uint16_t value = thousands + hundreds + tens + ones;
                    if((value > 0) && (value <= 6000))
                    {
                        switch(msg_var)
                        {
                            case SET_HYST:
                                int16_t hysteresis_shift = ((fanController.BB_controller.threshold_top - fanController.BB_controller.set_value) - (fanController.BB_controller.set_value - fanController.BB_controller.threshold_bottom))/2;
                                BBController_setParams(&fanController.BB_controller, fanController.BB_controller.set_value, value, hysteresis_shift);
                                break;
                            case SET_HYST_SHIFT:
                                uint16_t hysteresis = fanController.BB_controller.threshold_top - fanController.BB_controller.threshold_bottom;
                                BBController_setParams(&fanController.BB_controller, fanController.BB_controller.set_value, hysteresis, value);
                                break;
                            case SET_REF_VALUE:
                                FanController_setRefValue(&fanController, value);
                                break;
                            default:
                                break;
                        }
                    }
                    break;
                default:
                    break;
            }
            break;
        case COIL_CONF_MSG:
            switch(msg_var)
            {
                case SET_HEATER_CONF:
                    // BANG BANG
                    BBController_setParams(&(coilController.BB_controller), cccc2u8(&msg[MSG_BODY + BB_SET_VAL]), cccc2u8(&msg[MSG_BODY + BB_HYSTERESIS]), 0);
                    coilController.BB_controller.u_max = (float)BB_U_MAX * (float)ccc2u8(&msg[MSG_BODY + COIL_POWER]);
                    BBController_reset(&(coilController.BB_controller));
                    // PID
                    coilController.PID_controller.set_value = cccc2u8(&msg[MSG_BODY + PID_SET_VAL]);
                    coilController.PID_controller.Kp = ccc_cc2f(&msg[MSG_BODY + PID_KP]);
                    coilController.PID_controller.Ki = ccc_cc2f(&msg[MSG_BODY + PID_KI]);
                    coilController.PID_controller.Kd = ccc_cc2f(&msg[MSG_BODY + PID_KD]);
                    coilController.PID_controller.Kaw = ccc_cc2f(&msg[MSG_BODY + PID_KAW]);
                    PID_reset(&(coilController.PID_controller));
                    // COMMON
                    CoilController_setRefTemp(&coilController, c2u8(msg[MSG_BODY + LEFT_RIGHT]));
                    CoilController_setRefCoil(&coilController, c2u8(msg[MSG_BODY + COIL_POWER]));
                    CoilController_setController(&coilController, c2u8(msg[MSG_BODY + CONTROLLER_TYPE]));
                    CoilController_setMode(&coilController, c2u8(msg[MSG_BODY + ON_OFF]));
                    break;
                case SET_CONTROLLER:
                    // BANG_BANG / PID
                    uint8_t controller = c2u8(msg[MSG_BODY]);
                    if((controller == BANG_BANG) || (controller == PID))
                    {
                        CoilController_setController(&coilController, controller);
                    }
                    break;
                case SET_MODE:
                    // ON/OFF
                    uint8_t mode = c2u8(msg[MSG_BODY]);
                    if((mode == ON) || (mode == OFF))
                    {
                        CoilController_setMode(&coilController, mode);
                    }
                    break;
                case SET_KP:
                case SET_KI:
                case SET_KD:
                    // 000.00
                    float khundreds = c2u8(msg[MSG_BODY]) * 100.0f;
                    float ktens, kones, kdtens, kdhundreds;
                    ktens = c2u8(msg[MSG_BODY + 1]) * 10;
                    kones = c2u8(msg[MSG_BODY + 2]);
                    kdtens = c2u8(msg[MSG_BODY + 4]) / 10.0f;
                    kdhundreds = c2u8(msg[MSG_BODY + 5]) / 100.0f;
                    float Kx = khundreds + ktens + kones + kdtens + kdhundreds;
                    if((Kx >= 0.0f) && (Kx <= 1000.0f))
                    {
                        switch(msg_var)
                        {
                            case SET_KP:
                                coilController.PID_controller.Kp = Kx;
                                break;
                            case SET_KI:
                                coilController.PID_controller.Ki = Kx;
                                break;
                            case SET_KD:
                                coilController.PID_controller.Kd = Kx;
                                break;
                            default:
                                break;
                        }
                    }
                    break;
                case SET_HYST:
                case SET_HYST_SHIFT:
                case SET_REF_VALUE:
                    // Expected 0000 - 00100
                    uint16_t thousands, hundreds, tens, ones;
                    thousands = c2u8(msg[MSG_BODY]) * 1000;
                    hundreds = c2u8(msg[MSG_BODY + 1]) * 100;
                    tens = c2u8(msg[MSG_BODY + 2]) * 10;
                    ones = c2u8(msg[MSG_BODY + 3]);
                    uint16_t value = thousands + hundreds + tens + ones;
                    switch(msg_var)
                    {
                        case SET_HYST:
                            int16_t hysteresis_shift = ((coilController.BB_controller.threshold_top - coilController.BB_controller.set_value) - (coilController.BB_controller.set_value - coilController.BB_controller.threshold_bottom))/2;
                            if(value < 50)
                            {
                                BBController_setParams(&coilController.BB_controller, coilController.BB_controller.set_value, value, hysteresis_shift);
                            }
                            break;
                        case SET_HYST_SHIFT:
                            uint16_t hysteresis = coilController.BB_controller.threshold_top - coilController.BB_controller.threshold_bottom;
                            if(value < 50)
                            {
                                BBController_setParams(&coilController.BB_controller, coilController.BB_controller.set_value, hysteresis, value);
                            }
                            break;
                        case SET_REF_VALUE:
                            if(value < MAX_TEMPERATURE)
                            {
                                CoilController_setRefValue(&coilController, value);
                            }
                            break;
                        default:
                            break;
                    }
                    break;
                case SET_REF_COIL:
                    uint8_t coil = c2u8(msg[MSG_BODY]);
                    if((coil == COIL_A) || (coil == COIL_B))
                    {
                        CoilController_setRefCoil(&coilController, coil);
                    }
                    break;
                case SET_REF_TEMP:
                    uint8_t ref_temp = c2u8(msg[MSG_BODY]);
                    if((ref_temp == TEMP_BOTTOM) || (ref_temp == TEMP_TOP))
                    {
                        CoilController_setRefTemp(&coilController, ref_temp);
                    }
                    break;
                default:
                    break;
            }
            break;
        case FAN_DATA_MSG:
            char msg[200];
            int len;
            switch(msg_var)
            {
                case GET_ALL_FAN_DATA:
                    len = sprintf(msg, "ALLFAN_%d_%d_%d_%d_%d_%d_%d_%f_%f_%f_%f_%f_%f_%f_%d_%d_%f_%f_%f_%d_%d_%d_%d",
                    // Fan BB controller
                    fanController.BB_controller.set_value, fanController.BB_controller.threshold_top, fanController.BB_controller.threshold_bottom,
                    fanController.BB_controller.u_max, fanController.BB_controller.u_min, fanController.BB_controller.command,
                    // Fan PID controller
                    fanController.PID_controller.set_value, fanController.PID_controller.errror,
                    fanController.PID_controller.integral_sum, fanController.PID_controller.aw_integral_sum,
                    fanController.PID_controller.Kp, fanController.PID_controller.Ki, fanController.PID_controller.Kd,
                    fanController.PID_controller.Kaw, fanController.PID_controller.u, fanController.PID_controller.u_saturated,
                    fanController.PID_controller.u_p, fanController.PID_controller.u_i, fanController.PID_controller.u_d,
                    fanController.PID_controller.max, fanController.PID_controller.min, 
                    fanController.filter.value, fanController.mode
                    );
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_FAST_FAN_DATA:
                    len = sprintf(msg, "FASTFAN_%d_%f_%f_%f_%d_%d_%f_%f_%f_%d_%d",
                    // Fan BB controller
                    fanController.BB_controller.command,
                    // Fan PID controller
                    fanController.PID_controller.errror, fanController.PID_controller.integral_sum, fanController.PID_controller.aw_integral_sum,
                    fanController.PID_controller.u, fanController.PID_controller.u_saturated,
                    fanController.PID_controller.u_p, fanController.PID_controller.u_i, fanController.PID_controller.u_d,
                    fanController.filter.value, fanController.mode
                    );
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_BB_SET_VALUE:
                    len = sprintf(msg, "%d", fanController.BB_controller.set_value);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_BB_THRESHOLD_TOP:
                    len = sprintf(msg, "%d", fanController.BB_controller.threshold_top);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_BB_THRESHOLD_BOTTOM:
                    len = sprintf(msg, "%d", fanController.BB_controller.threshold_bottom);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_BB_U_MAX:
                    len = sprintf(msg, "%d", fanController.BB_controller.u_max);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_BB_U_MIN:
                    len = sprintf(msg, "%d", fanController.BB_controller.u_min);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_BB_CMD:
                    len = sprintf(msg, "%d", fanController.BB_controller.command);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_SET_VALUE:
                    len = sprintf(msg, "%d", fanController.PID_controller.set_value);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_ERROR:
                    len = sprintf(msg, "%f", fanController.PID_controller.errror);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_INTEGRAL_ERROR:
                    len = sprintf(msg, "%f", fanController.PID_controller.integral_sum);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_AW_INTEGRAL_ERROR:
                    len = sprintf(msg, "%f", fanController.PID_controller.aw_integral_sum);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_KP:
                    len = sprintf(msg, "%f", fanController.PID_controller.Kp);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_KI:
                    len = sprintf(msg, "%f", fanController.PID_controller.Ki);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_KD:
                    len = sprintf(msg, "%f", fanController.PID_controller.Kd);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_KAW:
                    len = sprintf(msg, "%f", fanController.PID_controller.Kaw);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_U:
                    len = sprintf(msg, "%d", fanController.PID_controller.u);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_U_SATURATED:
                    len = sprintf(msg, "%d", fanController.PID_controller.u_saturated);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_U_P:
                    len = sprintf(msg, "%f", fanController.PID_controller.u_p);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_U_I:
                    len = sprintf(msg, "%f", fanController.PID_controller.u_i);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_U_D:
                    len = sprintf(msg, "%f", fanController.PID_controller.u_d);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_MAX:
                    len = sprintf(msg, "%d", fanController.PID_controller.max);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_MIN:
                    len = sprintf(msg, "%d", fanController.PID_controller.min);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_FAN_SPEED:
                    len = sprintf(msg, "%d", fanController.filter.value);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_FAN_MODE:
                    len = sprintf(msg, "%d", fanController.mode);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                default:
                    break;
            }
            break;
        case COIL_DATA_MSG:
            switch(msg_var)
            {
                case GET_ALL_COIL_DATA:
                    len = sprintf(msg, "ALLCOIL_%d_%d_%d_%d_%d_%d_%d_%f_%f_%f_%f_%f_%f_%f_%d_%d_%f_%f_%f_%d_%d_%d_%d_%d",
                    // Fan BB controller
                    coilController.BB_controller.set_value, coilController.BB_controller.threshold_top, coilController.BB_controller.threshold_bottom,
                    coilController.BB_controller.u_max, coilController.BB_controller.u_min, coilController.BB_controller.command,
                    // Fan PID controller
                    coilController.PID_controller.set_value, coilController.PID_controller.errror,
                    coilController.PID_controller.integral_sum, coilController.PID_controller.aw_integral_sum,
                    coilController.PID_controller.Kp, coilController.PID_controller.Ki, coilController.PID_controller.Kd,
                    coilController.PID_controller.Kaw, coilController.PID_controller.u, coilController.PID_controller.u_saturated,
                    coilController.PID_controller.u_p, coilController.PID_controller.u_i, coilController.PID_controller.u_d,
                    coilController.PID_controller.max, coilController.PID_controller.min, 
                    (coilController.filters[0]).value, (coilController.filters[1]).value, coilController.mode
                    );
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_FAST_COIL_DATA:
                    len = sprintf(msg, "FASTCOIL_%d_%f_%f_%f_%d_%d_%f_%f_%f_%d_%d_%d",
                    // Fan BB controller
                    coilController.BB_controller.command,
                    // Fan PID controller
                    coilController.PID_controller.errror, coilController.PID_controller.integral_sum, coilController.PID_controller.aw_integral_sum,
                    coilController.PID_controller.u, coilController.PID_controller.u_saturated,
                    coilController.PID_controller.u_p, coilController.PID_controller.u_i, coilController.PID_controller.u_d,
                    (coilController.filters[0]).value, (coilController.filters[1]).value, coilController.mode
                    );
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_BB_SET_VALUE:
                    len = sprintf(msg, "%d", coilController.BB_controller.set_value);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_BB_THRESHOLD_TOP:
                    len = sprintf(msg, "%d", coilController.BB_controller.threshold_top);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_BB_THRESHOLD_BOTTOM:
                    len = sprintf(msg, "%d", coilController.BB_controller.threshold_bottom);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_BB_U_MAX:
                    len = sprintf(msg, "%d", coilController.BB_controller.u_max);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_BB_U_MIN:
                    len = sprintf(msg, "%d", coilController.BB_controller.u_min);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_BB_CMD:
                    len = sprintf(msg, "%d", coilController.BB_controller.command);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_SET_VALUE:
                    len = sprintf(msg, "%d", coilController.PID_controller.set_value);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_ERROR:
                    len = sprintf(msg, "%f", coilController.PID_controller.errror);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_INTEGRAL_ERROR:
                    len = sprintf(msg, "%f", coilController.PID_controller.integral_sum);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_AW_INTEGRAL_ERROR:
                    len = sprintf(msg, "%f", coilController.PID_controller.aw_integral_sum);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_KP:
                    len = sprintf(msg, "%f", coilController.PID_controller.Kp);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_KI:
                    len = sprintf(msg, "%f", coilController.PID_controller.Ki);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_KD:
                    len = sprintf(msg, "%f", coilController.PID_controller.Kd);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_KAW:
                    len = sprintf(msg, "%f", coilController.PID_controller.Kaw);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_U:
                    len = sprintf(msg, "%d", coilController.PID_controller.u);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_U_SATURATED:
                    len = sprintf(msg, "%d", coilController.PID_controller.u_saturated);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_U_P:
                    len = sprintf(msg, "%f", coilController.PID_controller.u_p);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_U_I:
                    len = sprintf(msg, "%f", coilController.PID_controller.u_i);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_U_D:
                    len = sprintf(msg, "%f", coilController.PID_controller.u_d);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_MAX:
                    len = sprintf(msg, "%d", coilController.PID_controller.max);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_PID_MIN:
                    len = sprintf(msg, "%d", coilController.PID_controller.min);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_COIL_TEMPERATURES:
                    len = sprintf(msg, "%d_%d", coilController.temperatures[TEMP_TOP], coilController.temperatures[TEMP_BOTTOM]);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                case GET_COIL_MODE:
                    len = sprintf(msg, "%d", coilController.mode);
                    CDC_Transmit_FS((uint8_t *)msg, len);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}
