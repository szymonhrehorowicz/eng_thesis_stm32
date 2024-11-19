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
#include "control/coil_controller.h"
#include "control/fan_controller.h"

// UserRxBufferFS[APP_RX_DATA_SIZE];
// UserTxBufferFS[APP_TX_DATA_SIZE];

void COM_checkConnection(void)
{
    if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
    {
        HAL_GPIO_WritePin(LED_USB_GPIO_Port, LED_USB_Pin, GPIO_PIN_SET);
    }else
    {
        HAL_GPIO_WritePin(LED_USB_GPIO_Port, LED_USB_Pin, GPIO_PIN_RESET);
    }
}

static const char * con_tx_str = "DEV_CON_ACK";

typedef enum
{
    APP_CON_REQ,
    FAN_CONF_MSG,
    COIL_CONF_MSG,
} MsgType_t;

typedef enum
{
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
} ControlMsg_t;

#define MSG_TYPE 0u
#define MSG_VAR  1u
#define MSG_BODY 2u

void COM_transmit(uint8_t *buffer, uint16_t len)
{
    CDC_Transmit_FS(buffer, len);
}

void COM_sendControlParameters(void)
{

}

void COM_sendMeasurements(void)
{

}

static inline uint8_t c2u8(char c)
{
    return (uint8_t)(c - '0');
}

void COM_translateMsg(uint8_t *msg, uint16_t len)
{
    uint8_t msg_var = c2u8(msg[MSG_VAR]);
    switch(c2u8(msg[MSG_TYPE]))
    {
        case APP_CON_REQ:
            CDC_Transmit_FS((uint8_t *)con_tx_str, strlen(con_tx_str));
            break;
        case FAN_CONF_MSG:
            switch(msg_var)
            {
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
                    // Expected 0 - 6000
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
        default:
            break;
    }
}
