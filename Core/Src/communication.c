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
#include "utils/char_translator.h"
#include "tim.h"

#define MAX_SILIENCE_TIME_MS 15000

volatile uint8_t isConnected = 0;

void COM_checkConnection(void)
{
    if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
    {
        HAL_GPIO_WritePin(LED_USB_GPIO_Port, LED_USB_Pin, GPIO_PIN_SET);
    }else
    {
        HAL_GPIO_WritePin(LED_USB_GPIO_Port, LED_USB_Pin, GPIO_PIN_RESET);
        isConnected = 0;
    }
}

volatile uint32_t lastMsgTimestamp = MAX_SILIENCE_TIME_MS;
volatile uint8_t wasAlreadyReset = 0;

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
    CTRL_REF_TYPE   = ON_OFF + 1,
    CTRL_REF_SLOPE  = CTRL_REF_TYPE + 1,
    CTRL_REF_AMPL   = CTRL_REF_SLOPE + 4,
    CTRL_REF_OMEGA  = CTRL_REF_AMPL + 4,
    COIL_TYPE       = CTRL_REF_OMEGA + 6, // 0/1
    LEFT_RIGHT      = COIL_TYPE + 1, // 0/1
    COIL_POWER      = LEFT_RIGHT + 1, // 0-100
} ConfMsgBody_t;



typedef enum
{
    APP_CON_REQ,
    FAN_CFG,
    COIL_CFG,
} IncommingMessages_t;

/*
0/1
Byte 0   : 0: FAN/COIL | 1: ALL/FAST | 2: ON/OFF | 3: TEMP_REF_TOP/TEP_REF_BOTTOM | 4: COIL_REF_TOP/COIL_REF_BOTTOM | 5: NA| 6: NA| 7: NA|

Byte 1-4 : timestamp

if [FAN] # 
 Byte 5-6: speed: u16
if [COIL]#
 Byte 5-8: temp_top: u16 | temp_bottom: u16

Rest of Bytes:
[ALL]
Bang-Bang: set_value: u16 | threshold_top: u16 | threshold_bottom: u16 | u_max: u16 | u_min: u16
PID      : set_value: u16 | error: f32 | int_sum: f32 | aw_int_sum: f32 | Kp: f32 | Ki: f32 | Kd: f32 | Kaw: f32 | u: u16 | u_sat: u16 | u_p: f32 | u_i: f32 | u_d: f32 | max: u16 | min: u16    

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

#define DEV_TYPE_FAN  0u
#define DEV_TYPE_COIL 1u
#define OUT_ALL  0u
#define OUT_FAST 1u

#define MSG_TYPE 0u

static volatile uint8_t msg[200];
static volatile uint8_t msg_idx = 0;

void COM_sendAllData(void)
{
    msg_idx = 0;
    memset(msg, 0, sizeof(msg));

    // Byte 0 : 0: ALL/FAST | 1: FAN OFF/ON | 2: COIL OFF/ON | 3: TEMP_REF_TOP/TEMP_REF_BOTTOM | 4: COIL_REF_TOP/COIL_REF_BOTTOM | 5: NA| 6: NA| 7: NA|
    msg[msg_idx] |= OUT_ALL << OUT_ALL_FAST;
    msg[msg_idx] |= fanController.mode << OUT_FAN_ON_OFF;
    msg[msg_idx] |= coilController.mode << OUT_COIL_ON_OFF;
    msg[msg_idx] |= coilController.ref_temp << OUT_TEMP_REF;
    msg[msg_idx] |= coilController.ref_coil << OUT_COIL_REF;
    msg_idx++;
    // Byte 1-4 : timestamp
    uint32_t timestamp = HAL_GetTick();
    msg[msg_idx] = (uint8_t)(timestamp >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)(timestamp >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)(timestamp >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(timestamp & 0x000000ff); msg_idx++;

    /*
        FAN DATA
    */
    // Bang Bang
        // cmd
    msg[msg_idx] = (uint8_t)fanController.BB_controller.command; msg_idx++;
        // Set value
    msg[msg_idx] = (uint8_t)(fanController.BB_controller.set_value >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.BB_controller.set_value & 0x00ff); msg_idx++;
        // Threshold top
    msg[msg_idx] = (uint8_t)(fanController.BB_controller.threshold_top >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.BB_controller.threshold_top & 0x00ff); msg_idx++;
        // Threshold bottom
    msg[msg_idx] = (uint8_t)(fanController.BB_controller.threshold_bottom >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.BB_controller.threshold_bottom & 0x00ff); msg_idx++;
        // u_max
    msg[msg_idx] = (uint8_t)(fanController.BB_controller.u_max >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.BB_controller.u_max & 0x00ff); msg_idx++;
        // u_min
    msg[msg_idx] = (uint8_t)(fanController.BB_controller.u_min >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.BB_controller.u_min & 0x00ff); msg_idx++;

    // PID
        // Set value
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.set_value >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.set_value & 0x00ff); msg_idx++;
        // Error
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.error) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.error) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.error) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.error) & 0x000000ff); msg_idx++;
        // Integral sum
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.integral_sum) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.integral_sum) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.integral_sum) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.integral_sum) & 0x000000ff); msg_idx++;
        // Ant-windup integral sum
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.aw_integral_sum) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.aw_integral_sum) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.aw_integral_sum) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.aw_integral_sum) & 0x000000ff); msg_idx++;
        // Kp
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Kp) >> 24) ; msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Kp) >> 16) ; msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Kp) >> 8) ; msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Kp) & 0x000000ff) ; msg_idx++;
        // Ki
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Ki) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Ki) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Ki)) >> 8; msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Ki)) & 0x000000ff; msg_idx++;
        // Kd
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Kd) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Kd) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Kd) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Kd) & 0x000000ff); msg_idx++;
        // Kaw
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Kaw) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Kaw) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Kaw) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.Kaw) & 0x000000ff); msg_idx++;
        // u
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.u >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.u & 0x00ff); msg_idx++;
        // u_sat
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.u_saturated >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.u_saturated & 0x00ff); msg_idx++;
        // u_p
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_p) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_p) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_p) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_p) & 0x000000ff); msg_idx++;
        // u_i
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_i) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_i) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_i) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_i) & 0x000000ff); msg_idx++;
        // u_d
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_d) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_d) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_d) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_d) & 0x000000ff); msg_idx++;
        // max
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.max >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.max & 0x00ff); msg_idx++;
        // min
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.min >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.min & 0x00ff); msg_idx++;

    /*
        COIL DATA
    */ 
    // Bang Bang
        // cmd
    msg[msg_idx] = (uint8_t)coilController.BB_controller.command; msg_idx++;
        // Set value
    msg[msg_idx] = (uint8_t)(coilController.BB_controller.set_value >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.BB_controller.set_value & 0x00ff); msg_idx++;
        // Threshold top
    msg[msg_idx] = (uint8_t)(coilController.BB_controller.threshold_top >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.BB_controller.threshold_top & 0x00ff); msg_idx++;
        // Threshold bottom
    msg[msg_idx] = (uint8_t)(coilController.BB_controller.threshold_bottom >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.BB_controller.threshold_bottom & 0x00ff); msg_idx++;
        // u_max
    msg[msg_idx] = (uint8_t)(coilController.BB_controller.u_max >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.BB_controller.u_max & 0x00ff); msg_idx++;
        // u_min
    msg[msg_idx] = (uint8_t)(coilController.BB_controller.u_min >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.BB_controller.u_min & 0x00ff); msg_idx++;

    // PID
        // Set value
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.set_value >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.set_value & 0x00ff); msg_idx++;
        // Error
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.error) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.error) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.error) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.error) & 0x000000ff); msg_idx++;
        // Integral sum
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.integral_sum) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.integral_sum) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.integral_sum) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.integral_sum) & 0x000000ff); msg_idx++;
        // Ant-windup integral sum
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.aw_integral_sum) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.aw_integral_sum) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.aw_integral_sum) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.aw_integral_sum) & 0x000000ff); msg_idx++;
        // Kp
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Kp) >> 24) ; msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Kp) >> 16) ; msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Kp) >> 8) ; msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Kp) & 0x000000ff) ; msg_idx++;
        // Ki
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Ki) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Ki) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Ki)) >> 8; msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Ki)) & 0x000000ff; msg_idx++;
        // Kd
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Kd) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Kd) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Kd) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Kd) & 0x000000ff); msg_idx++;
        // Kaw
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Kaw) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Kaw) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Kaw) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.Kaw) & 0x000000ff); msg_idx++;
        // u
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.u >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.u & 0x00ff); msg_idx++;
        // u_sat
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.u_saturated >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.u_saturated & 0x00ff); msg_idx++;
        // u_p
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_p) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_p) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_p) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_p) & 0x000000ff); msg_idx++;
        // u_i
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_i) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_i) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_i) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_i) & 0x000000ff); msg_idx++;
        // u_d
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_d) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_d) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_d) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_d) & 0x000000ff); msg_idx++;
        // max
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.max >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.max & 0x00ff); msg_idx++;
        // min
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.min >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.min & 0x00ff); msg_idx++;

    /*
        Byte 5-10: speed: u16 | temp_top: u16 | temp_bottom: u16
    */
    msg[msg_idx] = (uint8_t)(fanController.filter.value >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.filter.value & 0x00ff); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.filters[TEMP_TOP].value >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.filters[TEMP_TOP].value & 0x00ff); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.filters[TEMP_BOTTOM].value >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.filters[TEMP_BOTTOM].value & 0x00ff); msg_idx++;

    msg[msg_idx] = '\n';
    CDC_Transmit_FS((uint8_t *)msg, msg_idx);
}

void COM_sendFastData(void)
{
    msg_idx = 0;
    memset(msg, 0, sizeof(msg));

    // Byte 0 : 0: ALL/FAST | 1: FAN OFF/ON | 2: COIL OFF/ON | 3: TEMP_REF_TOP/TEMP_REF_BOTTOM | 4: COIL_REF_TOP/COIL_REF_BOTTOM | 5: NA| 6: NA| 7: NA|
    msg[msg_idx] |= OUT_FAST << OUT_ALL_FAST;
    msg[msg_idx] |= fanController.mode << OUT_FAN_ON_OFF;
    msg[msg_idx] |= coilController.mode << OUT_COIL_ON_OFF;
    msg[msg_idx] |= coilController.ref_temp << OUT_TEMP_REF;
    msg[msg_idx] |= coilController.ref_coil << OUT_COIL_REF;
    msg_idx++;
    // Byte 1-4 : timestamp
    uint32_t timestamp = HAL_GetTick();
    msg[msg_idx] = (uint8_t)(timestamp >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)(timestamp >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)(timestamp >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(timestamp & 0x000000ff); msg_idx++;

    /*
        FAN DATA
    */
    // Bang Bang
        // cmd
    msg[msg_idx] = (uint8_t)fanController.BB_controller.command; msg_idx++;

    // PID
        // Error
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.error) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.error) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.error) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.error) & 0x000000ff); msg_idx++;
        // Integral sum
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.integral_sum) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.integral_sum) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.integral_sum) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.integral_sum) & 0x000000ff); msg_idx++;
        // Ant-windup integral sum
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.aw_integral_sum) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.aw_integral_sum) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.aw_integral_sum) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.aw_integral_sum) & 0x000000ff); msg_idx++;
        // u
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.u >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.u & 0x00ff); msg_idx++;
        // u_sat
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.u_saturated >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.PID_controller.u_saturated & 0x00ff); msg_idx++;
        // u_p
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_p) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_p) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_p) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_p) & 0x000000ff); msg_idx++;
        // u_i
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_i) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_i) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_i) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_i) & 0x000000ff); msg_idx++;
        // u_d
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_d) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_d) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_d) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(fanController.PID_controller.u_d) & 0x000000ff); msg_idx++;

    /*
        COIL DATA
    */ 
    // Bang Bang
        // cmd
    msg[msg_idx] = (uint8_t)coilController.BB_controller.command; msg_idx++;

    // PID
        // Error
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.error) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.error) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.error) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.error) & 0x000000ff); msg_idx++;
        // Integral sum
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.integral_sum) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.integral_sum) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.integral_sum) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.integral_sum) & 0x000000ff); msg_idx++;
        // Ant-windup integral sum
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.aw_integral_sum) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.aw_integral_sum) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.aw_integral_sum) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.aw_integral_sum) & 0x000000ff); msg_idx++;
        // u
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.u >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.u & 0x00ff); msg_idx++;
        // u_sat
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.u_saturated >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.PID_controller.u_saturated & 0x00ff); msg_idx++;
        // u_p
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_p) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_p) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_p) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_p) & 0x000000ff); msg_idx++;
        // u_i
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_i) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_i) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_i) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_i) & 0x000000ff); msg_idx++;
        // u_d
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_d) >> 24); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_d) >> 16); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_d) >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)((uint32_t)(coilController.PID_controller.u_d) & 0x000000ff); msg_idx++;

    /*
        Byte 5-10: speed: u16 | temp_top: u16 | temp_bottom: u16
    */
    msg[msg_idx] = (uint8_t)(fanController.filter.value >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(fanController.filter.value & 0x00ff); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.filters[TEMP_TOP].value >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.filters[TEMP_TOP].value & 0x00ff); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.filters[TEMP_BOTTOM].value >> 8); msg_idx++;
    msg[msg_idx] = (uint8_t)(coilController.filters[TEMP_BOTTOM].value & 0x00ff); msg_idx++;

    msg[msg_idx] = '\n';
    CDC_Transmit_FS((uint8_t *)msg, msg_idx);
}

void COM_translateMsg(uint8_t *msg, uint16_t len)
{
    static const char con_tx_str[] = {1,'\n', '\0'};
    uint8_t msg_type = c2u8(msg[MSG_TYPE]);

    switch(msg_type)
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

            switch(fanController.control_reference.type)
            {
                case STEP:
                    ControlReference_setStepReference(
                        &fanController.control_reference,
                        fan_mode == 1 ? fan_set_value : 0);
                    break;
                case RAMP:
                    ControlReference_setRampReference(
                        &fanController.control_reference,
                        fan_mode == 1 ? fan_set_value : 0,
                        cccc2u16(&msg[MSG_BODY + CTRL_REF_SLOPE]));
                    break;
                case SINEWAVE:
                    ControlReference_setSineReference(
                        &fanController.control_reference,
                        fan_mode == 1 ? fan_set_value : 0,
                        cccc2u16(&msg[MSG_BODY + CTRL_REF_AMPL]),
                        ccc_cc2f(&msg[MSG_BODY + CTRL_REF_OMEGA]));
                    break;
                default:
                    break;
            }
            // BANG BANG
            //BBController_reset(&(fanController.BB_controller));
            BBController_setParams(&(fanController.BB_controller), cccc2u16(&msg[MSG_BODY + BB_SET_VAL]), cccc2u16(&msg[MSG_BODY + BB_HYSTERESIS]), 0);
            // PID
            fanController.PID_controller.Kp = ccc_cc2f(&msg[MSG_BODY + PID_KP]);
            fanController.PID_controller.Ki = ccc_cc2f(&msg[MSG_BODY + PID_KI]);
            fanController.PID_controller.Kd = ccc_cc2f(&msg[MSG_BODY + PID_KD]);
            fanController.PID_controller.Kaw = ccc_cc2f(&msg[MSG_BODY + PID_KAW]);
            // COMMON
            FanController_setController(&fanController, c2u8(msg[MSG_BODY + CONTROLLER_TYPE]));
            FanController_setMode(&fanController, fan_mode);    
            break;
        case COIL_CFG:
            // CONTROL REFERENCE
            uint16_t coil_set_value = cccc2u16(&msg[MSG_BODY + PID_SET_VAL]);
            uint8_t coil_mode = c2u8(msg[MSG_BODY + ON_OFF]);

            coilController.control_reference.type = c2u8(msg[MSG_BODY + CTRL_REF_TYPE]);
            
            switch(coilController.control_reference.type)
            {
                case STEP:
                    ControlReference_setStepReference(
                        &coilController.control_reference,
                        coil_mode == 1 ? coil_set_value : 0);
                    break;
                case RAMP:
                    ControlReference_setRampReference(
                        &coilController.control_reference,
                        coil_mode == 1 ? coil_set_value : 0,
                        cccc2u16(&msg[MSG_BODY + CTRL_REF_SLOPE]));
                    break;
                case SINEWAVE:
                    ControlReference_setSineReference(
                        &coilController.control_reference,
                        coil_mode == 1 ? coil_set_value : 0,
                        cccc2u16(&msg[MSG_BODY + CTRL_REF_AMPL]),
                        ccc_cc2f(&msg[MSG_BODY + CTRL_REF_OMEGA]));
                    break;
                default:
                   break;
            }
            // BANG BANG
            // BBController_reset(&(coilController.BB_controller));
            BBController_setParams(&(coilController.BB_controller), cccc2u16(&msg[MSG_BODY + BB_SET_VAL]), cccc2u16(&msg[MSG_BODY + BB_HYSTERESIS]), 0);
            coilController.BB_controller.u_max = c2u8(msg[MSG_BODY + CONTROLLER_TYPE]) == BANG_BANG ?
                    (uint16_t)((float)BB_U_MAX * ((float)ccc2u8(&msg[MSG_BODY + COIL_POWER])/100.0f)) :
                    BB_U_MAX;
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
            break;
        default:
            break;
    }
}
