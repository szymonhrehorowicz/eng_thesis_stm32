/*
 * communication.h
 *
 *  Created on: Nov 18, 2024
 *      Author: Szymon
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include "stdint.h"

typedef enum
{
    ALL_FAN_DATA,
    FAST_FAN_DATA,
    ALL_COIL_DATA,
    FAST_COIL_DATA,
} OutgoingMessages_t;

void COM_checkConnection(void);

void COM_sendAllData(void);
void COM_sendFastData(void);

void COM_translateMsg(uint8_t *msg, uint16_t len);

extern volatile uint32_t lastMsgTimestamp;
extern volatile uint8_t wasAlreadyReset;
extern volatile uint8_t isConnected;

#endif /* INC_COMMUNICATION_H_ */
