/*
 * communication.h
 *
 *  Created on: Nov 18, 2024
 *      Author: Szymon
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include "stdint.h"

uint8_t COM_checkConnection(void);
void COM_translateMsg(uint8_t *msg, uint16_t len);

#endif /* INC_COMMUNICATION_H_ */
