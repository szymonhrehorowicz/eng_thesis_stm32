/*
 * char_translator.h
 *
 *  Created on: Dec 5, 2024
 *      Author: Szymon
 */

#ifndef INC_UTILS_CHAR_TRANSLATOR_H_
#define INC_UTILS_CHAR_TRANSLATOR_H_

#include "stdint.h"

uint8_t c2u8(uint8_t c);      // 1 char to uint8
uint8_t cc2u8(uint8_t *c);    // 2 chars to uint8
uint8_t ccc2u8(uint8_t *c);   // 3 chars to uint8
uint16_t cccc2u16(uint8_t *c); // 4 chars to uint8
float ccc_cc2f(uint8_t *c);   // char containg a float as 000.00 to float

#endif /* INC_UTILS_CHAR_TRANSLATOR_H_ */
