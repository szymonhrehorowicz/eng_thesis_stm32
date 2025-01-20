/*
 * char_translator.c
 *
 *  Created on: Dec 5, 2024
 *      Author: Szymon
 */
#include "utils/char_translator.h"

uint8_t c2u8(uint8_t c)
{
    return (uint8_t)(c - '0');
}

uint8_t cc2u8(uint8_t *c)
{
    return ((uint8_t)(c[0] - '0') * 10) + ((uint8_t)(c[1]) - '0');
}

uint8_t ccc2u8(uint8_t *c)
{
    return ((uint8_t)(c[0] - '0') * 100) + ((uint8_t)(c[1] - '0') * 10) + ((uint8_t)(c[2]) - '0');
}

uint16_t cccc2u16(uint8_t *c)
{
    uint16_t thousands, hundreds, tens, ones;
    thousands = c2u8(c[0]) * 1000;
    hundreds = c2u8(c[1]) * 100;
    tens = c2u8(c[2]) * 10;
    ones = c2u8(c[3]);
    return thousands + hundreds + tens + ones;
}

float ccc_cc2f(uint8_t *c)
{
    float khundreds = c2u8(c[0]) * 100.0f;
    float ktens, kones, kdtens, kdhundreds;
    ktens = c2u8(c[1]) * 10;
    kones = c2u8(c[2]);
    kdtens = c2u8(c[4]) / 10.0f;
    kdhundreds = c2u8(c[5]) / 100.0f;
    return khundreds + ktens + kones + kdtens + kdhundreds;
}

void float_to_bytes(uint8_t* buffer, uint8_t *idx, float *value)
{
    uint8_t *byte_ptr = (uint8_t*)value;

    buffer[*idx] = byte_ptr[3]; *idx += 1;
    buffer[*idx] = byte_ptr[2]; *idx += 1;
    buffer[*idx] = byte_ptr[1]; *idx += 1;
    buffer[*idx] = byte_ptr[0]; *idx += 1;
}
