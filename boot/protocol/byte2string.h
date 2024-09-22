#ifndef __BYTE_2_STRING_H
#define __BYTE_2_STRING_H
#include <stdint.h>


void OneByteToStr(uint8_t byte, uint8_t *str);
void MultiByteToStr(uint8_t *byteBuf, uint16_t len, uint8_t *str);


#endif

