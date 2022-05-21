#ifndef PACK_DATA_H
#define PACK_DATA_H

#include "stdint.h"


void packi16(uint8_t *buf, uint16_t i);
void packi32(uint8_t *buf, uint32_t i);
uint16_t unpacku16(uint8_t *buf);
uint32_t unpacku32(uint8_t *buf);




uint64_t pack754(long double f, unsigned bits, unsigned expbits);
#define pack754_32(f) (pack754((f), 32, 8))
#define pack754_64(f) (pack754((f), 64, 11))
#define unpack754_32(i) (unpack754((i), 32, 8))
#define unpack754_64(i) (unpack754((i), 64, 11))


#endif
