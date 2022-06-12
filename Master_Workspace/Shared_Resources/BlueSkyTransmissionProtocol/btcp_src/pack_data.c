#include "pack_data.h"




// Note: arrayToFloat and floatToArray are not cross platform
// They only work on hardware where floating point values are stored in IEEE 754 format.
// STM32 uses IEEE 754 format

float arrayToFloat(uint8_t* aryPtr) {

	float val = 0;
	uint8_t* valPtr = (uint8_t*)&val;


	for (int aryIdx = sizeof(val)-1; aryIdx >= 0; aryIdx--) {
		*valPtr = aryPtr[aryIdx];
		valPtr++;
	}
	return val;
}

void floatToArray(float val, uint8_t* aryPtr) {
    int aryIdx = sizeof(val)-1;
    uint8_t* ptr = (uint8_t*)&val;
    for(; aryIdx >= 0; aryIdx--){
    	aryPtr[aryIdx] = *ptr;
    	ptr++;
    }
}



// Everything below works cross platform
/* ===========================================================*/

/*
** packi16() -- store a 16-bit unsigned into a uint8_t buffer (like htons())
*/
void packi16(uint8_t *buf, uint16_t i)
{
    *buf++ = i>>8; *buf++ = i;
}

/*
** packi32() -- store a 32-bit unsigned into a uint8_t buffer (like htonl())
*/
void packi32(uint8_t *buf, uint32_t i)
{
    *buf++ = i>>24; *buf++ = i>>16;
    *buf++ = i>>8;  *buf++ = i;
}

/*
** unpacku16() -- unpack a 16-bit unsigned from a uint8_t buffer (like ntohs())
*/
uint16_t unpacku16(uint8_t *buf)
{
    return ((uint16_t)buf[0]<<8) | buf[1];
}

/*
** unpacku32() -- unpack a 32-bit unsigned from a uint8_t buffer (like ntohl())
*/
uint32_t unpacku32(uint8_t *buf)
{
    return ((uint32_t)buf[0]<<24) |
           ((uint32_t)buf[1]<<16) |
           ((uint32_t)buf[2]<<8)  |
           buf[3];
}




uint64_t pack754(long double f, unsigned bits, unsigned expbits)
{
    long double fnorm;
    int shift;
    long long sign, exp, significand;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (f == 0.0) return 0; // get this special case out of the way

    // check sign and begin normalization
    if (f < 0) { sign = 1; fnorm = -f; }
    else { sign = 0; fnorm = f; }

    // get the normalized form of f and track the exponent
    shift = 0;
    while (fnorm >= 2.0) { fnorm /= 2.0; shift++; }
    while (fnorm < 1.0) { fnorm *= 2.0; shift--; }
    fnorm = fnorm - 1.0;

    // calculate the binary form (non-float) of the significand data
    significand = fnorm * ((1LL << significandbits) + 0.5f);

    // get the biased exponent
    exp = shift + ((1 << (expbits - 1)) - 1); // shift + bias

    // return the final answer
    return (sign << (bits - 1)) | (exp << (bits - expbits - 1)) | significand;
}

long double unpack754(uint64_t i, unsigned bits, unsigned expbits)
{
    long double result;
    long long shift;
    unsigned bias;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (i == 0) return 0.0;

    // pull the significand
    result = (i & ((1LL << significandbits) - 1)); // mask
    result /= (1LL << significandbits); // convert back to float
    result += 1.0f; // add the one back on

    // deal with the exponent
    bias = (1 << (expbits - 1)) - 1;
    shift = ((i >> significandbits) & ((1LL << expbits) - 1)) - bias;
    while (shift > 0) { result *= 2.0; shift--; }
    while (shift < 0) { result /= 2.0; shift++; }

    // sign it
    result *= (i >> (bits - 1)) & 1 ? -1.0 : 1.0;

    return result;
}


