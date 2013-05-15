#ifndef _MICRO_ECDH_H_
#define _MICRO_ECDH_H_

#include <stdint.h>

/* Optimization settings. Define as 1 to enable an optimization, 0 to disable it.
ECC_SQUARE_FUNC - If enabled, this will cause a specific function to be used for (scalar) squaring instead of the generic
                  multiplication function. Improves speed by about 1-4% (or more if 32-bit multiplications are slow).
ECC_USE_NAF - If enabled, this will convert the private key to a non-adjacent form before point multiplication.
              Improves speed by about 10%.
*/
#define ECC_SQUARE_FUNC 1
#define ECC_USE_NAF 1
#define ECC_MULT64 1

#define ECC_CURVE secp160r1

#define secp128r1 4
#define secp160r1 5
#define secp192r1 6
#define secp224r1 7
#define secp256r1 8

#if (ECC_CURVE != secp128r1 && ECC_CURVE != secp160r1 && ECC_CURVE != secp192r1 && ECC_CURVE != secp224r1 && ECC_CURVE != secp256r1)
    #error "Must define ECC_CURVE to one of the available curves"
#endif

#define NUM_ECC_DIGITS ECC_CURVE

typedef struct EccPoint
{
	uint32_t x[NUM_ECC_DIGITS];
	uint32_t y[NUM_ECC_DIGITS];
} EccPoint;

int ecdh_shared_secret(uint32_t p_secret[NUM_ECC_DIGITS], EccPoint *p_publicKey, uint32_t p_privateKey[NUM_ECC_DIGITS]);
int ecdh_make_key(EccPoint *p_publicKey, uint32_t p_privateKey[NUM_ECC_DIGITS], uint32_t p_random[NUM_ECC_DIGITS]);

#endif /* _MICRO_ECDH_H_ */
