#ifndef _MICRO_ECDH_H_
#define _MICRO_ECDH_H_

#include <stdint.h>

#define secp128r1 4
#define secp160r1 5
#define secp192r1 6
#define secp224r1 7
#define secp256r1 8

#define ECC_CURVE secp160r1

#if !(ECC_CURVE)
    #error "Must define ECC_CURVE to one of the available curves"
#endif

#define NUM_ECC_DIGITS ECC_CURVE

typedef struct EccPoint
{
	uint32_t x[NUM_ECC_DIGITS];
	uint32_t y[NUM_ECC_DIGITS];
} EccPoint;

typedef struct Curve
{
	uint32_t p[NUM_ECC_DIGITS];
    /* the other curve parameters are not necessary to compute the shared secret. */
} Curve;

int ecdh_shared_secret(uint8_t *p_secret, unsigned int p_len, EccPoint *p_publicKey, uint32_t *p_privateKey);

#endif /* _MICRO_ECDH_H_ */
