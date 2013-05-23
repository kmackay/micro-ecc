#ifndef _MICRO_ECDH_H_
#define _MICRO_ECDH_H_

#include <stdint.h>

/* Optimization settings. Define as 1 to enable an optimization, 0 to disable it.
ECC_SQUARE_FUNC - If enabled, this will cause a specific function to be used for (scalar) squaring instead of the generic
                  multiplication function. Improves speed by about 8% .
ECC_USE_NAF - If enabled, this will convert the private key to a non-adjacent form before point multiplication.
              Improves speed by about 10%.
ECC_SOFT_MULT64 - For platforms that do not have instructions to allow a fast 64x64 bit multiply (eg Cortex-M0), this option
                  enables code to do 64x64 bit multiplies faster than libgcc does. Improves speed by about 6% on Cortex-M0
                  (with 32-cycle multiply instruction). Do not enable on platforms that have a fast 64x64 bit multiply.
*/
#define ECC_SQUARE_FUNC 0
#define ECC_USE_NAF 0
#define ECC_SOFT_MULT64 1

#define ECC_CURVE secp192r1

#define secp128r1 4
#define secp192r1 6
#define secp256r1 8
#define secp384r1 12

#if (ECC_CURVE != secp128r1 && ECC_CURVE != secp192r1 && ECC_CURVE != secp256r1 && ECC_CURVE != secp384r1)
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
int ecc_valid_public_key(EccPoint *p_publicKey);

/* Note: It is recommended that you hash the result of ecdh_shared_secret before using it for symmetric encryption or HMAC.
If you do not hash the shared secret, you must call ecc_valid_public_key() to verify that the remote side's public key is valid.
If this is not done, an attacker could create a public key that would cause your use of the shared secret to leak information
about your private key. */

int ecdsa_sign(uint32_t p_privateKey[NUM_ECC_DIGITS], uint32_t p_random[NUM_ECC_DIGITS], uint32_t p_hash[NUM_ECC_DIGITS],
    uint32_t r[NUM_ECC_DIGITS], uint32_t s[NUM_ECC_DIGITS]);
int ecdsa_verify(EccPoint *p_publicKey, uint32_t p_hash[NUM_ECC_DIGITS], uint32_t r[NUM_ECC_DIGITS], uint32_t s[NUM_ECC_DIGITS]);

#endif /* _MICRO_ECDH_H_ */
