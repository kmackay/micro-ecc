#ifndef _AVR_ECC_H_
#define _AVR_ECC_H_

#include <stdint.h>

/* Optimization settings. Define as 1 to enable an optimization, 0 to disable it.
ECC_SQUARE_FUNC - If enabled, this will cause a specific function to be used for (scalar) squaring instead of the generic
                  multiplication function.
*/
#define ECC_SQUARE_FUNC 1

/* Inline assembly options.
Inline assembly (gcc format) is provided for selected operations for AVR (requires MUL support).
*/
#define ecc_asm_none   0
#define ecc_asm_avr    1
#ifndef ECC_ASM
    #define ECC_ASM ecc_asm_none
#endif

#define ECC_CONCAT1(a, b) a##b
#define ECC_CONCAT(a, b) ECC_CONCAT1(a, b)

/* Curve selection options. */
#define secp128r1 1
#define secp160r1 2
#define secp192r1 3
#define secp256r1 4

#ifndef ECC_CURVE
    #define ECC_CURVE secp160r1
#endif

#define ecc_size_1 16
#define ecc_size_2 20
#define ecc_size_3 24
#define ecc_size_4 32

#define ECC_BYTES ECC_CONCAT(ecc_size_, ECC_CURVE)

#ifdef __cplusplus
extern "C"
{
#endif

/* RNG_Function type
The RNG function should fill p_size random bytes into p_dest. It should return 1 if
p_dest was filled with random data, or 0 if the random data could not be generated.
*/
typedef int (*RNG_Function)(uint8_t *p_dest, unsigned p_size);

/* ecc_set_rng() function.
Set the function that will be used to generate random bytes. The RNG function should
return 1 if the random data was generated, or 0 if the random data could not be generated.

This must be called before ecc_make_key(), ecdh_shared_secret(), or ecdsa_sign() are used.
    
Inputs:
    p_rng  - The function that will be used to generate random bytes.
*/
void ecc_set_rng(RNG_Function p_rng);

/* ecc_make_key() function.
Create a public/private key pair.
    
Outputs:
    p_publicKey  - Will be filled in with the public key.
    p_privateKey - Will be filled in with the private key.

Returns 1 if the key pair was generated successfully, 0 if an error occurred.
*/
int ecc_make_key(uint8_t p_publicKey[ECC_BYTES+1], uint8_t p_privateKey[ECC_BYTES]);

/* ecdh_shared_secret() function.
Compute a shared secret given your secret key and someone else's public key.
Note: It is recommended that you hash the result of ecdh_shared_secret before using it for symmetric encryption or HMAC.

Inputs:
    p_publicKey  - The public key of the remote party.
    p_privateKey - Your private key.

Outputs:
    p_secret - Will be filled in with the shared secret value.

Returns 1 if the shared secret was generated successfully, 0 if an error occurred.
*/
int ecdh_shared_secret(const uint8_t p_publicKey[ECC_BYTES+1], const uint8_t p_privateKey[ECC_BYTES], uint8_t p_secret[ECC_BYTES]);

#ifdef __cplusplus
} /* end of extern "C" */
#endif

#endif /* _EASY_ECC_H_ */
