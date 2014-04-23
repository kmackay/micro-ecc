/* Copyright 2013 Kenneth MacKay. Licensed under the BSD 2-clause license. */

#ifndef _MICRO_ECC_H_
#define _MICRO_ECC_H_

#include <stdint.h>

/* Optimization settings. Define as 1 to enable an optimization, 0 to disable it.
ECC_SQUARE_FUNC - If enabled, this will cause a specific function to be used for (scalar) squaring instead of the generic
                  multiplication function. Improves speed by about 8% .
*/
#define ECC_SQUARE_FUNC 1

/* Inline assembly options.
Inline assembly (gcc format) is provided for selected operations for Thumb and Thumb2/ARM.
Improves speed by about 57% on Cortex-M0 when using ecc_asm_thumb.

Note: You must choose the appropriate option for your target architecture, or compilation will fail
with strange assembler messages.
*/
#define ecc_asm_none   0
#define ecc_asm_thumb  1 /* ARM Thumb assembly (including Cortex-M0) */
#define ecc_asm_thumb2 2 /* ARM Thumb-2 assembly (eg Cortex-M3) */
#define ecc_asm_arm    3 /* Regular ARM assembly */
#ifndef ECC_ASM
    #define ECC_ASM ecc_asm_none
#endif

#define ECC_CONCAT1(a, b) a##b
#define ECC_CONCAT(a, b) ECC_CONCAT1(a, b)

/* Curve selection options. */
#define secp128r1 1
#define secp192r1 2
#define secp256r1 3
#define secp384r1 4
#define secp256k1 5

#ifndef ECC_CURVE
    #define ECC_CURVE secp256r1
#endif

#define ecc_size_1 4
#define ecc_size_2 6
#define ecc_size_3 8
#define ecc_size_4 12
#define ecc_size_5 8

#define NUM_ECC_DIGITS ECC_CONCAT(ecc_size_, ECC_CURVE)

typedef struct EccPoint
{
    uint32_t x[NUM_ECC_DIGITS];
    uint32_t y[NUM_ECC_DIGITS];
} EccPoint;

/* ecc_make_key() function.
Create a public/private key pair.

You must use a new nonpredictable random number to generate each new key pair.

Outputs:
    p_publicKey  - Will be filled in with the point representing the public key.
    p_privateKey - Will be filled in with the private key.

Inputs:
    p_random - The random number to use to generate the key pair.

Returns 1 if the key pair was generated successfully, 0 if an error occurred. If 0 is returned,
try again with a different random number.
*/
int ecc_make_key(EccPoint *p_publicKey, uint32_t p_privateKey[NUM_ECC_DIGITS], uint32_t p_random[NUM_ECC_DIGITS]);

/* ecc_valid_public_key() function.
Determine whether or not a given point is on the chosen elliptic curve (ie, is a valid public key).

Inputs:
    p_publicKey - The point to check.

Returns 1 if the given point is valid, 0 if it is invalid.
*/
int ecc_valid_public_key(EccPoint *p_publicKey);

/* ecdh_shared_secret() function.
Compute a shared secret given your secret key and someone else's public key.

Optionally, you can provide a random multiplier for resistance to DPA attacks. The random multiplier
should probably be different for each invocation of ecdh_shared_secret().

Outputs:
    p_secret - Will be filled in with the shared secret value.
    
Inputs:
    p_publicKey  - The public key of the remote party.
    p_privateKey - Your private key.
    p_random     - An optional random number to resist DPA attacks. Pass in NULL if DPA attacks are not a concern.

Returns 1 if the shared secret was computed successfully, 0 otherwise.

Note: It is recommended that you hash the result of ecdh_shared_secret before using it for symmetric encryption or HMAC.
If you do not hash the shared secret, you must call ecc_valid_public_key() to verify that the remote side's public key is valid.
If this is not done, an attacker could create a public key that would cause your use of the shared secret to leak information
about your private key. */
int ecdh_shared_secret(uint32_t p_secret[NUM_ECC_DIGITS], EccPoint *p_publicKey, uint32_t p_privateKey[NUM_ECC_DIGITS], uint32_t p_random[NUM_ECC_DIGITS]);

/* ecdsa_sign() function.
Generate an ECDSA signature for a given hash value.

Usage: Compute a hash of the data you wish to sign (SHA-2 is recommended) and pass it in to
this function along with your private key and a random number.
You must use a new nonpredictable random number to generate each new signature.

Outputs:
    r, s - Will be filled in with the signature values.

Inputs:
    p_privateKey - Your private key.
    p_random     - The random number to use to generate the signature.
    p_hash       - The message hash to sign.

Returns 1 if the signature generated successfully, 0 if an error occurred. If 0 is returned,
try again with a different random number.
*/
int ecdsa_sign(uint32_t r[NUM_ECC_DIGITS], uint32_t s[NUM_ECC_DIGITS], uint32_t p_privateKey[NUM_ECC_DIGITS],
    uint32_t p_random[NUM_ECC_DIGITS], uint32_t p_hash[NUM_ECC_DIGITS]);

/* ecdsa_verify() function.
Verify an ECDSA signature.

Usage: Compute the hash of the signed data using the same hash as the signer and
pass it to this function along with the signer's public key and the signature values (r and s).

Inputs:
    p_publicKey - The signer's public key
    p_hash      - The hash of the signed data.
    r, s        - The signature values.

Returns 1 if the signature is valid, 0 if it is invalid.
*/
int ecdsa_verify(EccPoint *p_publicKey, uint32_t p_hash[NUM_ECC_DIGITS], uint32_t r[NUM_ECC_DIGITS], uint32_t s[NUM_ECC_DIGITS]);

/* ecc_bytes2native() function.
Convert an integer in standard octet representation to the native format.

Outputs:
    p_native - Will be filled in with the native integer value.

Inputs:
    p_bytes - The standard octet representation of the integer to convert.
*/
void ecc_bytes2native(uint32_t p_native[NUM_ECC_DIGITS], uint8_t p_bytes[NUM_ECC_DIGITS*4]);

/* ecc_native2bytes() function.
Convert an integer in native format to the standard octet representation.

Outputs:
    p_bytes - Will be filled in with the standard octet representation of the integer.

Inputs:
    p_native - The native integer value to convert.
*/
void ecc_native2bytes(uint8_t p_bytes[NUM_ECC_DIGITS*4], uint32_t p_native[NUM_ECC_DIGITS]);

/* ecc_point_compress() function.
Compress a point from native format into the standard compressed octet representation.

Outputs:
    p_compressed  - Will be filled in with the compressed point representation.

Inputs:
    p_point - The point to compress.
*/
void ecc_point_compress(uint8_t p_compressed[NUM_ECC_DIGITS*4 + 1], EccPoint *p_point);

/* ecc_point_compress() function.
Decompress a point from the standard compressed octet representation to native format.

Outputs:
    p_point  - Will be filled in with the native point representation.

Inputs:
    p_compressed - The standard compressed octet representation of the point.
*/
void ecc_point_decompress(EccPoint *p_point, uint8_t p_compressed[NUM_ECC_DIGITS*4 + 1]);

#endif /* _MICRO_ECC_H_ */
