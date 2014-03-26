#ifndef _AVR_ECC_H_
#define _AVR_ECC_H_

#include <stdint.h>

/* Platform selection options.
If ECC_PLATFORM is not defined, the code will try to guess it based on compiler macros.
Possible values for ECC_PLATFORM are defined below: */
#define ecc_arch_other 0
#define ecc_x86        1
#define ecc_x86_64     2
#define ecc_arm        3
#define ecc_arm_thumb  4
#define ecc_avr        5

/* If desired, you can define ECC_WORD_SIZE as appropriate for your platform (1, 4, or 8 bytes).
If ECC_WORD_SIZE is not explicitly defined then it will be automatically set based on your platform. */

/* Inline assembly options.
ecc_asm_none  - Use standard C99 only.
ecc_asm_small - Use GCC inline assembly for the target platform (if available), optimized for minimum size.
ecc_asm_fast  - Use GCC inline assembly optimized for maximum speed. */
#define ecc_asm_none  0
#define ecc_asm_small 1
#define ecc_asm_fast  2
#ifndef ECC_ASM
    #define ECC_ASM ecc_asm_fast
#endif

/* Curve selection options. */
#define secp160r1 1
#define secp192r1 2
#define secp256r1 3
#ifndef ECC_CURVE
    #define ECC_CURVE secp160r1
#endif

/* Optimization settings. Define as 1 to enable an optimization, 0 to disable it.
ECC_SQUARE_FUNC - If enabled, this will cause a specific function to be used for (scalar) squaring instead of the generic
                  multiplication function. This will make things faster by about 8% but increases the code size. */
#define ECC_SQUARE_FUNC 1


#define ECC_CONCAT1(a, b) a##b
#define ECC_CONCAT(a, b) ECC_CONCAT1(a, b)

#define ecc_size_1 20 /* secp160r1 */
#define ecc_size_2 24 /* secp192r1 */
#define ecc_size_3 32 /* secp256r1 */

#define ECC_BYTES ECC_CONCAT(ecc_size_, ECC_CURVE)

#ifdef __cplusplus
extern "C"
{
#endif

/* RNG_Function type
The RNG function should fill p_size random bytes into p_dest. It should return 1 if
p_dest was filled with random data, or 0 if the random data could not be generated.
The filled-in values should be either truly random, or from a cryptographically-secure PRNG.

A correctly functioning RNG function must be set (using ecc_set_rng()) before calling
ecc_make_key() or ecdsa_sign().

A correct RNG function is set by default when building for Windows, Linux, or OS X.
If you are building on another POSIX-compliant system that supports /dev/random or /dev/urandom,
you can define ECC_POSIX to use the predefined RNG. For embedded platforms there is no predefined
RNG function; you must provide your own.
*/
typedef int (*RNG_Function)(uint8_t *p_dest, unsigned p_size);

/* ecc_set_rng() function.
Set the function that will be used to generate random bytes. The RNG function should
return 1 if the random data was generated, or 0 if the random data could not be generated.

On platforms where there is no predefined RNG function (eg embedded platforms), this must
be called before ecc_make_key() or ecdsa_sign() are used.
    
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
int ecc_make_key(uint8_t p_publicKey[ECC_BYTES*2], uint8_t p_privateKey[ECC_BYTES]);

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
int ecdh_shared_secret(const uint8_t p_publicKey[ECC_BYTES*2], const uint8_t p_privateKey[ECC_BYTES], uint8_t p_secret[ECC_BYTES]);

/* ecc_compress() function.
Compress a public key.

Inputs:
    p_publicKey - The public key to compress.

Outputs:
    p_compressed - Will be filled in with the compressed public key.
*/
void ecc_compress(uint8_t p_publicKey[ECC_BYTES*2], uint8_t p_compressed[ECC_BYTES+1]);

/* ecc_decompress() function.
Decompress a compressed public key.

Inputs:
    p_compressed - The compressed public key.

Outputs:
    p_publicKey - Will be filled in with the decompressed public key.
*/
void ecc_decompress(uint8_t p_compressed[ECC_BYTES+1], uint8_t p_publicKey[ECC_BYTES*2]);

/* ecdsa_sign() function.
Generate an ECDSA signature for a given hash value.

Usage: Compute a hash of the data you wish to sign (SHA-2 is recommended) and pass it in to
this function along with your private key.

Inputs:
    p_privateKey - Your private key.
    p_hash       - The message hash to sign.

Outputs:
    p_signature  - Will be filled in with the signature value.

Returns 1 if the signature generated successfully, 0 if an error occurred.
*/
int ecdsa_sign(const uint8_t p_privateKey[ECC_BYTES], const uint8_t p_hash[ECC_BYTES], uint8_t p_signature[ECC_BYTES*2]);

/* ecdsa_verify() function.
Verify an ECDSA signature.

Usage: Compute the hash of the signed data using the same hash as the signer and
pass it to this function along with the signer's public key and the signature values (r and s).

Inputs:
    p_publicKey - The signer's public key
    p_hash      - The hash of the signed data.
    p_signature - The signature value.

Returns 1 if the signature is valid, 0 if it is invalid.
*/
int ecdsa_verify(const uint8_t p_publicKey[ECC_BYTES*2], const uint8_t p_hash[ECC_BYTES], const uint8_t p_signature[ECC_BYTES*2]);

#ifdef __cplusplus
} /* end of extern "C" */
#endif

#endif /* _AVR_ECC_H_ */
