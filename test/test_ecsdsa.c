/* Copyright 2014, Kenneth MacKay. Licensed under the BSD 2-clause license.
 *
 * ECSDSA sign/verify tests - known-answer test vectors plus random round-trip tests.
 *
 * Two variants (differ only in what is hashed to produce R):
 *   Optimized: R = h(x1 || M)        -- x coordinate only
 *   Standard:  R = h(x1 || y1 || M)  -- both coordinates
 */

#include "uECC.h"

#if !(uECC_SUPPORTS_ECSDSA_OPTIMIZED || uECC_SUPPORTS_ECSDSA_STANDARD)
#include <stdio.h>
int main(void) { printf("ECSDSA disabled\n"); return 0; }
#else

/*
 * Enable the built-in hash adapters for this test.
 *
 * NOTE: uECC_SUPPORTS_SHA256 and uECC_SUPPORTS_SHA512 default to 0 in
 * uECC_hash.h so that projects which already carry a hash library do not
 * compile a second copy.  We opt in explicitly here for testing purposes.
 */
#define uECC_SUPPORTS_SHA256 1
#define uECC_SUPPORTS_SHA512 1
#include "uECC_hash.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* ---- helpers ---- */

static void print_hex(const char *label, const uint8_t *data, unsigned len) {
    unsigned i;
    printf("%s: ", label);
    for (i = 0; i < len; ++i) printf("%02x", data[i]);
    printf("\n");
}

/* ---- deterministic RNG for sign vector tests ----
 *
 * Injects a known k on the first call (reversed to native LE VLI layout).
 * Returns value 1 on the second call (blinding factor).
 */
static const uint8_t *s_rng_k_be;
static unsigned       s_rng_k_len;
static int            s_rng_calls;

static int vector_sign_rng(uint8_t *dest, unsigned size) {
    unsigned i;
    if (s_rng_calls == 0 && size == s_rng_k_len) {
        for (i = 0; i < size; ++i)
            dest[i] = s_rng_k_be[size - 1 - i];
        s_rng_calls++;
        return 1;
    }
    memset(dest, 0, size);
    if (size > 0) dest[0] = 1;
    s_rng_calls++;
    return 1;
}

/* ---- EC-SDSA known-answer test vectors ----
 *
 * Vector 1: secp256r1 / SHA-256 / M = "abc"
 * Vector 2: secp256r1 / SHA-512 / M = "abc"  (same key and k as vector 1)
 *   private key, public key, ephemeral k, and both variant signatures.
 */

/* Shared key material and k for vectors 1 and 2 */
static const uint8_t vec_priv[32] = {
    0xC9,0xAF,0xA9,0xD8,0x45,0xBA,0x75,0x16,0x6B,0x5C,0x21,0x57,0x67,0xB1,0xD6,0x93,
    0x4E,0x50,0xC3,0xDB,0x36,0xE8,0x9B,0x12,0x7B,0x8A,0x62,0x2B,0x12,0x0F,0x67,0x21
};
static const uint8_t vec_pub[64] = {
    /* Qx */
    0x60,0xFE,0xD4,0xBA,0x25,0x5A,0x9D,0x31,0xC9,0x61,0xEB,0x74,0xC6,0x35,0x6D,0x68,
    0xC0,0x49,0xB8,0x92,0x3B,0x61,0xFA,0x6C,0xE6,0x69,0x62,0x2E,0x60,0xF2,0x9F,0xB6,
    /* Qy */
    0x79,0x03,0xFE,0x10,0x08,0xB8,0xBC,0x99,0xA4,0x1A,0xE9,0xE9,0x56,0x28,0xBC,0x64,
    0xF2,0xF1,0xB2,0x0C,0x2D,0x7E,0x9F,0x51,0x77,0xA3,0xC2,0x94,0xD4,0x46,0x22,0x99
};
static const uint8_t vec_k_be[32] = {
    0xAF,0x2B,0xDB,0xE1,0xAA,0x9B,0x6E,0xC1,0xE2,0xAD,0xE1,0xD6,0x94,0xF4,0x1F,0xC7,
    0x1A,0x83,0x1D,0x02,0x68,0xE9,0x89,0x15,0x62,0x11,0x3D,0x8A,0x62,0xAD,0xD1,0xBF
};
static const uint8_t vec_msg[] = { 0x61, 0x62, 0x63 }; /* "abc" */

/* =========================================================================
 * SHA-256 vector tests
 * ========================================================================= */

#if uECC_SUPPORTS_ECSDSA_OPTIMIZED && uECC_SUPPORTS_secp256r1
static int test_ecsdsa_vector_sha256_optimized(void) {
    /* Vector 1 optimized-variant signature: R = h256(x1 || M) */
    static const uint8_t sig[64] = {
        /* R */
        0x59,0x50,0x47,0x06,0x6F,0xDC,0x55,0x53,0x7D,0xCB,0x03,0xC8,0x8F,0xCA,0xDC,0x73,
        0xAD,0x60,0x0F,0x39,0x9A,0x10,0x12,0xC1,0xA9,0xC2,0x85,0x1F,0x68,0x89,0x80,0xE3,
        /* S */
        0x69,0x65,0x9E,0x5C,0x8A,0xD4,0x86,0xC8,0x31,0xEE,0x12,0x20,0xE1,0x9F,0x09,0x34,
        0x73,0x74,0xF5,0x2F,0xB9,0xD8,0xEB,0x8A,0xCA,0x98,0x7F,0x65,0xFF,0x72,0x66,0x1C
    };
    uECC_SHA256_HashContext hctx;
    uECC_SHA256_HashContext_init(&hctx);
    uECC_Curve curve = uECC_secp256r1();
    uint8_t computed_sig[64];
    uint8_t bad[64];
    uECC_RNG_Function old_rng;

    old_rng = uECC_get_rng();
    s_rng_k_be  = vec_k_be;
    s_rng_k_len = 32;
    s_rng_calls = 0;
    uECC_set_rng(vector_sign_rng);
    if (!uECC_ecsdsa_sign_optimized(vec_priv, vec_msg, sizeof(vec_msg),
                                    &hctx.uECC, computed_sig, curve)) {
        uECC_set_rng(old_rng);
        printf("FAIL: vec1 sha256 optimized sign failed\n");
        return 0;
    }
    uECC_set_rng(old_rng);

    if (memcmp(computed_sig, sig, 64) != 0) {
        printf("FAIL: vec1 sha256 optimized sign: R or S mismatch\n");
        print_hex("  got R", computed_sig,      32);
        print_hex("  exp R", sig,               32);
        print_hex("  got S", computed_sig + 32, 32);
        print_hex("  exp S", sig + 32,          32);
        return 0;
    }

    if (!uECC_ecsdsa_verify_optimized(vec_pub, vec_msg, sizeof(vec_msg),
                                      &hctx.uECC, sig, curve)) {
        printf("FAIL: vec1 sha256 optimized verify: valid sig rejected\n");
        return 0;
    }
    memcpy(bad, sig, 64); bad[0] ^= 0x01;
    if (uECC_ecsdsa_verify_optimized(vec_pub, vec_msg, sizeof(vec_msg),
                                     &hctx.uECC, bad, curve)) {
        printf("FAIL: vec1 sha256 optimized verify: modified R accepted\n");
        return 0;
    }
    memcpy(bad, sig, 64); bad[32] ^= 0x01;
    if (uECC_ecsdsa_verify_optimized(vec_pub, vec_msg, sizeof(vec_msg),
                                     &hctx.uECC, bad, curve)) {
        printf("FAIL: vec1 sha256 optimized verify: modified S accepted\n");
        return 0;
    }
    {
        uint8_t bad_msg[] = { 0x61, 0x62, 0x64 };
        if (uECC_ecsdsa_verify_optimized(vec_pub, bad_msg, sizeof(bad_msg),
                                         &hctx.uECC, sig, curve)) {
            printf("FAIL: vec1 sha256 optimized verify: modified message accepted\n");
            return 0;
        }
    }
    printf("PASS: EC-SDSA optimized secp256r1/SHA-256 vector (sign+verify)\n");
    return 1;
}
#endif

#if uECC_SUPPORTS_ECSDSA_STANDARD && uECC_SUPPORTS_secp256r1
static int test_ecsdsa_vector_sha256_standard(void) {
    /* Vector 1 standard-variant signature: R = h256(x1 || y1 || M) */
    static const uint8_t sig[64] = {
        /* R */
        0x41,0xA8,0x23,0x68,0xB5,0x1B,0x34,0xD7,0x77,0x44,0xFC,0x08,0x2A,0xAD,0xC7,0x36,
        0xB4,0xFE,0x19,0x65,0xDA,0xB7,0x29,0xBC,0x65,0xD6,0x18,0x5B,0x8D,0x3F,0x37,0xF8,
        /* S */
        0xA4,0x67,0x45,0x2B,0x0C,0x04,0xAC,0x5B,0x4F,0x92,0xE0,0x96,0x8D,0xA1,0x42,0xA6,
        0x61,0x19,0xC2,0xEE,0xE5,0x0E,0x5A,0x50,0xB7,0x7B,0x12,0xB8,0x30,0x33,0x84,0x74
    };
    uECC_SHA256_HashContext hctx;
    uECC_SHA256_HashContext_init(&hctx);
    uECC_Curve curve = uECC_secp256r1();
    uint8_t computed_sig[64];
    uint8_t bad[64];
    uECC_RNG_Function old_rng;

    old_rng = uECC_get_rng();
    s_rng_k_be  = vec_k_be;
    s_rng_k_len = 32;
    s_rng_calls = 0;
    uECC_set_rng(vector_sign_rng);
    if (!uECC_ecsdsa_sign_standard(vec_priv, vec_msg, sizeof(vec_msg),
                                   &hctx.uECC, computed_sig, curve)) {
        uECC_set_rng(old_rng);
        printf("FAIL: vec1 sha256 standard sign failed\n");
        return 0;
    }
    uECC_set_rng(old_rng);

    if (memcmp(computed_sig, sig, 64) != 0) {
        printf("FAIL: vec1 sha256 standard sign: R or S mismatch\n");
        print_hex("  got R", computed_sig,      32);
        print_hex("  exp R", sig,               32);
        print_hex("  got S", computed_sig + 32, 32);
        print_hex("  exp S", sig + 32,          32);
        return 0;
    }

    if (!uECC_ecsdsa_verify_standard(vec_pub, vec_msg, sizeof(vec_msg),
                                     &hctx.uECC, sig, curve)) {
        printf("FAIL: vec1 sha256 standard verify: valid sig rejected\n");
        return 0;
    }
    memcpy(bad, sig, 64); bad[0] ^= 0x01;
    if (uECC_ecsdsa_verify_standard(vec_pub, vec_msg, sizeof(vec_msg),
                                    &hctx.uECC, bad, curve)) {
        printf("FAIL: vec1 sha256 standard verify: modified R accepted\n");
        return 0;
    }
    memcpy(bad, sig, 64); bad[32] ^= 0x01;
    if (uECC_ecsdsa_verify_standard(vec_pub, vec_msg, sizeof(vec_msg),
                                    &hctx.uECC, bad, curve)) {
        printf("FAIL: vec1 sha256 standard verify: modified S accepted\n");
        return 0;
    }
    {
        uint8_t bad_msg[] = { 0x61, 0x62, 0x64 };
        if (uECC_ecsdsa_verify_standard(vec_pub, bad_msg, sizeof(bad_msg),
                                        &hctx.uECC, sig, curve)) {
            printf("FAIL: vec1 sha256 standard verify: modified message accepted\n");
            return 0;
        }
    }
    printf("PASS: EC-SDSA standard secp256r1/SHA-256 vector (sign+verify)\n");
    return 1;
}
#endif

/* =========================================================================
 * SHA-512 vector tests
 * sig format: R(64 bytes) || S(32 bytes) = 96 bytes for secp256r1
 * ========================================================================= */

#if uECC_SUPPORTS_SHA512

#if uECC_SUPPORTS_ECSDSA_OPTIMIZED && uECC_SUPPORTS_secp256r1
static int test_ecsdsa_vector_sha512_optimized(void) {
    /* Vector 2 optimized-variant signature: R = h512(x1 || M) */
    static const uint8_t sig[96] = {
        /* R (64 bytes) */
        0xD7,0x6F,0xA5,0x59,0xAC,0x1D,0x6C,0x15,0x18,0xAF,0xDF,0x3A,0x94,0x37,0xB4,0x05,
        0x21,0xEA,0x59,0xA4,0x7B,0x58,0xF1,0xF0,0xF6,0xA2,0x03,0x27,0x1F,0x75,0x65,0x4E,
        0x88,0xAF,0x12,0x3F,0x44,0x0B,0x46,0x7C,0x20,0x02,0x62,0xA1,0xE6,0x0F,0xB8,0x8A,
        0xA4,0xF6,0x0F,0x93,0xDA,0x58,0x96,0xD6,0x18,0xD6,0xD3,0x89,0x59,0xD0,0x8C,0x77,
        /* S (32 bytes) */
        0x0F,0x58,0x0A,0x62,0xC8,0x78,0x67,0x5C,0xCD,0xE9,0x37,0x0A,0x26,0x5A,0x5B,0x41,
        0x10,0x30,0xA1,0xA7,0xDC,0x94,0x36,0xED,0x32,0x76,0x3C,0x57,0xE1,0xA8,0xD3,0x55
    };
    uECC_SHA512_HashContext hctx;
    uECC_SHA512_HashContext_init(&hctx);
    uECC_Curve curve = uECC_secp256r1();
    uint8_t computed_sig[96];
    uint8_t bad[96];
    uECC_RNG_Function old_rng;

    old_rng = uECC_get_rng();
    s_rng_k_be  = vec_k_be;
    s_rng_k_len = 32;
    s_rng_calls = 0;
    uECC_set_rng(vector_sign_rng);
    if (!uECC_ecsdsa_sign_optimized(vec_priv, vec_msg, sizeof(vec_msg),
                                    &hctx.uECC, computed_sig, curve)) {
        uECC_set_rng(old_rng);
        printf("FAIL: vec2 sha512 optimized sign failed\n");
        return 0;
    }
    uECC_set_rng(old_rng);

    if (memcmp(computed_sig, sig, 96) != 0) {
        printf("FAIL: vec2 sha512 optimized sign: R or S mismatch\n");
        print_hex("  got R", computed_sig,      64);
        print_hex("  exp R", sig,               64);
        print_hex("  got S", computed_sig + 64, 32);
        print_hex("  exp S", sig + 64,          32);
        return 0;
    }

    if (!uECC_ecsdsa_verify_optimized(vec_pub, vec_msg, sizeof(vec_msg),
                                      &hctx.uECC, sig, curve)) {
        printf("FAIL: vec2 sha512 optimized verify: valid sig rejected\n");
        return 0;
    }
    memcpy(bad, sig, 96); bad[0] ^= 0x01;
    if (uECC_ecsdsa_verify_optimized(vec_pub, vec_msg, sizeof(vec_msg),
                                     &hctx.uECC, bad, curve)) {
        printf("FAIL: vec2 sha512 optimized verify: modified R accepted\n");
        return 0;
    }
    memcpy(bad, sig, 96); bad[64] ^= 0x01;
    if (uECC_ecsdsa_verify_optimized(vec_pub, vec_msg, sizeof(vec_msg),
                                     &hctx.uECC, bad, curve)) {
        printf("FAIL: vec2 sha512 optimized verify: modified S accepted\n");
        return 0;
    }
    {
        uint8_t bad_msg[] = { 0x61, 0x62, 0x64 };
        if (uECC_ecsdsa_verify_optimized(vec_pub, bad_msg, sizeof(bad_msg),
                                         &hctx.uECC, sig, curve)) {
            printf("FAIL: vec2 sha512 optimized verify: modified message accepted\n");
            return 0;
        }
    }
    printf("PASS: EC-SDSA optimized secp256r1/SHA-512 vector (sign+verify)\n");
    return 1;
}
#endif

#if uECC_SUPPORTS_ECSDSA_STANDARD && uECC_SUPPORTS_secp256r1
static int test_ecsdsa_vector_sha512_standard(void) {
    /* Vector 2 standard-variant signature: R = h512(x1 || y1 || M) */
    static const uint8_t sig[96] = {
        /* R (64 bytes) */
        0x08,0xAA,0xD4,0xF7,0x82,0x89,0xCE,0xED,0xCC,0xD4,0x76,0xD7,0x71,0x61,0xF2,0xE2,
        0x28,0xF2,0xE1,0x3F,0x44,0x09,0x21,0x5C,0x80,0xF1,0x4C,0x70,0xE3,0xD9,0xD5,0xCE,
        0x8F,0x0F,0xFA,0x9B,0xA8,0x43,0xEC,0xEF,0xA0,0x66,0x26,0x4A,0x22,0xF4,0x25,0x76,
        0x2D,0x62,0xC0,0x94,0x02,0x4A,0xAC,0x7E,0x3D,0xF3,0x87,0xDA,0xB7,0xB7,0x39,0xDF,
        /* S (32 bytes) */
        0x43,0xEA,0x89,0xF6,0x75,0xF0,0x32,0x9D,0x3F,0xDF,0x95,0xCA,0xDB,0x45,0xFB,0x97,
        0x4E,0xAD,0xE7,0x0A,0x3C,0x19,0x98,0x6A,0xE3,0xC7,0x64,0x8F,0x6A,0xFA,0xE3,0x90
    };
    uECC_SHA512_HashContext hctx;
    uECC_SHA512_HashContext_init(&hctx);
    uECC_Curve curve = uECC_secp256r1();
    uint8_t computed_sig[96];
    uint8_t bad[96];
    uECC_RNG_Function old_rng;

    old_rng = uECC_get_rng();
    s_rng_k_be  = vec_k_be;
    s_rng_k_len = 32;
    s_rng_calls = 0;
    uECC_set_rng(vector_sign_rng);
    if (!uECC_ecsdsa_sign_standard(vec_priv, vec_msg, sizeof(vec_msg),
                                   &hctx.uECC, computed_sig, curve)) {
        uECC_set_rng(old_rng);
        printf("FAIL: vec2 sha512 standard sign failed\n");
        return 0;
    }
    uECC_set_rng(old_rng);

    if (memcmp(computed_sig, sig, 96) != 0) {
        printf("FAIL: vec2 sha512 standard sign: R or S mismatch\n");
        print_hex("  got R", computed_sig,      64);
        print_hex("  exp R", sig,               64);
        print_hex("  got S", computed_sig + 64, 32);
        print_hex("  exp S", sig + 64,          32);
        return 0;
    }

    if (!uECC_ecsdsa_verify_standard(vec_pub, vec_msg, sizeof(vec_msg),
                                     &hctx.uECC, sig, curve)) {
        printf("FAIL: vec2 sha512 standard verify: valid sig rejected\n");
        return 0;
    }
    memcpy(bad, sig, 96); bad[0] ^= 0x01;
    if (uECC_ecsdsa_verify_standard(vec_pub, vec_msg, sizeof(vec_msg),
                                    &hctx.uECC, bad, curve)) {
        printf("FAIL: vec2 sha512 standard verify: modified R accepted\n");
        return 0;
    }
    memcpy(bad, sig, 96); bad[64] ^= 0x01;
    if (uECC_ecsdsa_verify_standard(vec_pub, vec_msg, sizeof(vec_msg),
                                    &hctx.uECC, bad, curve)) {
        printf("FAIL: vec2 sha512 standard verify: modified S accepted\n");
        return 0;
    }
    {
        uint8_t bad_msg[] = { 0x61, 0x62, 0x64 };
        if (uECC_ecsdsa_verify_standard(vec_pub, bad_msg, sizeof(bad_msg),
                                        &hctx.uECC, sig, curve)) {
            printf("FAIL: vec2 sha512 standard verify: modified message accepted\n");
            return 0;
        }
    }
    printf("PASS: EC-SDSA standard secp256r1/SHA-512 vector (sign+verify)\n");
    return 1;
}
#endif

#endif /* uECC_SUPPORTS_SHA512 */

/* =========================================================================
 * Random round-trip tests
 * sig buffer: 96 bytes covers R(64)+S(32) for SHA-512+secp256r1 (largest case)
 * ========================================================================= */

#if uECC_SUPPORTS_ECSDSA_OPTIMIZED
static int test_ecsdsa_optimized_sign_verify(uECC_Curve curve, int verbose) {
    uint8_t private_key[32] = {0};
    uint8_t public_key[64]  = {0};
    uint8_t sig[96] = {0};
    uint8_t msg[32];
    uECC_SHA256_HashContext hctx;
    uECC_SHA256_HashContext_init(&hctx);
    int num_bytes = uECC_curve_private_key_size(curve);

    if (!uECC_make_key(public_key, private_key, curve)) {
        printf("FAIL: uECC_make_key\n"); return 0;
    }
    memcpy(msg, public_key, sizeof(msg));

    if (!uECC_ecsdsa_sign_optimized(private_key, msg, sizeof(msg),
                                    &hctx.uECC, sig, curve)) {
        printf("FAIL: uECC_ecsdsa_sign_optimized\n"); return 0;
    }
    if (!uECC_ecsdsa_verify_optimized(public_key, msg, sizeof(msg),
                                      &hctx.uECC, sig, curve)) {
        printf("FAIL: uECC_ecsdsa_verify_optimized (valid sig rejected)\n"); return 0;
    }

    msg[0] ^= 0x01;
    if (uECC_ecsdsa_verify_optimized(public_key, msg, sizeof(msg),
                                     &hctx.uECC, sig, curve)) {
        printf("FAIL: optimized verify accepted modified message\n"); return 0;
    }
    msg[0] ^= 0x01;

    sig[0] ^= 0x01;
    if (uECC_ecsdsa_verify_optimized(public_key, msg, sizeof(msg),
                                     &hctx.uECC, sig, curve)) {
        printf("FAIL: optimized verify accepted modified R\n"); return 0;
    }
    sig[0] ^= 0x01;

    sig[32] ^= 0x01;
    if (uECC_ecsdsa_verify_optimized(public_key, msg, sizeof(msg),
                                     &hctx.uECC, sig, curve)) {
        printf("FAIL: optimized verify accepted modified S\n"); return 0;
    }
    sig[32] ^= 0x01;

    if (verbose) {
        print_hex("  priv", private_key, num_bytes);
        print_hex("  pub ", public_key,  2 * num_bytes);
        print_hex("  R   ", sig,         32);
        print_hex("  S   ", sig + 32,    num_bytes);
    }
    return 1;
}
#endif /* uECC_SUPPORTS_ECSDSA_OPTIMIZED */

#if uECC_SUPPORTS_ECSDSA_STANDARD
static int test_ecsdsa_standard_sign_verify(uECC_Curve curve, int verbose) {
    uint8_t private_key[32] = {0};
    uint8_t public_key[64]  = {0};
    uint8_t sig[96] = {0};
    uint8_t msg[32];
    uECC_SHA256_HashContext hctx;
    uECC_SHA256_HashContext_init(&hctx);
    int num_bytes = uECC_curve_private_key_size(curve);

    if (!uECC_make_key(public_key, private_key, curve)) {
        printf("FAIL: uECC_make_key\n"); return 0;
    }
    memcpy(msg, public_key, sizeof(msg));

    if (!uECC_ecsdsa_sign_standard(private_key, msg, sizeof(msg),
                                   &hctx.uECC, sig, curve)) {
        printf("FAIL: uECC_ecsdsa_sign_standard\n"); return 0;
    }
    if (!uECC_ecsdsa_verify_standard(public_key, msg, sizeof(msg),
                                     &hctx.uECC, sig, curve)) {
        printf("FAIL: uECC_ecsdsa_verify_standard (valid sig rejected)\n"); return 0;
    }

    msg[0] ^= 0x01;
    if (uECC_ecsdsa_verify_standard(public_key, msg, sizeof(msg),
                                    &hctx.uECC, sig, curve)) {
        printf("FAIL: standard verify accepted modified message\n"); return 0;
    }
    msg[0] ^= 0x01;

    sig[0] ^= 0x01;
    if (uECC_ecsdsa_verify_standard(public_key, msg, sizeof(msg),
                                    &hctx.uECC, sig, curve)) {
        printf("FAIL: standard verify accepted modified R\n"); return 0;
    }
    sig[0] ^= 0x01;

    sig[32] ^= 0x01;
    if (uECC_ecsdsa_verify_standard(public_key, msg, sizeof(msg),
                                    &hctx.uECC, sig, curve)) {
        printf("FAIL: standard verify accepted modified S\n"); return 0;
    }
    sig[32] ^= 0x01;

    if (verbose) {
        print_hex("  priv", private_key, num_bytes);
        print_hex("  pub ", public_key,  2 * num_bytes);
        print_hex("  R   ", sig,         32);
        print_hex("  S   ", sig + 32,    num_bytes);
    }
    return 1;
}
#endif /* uECC_SUPPORTS_ECSDSA_STANDARD */

#if uECC_SUPPORTS_SHA512

#if uECC_SUPPORTS_ECSDSA_OPTIMIZED
static int test_ecsdsa_sha512_optimized_sign_verify(uECC_Curve curve) {
    uint8_t private_key[32] = {0};
    uint8_t public_key[64]  = {0};
    uint8_t sig[96] = {0}; /* R(64) + S(up to 32) */
    uint8_t msg[32];
    uECC_SHA512_HashContext hctx;
    uECC_SHA512_HashContext_init(&hctx);

    if (!uECC_make_key(public_key, private_key, curve)) {
        printf("FAIL: uECC_make_key\n"); return 0;
    }
    memcpy(msg, public_key, sizeof(msg));

    if (!uECC_ecsdsa_sign_optimized(private_key, msg, sizeof(msg),
                                    &hctx.uECC, sig, curve)) {
        printf("FAIL: uECC_ecsdsa_sign_optimized (sha512)\n"); return 0;
    }
    if (!uECC_ecsdsa_verify_optimized(public_key, msg, sizeof(msg),
                                      &hctx.uECC, sig, curve)) {
        printf("FAIL: uECC_ecsdsa_verify_optimized sha512 (valid sig rejected)\n"); return 0;
    }

    msg[0] ^= 0x01;
    if (uECC_ecsdsa_verify_optimized(public_key, msg, sizeof(msg),
                                     &hctx.uECC, sig, curve)) {
        printf("FAIL: sha512 optimized verify accepted modified message\n"); return 0;
    }
    msg[0] ^= 0x01;

    sig[0] ^= 0x01;
    if (uECC_ecsdsa_verify_optimized(public_key, msg, sizeof(msg),
                                     &hctx.uECC, sig, curve)) {
        printf("FAIL: sha512 optimized verify accepted modified R\n"); return 0;
    }
    sig[0] ^= 0x01;

    sig[64] ^= 0x01;
    if (uECC_ecsdsa_verify_optimized(public_key, msg, sizeof(msg),
                                     &hctx.uECC, sig, curve)) {
        printf("FAIL: sha512 optimized verify accepted modified S\n"); return 0;
    }
    sig[64] ^= 0x01;

    return 1;
}
#endif

#if uECC_SUPPORTS_ECSDSA_STANDARD
static int test_ecsdsa_sha512_standard_sign_verify(uECC_Curve curve) {
    uint8_t private_key[32] = {0};
    uint8_t public_key[64]  = {0};
    uint8_t sig[96] = {0}; /* R(64) + S(up to 32) */
    uint8_t msg[32];
    uECC_SHA512_HashContext hctx;
    uECC_SHA512_HashContext_init(&hctx);

    if (!uECC_make_key(public_key, private_key, curve)) {
        printf("FAIL: uECC_make_key\n"); return 0;
    }
    memcpy(msg, public_key, sizeof(msg));

    if (!uECC_ecsdsa_sign_standard(private_key, msg, sizeof(msg),
                                   &hctx.uECC, sig, curve)) {
        printf("FAIL: uECC_ecsdsa_sign_standard (sha512)\n"); return 0;
    }
    if (!uECC_ecsdsa_verify_standard(public_key, msg, sizeof(msg),
                                     &hctx.uECC, sig, curve)) {
        printf("FAIL: uECC_ecsdsa_verify_standard sha512 (valid sig rejected)\n"); return 0;
    }

    msg[0] ^= 0x01;
    if (uECC_ecsdsa_verify_standard(public_key, msg, sizeof(msg),
                                    &hctx.uECC, sig, curve)) {
        printf("FAIL: sha512 standard verify accepted modified message\n"); return 0;
    }
    msg[0] ^= 0x01;

    sig[0] ^= 0x01;
    if (uECC_ecsdsa_verify_standard(public_key, msg, sizeof(msg),
                                    &hctx.uECC, sig, curve)) {
        printf("FAIL: sha512 standard verify accepted modified R\n"); return 0;
    }
    sig[0] ^= 0x01;

    sig[64] ^= 0x01;
    if (uECC_ecsdsa_verify_standard(public_key, msg, sizeof(msg),
                                    &hctx.uECC, sig, curve)) {
        printf("FAIL: sha512 standard verify accepted modified S\n"); return 0;
    }
    sig[64] ^= 0x01;

    return 1;
}
#endif

#endif /* uECC_SUPPORTS_SHA512 */

/* =========================================================================
 * main
 * ========================================================================= */

int main(void) {
    int c, i;
    const struct uECC_Curve_t *curves[5];
    const char *names[5];
    int num_curves = 0;

#if uECC_SUPPORTS_secp160r1
    curves[num_curves] = uECC_secp160r1(); names[num_curves++] = "secp160r1";
#endif
#if uECC_SUPPORTS_secp192r1
    curves[num_curves] = uECC_secp192r1(); names[num_curves++] = "secp192r1";
#endif
#if uECC_SUPPORTS_secp224r1
    curves[num_curves] = uECC_secp224r1(); names[num_curves++] = "secp224r1";
#endif
#if uECC_SUPPORTS_secp256r1
    curves[num_curves] = uECC_secp256r1(); names[num_curves++] = "secp256r1";
#endif
#if uECC_SUPPORTS_secp256k1
    curves[num_curves] = uECC_secp256k1(); names[num_curves++] = "secp256k1";
#endif

    printf("Testing EC-SDSA\n");

    /* ---- SHA-256 vector tests ---- */
#if uECC_SUPPORTS_ECSDSA_OPTIMIZED && uECC_SUPPORTS_secp256r1
    printf("\nSHA-256 optimized variant vector test:\n");
    if (!test_ecsdsa_vector_sha256_optimized()) return 1;
#endif
#if uECC_SUPPORTS_ECSDSA_STANDARD && uECC_SUPPORTS_secp256r1
    printf("\nSHA-256 standard variant vector test:\n");
    if (!test_ecsdsa_vector_sha256_standard()) return 1;
#endif

    /* ---- SHA-512 vector tests ---- */
#if uECC_SUPPORTS_SHA512
#if uECC_SUPPORTS_ECSDSA_OPTIMIZED && uECC_SUPPORTS_secp256r1
    printf("\nSHA-512 optimized variant vector test:\n");
    if (!test_ecsdsa_vector_sha512_optimized()) return 1;
#endif
#if uECC_SUPPORTS_ECSDSA_STANDARD && uECC_SUPPORTS_secp256r1
    printf("\nSHA-512 standard variant vector test:\n");
    if (!test_ecsdsa_vector_sha512_standard()) return 1;
#endif
#endif /* uECC_SUPPORTS_SHA512 */

    /* ---- SHA-256 random round-trip tests ---- */
#if uECC_SUPPORTS_ECSDSA_OPTIMIZED
    printf("\nEC-SDSA SHA-256 optimized random round-trip tests (64 per curve):\n");
    for (c = 0; c < num_curves; ++c) {
        printf("  %s:", names[c]); fflush(stdout);
        for (i = 0; i < 64; ++i) {
            printf("."); fflush(stdout);
            if (!test_ecsdsa_optimized_sign_verify(curves[c], 0)) return 1;
        }
        printf(" PASS\n");
    }
#endif
#if uECC_SUPPORTS_ECSDSA_STANDARD
    printf("\nEC-SDSA SHA-256 standard random round-trip tests (64 per curve):\n");
    for (c = 0; c < num_curves; ++c) {
        printf("  %s:", names[c]); fflush(stdout);
        for (i = 0; i < 64; ++i) {
            printf("."); fflush(stdout);
            if (!test_ecsdsa_standard_sign_verify(curves[c], 0)) return 1;
        }
        printf(" PASS\n");
    }
#endif

    /* ---- SHA-512 random round-trip tests ---- */
#if uECC_SUPPORTS_SHA512
#if uECC_SUPPORTS_ECSDSA_OPTIMIZED
    printf("\nEC-SDSA SHA-512 optimized random round-trip tests (64 per curve):\n");
    for (c = 0; c < num_curves; ++c) {
        printf("  %s:", names[c]); fflush(stdout);
        for (i = 0; i < 64; ++i) {
            printf("."); fflush(stdout);
            if (!test_ecsdsa_sha512_optimized_sign_verify(curves[c])) return 1;
        }
        printf(" PASS\n");
    }
#endif
#if uECC_SUPPORTS_ECSDSA_STANDARD
    printf("\nEC-SDSA SHA-512 standard random round-trip tests (64 per curve):\n");
    for (c = 0; c < num_curves; ++c) {
        printf("  %s:", names[c]); fflush(stdout);
        for (i = 0; i < 64; ++i) {
            printf("."); fflush(stdout);
            if (!test_ecsdsa_sha512_standard_sign_verify(curves[c])) return 1;
        }
        printf(" PASS\n");
    }
#endif
#endif /* uECC_SUPPORTS_SHA512 */

    return 0;
}

#endif /* uECC_SUPPORTS_ECSDSA_OPTIMIZED || uECC_SUPPORTS_ECSDSA_STANDARD */
