/* Copyright 2014, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#ifndef uECC_HASH_H_
#define uECC_HASH_H_

/*
 * uECC_hash.h — optional built-in SHA-256 and SHA-512 hash context adapters
 *               for use with uECC_sign_deterministic() and uECC_ecsdsa_*().
 *
 * DISABLED BY DEFAULT.  Many embedded projects already carry a hash library
 * (mbed TLS, wolfCrypt, RIOT crypto, etc.) and should not pay the code-size
 * cost of a second implementation.  Enable only what you actually need:
 *
 *   #define uECC_SUPPORTS_SHA256 1   // before including this header
 *   #define uECC_SUPPORTS_SHA512 1   // before including this header
 *
 * Both macros default to 0 so that simply including uECC_hash.h has no effect
 * unless you opt in.
 */

#include "uECC.h"
#include <stdint.h>

/* ---- opt-in gates (default: disabled) ----------------------------------- */

#ifndef uECC_SUPPORTS_SHA256
    #define uECC_SUPPORTS_SHA256 0   /* set to 1 to enable the built-in SHA-256 */
#endif

#ifndef uECC_SUPPORTS_SHA512
    #define uECC_SUPPORTS_SHA512 0   /* set to 1 to enable the built-in SHA-512 */
#endif

/* ---- SHA-256 ------------------------------------------------------------- */

#if uECC_SUPPORTS_SHA256

/*
 * Self-contained SHA-256 uECC_HashContext.
 *
 * The uECC_HashContext pointer (first member) is safe to cast to/from a
 * pointer to this struct.  The embedded tmp[] array satisfies the
 * uECC_HashContext.tmp requirement of >= (2*result_size + block_size) bytes
 * without any external allocation.
 *
 * Usage:
 *   uECC_SHA256_HashContext ctx;
 *   uECC_SHA256_HashContext_init(&ctx);
 *   uECC_ecsdsa_sign_optimized(priv, msg, len, &ctx.uECC, sig, curve);
 */
typedef struct {
    uECC_HashContext uECC;       /* must be first — cast-compatible */
    uint32_t state[8];
    uint8_t  buf[64];
    uint64_t bits;
    uint32_t buf_len;
    uint8_t  tmp[2 * 32 + 64];  /* scratch for uECC_HashContext.tmp */
} uECC_SHA256_HashContext;

/*
 * Initialise *ctx and wire all function pointers.
 * Must be called before passing &ctx.uECC to any uECC function.
 * Safe to call multiple times (re-initialises).
 */
void uECC_SHA256_HashContext_init(uECC_SHA256_HashContext *ctx);

#endif /* uECC_SUPPORTS_SHA256 */

/* ---- SHA-512 ------------------------------------------------------------- */

#if uECC_SUPPORTS_SHA512

/*
 * Self-contained SHA-512 uECC_HashContext.
 *
 * With SHA-512 the ECSDSA R value is 64 bytes; for 256-bit curves only the
 * leading 256 bits are used as the scalar r (bits2int truncates to curve
 * order width).  The signature format is R(64) || S(num_bytes).
 *
 * Usage:
 *   uECC_SHA512_HashContext ctx;
 *   uECC_SHA512_HashContext_init(&ctx);
 *   uECC_ecsdsa_sign_optimized(priv, msg, len, &ctx.uECC, sig, curve);
 *   // sig must be at least 64 + curve->num_bytes bytes
 */
typedef struct {
    uECC_HashContext uECC;        /* must be first — cast-compatible */
    uint64_t state[8];
    uint8_t  buf[128];
    uint64_t bits;               /* bit count (supports messages up to 2^64-1 bits) */
    uint32_t buf_len;
    uint8_t  tmp[2 * 64 + 128]; /* scratch for uECC_HashContext.tmp */
} uECC_SHA512_HashContext;

/*
 * Initialise *ctx and wire all function pointers.
 * Must be called before passing &ctx.uECC to any uECC function.
 * Safe to call multiple times (re-initialises).
 */
void uECC_SHA512_HashContext_init(uECC_SHA512_HashContext *ctx);

#endif /* uECC_SUPPORTS_SHA512 */

#endif /* uECC_HASH_H_ */
