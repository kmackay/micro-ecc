/* Copyright 2014, Kenneth MacKay. Licensed under the BSD 2-clause license. */

/*
 * uECC_hash.c — built-in SHA-256 and SHA-512 hash adapters.
 *
 * Both implementations are DISABLED BY DEFAULT.  See uECC_hash.h for how to
 * opt in.  If you have already included a hash library in your project you
 * should NOT compile this file; implement the three uECC_HashContext callbacks
 * (init_hash / update_hash / finish_hash) against your existing library
 * instead.
 */

#include "uECC_hash.h"

/* =========================================================================
 * SHA-256
 * ========================================================================= */

#if uECC_SUPPORTS_SHA256

#define ROR32(x, n) (((x) >> (n)) | ((x) << (32 - (n))))

#define SHA256_S0(x) (ROR32(x, 2)  ^ ROR32(x, 13) ^ ROR32(x, 22))
#define SHA256_S1(x) (ROR32(x, 6)  ^ ROR32(x, 11) ^ ROR32(x, 25))
#define SHA256_s0(x) (ROR32(x, 7)  ^ ROR32(x, 18) ^ ((x) >>  3))
#define SHA256_s1(x) (ROR32(x, 17) ^ ROR32(x, 19) ^ ((x) >> 10))
#define SHA256_CH(x, y, z)  (((x) & (y)) ^ (~(x) & (z)))
#define SHA256_MAJ(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))

static const uint32_t sha256_K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

static void sha256_compress(uECC_SHA256_HashContext *ctx, const uint8_t *block) {
    uint32_t W[64];
    uint32_t a, b, c, d, e, f, g, h, T1, T2;
    int i;

    for (i = 0; i < 16; ++i) {
        W[i] = ((uint32_t)block[i*4    ] << 24)
             | ((uint32_t)block[i*4 + 1] << 16)
             | ((uint32_t)block[i*4 + 2] <<  8)
             |  (uint32_t)block[i*4 + 3];
    }
    for (i = 16; i < 64; ++i) {
        W[i] = SHA256_s1(W[i-2]) + W[i-7] + SHA256_s0(W[i-15]) + W[i-16];
    }

    a = ctx->state[0]; b = ctx->state[1]; c = ctx->state[2]; d = ctx->state[3];
    e = ctx->state[4]; f = ctx->state[5]; g = ctx->state[6]; h = ctx->state[7];

    for (i = 0; i < 64; ++i) {
        T1 = h + SHA256_S1(e) + SHA256_CH(e, f, g) + sha256_K[i] + W[i];
        T2 = SHA256_S0(a) + SHA256_MAJ(a, b, c);
        h = g; g = f; f = e; e = d + T1;
        d = c; c = b; b = a; a = T1 + T2;
    }

    ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c; ctx->state[3] += d;
    ctx->state[4] += e; ctx->state[5] += f; ctx->state[6] += g; ctx->state[7] += h;
}

static void sha256_init_hash(const uECC_HashContext *base) {
    uECC_SHA256_HashContext *ctx = (uECC_SHA256_HashContext *)base;
    ctx->state[0] = 0x6a09e667; ctx->state[1] = 0xbb67ae85;
    ctx->state[2] = 0x3c6ef372; ctx->state[3] = 0xa54ff53a;
    ctx->state[4] = 0x510e527f; ctx->state[5] = 0x9b05688c;
    ctx->state[6] = 0x1f83d9ab; ctx->state[7] = 0x5be0cd19;
    ctx->bits = 0;
    ctx->buf_len = 0;
}

static void sha256_update_hash(const uECC_HashContext *base,
                               const uint8_t *data, unsigned len) {
    uECC_SHA256_HashContext *ctx = (uECC_SHA256_HashContext *)base;
    unsigned i;
    ctx->bits += (uint64_t)len * 8;
    for (i = 0; i < len; ++i) {
        ctx->buf[ctx->buf_len++] = data[i];
        if (ctx->buf_len == 64) {
            sha256_compress(ctx, ctx->buf);
            ctx->buf_len = 0;
        }
    }
}

static void sha256_finish_hash(const uECC_HashContext *base, uint8_t *out) {
    uECC_SHA256_HashContext *ctx = (uECC_SHA256_HashContext *)base;
    uint64_t bits = ctx->bits;
    int i;

    ctx->buf[ctx->buf_len++] = 0x80;
    if (ctx->buf_len > 56) {
        while (ctx->buf_len < 64) ctx->buf[ctx->buf_len++] = 0;
        sha256_compress(ctx, ctx->buf);
        ctx->buf_len = 0;
    }
    while (ctx->buf_len < 56) ctx->buf[ctx->buf_len++] = 0;
    for (i = 7; i >= 0; --i) {
        ctx->buf[56 + (7 - i)] = (uint8_t)(bits >> (i * 8));
    }
    sha256_compress(ctx, ctx->buf);

    for (i = 0; i < 8; ++i) {
        out[i*4    ] = (uint8_t)(ctx->state[i] >> 24);
        out[i*4 + 1] = (uint8_t)(ctx->state[i] >> 16);
        out[i*4 + 2] = (uint8_t)(ctx->state[i] >>  8);
        out[i*4 + 3] = (uint8_t)(ctx->state[i]);
    }
}

void uECC_SHA256_HashContext_init(uECC_SHA256_HashContext *ctx) {
    ctx->uECC.init_hash   = sha256_init_hash;
    ctx->uECC.update_hash = sha256_update_hash;
    ctx->uECC.finish_hash = sha256_finish_hash;
    ctx->uECC.block_size  = 64;
    ctx->uECC.result_size = 32;
    ctx->uECC.tmp         = ctx->tmp;
}

#undef ROR32
#undef SHA256_S0
#undef SHA256_S1
#undef SHA256_s0
#undef SHA256_s1
#undef SHA256_CH
#undef SHA256_MAJ

#endif /* uECC_SUPPORTS_SHA256 */

/* =========================================================================
 * SHA-512
 * ========================================================================= */

#if uECC_SUPPORTS_SHA512

#define ROR64(x, n) (((x) >> (n)) | ((x) << (64 - (n))))

#define SHA512_S0(x) (ROR64(x, 28) ^ ROR64(x, 34) ^ ROR64(x, 39))
#define SHA512_S1(x) (ROR64(x, 14) ^ ROR64(x, 18) ^ ROR64(x, 41))
#define SHA512_s0(x) (ROR64(x,  1) ^ ROR64(x,  8) ^ ((x) >>  7))
#define SHA512_s1(x) (ROR64(x, 19) ^ ROR64(x, 61) ^ ((x) >>  6))
#define SHA512_CH(x, y, z)  (((x) & (y)) ^ (~(x) & (z)))
#define SHA512_MAJ(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))

static const uint64_t sha512_K[80] = {
    0x428a2f98d728ae22ULL, 0x7137449123ef65cdULL,
    0xb5c0fbcfec4d3b2fULL, 0xe9b5dba58189dbbcULL,
    0x3956c25bf348b538ULL, 0x59f111f1b605d019ULL,
    0x923f82a4af194f9bULL, 0xab1c5ed5da6d8118ULL,
    0xd807aa98a3030242ULL, 0x12835b0145706fbeULL,
    0x243185be4ee4b28cULL, 0x550c7dc3d5ffb4e2ULL,
    0x72be5d74f27b896fULL, 0x80deb1fe3b1696b1ULL,
    0x9bdc06a725c71235ULL, 0xc19bf174cf692694ULL,
    0xe49b69c19ef14ad2ULL, 0xefbe4786384f25e3ULL,
    0x0fc19dc68b8cd5b5ULL, 0x240ca1cc77ac9c65ULL,
    0x2de92c6f592b0275ULL, 0x4a7484aa6ea6e483ULL,
    0x5cb0a9dcbd41fbd4ULL, 0x76f988da831153b5ULL,
    0x983e5152ee66dfabULL, 0xa831c66d2db43210ULL,
    0xb00327c898fb213fULL, 0xbf597fc7beef0ee4ULL,
    0xc6e00bf33da88fc2ULL, 0xd5a79147930aa725ULL,
    0x06ca6351e003826fULL, 0x142929670a0e6e70ULL,
    0x27b70a8546d22ffcULL, 0x2e1b21385c26c926ULL,
    0x4d2c6dfc5ac42aedULL, 0x53380d139d95b3dfULL,
    0x650a73548baf63deULL, 0x766a0abb3c77b2a8ULL,
    0x81c2c92e47edaee6ULL, 0x92722c851482353bULL,
    0xa2bfe8a14cf10364ULL, 0xa81a664bbc423001ULL,
    0xc24b8b70d0f89791ULL, 0xc76c51a30654be30ULL,
    0xd192e819d6ef5218ULL, 0xd69906245565a910ULL,
    0xf40e35855771202aULL, 0x106aa07032bbd1b8ULL,
    0x19a4c116b8d2d0c8ULL, 0x1e376c085141ab53ULL,
    0x2748774cdf8eeb99ULL, 0x34b0bcb5e19b48a8ULL,
    0x391c0cb3c5c95a63ULL, 0x4ed8aa4ae3418acbULL,
    0x5b9cca4f7763e373ULL, 0x682e6ff3d6b2b8a3ULL,
    0x748f82ee5defb2fcULL, 0x78a5636f43172f60ULL,
    0x84c87814a1f0ab72ULL, 0x8cc702081a6439ecULL,
    0x90befffa23631e28ULL, 0xa4506cebde82bde9ULL,
    0xbef9a3f7b2c67915ULL, 0xc67178f2e372532bULL,
    0xca273eceea26619cULL, 0xd186b8c721c0c207ULL,
    0xeada7dd6cde0eb1eULL, 0xf57d4f7fee6ed178ULL,
    0x06f067aa72176fbaULL, 0x0a637dc5a2c898a6ULL,
    0x113f9804bef90daeULL, 0x1b710b35131c471bULL,
    0x28db77f523047d84ULL, 0x32caab7b40c72493ULL,
    0x3c9ebe0a15c9bebcULL, 0x431d67c49c100d4cULL,
    0x4cc5d4becb3e42b6ULL, 0x597f299cfc657e2aULL,
    0x5fcb6fab3ad6faecULL, 0x6c44198c4a475817ULL
};

static void sha512_compress(uECC_SHA512_HashContext *ctx, const uint8_t *block) {
    uint64_t W[80];
    uint64_t a, b, c, d, e, f, g, h, T1, T2;
    int i;

    for (i = 0; i < 16; ++i) {
        W[i] = ((uint64_t)block[i*8    ] << 56)
             | ((uint64_t)block[i*8 + 1] << 48)
             | ((uint64_t)block[i*8 + 2] << 40)
             | ((uint64_t)block[i*8 + 3] << 32)
             | ((uint64_t)block[i*8 + 4] << 24)
             | ((uint64_t)block[i*8 + 5] << 16)
             | ((uint64_t)block[i*8 + 6] <<  8)
             |  (uint64_t)block[i*8 + 7];
    }
    for (i = 16; i < 80; ++i) {
        W[i] = SHA512_s1(W[i-2]) + W[i-7] + SHA512_s0(W[i-15]) + W[i-16];
    }

    a = ctx->state[0]; b = ctx->state[1]; c = ctx->state[2]; d = ctx->state[3];
    e = ctx->state[4]; f = ctx->state[5]; g = ctx->state[6]; h = ctx->state[7];

    for (i = 0; i < 80; ++i) {
        T1 = h + SHA512_S1(e) + SHA512_CH(e, f, g) + sha512_K[i] + W[i];
        T2 = SHA512_S0(a) + SHA512_MAJ(a, b, c);
        h = g; g = f; f = e; e = d + T1;
        d = c; c = b; b = a; a = T1 + T2;
    }

    ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c; ctx->state[3] += d;
    ctx->state[4] += e; ctx->state[5] += f; ctx->state[6] += g; ctx->state[7] += h;
}

static void sha512_init_hash(const uECC_HashContext *base) {
    uECC_SHA512_HashContext *ctx = (uECC_SHA512_HashContext *)base;
    ctx->state[0] = 0x6a09e667f3bcc908ULL; ctx->state[1] = 0xbb67ae8584caa73bULL;
    ctx->state[2] = 0x3c6ef372fe94f82bULL; ctx->state[3] = 0xa54ff53a5f1d36f1ULL;
    ctx->state[4] = 0x510e527fade682d1ULL; ctx->state[5] = 0x9b05688c2b3e6c1fULL;
    ctx->state[6] = 0x1f83d9abfb41bd6bULL; ctx->state[7] = 0x5be0cd19137e2179ULL;
    ctx->bits = 0;
    ctx->buf_len = 0;
}

static void sha512_update_hash(const uECC_HashContext *base,
                               const uint8_t *data, unsigned len) {
    uECC_SHA512_HashContext *ctx = (uECC_SHA512_HashContext *)base;
    unsigned i;
    ctx->bits += (uint64_t)len * 8;
    for (i = 0; i < len; ++i) {
        ctx->buf[ctx->buf_len++] = data[i];
        if (ctx->buf_len == 128) {
            sha512_compress(ctx, ctx->buf);
            ctx->buf_len = 0;
        }
    }
}

static void sha512_finish_hash(const uECC_HashContext *base, uint8_t *out) {
    uECC_SHA512_HashContext *ctx = (uECC_SHA512_HashContext *)base;
    uint64_t bits = ctx->bits;
    int i;

    ctx->buf[ctx->buf_len++] = 0x80;
    if (ctx->buf_len > 112) {
        while (ctx->buf_len < 128) ctx->buf[ctx->buf_len++] = 0;
        sha512_compress(ctx, ctx->buf);
        ctx->buf_len = 0;
    }
    while (ctx->buf_len < 112) ctx->buf[ctx->buf_len++] = 0;
    /* 128-bit length field: high 64 bits are always 0 (supports up to 2^64-1 bits) */
    for (i = 0; i < 8; ++i) ctx->buf[112 + i] = 0;
    for (i = 7; i >= 0; --i) {
        ctx->buf[120 + (7 - i)] = (uint8_t)(bits >> (i * 8));
    }
    sha512_compress(ctx, ctx->buf);

    for (i = 0; i < 8; ++i) {
        out[i*8    ] = (uint8_t)(ctx->state[i] >> 56);
        out[i*8 + 1] = (uint8_t)(ctx->state[i] >> 48);
        out[i*8 + 2] = (uint8_t)(ctx->state[i] >> 40);
        out[i*8 + 3] = (uint8_t)(ctx->state[i] >> 32);
        out[i*8 + 4] = (uint8_t)(ctx->state[i] >> 24);
        out[i*8 + 5] = (uint8_t)(ctx->state[i] >> 16);
        out[i*8 + 6] = (uint8_t)(ctx->state[i] >>  8);
        out[i*8 + 7] = (uint8_t)(ctx->state[i]);
    }
}

void uECC_SHA512_HashContext_init(uECC_SHA512_HashContext *ctx) {
    ctx->uECC.init_hash   = sha512_init_hash;
    ctx->uECC.update_hash = sha512_update_hash;
    ctx->uECC.finish_hash = sha512_finish_hash;
    ctx->uECC.block_size  = 128;
    ctx->uECC.result_size = 64;
    ctx->uECC.tmp         = ctx->tmp;
}

#undef ROR64
#undef SHA512_S0
#undef SHA512_S1
#undef SHA512_s0
#undef SHA512_s1
#undef SHA512_CH
#undef SHA512_MAJ

#endif /* uECC_SUPPORTS_SHA512 */
