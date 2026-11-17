/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#ifndef _UECC_CURVE_SPECIFIC_H_
#define _UECC_CURVE_SPECIFIC_H_

#define num_bytes_secp160r1 20
#define num_bytes_secp192r1 24
#define num_bytes_secp224r1 28
#define num_bytes_secp256r1 32
#define num_bytes_secp256k1 32

#if (uECC_WORD_SIZE == 1)

#define num_words_secp160r1 20
#define num_words_secp192r1 24
#define num_words_secp224r1 28
#define num_words_secp256r1 32
#define num_words_secp256k1 32

#define BYTES_TO_WORDS_8(a, b, c, d, e, f, g, h) \
    0x##a, 0x##b, 0x##c, 0x##d, 0x##e, 0x##f, 0x##g, 0x##h
#define BYTES_TO_WORDS_4(a, b, c, d) 0x##a, 0x##b, 0x##c, 0x##d

#elif (uECC_WORD_SIZE == 4)

#define num_words_secp160r1 5
#define num_words_secp192r1 6
#define num_words_secp224r1 7
#define num_words_secp256r1 8
#define num_words_secp256k1 8

#define BYTES_TO_WORDS_8(a, b, c, d, e, f, g, h) 0x##d##c##b##a, 0x##h##g##f##e
#define BYTES_TO_WORDS_4(a, b, c, d) 0x##d##c##b##a

#elif (uECC_WORD_SIZE == 8)

#define num_words_secp160r1 3
#define num_words_secp192r1 3
#define num_words_secp224r1 4
#define num_words_secp256r1 4
#define num_words_secp256k1 4

#define BYTES_TO_WORDS_8(a, b, c, d, e, f, g, h) 0x##h##g##f##e##d##c##b##a##ull
#define BYTES_TO_WORDS_4(a, b, c, d) 0x##d##c##b##a##ull

#endif /* uECC_WORD_SIZE */

#if uECC_SUPPORTS_secp160r1 || uECC_SUPPORTS_secp192r1 || \
    uECC_SUPPORTS_secp224r1 || uECC_SUPPORTS_secp256r1
static void double_jacobian_default(uECC_word_t * X1,
                                    uECC_word_t * Y1,
                                    uECC_word_t * Z1,
                                    uECC_Curve curve) {
    /* t1 = X, t2 = Y, t3 = Z */
    uECC_word_t t4[uECC_MAX_WORDS];
    uECC_word_t t5[uECC_MAX_WORDS];
    wordcount_t num_words = curve->num_words;

    if (uECC_vli_isZero(Z1, num_words)) {
        return;
    }

    uECC_vli_modSquare_fast(t4, Y1, curve);   /* t4 = y1^2 */
    uECC_vli_modMult_fast(t5, X1, t4, curve); /* t5 = x1*y1^2 = A */
    uECC_vli_modSquare_fast(t4, t4, curve);   /* t4 = y1^4 */
    uECC_vli_modMult_fast(Y1, Y1, Z1, curve); /* t2 = y1*z1 = z3 */
    uECC_vli_modSquare_fast(Z1, Z1, curve);   /* t3 = z1^2 */

    uECC_vli_modAdd(X1, X1, Z1, curve->p, num_words); /* t1 = x1 + z1^2 */
    uECC_vli_modAdd(Z1, Z1, Z1, curve->p, num_words); /* t3 = 2*z1^2 */
    uECC_vli_modSub(Z1, X1, Z1, curve->p, num_words); /* t3 = x1 - z1^2 */
    uECC_vli_modMult_fast(X1, X1, Z1, curve);                /* t1 = x1^2 - z1^4 */

    uECC_vli_modAdd(Z1, X1, X1, curve->p, num_words); /* t3 = 2*(x1^2 - z1^4) */
    uECC_vli_modAdd(X1, X1, Z1, curve->p, num_words); /* t1 = 3*(x1^2 - z1^4) */
    if (uECC_vli_testBit(X1, 0)) {
        uECC_word_t l_carry = uECC_vli_add(X1, X1, curve->p, num_words);
        uECC_vli_rshift1(X1, num_words);
        X1[num_words - 1] |= l_carry << (uECC_WORD_BITS - 1);
    } else {
        uECC_vli_rshift1(X1, num_words);
    }
    /* t1 = 3/2*(x1^2 - z1^4) = B */

    uECC_vli_modSquare_fast(Z1, X1, curve);                  /* t3 = B^2 */
    uECC_vli_modSub(Z1, Z1, t5, curve->p, num_words); /* t3 = B^2 - A */
    uECC_vli_modSub(Z1, Z1, t5, curve->p, num_words); /* t3 = B^2 - 2A = x3 */
    uECC_vli_modSub(t5, t5, Z1, curve->p, num_words); /* t5 = A - x3 */
    uECC_vli_modMult_fast(X1, X1, t5, curve);                /* t1 = B * (A - x3) */
    uECC_vli_modSub(t4, X1, t4, curve->p, num_words); /* t4 = B * (A - x3) - y1^4 = y3 */

    uECC_vli_set(X1, Z1, num_words);
    uECC_vli_set(Z1, Y1, num_words);
    uECC_vli_set(Y1, t4, num_words);
}

/* Computes result = x^3 + ax + b. result must not overlap x. */
static void x_side_default(uECC_word_t *result, const uECC_word_t *x, uECC_Curve curve) {
    uECC_word_t _3[uECC_MAX_WORDS] = {3}; /* -a = 3 */
    wordcount_t num_words = curve->num_words;

    uECC_vli_modSquare_fast(result, x, curve);                             /* r = x^2 */
    uECC_vli_modSub(result, result, _3, curve->p, num_words);       /* r = x^2 - 3 */
    uECC_vli_modMult_fast(result, result, x, curve);                       /* r = x^3 - 3x */
    uECC_vli_modAdd(result, result, curve->b, curve->p, num_words); /* r = x^3 - 3x + b */
}
#endif /* uECC_SUPPORTS_secp... */

#if uECC_SUPPORT_COMPRESSED_POINT
#if uECC_SUPPORTS_secp160r1 || uECC_SUPPORTS_secp192r1 || \
    uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1
/* Compute a = sqrt(a) (mod curve_p). */
static void mod_sqrt_default(uECC_word_t *a, uECC_Curve curve) {
    bitcount_t i;
    uECC_word_t p1[uECC_MAX_WORDS] = {1};
    uECC_word_t l_result[uECC_MAX_WORDS] = {1};
    wordcount_t num_words = curve->num_words;

    /* When curve->p == 3 (mod 4), we can compute
       sqrt(a) = a^((curve->p + 1) / 4) (mod curve->p). */
    uECC_vli_add(p1, curve->p, p1, num_words); /* p1 = curve_p + 1 */
    for (i = uECC_vli_numBits(p1, num_words) - 1; i > 1; --i) {
        uECC_vli_modSquare_fast(l_result, l_result, curve);
        if (uECC_vli_testBit(p1, i)) {
            uECC_vli_modMult_fast(l_result, l_result, a, curve);
        }
    }
    uECC_vli_set(a, l_result, num_words);
}
#endif /* uECC_SUPPORTS_secp... */
#endif /* uECC_SUPPORT_COMPRESSED_POINT */

#if (uECC_OPTIMIZATION_LEVEL > 0) && (uECC_WORD_SIZE == 4)
/* 32-bit constant-time fast reductions. Each vli_mmod_fast first folds the high
   half of the product into the low half with 64-bit column accumulators, giving
   r + c * 2^N (N = 32 * num_words). With d = 2^N - p (small and sparse),
   fold(r, c) computes r + c * d and returns the new carry. One more fold leaves
   r + c2 * 2^N with c2 in {-1, 0, 1} and a value in (-p, 2p).

   Since the result fits in N bits, folding -1 adds p and folding +1 subtracts p.
   MMOD_FINISH folds s = (c2 | 1) (c2, or 1 if c2 == 0); if c2 was 0 and that
   fold did not carry (that is, r < p), it undoes it by folding -1. */
#define MMOD_FINISH(fold, r, c) do { \
    int32_t c2_ = fold((r), (c)); \
    int32_t k_ = fold((r), c2_ | 1); \
    int32_t t_ = c2_ | k_; \
    fold((r), (int32_t)(((uint32_t)(t_ | -t_) >> 31) - 1)); \
} while (0)
#endif /* (uECC_OPTIMIZATION_LEVEL > 0) && (uECC_WORD_SIZE == 4) */

#if (uECC_OPTIMIZATION_LEVEL > 0) && (uECC_WORD_SIZE == 1) && \
    ((uECC_SUPPORTS_secp160r1 && !asm_mmod_fast_secp160r1) || \
     (uECC_SUPPORTS_secp192r1 && !asm_mmod_fast_secp192r1) || \
     (uECC_SUPPORTS_secp224r1 && !asm_mmod_fast_secp224r1) || \
     (uECC_SUPPORTS_secp256r1 && !asm_mmod_fast_secp256r1) || \
     (uECC_SUPPORTS_secp256k1 && !asm_mmod_fast_secp256k1))
/* vli_mmod_columns8 and vli_mmod_fold_select8 are only used by the NIST
   (secp192r1 / secp224r1 / secp256r1) C reductions. */
#define uECC_MMOD8_NIST_C ((uECC_SUPPORTS_secp192r1 && !asm_mmod_fast_secp192r1) || \
                           (uECC_SUPPORTS_secp224r1 && !asm_mmod_fast_secp224r1) || \
                           (uECC_SUPPORTS_secp256r1 && !asm_mmod_fast_secp256r1))
/* 8-bit constant-time fast reductions. All shifts in the NIST reductions are
   whole 32-bit words, so the reduction can be computed one byte column at a
   time with a 16-bit accumulator: byte b of the result receives byte (b & 3)
   of each 32-bit source word listed for its word column (b >> 2). */

/* Computes result = low half of product plus/minus the high-half words listed
   in terms, one 32-bit word column at a time (with a 16-bit accumulator per byte).
   terms: for each 32-bit result word, the list of source words to add
   (0..7) or subtract (0x80 | word), terminated by 0xFF; words with
   coefficient 2 or 3 are listed 2 or 3 times. Returns the (small, signed)
   carry out of the top byte. */
#if uECC_MMOD8_NIST_C
static int16_t vli_mmod_columns8(uint8_t *result,
                                 const uint8_t *product,
                                 const uint8_t *terms,
                                 wordcount_t num_bytes) {
    int16_t carry = 0;
    wordcount_t w;
    for (w = 0; w < num_bytes; w += 4) {
        int16_t a0 = product[w] + carry;
        int16_t a1 = product[w + 1];
        int16_t a2 = product[w + 2];
        int16_t a3 = product[w + 3];
        uint8_t e;
        while ((e = *terms++) != 0xFF) {
            const uint8_t *src = product + num_bytes + 4 * (e & 0x7F);
            if (e & 0x80) {
                a0 -= src[0]; a1 -= src[1]; a2 -= src[2]; a3 -= src[3];
            } else {
                a0 += src[0]; a1 += src[1]; a2 += src[2]; a3 += src[3];
            }
        }
        result[w] = (uint8_t)a0;
        a1 += a0 >> 8;
        result[w + 1] = (uint8_t)a1;
        a2 += a1 >> 8;
        result[w + 2] = (uint8_t)a2;
        a3 += a2 >> 8;
        result[w + 3] = (uint8_t)a3;
        carry = a3 >> 8;
    }
    return carry;
}
#endif /* uECC_MMOD8_NIST_C */

/* Adds p (c2 < 0), subtracts p (c2 > 0, or c2 == 0 and r >= p) or adds 0.
   r + c2 * 2^N is in (-p, 2p), c2 in {-1, 0, 1}; cmp is the borrow of r - p
   (0 or -1), computed by the caller during its last pass over r. */
static void vli_mmod_select8(uint8_t *r, int8_t c2, int8_t cmp, const uint8_t *p,
                             wordcount_t num_bytes) {
    uint16_t sum;
    uint8_t ge, neg, pos, m, s;
    wordcount_t b;
    ge = ~(uint8_t)cmp;                      /* 0xFF if r >= p */
    neg = 0 - ((uint8_t)c2 >> 7);            /* 0xFF if c2 < 0 */
    pos = 0 - ((uint8_t)(0 - (uint8_t)c2) >> 7); /* 0xFF if c2 > 0 */
    s = pos | (~(neg | pos) & ge);           /* subtract p */
    m = neg | s;                             /* add or subtract p */
    sum = s & 1;                             /* r + ~p + 1 = r - p */
    for (b = 0; b < num_bytes; ++b) {
        sum += r[b] + (uint8_t)((p[b] & m) ^ s);
        r[b] = (uint8_t)sum;
        sum >>= 8;
    }
}

/* r += c * d, where d = sum(digits[w] * 2^(32 w)) with digits in {-1, 0, 1},
   then the final reduction (see vli_mmod_select8). */
#if uECC_MMOD8_NIST_C
static void vli_mmod_fold_select8(uint8_t *r, int16_t c, const int8_t *digits,
                                  const uint8_t *p, wordcount_t num_bytes) {
    int16_t acc = 0;
    int16_t cmp = 0;
    wordcount_t b;
    for (b = 0; b < num_bytes; ++b) {
        acc += r[b];
        if ((b & 3) == 0) {
            int8_t d = digits[b >> 2];
            if (d > 0) {
                acc += c;
            } else if (d < 0) {
                acc -= c;
            }
        }
        r[b] = (uint8_t)acc;
        acc >>= 8;
        cmp += (int16_t)r[b] - p[b];         /* Track the borrow of r - p. */
        cmp >>= 8;
    }
    vli_mmod_select8(r, (int8_t)acc, (int8_t)cmp, p, num_bytes);
}
#endif /* uECC_MMOD8_NIST_C */
#undef uECC_MMOD8_NIST_C
#endif /* (uECC_OPTIMIZATION_LEVEL > 0) && (uECC_WORD_SIZE == 1) && C reduction needed */

#if uECC_SUPPORTS_secp160r1

#if (uECC_OPTIMIZATION_LEVEL > 0)
static void vli_mmod_fast_secp160r1(uECC_word_t *result, uECC_word_t *product);
#endif

static const struct uECC_Curve_t curve_secp160r1 = {
    num_words_secp160r1,
    num_bytes_secp160r1,
    161, /* num_n_bits */
    { BYTES_TO_WORDS_8(FF, FF, FF, 7F, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_4(FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(57, 22, 75, CA, D3, AE, 27, F9),
        BYTES_TO_WORDS_8(C8, F4, 01, 00, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(00, 00, 00, 00, 01, 00, 00, 00) },
    { BYTES_TO_WORDS_8(82, FC, CB, 13, B9, 8B, C3, 68),
        BYTES_TO_WORDS_8(89, 69, 64, 46, 28, 73, F5, 8E),
        BYTES_TO_WORDS_4(68, B5, 96, 4A),

        BYTES_TO_WORDS_8(32, FB, C5, 7A, 37, 51, 23, 04),
        BYTES_TO_WORDS_8(12, C9, DC, 59, 7D, 94, 68, 31),
        BYTES_TO_WORDS_4(55, 28, A6, 23) },
    { BYTES_TO_WORDS_8(45, FA, 65, C5, AD, D4, D4, 81),
        BYTES_TO_WORDS_8(9F, F8, AC, 65, 8B, 7A, BD, 54),
        BYTES_TO_WORDS_4(FC, BE, 97, 1C) },
    &double_jacobian_default,
#if uECC_SUPPORT_COMPRESSED_POINT
    &mod_sqrt_default,
#endif
    &x_side_default,
#if (uECC_OPTIMIZATION_LEVEL > 0)
    &vli_mmod_fast_secp160r1
#endif
};

uECC_Curve uECC_secp160r1(void) { return &curve_secp160r1; }

#if uECC_ARM_COMPACT_L1 && (uECC_WORD_SIZE == 4)
/* Smaller, variable-time reduction (level 1 on ARM; see uECC.c). */
static void omega_mult_secp160r1(uECC_word_t *result, const uECC_word_t *right);
static void vli_mmod_fast_secp160r1(uECC_word_t *result, uECC_word_t *product) {
    uECC_word_t tmp[2 * num_words_secp160r1];
    uECC_word_t carry;

    uECC_vli_clear(tmp, num_words_secp160r1);
    uECC_vli_clear(tmp + num_words_secp160r1, num_words_secp160r1);

    omega_mult_secp160r1(tmp, product + num_words_secp160r1); /* (Rq, q) = q * c */

    carry = uECC_vli_add(result, product, tmp, num_words_secp160r1); /* (C, r) = r + q */
    uECC_vli_clear(product, num_words_secp160r1);
    omega_mult_secp160r1(product, tmp + num_words_secp160r1); /* Rq*c */
    carry += uECC_vli_add(result, result, product, num_words_secp160r1); /* (C1, r) = r + Rq*c */

    while (carry > 0) {
        --carry;
        uECC_vli_sub(result, result, curve_secp160r1.p, num_words_secp160r1);
    }
    if (uECC_vli_cmp_unsafe(result, curve_secp160r1.p, num_words_secp160r1) > 0) {
        uECC_vli_sub(result, result, curve_secp160r1.p, num_words_secp160r1);
    }
}
static void omega_mult_secp160r1(uint32_t *result, const uint32_t *right) {
    uint32_t carry;
    unsigned i;

    /* Multiply by (2^31 + 1). */
    uECC_vli_set(result + 1, right, num_words_secp160r1); /* 2^32 */
    uECC_vli_rshift1(result + 1, num_words_secp160r1); /* 2^31 */
    result[0] = right[0] << 31; /* get last bit from shift */

    carry = uECC_vli_add(result, result, right, num_words_secp160r1); /* 2^31 + 1 */
    for (i = num_words_secp160r1; carry; ++i) {
        uint64_t sum = (uint64_t)result[i] + carry;
        result[i] = (uint32_t)sum;
        carry = sum >> 32;
    }
}
#define asm_mmod_fast_secp160r1 1 /* skip the constant-time C version below */
#endif

#if (uECC_OPTIMIZATION_LEVEL > 0 && !asm_mmod_fast_secp160r1)
/* Computes result = product % curve_p
    see http://www.isys.uni-klu.ac.at/PDF/2001-0126-MT.pdf page 354

    Note that this only works if log2(omega) < log2(p) / 2 */
#if (uECC_WORD_SIZE == 8)
static void omega_mult_secp160r1(uECC_word_t *result, const uECC_word_t *right);
#endif
#if uECC_WORD_SIZE == 8
static void vli_mmod_fast_secp160r1(uECC_word_t *result, uECC_word_t *product) {
    uECC_word_t tmp[2 * num_words_secp160r1];
    uECC_word_t copy;

    uECC_vli_clear(tmp, num_words_secp160r1);
    uECC_vli_clear(tmp + num_words_secp160r1, num_words_secp160r1);

    omega_mult_secp160r1(tmp, product + num_words_secp160r1 - 1); /* (Rq, q) = q * c */

    product[num_words_secp160r1 - 1] &= 0xffffffff;
    copy = tmp[num_words_secp160r1 - 1];
    tmp[num_words_secp160r1 - 1] &= 0xffffffff;
    uECC_vli_add(result, product, tmp, num_words_secp160r1); /* (C, r) = r + q */
    uECC_vli_clear(product, num_words_secp160r1);
    tmp[num_words_secp160r1 - 1] = copy;
    omega_mult_secp160r1(product, tmp + num_words_secp160r1 - 1); /* Rq*c */
    uECC_vli_add(result, result, product, num_words_secp160r1); /* (C1, r) = r + Rq*c */

    while (uECC_vli_cmp_unsafe(result, curve_secp160r1.p, num_words_secp160r1) > 0) {
        uECC_vli_sub(result, result, curve_secp160r1.p, num_words_secp160r1);
    }
}

static void omega_mult_secp160r1(uint64_t *result, const uint64_t *right) {
    uint32_t carry;
    unsigned i;

    /* Multiply by (2^31 + 1). */
    carry = 0;
    for (i = 0; i < num_words_secp160r1; ++i) {
        uint64_t tmp = (right[i] >> 32) | (right[i + 1] << 32);
        result[i] = (tmp << 31) + tmp + carry;
        carry = (tmp >> 33) + (result[i] < tmp || (carry && result[i] == tmp));
    }
    result[i] = carry;
}
#elif uECC_WORD_SIZE == 4
/* r += c * (2^31 + 1), where 2^160 = 2^31 + 1 (mod p). Returns the carry. */
static int32_t mmod_fold_secp160r1(uint32_t *r, int64_t c) {
    int64_t acc;
    acc = (int64_t)r[0] + c + c * ((int64_t)1 << 31); r[0] = (uint32_t)acc; acc >>= 32;
    acc += r[1];                                r[1] = (uint32_t)acc; acc >>= 32;
    acc += r[2];                                r[2] = (uint32_t)acc; acc >>= 32;
    acc += r[3];                                r[3] = (uint32_t)acc; acc >>= 32;
    acc += r[4];                                r[4] = (uint32_t)acc; acc >>= 32;
    return (int32_t)acc;
}

static void vli_mmod_fast_secp160r1(uint32_t *result, uint32_t *product) {
    const uint32_t a0 = product[5], a1 = product[6], a2 = product[7],
                   a3 = product[8], a4 = product[9];
    uint64_t acc;
    uint32_t c;

    /* result = lo + hi + hi * 2^31; the part above 2^160 goes in c. */
    acc = (uint64_t)product[0] + a0 + (uint32_t)(a0 << 31);
    result[0] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[1] + a1 + ((a1 << 31) | (a0 >> 1));
    result[1] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[2] + a2 + ((a2 << 31) | (a1 >> 1));
    result[2] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[3] + a3 + ((a3 << 31) | (a2 >> 1));
    result[3] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[4] + a4 + ((a4 << 31) | (a3 >> 1));
    result[4] = (uint32_t)acc; acc >>= 32;
    c = (uint32_t)acc + (a4 >> 1);

    MMOD_FINISH(mmod_fold_secp160r1, result, c);
}
#else /* uECC_WORD_SIZE == 1 */
/* Byte i of x * 2^7 (x has n bytes, i in [0, n]). */
#define SHL7_SECP160R1(x, i, n) \
    ((uint8_t)(((i) < (n) ? (x)[i] << 7 : 0) | ((i) > 0 ? (x)[(i) - 1] >> 1 : 0)))

/* Same approach as the 32-bit version, one byte column at a time: hi * 2^31 is
   hi * 2^7 shifted up 3 bytes. */
static void vli_mmod_fast_secp160r1(uint8_t *result, uint8_t *product) {
    const uint8_t *hi = product + 20;
    const uint8_t *p = curve_secp160r1.p;
    uint8_t c[4];
    uint16_t acc = 0;
    int16_t cmp = 0;
    uint8_t b;

    /* result = lo + hi + hi * 2^31 (2^160 = 2^31 + 1 (mod p)); carry -> c. */
    for (b = 0; b < 3; ++b) {
        acc += product[b] + hi[b];
        result[b] = (uint8_t)acc;
        acc >>= 8;
    }
    for (b = 3; b < 20; ++b) {
        acc += product[b] + hi[b] + SHL7_SECP160R1(hi, b - 3, 20);
        result[b] = (uint8_t)acc;
        acc >>= 8;
    }
    for (b = 20; b < 24; ++b) {
        acc += SHL7_SECP160R1(hi, b - 3, 20);
        c[b - 20] = (uint8_t)acc;
        acc >>= 8;
    }

    /* Fold the carry the same way, tracking the borrow of r - p. */
    acc = 0;
    for (b = 0; b < 20; ++b) {
        acc += result[b];
        if (b < 4) {
            acc += c[b];
        }
        if (b >= 3 && b < 8) {
            acc += SHL7_SECP160R1(c, b - 3, 4);
        }
        result[b] = (uint8_t)acc;
        acc >>= 8;
        cmp += (int16_t)result[b] - p[b];
        cmp >>= 8;
    }
    vli_mmod_select8(result, (int8_t)acc, (int8_t)cmp, p, 20);
}
#endif

#endif /* (uECC_OPTIMIZATION_LEVEL > 0 && !asm_mmod_fast_secp160r1) */

#endif /* uECC_SUPPORTS_secp160r1 */

#if uECC_SUPPORTS_secp192r1

#if (uECC_OPTIMIZATION_LEVEL > 0)
static void vli_mmod_fast_secp192r1(uECC_word_t *result, uECC_word_t *product);
#endif

static const struct uECC_Curve_t curve_secp192r1 = {
    num_words_secp192r1,
    num_bytes_secp192r1,
    192, /* num_n_bits */
    { BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FE, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(31, 28, D2, B4, B1, C9, 6B, 14),
        BYTES_TO_WORDS_8(36, F8, DE, 99, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(12, 10, FF, 82, FD, 0A, FF, F4),
        BYTES_TO_WORDS_8(00, 88, A1, 43, EB, 20, BF, 7C),
        BYTES_TO_WORDS_8(F6, 90, 30, B0, 0E, A8, 8D, 18),

        BYTES_TO_WORDS_8(11, 48, 79, 1E, A1, 77, F9, 73),
        BYTES_TO_WORDS_8(D5, CD, 24, 6B, ED, 11, 10, 63),
        BYTES_TO_WORDS_8(78, DA, C8, FF, 95, 2B, 19, 07) },
    { BYTES_TO_WORDS_8(B1, B9, 46, C1, EC, DE, B8, FE),
        BYTES_TO_WORDS_8(49, 30, 24, 72, AB, E9, A7, 0F),
        BYTES_TO_WORDS_8(E7, 80, 9C, E5, 19, 05, 21, 64) },
    &double_jacobian_default,
#if uECC_SUPPORT_COMPRESSED_POINT
    &mod_sqrt_default,
#endif
    &x_side_default,
#if (uECC_OPTIMIZATION_LEVEL > 0)
    &vli_mmod_fast_secp192r1
#endif
};

uECC_Curve uECC_secp192r1(void) { return &curve_secp192r1; }

#if uECC_ARM_COMPACT_L1 && (uECC_WORD_SIZE == 4)
/* Smaller, variable-time reduction (level 1 on ARM; see uECC.c). */
static void vli_mmod_fast_secp192r1(uint32_t *result, uint32_t *product) {
    uint32_t tmp[num_words_secp192r1];
    int carry;

    uECC_vli_set(result, product, num_words_secp192r1);

    uECC_vli_set(tmp, &product[6], num_words_secp192r1);
    carry = uECC_vli_add(result, result, tmp, num_words_secp192r1);

    tmp[0] = tmp[1] = 0;
    tmp[2] = product[6];
    tmp[3] = product[7];
    tmp[4] = product[8];
    tmp[5] = product[9];
    carry += uECC_vli_add(result, result, tmp, num_words_secp192r1);

    tmp[0] = tmp[2] = product[10];
    tmp[1] = tmp[3] = product[11];
    tmp[4] = tmp[5] = 0;
    carry += uECC_vli_add(result, result, tmp, num_words_secp192r1);

    while (carry || uECC_vli_cmp_unsafe(curve_secp192r1.p, result, num_words_secp192r1) != 1) {
        carry -= uECC_vli_sub(result, result, curve_secp192r1.p, num_words_secp192r1);
    }
}
#define asm_mmod_fast_secp192r1 1 /* skip the constant-time C version below */
#endif

#if (uECC_OPTIMIZATION_LEVEL > 0 && !asm_mmod_fast_secp192r1)
/* Computes result = product % curve_p.
   See algorithm 5 and 6 from http://www.isys.uni-klu.ac.at/PDF/2001-0126-MT.pdf */
#if uECC_WORD_SIZE == 1
/* Sums the terms column by column, then folds the carry using digits
   (see vli_mmod_columns8 and vli_mmod_fold_select8). */
static void vli_mmod_fast_secp192r1(uint8_t *result, uint8_t *product) {
    static const uint8_t terms[] = {
        0, 4, 0xFF,
        1, 5, 0xFF,
        2, 0, 4, 0xFF,
        3, 1, 5, 0xFF,
        4, 2, 0xFF,
        5, 3, 0xFF };
    static const int8_t digits[] = { 1, 0, 1, 0, 0, 0 }; /* 2^192 = 2^64 + 1 */
    int16_t c = vli_mmod_columns8(result, product, terms, num_words_secp192r1);
    vli_mmod_fold_select8(result, c, digits, curve_secp192r1.p, num_words_secp192r1);
}
#elif uECC_WORD_SIZE == 4
/* r += c * (2^64 + 1), where 2^192 = 2^64 + 1 (mod p). Returns the carry. */
static int32_t mmod_fold_secp192r1(uint32_t *r, int64_t c) {
    int64_t acc;
    acc = (int64_t)r[0] + c; r[0] = (uint32_t)acc; acc >>= 32;
    acc += r[1];             r[1] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)r[2] + c; r[2] = (uint32_t)acc; acc >>= 32;
    acc += r[3];              r[3] = (uint32_t)acc; acc >>= 32;
    acc += r[4];              r[4] = (uint32_t)acc; acc >>= 32;
    acc += r[5];             r[5] = (uint32_t)acc; acc >>= 32;
    return (int32_t)acc;
}

/* result = lo + hi * (2^64 + 1), computed column by column, then MMOD_FINISH. */
static void vli_mmod_fast_secp192r1(uint32_t *result, uint32_t *product) {
    const uint32_t a0 = product[6], a1 = product[7], a2 = product[8],
                   a3 = product[9], a4 = product[10], a5 = product[11];
    uint64_t acc;
    uint32_t c;

    acc = (uint64_t)product[0] + a0 + a4;
    result[0] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[1] + a1 + a5;
    result[1] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[2] + a2 + a0 + a4;
    result[2] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[3] + a3 + a1 + a5;
    result[3] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[4] + a4 + a2;
    result[4] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[5] + a5 + a3;
    result[5] = (uint32_t)acc; acc >>= 32;
    c = (uint32_t)acc;

    MMOD_FINISH(mmod_fold_secp192r1, result, c);
}
#else
static void vli_mmod_fast_secp192r1(uint64_t *result, uint64_t *product) {
    uint64_t tmp[num_words_secp192r1];
    int carry;

    uECC_vli_set(result, product, num_words_secp192r1);

    uECC_vli_set(tmp, &product[3], num_words_secp192r1);
    carry = (int)uECC_vli_add(result, result, tmp, num_words_secp192r1);

    tmp[0] = 0;
    tmp[1] = product[3];
    tmp[2] = product[4];
    carry += uECC_vli_add(result, result, tmp, num_words_secp192r1);

    tmp[0] = tmp[1] = product[5];
    tmp[2] = 0;
    carry += uECC_vli_add(result, result, tmp, num_words_secp192r1);

    while (carry || uECC_vli_cmp_unsafe(curve_secp192r1.p, result, num_words_secp192r1) != 1) {
        carry -= uECC_vli_sub(result, result, curve_secp192r1.p, num_words_secp192r1);
    }
}
#endif /* uECC_WORD_SIZE */
#endif /* (uECC_OPTIMIZATION_LEVEL > 0 && !asm_mmod_fast_secp192r1) */

#endif /* uECC_SUPPORTS_secp192r1 */

#if uECC_SUPPORTS_secp224r1

#if uECC_SUPPORT_COMPRESSED_POINT
static void mod_sqrt_secp224r1(uECC_word_t *a, uECC_Curve curve);
#endif
#if (uECC_OPTIMIZATION_LEVEL > 0)
static void vli_mmod_fast_secp224r1(uECC_word_t *result, uECC_word_t *product);
#endif

static const struct uECC_Curve_t curve_secp224r1 = {
    num_words_secp224r1,
    num_bytes_secp224r1,
    224, /* num_n_bits */
    { BYTES_TO_WORDS_8(01, 00, 00, 00, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(00, 00, 00, 00, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_4(FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(3D, 2A, 5C, 5C, 45, 29, DD, 13),
        BYTES_TO_WORDS_8(3E, F0, B8, E0, A2, 16, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_4(FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(21, 1D, 5C, 11, D6, 80, 32, 34),
        BYTES_TO_WORDS_8(22, 11, C2, 56, D3, C1, 03, 4A),
        BYTES_TO_WORDS_8(B9, 90, 13, 32, 7F, BF, B4, 6B),
        BYTES_TO_WORDS_4(BD, 0C, 0E, B7),

        BYTES_TO_WORDS_8(34, 7E, 00, 85, 99, 81, D5, 44),
        BYTES_TO_WORDS_8(64, 47, 07, 5A, A0, 75, 43, CD),
        BYTES_TO_WORDS_8(E6, DF, 22, 4C, FB, 23, F7, B5),
        BYTES_TO_WORDS_4(88, 63, 37, BD) },
    { BYTES_TO_WORDS_8(B4, FF, 55, 23, 43, 39, 0B, 27),
        BYTES_TO_WORDS_8(BA, D8, BF, D7, B7, B0, 44, 50),
        BYTES_TO_WORDS_8(56, 32, 41, F5, AB, B3, 04, 0C),
        BYTES_TO_WORDS_4(85, 0A, 05, B4) },
    &double_jacobian_default,
#if uECC_SUPPORT_COMPRESSED_POINT
    &mod_sqrt_secp224r1,
#endif
    &x_side_default,
#if (uECC_OPTIMIZATION_LEVEL > 0)
    &vli_mmod_fast_secp224r1
#endif
};

uECC_Curve uECC_secp224r1(void) { return &curve_secp224r1; }


#if uECC_SUPPORT_COMPRESSED_POINT
/* Routine 3.2.4 RS;  from http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void mod_sqrt_secp224r1_rs(uECC_word_t *d1,
                                  uECC_word_t *e1,
                                  uECC_word_t *f1,
                                  const uECC_word_t *d0,
                                  const uECC_word_t *e0,
                                  const uECC_word_t *f0) {
    uECC_word_t t[num_words_secp224r1];

    uECC_vli_modSquare_fast(t, d0, &curve_secp224r1);                    /* t <-- d0 ^ 2 */
    uECC_vli_modMult_fast(e1, d0, e0, &curve_secp224r1);                 /* e1 <-- d0 * e0 */
    uECC_vli_modAdd(d1, t, f0, curve_secp224r1.p, num_words_secp224r1);  /* d1 <-- t  + f0 */
    uECC_vli_modAdd(e1, e1, e1, curve_secp224r1.p, num_words_secp224r1); /* e1 <-- e1 + e1 */
    uECC_vli_modMult_fast(f1, t, f0, &curve_secp224r1);                  /* f1 <-- t  * f0 */
    uECC_vli_modAdd(f1, f1, f1, curve_secp224r1.p, num_words_secp224r1); /* f1 <-- f1 + f1 */
    uECC_vli_modAdd(f1, f1, f1, curve_secp224r1.p, num_words_secp224r1); /* f1 <-- f1 + f1 */
}

/* Routine 3.2.5 RSS;  from http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void mod_sqrt_secp224r1_rss(uECC_word_t *d1,
                                   uECC_word_t *e1,
                                   uECC_word_t *f1,
                                   const uECC_word_t *d0,
                                   const uECC_word_t *e0,
                                   const uECC_word_t *f0,
                                   const bitcount_t j) {
    bitcount_t i;

    uECC_vli_set(d1, d0, num_words_secp224r1); /* d1 <-- d0 */
    uECC_vli_set(e1, e0, num_words_secp224r1); /* e1 <-- e0 */
    uECC_vli_set(f1, f0, num_words_secp224r1); /* f1 <-- f0 */
    for (i = 1; i <= j; i++) {
        mod_sqrt_secp224r1_rs(d1, e1, f1, d1, e1, f1); /* RS (d1,e1,f1,d1,e1,f1) */
    }
}

/* Routine 3.2.6 RM;  from http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void mod_sqrt_secp224r1_rm(uECC_word_t *d2,
                                  uECC_word_t *e2,
                                  uECC_word_t *f2,
                                  const uECC_word_t *c,
                                  const uECC_word_t *d0,
                                  const uECC_word_t *e0,
                                  const uECC_word_t *d1,
                                  const uECC_word_t *e1) {
    uECC_word_t t1[num_words_secp224r1];
    uECC_word_t t2[num_words_secp224r1];

    uECC_vli_modMult_fast(t1, e0, e1, &curve_secp224r1); /* t1 <-- e0 * e1 */
    uECC_vli_modMult_fast(t1, t1, c, &curve_secp224r1);  /* t1 <-- t1 * c */
    /* t1 <-- p  - t1 */
    uECC_vli_modSub(t1, curve_secp224r1.p, t1, curve_secp224r1.p, num_words_secp224r1);
    uECC_vli_modMult_fast(t2, d0, d1, &curve_secp224r1);                 /* t2 <-- d0 * d1 */
    uECC_vli_modAdd(t2, t2, t1, curve_secp224r1.p, num_words_secp224r1); /* t2 <-- t2 + t1 */
    uECC_vli_modMult_fast(t1, d0, e1, &curve_secp224r1);                 /* t1 <-- d0 * e1 */
    uECC_vli_modMult_fast(e2, d1, e0, &curve_secp224r1);                 /* e2 <-- d1 * e0 */
    uECC_vli_modAdd(e2, e2, t1, curve_secp224r1.p, num_words_secp224r1); /* e2 <-- e2 + t1 */
    uECC_vli_modSquare_fast(f2, e2, &curve_secp224r1);                   /* f2 <-- e2^2 */
    uECC_vli_modMult_fast(f2, f2, c, &curve_secp224r1);                  /* f2 <-- f2 * c */
    /* f2 <-- p  - f2 */
    uECC_vli_modSub(f2, curve_secp224r1.p, f2, curve_secp224r1.p, num_words_secp224r1);
    uECC_vli_set(d2, t2, num_words_secp224r1); /* d2 <-- t2 */
}

/* Routine 3.2.7 RP;  from http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void mod_sqrt_secp224r1_rp(uECC_word_t *d1,
                                  uECC_word_t *e1,
                                  uECC_word_t *f1,
                                  const uECC_word_t *c,
                                  const uECC_word_t *r) {
    wordcount_t i;
    wordcount_t pow2i = 1;
    uECC_word_t d0[num_words_secp224r1];
    uECC_word_t e0[num_words_secp224r1] = {1}; /* e0 <-- 1 */
    uECC_word_t f0[num_words_secp224r1];

    uECC_vli_set(d0, r, num_words_secp224r1); /* d0 <-- r */
    /* f0 <-- p  - c */
    uECC_vli_modSub(f0, curve_secp224r1.p, c, curve_secp224r1.p, num_words_secp224r1);
    for (i = 0; i <= 6; i++) {
        mod_sqrt_secp224r1_rss(d1, e1, f1, d0, e0, f0, pow2i); /* RSS (d1,e1,f1,d0,e0,f0,2^i) */
        mod_sqrt_secp224r1_rm(d1, e1, f1, c, d1, e1, d0, e0);  /* RM (d1,e1,f1,c,d1,e1,d0,e0) */
        uECC_vli_set(d0, d1, num_words_secp224r1); /* d0 <-- d1 */
        uECC_vli_set(e0, e1, num_words_secp224r1); /* e0 <-- e1 */
        uECC_vli_set(f0, f1, num_words_secp224r1); /* f0 <-- f1 */
        pow2i *= 2;
    }
}

/* Compute a = sqrt(a) (mod curve_p). */
/* Routine 3.2.8 mp_mod_sqrt_224; from http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void mod_sqrt_secp224r1(uECC_word_t *a, uECC_Curve curve) {
    bitcount_t i;
    uECC_word_t e1[num_words_secp224r1];
    uECC_word_t f1[num_words_secp224r1];
    uECC_word_t d0[num_words_secp224r1];
    uECC_word_t e0[num_words_secp224r1];
    uECC_word_t f0[num_words_secp224r1];
    uECC_word_t d1[num_words_secp224r1];

    /* s = a; using constant instead of random value */
    mod_sqrt_secp224r1_rp(d0, e0, f0, a, a);           /* RP (d0, e0, f0, c, s) */
    mod_sqrt_secp224r1_rs(d1, e1, f1, d0, e0, f0);     /* RS (d1, e1, f1, d0, e0, f0) */
    for (i = 1; i <= 95; i++) {
        uECC_vli_set(d0, d1, num_words_secp224r1);          /* d0 <-- d1 */
        uECC_vli_set(e0, e1, num_words_secp224r1);          /* e0 <-- e1 */
        uECC_vli_set(f0, f1, num_words_secp224r1);          /* f0 <-- f1 */
        mod_sqrt_secp224r1_rs(d1, e1, f1, d0, e0, f0); /* RS (d1, e1, f1, d0, e0, f0) */
        if (uECC_vli_isZero(d1, num_words_secp224r1)) {     /* if d1 == 0 */
                break;
        }
    }
    uECC_vli_modInv(f1, e0, curve_secp224r1.p, num_words_secp224r1); /* f1 <-- 1 / e0 */
    uECC_vli_modMult_fast(a, d0, f1, &curve_secp224r1);              /* a  <-- d0 / e0 */
}
#endif /* uECC_SUPPORT_COMPRESSED_POINT */

#if uECC_ARM_COMPACT_L1 && (uECC_WORD_SIZE == 4)
/* Smaller, variable-time reduction (level 1 on ARM; see uECC.c). */
static void vli_mmod_fast_secp224r1(uint32_t *result, uint32_t *product)
{
    uint32_t tmp[num_words_secp224r1];
    int carry;

    /* t */
    uECC_vli_set(result, product, num_words_secp224r1);

    /* s1 */
    tmp[0] = tmp[1] = tmp[2] = 0;
    tmp[3] = product[7];
    tmp[4] = product[8];
    tmp[5] = product[9];
    tmp[6] = product[10];
    carry = uECC_vli_add(result, result, tmp, num_words_secp224r1);

    /* s2 */
    tmp[3] = product[11];
    tmp[4] = product[12];
    tmp[5] = product[13];
    tmp[6] = 0;
    carry += uECC_vli_add(result, result, tmp, num_words_secp224r1);

    /* d1 */
    tmp[0] = product[7];
    tmp[1] = product[8];
    tmp[2] = product[9];
    tmp[3] = product[10];
    tmp[4] = product[11];
    tmp[5] = product[12];
    tmp[6] = product[13];
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp224r1);

    /* d2 */
    tmp[0] = product[11];
    tmp[1] = product[12];
    tmp[2] = product[13];
    tmp[3] = tmp[4] = tmp[5] = tmp[6] = 0;
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp224r1);

    if (carry < 0) {
        do {
            carry += uECC_vli_add(result, result, curve_secp224r1.p, num_words_secp224r1);
        } while (carry < 0);
    } else {
        while (carry || uECC_vli_cmp_unsafe(curve_secp224r1.p, result, num_words_secp224r1) != 1) {
            carry -= uECC_vli_sub(result, result, curve_secp224r1.p, num_words_secp224r1);
        }
    }
}
#define asm_mmod_fast_secp224r1 1 /* skip the constant-time C version below */
#endif

#if (uECC_OPTIMIZATION_LEVEL > 0 && !asm_mmod_fast_secp224r1)
/* Computes result = product % curve_p
   from http://www.nsa.gov/ia/_files/nist-routines.pdf */
#if uECC_WORD_SIZE == 1
/* Sums the terms column by column, then folds the carry using digits
   (see vli_mmod_columns8 and vli_mmod_fold_select8). */
static void vli_mmod_fast_secp224r1(uint8_t *result, uint8_t *product) {
    static const uint8_t terms[] = {
        0x80 | 0, 0x80 | 4, 0xFF,
        0x80 | 1, 0x80 | 5, 0xFF,
        0x80 | 2, 0x80 | 6, 0xFF,
        0, 4, 0x80 | 3, 0xFF,
        1, 5, 0x80 | 4, 0xFF,
        2, 6, 0x80 | 5, 0xFF,
        3, 0x80 | 6, 0xFF };
    static const int8_t digits[] = { -1, 0, 0, 1, 0, 0, 0 }; /* 2^224 = 2^96 - 1 */
    int16_t c = vli_mmod_columns8(result, product, terms, num_words_secp224r1);
    vli_mmod_fold_select8(result, c, digits, curve_secp224r1.p, num_words_secp224r1);
}
#elif uECC_WORD_SIZE == 4
/* r += c * (2^96 - 1), where 2^224 = 2^96 - 1 (mod p). Returns the carry. */
static int32_t mmod_fold_secp224r1(uint32_t *r, int64_t c) {
    int64_t acc;
    acc = (int64_t)r[0] - c; r[0] = (uint32_t)acc; acc >>= 32;
    acc += r[1];             r[1] = (uint32_t)acc; acc >>= 32;
    acc += r[2];             r[2] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)r[3] + c; r[3] = (uint32_t)acc; acc >>= 32;
    acc += r[4];             r[4] = (uint32_t)acc; acc >>= 32;
    acc += r[5];             r[5] = (uint32_t)acc; acc >>= 32;
    acc += r[6];             r[6] = (uint32_t)acc; acc >>= 32;
    return (int32_t)acc;
}

/* t + s1 + s2 - d1 - d2, from http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void vli_mmod_fast_secp224r1(uint32_t *result, uint32_t *product) {
    const uint32_t a0 = product[7], a1 = product[8], a2 = product[9],
                   a3 = product[10], a4 = product[11], a5 = product[12],
                   a6 = product[13];
    int64_t acc;
    int32_t c;

    acc = (int64_t)product[0] - a0 - a4;
    result[0] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)product[1] - a1 - a5;
    result[1] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)product[2] - a2 - a6;
    result[2] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)product[3] + a0 + a4 - a3;
    result[3] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)product[4] + a1 + a5 - a4;
    result[4] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)product[5] + a2 + a6 - a5;
    result[5] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)product[6] + a3 - a6;
    result[6] = (uint32_t)acc; acc >>= 32;
    c = (int32_t)acc;

    MMOD_FINISH(mmod_fold_secp224r1, result, c);
}
#else
static void vli_mmod_fast_secp224r1(uint64_t *result, uint64_t *product)
{
    uint64_t tmp[num_words_secp224r1];
    int carry = 0;

    /* t */
    uECC_vli_set(result, product, num_words_secp224r1);
    result[num_words_secp224r1 - 1] &= 0xffffffff;

    /* s1 */
    tmp[0] = 0;
    tmp[1] = product[3] & 0xffffffff00000000ull;
    tmp[2] = product[4];
    tmp[3] = product[5] & 0xffffffff;
    uECC_vli_add(result, result, tmp, num_words_secp224r1);

    /* s2 */
    tmp[1] = product[5] & 0xffffffff00000000ull;
    tmp[2] = product[6];
    tmp[3] = 0;
    uECC_vli_add(result, result, tmp, num_words_secp224r1);

    /* d1 */
    tmp[0] = (product[3] >> 32) | (product[4] << 32);
    tmp[1] = (product[4] >> 32) | (product[5] << 32);
    tmp[2] = (product[5] >> 32) | (product[6] << 32);
    tmp[3] = product[6] >> 32;
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp224r1);

    /* d2 */
    tmp[0] = (product[5] >> 32) | (product[6] << 32);
    tmp[1] = product[6] >> 32;
    tmp[2] = tmp[3] = 0;
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp224r1);

    if (carry < 0) {
        do {
            carry += uECC_vli_add(result, result, curve_secp224r1.p, num_words_secp224r1);
        } while (carry < 0);
    } else {
        while (uECC_vli_cmp_unsafe(curve_secp224r1.p, result, num_words_secp224r1) != 1) {
            uECC_vli_sub(result, result, curve_secp224r1.p, num_words_secp224r1);
        }
    }
}
#endif /* uECC_WORD_SIZE */
#endif /* (uECC_OPTIMIZATION_LEVEL > 0 && !asm_mmod_fast_secp224r1) */

#endif /* uECC_SUPPORTS_secp224r1 */

#if uECC_SUPPORTS_secp256r1

#if (uECC_OPTIMIZATION_LEVEL > 0)
static void vli_mmod_fast_secp256r1(uECC_word_t *result, uECC_word_t *product);
#endif

static const struct uECC_Curve_t curve_secp256r1 = {
    num_words_secp256r1,
    num_bytes_secp256r1,
    256, /* num_n_bits */
    { BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(00, 00, 00, 00, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(01, 00, 00, 00, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(51, 25, 63, FC, C2, CA, B9, F3),
        BYTES_TO_WORDS_8(84, 9E, 17, A7, AD, FA, E6, BC),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(00, 00, 00, 00, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(96, C2, 98, D8, 45, 39, A1, F4),
        BYTES_TO_WORDS_8(A0, 33, EB, 2D, 81, 7D, 03, 77),
        BYTES_TO_WORDS_8(F2, 40, A4, 63, E5, E6, BC, F8),
        BYTES_TO_WORDS_8(47, 42, 2C, E1, F2, D1, 17, 6B),

        BYTES_TO_WORDS_8(F5, 51, BF, 37, 68, 40, B6, CB),
        BYTES_TO_WORDS_8(CE, 5E, 31, 6B, 57, 33, CE, 2B),
        BYTES_TO_WORDS_8(16, 9E, 0F, 7C, 4A, EB, E7, 8E),
        BYTES_TO_WORDS_8(9B, 7F, 1A, FE, E2, 42, E3, 4F) },
    { BYTES_TO_WORDS_8(4B, 60, D2, 27, 3E, 3C, CE, 3B),
        BYTES_TO_WORDS_8(F6, B0, 53, CC, B0, 06, 1D, 65),
        BYTES_TO_WORDS_8(BC, 86, 98, 76, 55, BD, EB, B3),
        BYTES_TO_WORDS_8(E7, 93, 3A, AA, D8, 35, C6, 5A) },
    &double_jacobian_default,
#if uECC_SUPPORT_COMPRESSED_POINT
    &mod_sqrt_default,
#endif
    &x_side_default,
#if (uECC_OPTIMIZATION_LEVEL > 0)
    &vli_mmod_fast_secp256r1
#endif
};

uECC_Curve uECC_secp256r1(void) { return &curve_secp256r1; }


#if uECC_ARM_COMPACT_L1 && (uECC_WORD_SIZE == 4)
/* Smaller, variable-time reduction (level 1 on ARM; see uECC.c). */
static void vli_mmod_fast_secp256r1(uint32_t *result, uint32_t *product) {
    uint32_t tmp[num_words_secp256r1];
    int carry;

    /* t */
    uECC_vli_set(result, product, num_words_secp256r1);

    /* s1 */
    tmp[0] = tmp[1] = tmp[2] = 0;
    tmp[3] = product[11];
    tmp[4] = product[12];
    tmp[5] = product[13];
    tmp[6] = product[14];
    tmp[7] = product[15];
    carry = uECC_vli_add(tmp, tmp, tmp, num_words_secp256r1);
    carry += uECC_vli_add(result, result, tmp, num_words_secp256r1);

    /* s2 */
    tmp[3] = product[12];
    tmp[4] = product[13];
    tmp[5] = product[14];
    tmp[6] = product[15];
    tmp[7] = 0;
    carry += uECC_vli_add(tmp, tmp, tmp, num_words_secp256r1);
    carry += uECC_vli_add(result, result, tmp, num_words_secp256r1);

    /* s3 */
    tmp[0] = product[8];
    tmp[1] = product[9];
    tmp[2] = product[10];
    tmp[3] = tmp[4] = tmp[5] = 0;
    tmp[6] = product[14];
    tmp[7] = product[15];
    carry += uECC_vli_add(result, result, tmp, num_words_secp256r1);

    /* s4 */
    tmp[0] = product[9];
    tmp[1] = product[10];
    tmp[2] = product[11];
    tmp[3] = product[13];
    tmp[4] = product[14];
    tmp[5] = product[15];
    tmp[6] = product[13];
    tmp[7] = product[8];
    carry += uECC_vli_add(result, result, tmp, num_words_secp256r1);

    /* d1 */
    tmp[0] = product[11];
    tmp[1] = product[12];
    tmp[2] = product[13];
    tmp[3] = tmp[4] = tmp[5] = 0;
    tmp[6] = product[8];
    tmp[7] = product[10];
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp256r1);

    /* d2 */
    tmp[0] = product[12];
    tmp[1] = product[13];
    tmp[2] = product[14];
    tmp[3] = product[15];
    tmp[4] = tmp[5] = 0;
    tmp[6] = product[9];
    tmp[7] = product[11];
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp256r1);

    /* d3 */
    tmp[0] = product[13];
    tmp[1] = product[14];
    tmp[2] = product[15];
    tmp[3] = product[8];
    tmp[4] = product[9];
    tmp[5] = product[10];
    tmp[6] = 0;
    tmp[7] = product[12];
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp256r1);

    /* d4 */
    tmp[0] = product[14];
    tmp[1] = product[15];
    tmp[2] = 0;
    tmp[3] = product[9];
    tmp[4] = product[10];
    tmp[5] = product[11];
    tmp[6] = 0;
    tmp[7] = product[13];
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp256r1);

    if (carry < 0) {
        do {
            carry += uECC_vli_add(result, result, curve_secp256r1.p, num_words_secp256r1);
        } while (carry < 0);
    } else {
        while (carry || uECC_vli_cmp_unsafe(curve_secp256r1.p, result, num_words_secp256r1) != 1) {
            carry -= uECC_vli_sub(result, result, curve_secp256r1.p, num_words_secp256r1);
        }
    }
}
#define asm_mmod_fast_secp256r1 1 /* skip the constant-time C version below */
#endif

#if (uECC_OPTIMIZATION_LEVEL > 0 && !asm_mmod_fast_secp256r1)
/* Computes result = product % curve_p
   from http://www.nsa.gov/ia/_files/nist-routines.pdf */
#if uECC_WORD_SIZE == 1
/* Sums the terms column by column, then folds the carry using digits
   (see vli_mmod_columns8 and vli_mmod_fold_select8). */
static void vli_mmod_fast_secp256r1(uint8_t *result, uint8_t *product) {
    static const uint8_t terms[] = {
        0, 1, 0x80 | 3, 0x80 | 4, 0x80 | 5, 0x80 | 6, 0xFF,
        1, 2, 0x80 | 4, 0x80 | 5, 0x80 | 6, 0x80 | 7, 0xFF,
        2, 3, 0x80 | 5, 0x80 | 6, 0x80 | 7, 0xFF,
        3, 3, 4, 4, 5, 0x80 | 7, 0x80 | 0, 0x80 | 1, 0xFF,
        4, 4, 5, 5, 6, 0x80 | 1, 0x80 | 2, 0xFF,
        5, 5, 6, 6, 7, 0x80 | 2, 0x80 | 3, 0xFF,
        6, 6, 6, 7, 7, 5, 0x80 | 0, 0x80 | 1, 0xFF,
        7, 7, 7, 0, 0x80 | 2, 0x80 | 3, 0x80 | 4, 0x80 | 5, 0xFF };
    /* 2^256 = 2^224 - 2^192 - 2^96 + 1 */
    static const int8_t digits[] = { 1, 0, 0, -1, 0, 0, -1, 1 };
    int16_t c = vli_mmod_columns8(result, product, terms, num_words_secp256r1);
    vli_mmod_fold_select8(result, c, digits, curve_secp256r1.p, num_words_secp256r1);
}
#elif uECC_WORD_SIZE == 4
/* r += c * (2^224 - 2^192 - 2^96 + 1), where 2^256 = 2^224 - 2^192 - 2^96 + 1
   (mod p). Returns the carry. */
static int32_t mmod_fold_secp256r1(uint32_t *r, int64_t c) {
    int64_t acc;
    acc = (int64_t)r[0] + c; r[0] = (uint32_t)acc; acc >>= 32;
    acc += r[1];             r[1] = (uint32_t)acc; acc >>= 32;
    acc += r[2];             r[2] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)r[3] - c; r[3] = (uint32_t)acc; acc >>= 32;
    acc += r[4];             r[4] = (uint32_t)acc; acc >>= 32;
    acc += r[5];             r[5] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)r[6] - c; r[6] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)r[7] + c; r[7] = (uint32_t)acc; acc >>= 32;
    return (int32_t)acc;
}

/* t + 2 s1 + 2 s2 + s3 + s4 - d1 - d2 - d3 - d4, computed column by column
   (from http://www.nsa.gov/ia/_files/nist-routines.pdf) */
static void vli_mmod_fast_secp256r1(uint32_t *result, uint32_t *product) {
    const uint32_t a0 = product[8], a1 = product[9], a2 = product[10],
                   a3 = product[11], a4 = product[12], a5 = product[13],
                   a6 = product[14], a7 = product[15];
    int64_t acc;
    int32_t c;

    acc = (int64_t)product[0] + a0 + a1 - a3 - a4 - a5 - a6;
    result[0] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)product[1] + a1 + a2 - a4 - a5 - a6 - a7;
    result[1] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)product[2] + a2 + a3 - a5 - a6 - a7;
    result[2] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)product[3] + a3 + a3 + a4 + a4 + a5 - a7 - a0 - a1;
    result[3] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)product[4] + a4 + a4 + a5 + a5 + a6 - a1 - a2;
    result[4] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)product[5] + a5 + a5 + a6 + a6 + a7 - a2 - a3;
    result[5] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)product[6] + a6 + a6 + a6 + a7 + a7 + a5 - a0 - a1;
    result[6] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)product[7] + a7 + a7 + a7 + a0 - a2 - a3 - a4 - a5;
    result[7] = (uint32_t)acc; acc >>= 32;
    c = (int32_t)acc;

    MMOD_FINISH(mmod_fold_secp256r1, result, c);
}
#else
static void vli_mmod_fast_secp256r1(uint64_t *result, uint64_t *product) {
    uint64_t tmp[num_words_secp256r1];
    int carry;

    /* t */
    uECC_vli_set(result, product, num_words_secp256r1);

    /* s1 */
    tmp[0] = 0;
    tmp[1] = product[5] & 0xffffffff00000000ull;
    tmp[2] = product[6];
    tmp[3] = product[7];
    carry = (int)uECC_vli_add(tmp, tmp, tmp, num_words_secp256r1);
    carry += uECC_vli_add(result, result, tmp, num_words_secp256r1);

    /* s2 */
    tmp[1] = product[6] << 32;
    tmp[2] = (product[6] >> 32) | (product[7] << 32);
    tmp[3] = product[7] >> 32;
    carry += uECC_vli_add(tmp, tmp, tmp, num_words_secp256r1);
    carry += uECC_vli_add(result, result, tmp, num_words_secp256r1);

    /* s3 */
    tmp[0] = product[4];
    tmp[1] = product[5] & 0xffffffff;
    tmp[2] = 0;
    tmp[3] = product[7];
    carry += uECC_vli_add(result, result, tmp, num_words_secp256r1);

    /* s4 */
    tmp[0] = (product[4] >> 32) | (product[5] << 32);
    tmp[1] = (product[5] >> 32) | (product[6] & 0xffffffff00000000ull);
    tmp[2] = product[7];
    tmp[3] = (product[6] >> 32) | (product[4] << 32);
    carry += uECC_vli_add(result, result, tmp, num_words_secp256r1);

    /* d1 */
    tmp[0] = (product[5] >> 32) | (product[6] << 32);
    tmp[1] = (product[6] >> 32);
    tmp[2] = 0;
    tmp[3] = (product[4] & 0xffffffff) | (product[5] << 32);
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp256r1);

    /* d2 */
    tmp[0] = product[6];
    tmp[1] = product[7];
    tmp[2] = 0;
    tmp[3] = (product[4] >> 32) | (product[5] & 0xffffffff00000000ull);
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp256r1);

    /* d3 */
    tmp[0] = (product[6] >> 32) | (product[7] << 32);
    tmp[1] = (product[7] >> 32) | (product[4] << 32);
    tmp[2] = (product[4] >> 32) | (product[5] << 32);
    tmp[3] = (product[6] << 32);
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp256r1);

    /* d4 */
    tmp[0] = product[7];
    tmp[1] = product[4] & 0xffffffff00000000ull;
    tmp[2] = product[5];
    tmp[3] = product[6] & 0xffffffff00000000ull;
    carry -= uECC_vli_sub(result, result, tmp, num_words_secp256r1);

    if (carry < 0) {
        do {
            carry += uECC_vli_add(result, result, curve_secp256r1.p, num_words_secp256r1);
        } while (carry < 0);
    } else {
        while (carry || uECC_vli_cmp_unsafe(curve_secp256r1.p, result, num_words_secp256r1) != 1) {
            carry -= uECC_vli_sub(result, result, curve_secp256r1.p, num_words_secp256r1);
        }
    }
}
#endif /* uECC_WORD_SIZE */
#endif /* (uECC_OPTIMIZATION_LEVEL > 0 && !asm_mmod_fast_secp256r1) */

#endif /* uECC_SUPPORTS_secp256r1 */

#if uECC_SUPPORTS_secp256k1

static void double_jacobian_secp256k1(uECC_word_t * X1,
                                      uECC_word_t * Y1,
                                      uECC_word_t * Z1,
                                      uECC_Curve curve);
static void x_side_secp256k1(uECC_word_t *result, const uECC_word_t *x, uECC_Curve curve);
#if (uECC_OPTIMIZATION_LEVEL > 0)
static void vli_mmod_fast_secp256k1(uECC_word_t *result, uECC_word_t *product);
#endif

static const struct uECC_Curve_t curve_secp256k1 = {
    num_words_secp256k1,
    num_bytes_secp256k1,
    256, /* num_n_bits */
    { BYTES_TO_WORDS_8(2F, FC, FF, FF, FE, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(41, 41, 36, D0, 8C, 5E, D2, BF),
        BYTES_TO_WORDS_8(3B, A0, 48, AF, E6, DC, AE, BA),
        BYTES_TO_WORDS_8(FE, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(98, 17, F8, 16, 5B, 81, F2, 59),
        BYTES_TO_WORDS_8(D9, 28, CE, 2D, DB, FC, 9B, 02),
        BYTES_TO_WORDS_8(07, 0B, 87, CE, 95, 62, A0, 55),
        BYTES_TO_WORDS_8(AC, BB, DC, F9, 7E, 66, BE, 79),

        BYTES_TO_WORDS_8(B8, D4, 10, FB, 8F, D0, 47, 9C),
        BYTES_TO_WORDS_8(19, 54, 85, A6, 48, B4, 17, FD),
        BYTES_TO_WORDS_8(A8, 08, 11, 0E, FC, FB, A4, 5D),
        BYTES_TO_WORDS_8(65, C4, A3, 26, 77, DA, 3A, 48) },
    { BYTES_TO_WORDS_8(07, 00, 00, 00, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(00, 00, 00, 00, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(00, 00, 00, 00, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(00, 00, 00, 00, 00, 00, 00, 00) },
    &double_jacobian_secp256k1,
#if uECC_SUPPORT_COMPRESSED_POINT
    &mod_sqrt_default,
#endif
    &x_side_secp256k1,
#if (uECC_OPTIMIZATION_LEVEL > 0)
    &vli_mmod_fast_secp256k1
#endif
};

uECC_Curve uECC_secp256k1(void) { return &curve_secp256k1; }


/* Double in place */
static void double_jacobian_secp256k1(uECC_word_t * X1,
                                      uECC_word_t * Y1,
                                      uECC_word_t * Z1,
                                      uECC_Curve curve) {
    /* t1 = X, t2 = Y, t3 = Z */
    uECC_word_t t4[num_words_secp256k1];
    uECC_word_t t5[num_words_secp256k1];

    if (uECC_vli_isZero(Z1, num_words_secp256k1)) {
        return;
    }

    uECC_vli_modSquare_fast(t5, Y1, curve);   /* t5 = y1^2 */
    uECC_vli_modMult_fast(t4, X1, t5, curve); /* t4 = x1*y1^2 = A */
    uECC_vli_modSquare_fast(X1, X1, curve);   /* t1 = x1^2 */
    uECC_vli_modSquare_fast(t5, t5, curve);   /* t5 = y1^4 */
    uECC_vli_modMult_fast(Z1, Y1, Z1, curve); /* t3 = y1*z1 = z3 */

    uECC_vli_modAdd(Y1, X1, X1, curve->p, num_words_secp256k1); /* t2 = 2*x1^2 */
    uECC_vli_modAdd(Y1, Y1, X1, curve->p, num_words_secp256k1); /* t2 = 3*x1^2 */
    if (uECC_vli_testBit(Y1, 0)) {
        uECC_word_t carry = uECC_vli_add(Y1, Y1, curve->p, num_words_secp256k1);
        uECC_vli_rshift1(Y1, num_words_secp256k1);
        Y1[num_words_secp256k1 - 1] |= carry << (uECC_WORD_BITS - 1);
    } else {
        uECC_vli_rshift1(Y1, num_words_secp256k1);
    }
    /* t2 = 3/2*(x1^2) = B */

    uECC_vli_modSquare_fast(X1, Y1, curve);                     /* t1 = B^2 */
    uECC_vli_modSub(X1, X1, t4, curve->p, num_words_secp256k1); /* t1 = B^2 - A */
    uECC_vli_modSub(X1, X1, t4, curve->p, num_words_secp256k1); /* t1 = B^2 - 2A = x3 */

    uECC_vli_modSub(t4, t4, X1, curve->p, num_words_secp256k1); /* t4 = A - x3 */
    uECC_vli_modMult_fast(Y1, Y1, t4, curve);                   /* t2 = B * (A - x3) */
    uECC_vli_modSub(Y1, Y1, t5, curve->p, num_words_secp256k1); /* t2 = B * (A - x3) - y1^4 = y3 */
}

/* Computes result = x^3 + b. result must not overlap x. */
static void x_side_secp256k1(uECC_word_t *result, const uECC_word_t *x, uECC_Curve curve) {
    uECC_vli_modSquare_fast(result, x, curve);                                /* r = x^2 */
    uECC_vli_modMult_fast(result, result, x, curve);                          /* r = x^3 */
    uECC_vli_modAdd(result, result, curve->b, curve->p, num_words_secp256k1); /* r = x^3 + b */
}

#if uECC_ARM_COMPACT_L1 && (uECC_WORD_SIZE == 4)
/* Smaller, variable-time reduction (level 1 on ARM; see uECC.c). */
static void omega_mult_secp256k1(uECC_word_t *result, const uECC_word_t *right);
static void vli_mmod_fast_secp256k1(uECC_word_t *result, uECC_word_t *product) {
    uECC_word_t tmp[2 * num_words_secp256k1];
    uECC_word_t carry;

    uECC_vli_clear(tmp, num_words_secp256k1);
    uECC_vli_clear(tmp + num_words_secp256k1, num_words_secp256k1);

    omega_mult_secp256k1(tmp, product + num_words_secp256k1); /* (Rq, q) = q * c */

    carry = uECC_vli_add(result, product, tmp, num_words_secp256k1); /* (C, r) = r + q       */
    uECC_vli_clear(product, num_words_secp256k1);
    omega_mult_secp256k1(product, tmp + num_words_secp256k1); /* Rq*c */
    carry += uECC_vli_add(result, result, product, num_words_secp256k1); /* (C1, r) = r + Rq*c */

    while (carry > 0) {
        --carry;
        uECC_vli_sub(result, result, curve_secp256k1.p, num_words_secp256k1);
    }
    if (uECC_vli_cmp_unsafe(result, curve_secp256k1.p, num_words_secp256k1) > 0) {
        uECC_vli_sub(result, result, curve_secp256k1.p, num_words_secp256k1);
    }
}

static void omega_mult_secp256k1(uint32_t * result, const uint32_t * right) {
    /* Multiply by (2^9 + 2^8 + 2^7 + 2^6 + 2^4 + 1). */
    uint32_t carry = 0;
    wordcount_t k;

    for (k = 0; k < num_words_secp256k1; ++k) {
        uint64_t p = (uint64_t)0x3D1 * right[k] + carry;
        result[k] = (uint32_t) p;
        carry = p >> 32;
    }
    result[num_words_secp256k1] = carry;
    /* add the 2^32 multiple */
    result[1 + num_words_secp256k1] =
        uECC_vli_add(result + 1, result + 1, right, num_words_secp256k1);
}
#define asm_mmod_fast_secp256k1 1 /* skip the constant-time C version below */
#endif

#if (uECC_OPTIMIZATION_LEVEL > 0 && !asm_mmod_fast_secp256k1)
#if (uECC_WORD_SIZE == 4)
/* r += c * (2^32 + 977), where 2^256 = 2^32 + 977 (mod p). Returns the carry. */
static int32_t mmod_fold_secp256k1(uint32_t *r, int64_t c) {
    int64_t acc;
    acc = (int64_t)r[0] + c * 977;                       r[0] = (uint32_t)acc; acc >>= 32;
    acc += (int64_t)r[1] + c;                            r[1] = (uint32_t)acc; acc >>= 32;
    acc += r[2];                                         r[2] = (uint32_t)acc; acc >>= 32;
    acc += r[3];                                         r[3] = (uint32_t)acc; acc >>= 32;
    acc += r[4];                                         r[4] = (uint32_t)acc; acc >>= 32;
    acc += r[5];                                         r[5] = (uint32_t)acc; acc >>= 32;
    acc += r[6];                                         r[6] = (uint32_t)acc; acc >>= 32;
    acc += r[7];                                         r[7] = (uint32_t)acc; acc >>= 32;
    return (int32_t)acc;
}

/* result = lo + hi * 977 + hi * 2^32, then MMOD_FINISH. */
static void vli_mmod_fast_secp256k1(uint32_t *result, uint32_t *product) {
    const uint32_t a0 = product[8], a1 = product[9], a2 = product[10],
                   a3 = product[11], a4 = product[12], a5 = product[13],
                   a6 = product[14], a7 = product[15];
    uint64_t acc;
    uint64_t c;

    acc = (uint64_t)product[0] + (uint64_t)a0 * 977;
    result[0] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[1] + (uint64_t)a1 * 977 + a0;
    result[1] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[2] + (uint64_t)a2 * 977 + a1;
    result[2] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[3] + (uint64_t)a3 * 977 + a2;
    result[3] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[4] + (uint64_t)a4 * 977 + a3;
    result[4] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[5] + (uint64_t)a5 * 977 + a4;
    result[5] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[6] + (uint64_t)a6 * 977 + a5;
    result[6] = (uint32_t)acc; acc >>= 32;
    acc += (uint64_t)product[7] + (uint64_t)a7 * 977 + a6;
    result[7] = (uint32_t)acc; acc >>= 32;
    c = acc + a7;

    MMOD_FINISH(mmod_fold_secp256k1, result, c);
}
#elif (uECC_WORD_SIZE == 1)
/* Column i of x * (2^32 + 977) before carries (x has n bytes). 977 = 0x3D1, so
   column i gets x[i] * 0xD1 + x[i - 1] * 0x03 + x[i - 4]. */
#define OMEGA_SECP256K1(x, i, n) \
    (((i) < (n) ? (x)[i] * (uint16_t)0xD1 : 0) + \
     ((i) >= 1 && (i) - 1 < (n) ? (x)[(i) - 1] * (uint16_t)0x03 : 0) + \
     ((i) >= 4 && (i) - 4 < (n) ? (x)[(i) - 4] : 0))

static void vli_mmod_fast_secp256k1(uint8_t *result, uint8_t *product) {
    const uint8_t *hi = product + 32;
    const uint8_t *p = curve_secp256k1.p;
    uint8_t c[5];
    uint16_t acc = 0;
    int16_t cmp = 0;
    uint8_t b;

    /* result = lo + hi * 977 + hi * 2^32 (2^256 = 2^32 + 977 (mod p)); carry -> c. */
    for (b = 0; b < 4; ++b) {
        acc += product[b] + OMEGA_SECP256K1(hi, b, 32);
        result[b] = (uint8_t)acc;
        acc >>= 8;
    }
    for (b = 4; b < 32; ++b) {
        acc += product[b] + hi[b] * (uint16_t)0xD1 + hi[b - 1] * (uint16_t)0x03 + hi[b - 4];
        result[b] = (uint8_t)acc;
        acc >>= 8;
    }
    for (b = 32; b < 37; ++b) {
        acc += OMEGA_SECP256K1(hi, b, 32);
        c[b - 32] = (uint8_t)acc;
        acc >>= 8;
    }

    /* Fold the carry the same way, tracking the borrow of r - p. */
    acc = 0;
    for (b = 0; b < 32; ++b) {
        acc += result[b];
        if (b < 9) {
            acc += OMEGA_SECP256K1(c, b, 5);
        }
        result[b] = (uint8_t)acc;
        acc >>= 8;
        cmp += (int16_t)result[b] - p[b];
        cmp >>= 8;
    }
    vli_mmod_select8(result, (int8_t)acc, (int8_t)cmp, p, 32);
}
#else
static void omega_mult_secp256k1(uECC_word_t *result, const uECC_word_t *right);
static void vli_mmod_fast_secp256k1(uECC_word_t *result, uECC_word_t *product) {
    uECC_word_t tmp[2 * num_words_secp256k1];
    uECC_word_t carry;

    uECC_vli_clear(tmp, num_words_secp256k1);
    uECC_vli_clear(tmp + num_words_secp256k1, num_words_secp256k1);

    omega_mult_secp256k1(tmp, product + num_words_secp256k1); /* (Rq, q) = q * c */

    carry = uECC_vli_add(result, product, tmp, num_words_secp256k1); /* (C, r) = r + q       */
    uECC_vli_clear(product, num_words_secp256k1);
    omega_mult_secp256k1(product, tmp + num_words_secp256k1); /* Rq*c */
    carry += uECC_vli_add(result, result, product, num_words_secp256k1); /* (C1, r) = r + Rq*c */

    while (carry > 0) {
        --carry;
        uECC_vli_sub(result, result, curve_secp256k1.p, num_words_secp256k1);
    }
    if (uECC_vli_cmp_unsafe(result, curve_secp256k1.p, num_words_secp256k1) > 0) {
        uECC_vli_sub(result, result, curve_secp256k1.p, num_words_secp256k1);
    }
}

#if uECC_WORD_SIZE == 8
static void omega_mult_secp256k1(uint64_t * result, const uint64_t * right) {
    uECC_word_t r0 = 0;
    uECC_word_t r1 = 0;
    uECC_word_t r2 = 0;
    wordcount_t k;

    /* Multiply by (2^32 + 2^9 + 2^8 + 2^7 + 2^6 + 2^4 + 1). */
    for (k = 0; k < num_words_secp256k1; ++k) {
        muladd(0x1000003D1ull, right[k], &r0, &r1, &r2);
        result[k] = r0;
        r0 = r1;
        r1 = r2;
        r2 = 0;
    }
    result[num_words_secp256k1] = r0;
}
#endif /* uECC_WORD_SIZE */
#endif /* uECC_WORD_SIZE */
#endif /* (uECC_OPTIMIZATION_LEVEL > 0 &&  && !asm_mmod_fast_secp256k1) */

#endif /* uECC_SUPPORTS_secp256k1 */

#endif /* _UECC_CURVE_SPECIFIC_H_ */
