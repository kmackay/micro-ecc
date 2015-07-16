/* Copyright 2014, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#include "uECC.h"

#define MAX_TRIES 64

#include "platform-specific.inc"

#if (uECC_WORD_SIZE == 1)
    #define uECC_MAX_WORDS 32
#elif (uECC_WORD_SIZE == 4)
    #define uECC_MAX_WORDS 8
#elif (uECC_WORD_SIZE == 8)
    #define uECC_MAX_WORDS 4
#endif /* uECC_WORD_SIZE */

struct uECC_Curve_t {
    wordcount_t num_words;
    wordcount_t num_n_words;
    wordcount_t num_bytes;
    uECC_word_t p[uECC_MAX_WORDS];
    uECC_word_t n[uECC_MAX_WORDS];
    uECC_word_t G[uECC_MAX_WORDS * 2];
    uECC_word_t b[uECC_MAX_WORDS];
    void (*double_jacobian)(uECC_word_t * X1,
                            uECC_word_t * Y1,
                            uECC_word_t * Z1,
                            uECC_Curve curve);
    void (*mod_sqrt)(uECC_word_t *a, uECC_Curve curve);
    void (*x_side)(uECC_word_t *result, const uECC_word_t *x, uECC_Curve curve);
};

static void double_jacobian_secp256k1(uECC_word_t * X1,
                                      uECC_word_t * Y1,
                                      uECC_word_t * Z1,
                                      uECC_Curve curve);
static void double_jacobian_default(uECC_word_t * X1,
                                    uECC_word_t * Y1,
                                    uECC_word_t * Z1,
                                    uECC_Curve curve);

static void mod_sqrt_default(uECC_word_t *a, uECC_Curve curve);
static void mod_sqrt_secp224r1(uECC_word_t *a, uECC_Curve curve);

static void x_side_default(uECC_word_t *result, const uECC_word_t *x, uECC_Curve curve);
static void x_side_secp256k1(uECC_word_t *result, const uECC_word_t *x, uECC_Curve curve);

#include "curve-specific.inc"

static uECC_RNG_Function g_rng_function = &default_RNG;

void uECC_set_rng(uECC_RNG_Function rng_function) {
    g_rng_function = rng_function;
}

static void vli_clear(uECC_word_t *vli, const wordcount_t num_words) {
    wordcount_t i;
    for (i = 0; i < num_words; ++i) {
        vli[i] = 0;
    }
}

/* Returns 1 if vli == 0, 0 otherwise. */
static uECC_word_t vli_isZero(const uECC_word_t *vli, const wordcount_t num_words) {
    wordcount_t i;
    for (i = 0; i < num_words; ++i) {
        if (vli[i]) {
            return 0;
        }
    }
    return 1;
}

/* Returns nonzero if bit 'bit' of vli is set. */
static uECC_word_t vli_testBit(const uECC_word_t *vli, bitcount_t bit) {
    return (vli[bit >> uECC_WORD_BITS_SHIFT] & ((uECC_word_t)1 << (bit & uECC_WORD_BITS_MASK)));
}

/* Counts the number of words in vli. */
static wordcount_t vli_numDigits(const uECC_word_t *vli, const wordcount_t max_words) {
    swordcount_t i;
    /* Search from the end until we find a non-zero digit.
       We do it in reverse because we expect that most digits will be nonzero. */
    for (i = max_words - 1; i >= 0 && vli[i] == 0; --i) {
    }

    return (i + 1);
}

/* Counts the number of bits required to represent vli. */
static bitcount_t vli_numBits(const uECC_word_t *vli, const wordcount_t max_words) {
    uECC_word_t i;
    uECC_word_t digit;

    wordcount_t num_digits = vli_numDigits(vli, max_words);
    if (num_digits == 0) {
        return 0;
    }

    digit = vli[num_digits - 1];
    for (i = 0; digit; ++i) {
        digit >>= 1;
    }

    return (((bitcount_t)(num_digits - 1) << uECC_WORD_BITS_SHIFT) + i);
}

/* Sets dest = src. */
static void vli_set(uECC_word_t *dest, const uECC_word_t *src, const wordcount_t num_words) {
    wordcount_t i;
    for (i = 0; i < num_words; ++i) {
        dest[i] = src[i];
    }
}

/* Returns sign of left - right. */
static cmpresult_t vli_cmp(const uECC_word_t *left,
                           const uECC_word_t *right,
                           const wordcount_t num_words) {
    swordcount_t i;
    for (i = num_words - 1; i >= 0; --i) {
        if (left[i] > right[i]) {
            return 1;
        } else if (left[i] < right[i]) {
            return -1;
        }
    }
    return 0;
}

/* Computes vli = vli >> 1. */
static void vli_rshift1(uECC_word_t *vli, const wordcount_t num_words) {
    uECC_word_t *end = vli;
    uECC_word_t carry = 0;
    
    vli += num_words;
    while (vli-- > end) {
        uECC_word_t temp = *vli;
        *vli = (temp >> 1) | carry;
        carry = temp << (uECC_WORD_BITS - 1);
    }
}

/* Computes result = left + right, returning carry. Can modify in place. */
static uECC_word_t vli_add(uECC_word_t *result,
                           const uECC_word_t *left,
                           const uECC_word_t *right,
                           const wordcount_t num_words) {
    uECC_word_t carry = 0;
    wordcount_t i;
    for (i = 0; i < num_words; ++i) {
        uECC_word_t sum = left[i] + right[i] + carry;
        if (sum != left[i]) {
            carry = (sum < left[i]);
        }
        result[i] = sum;
    }
    return carry;
}

/* Computes result = left - right, returning borrow. Can modify in place. */
static uECC_word_t vli_sub(uECC_word_t *result,
                           const uECC_word_t *left,
                           const uECC_word_t *right,
                           const wordcount_t num_words) {
    uECC_word_t borrow = 0;
    wordcount_t i;
    for (i = 0; i < num_words; ++i) {
        uECC_word_t diff = left[i] - right[i] - borrow;
        if (diff != left[i]) {
            borrow = (diff > left[i]);
        }
        result[i] = diff;
    }
    return borrow;
}

static void muladd(uECC_word_t a,
                   uECC_word_t b,
                   uECC_word_t *r0,
                   uECC_word_t *r1,
                   uECC_word_t *r2) {
#if uECC_WORD_SIZE == 8 && !SUPPORTS_INT128
    uint64_t a0 = a & 0xffffffffull;
    uint64_t a1 = a >> 32;
    uint64_t b0 = b & 0xffffffffull;
    uint64_t b1 = b >> 32;

    uint64_t i0 = a0 * b0;
    uint64_t i1 = a0 * b1;
    uint64_t i2 = a1 * b0;
    uint64_t i3 = a1 * b1;

    uint64_t p0, p1;

    i2 += (i0 >> 32);
    i2 += i1;
    if (i2 < i1) { // overflow
        i3 += 0x100000000ull;
    }

    p0 = (i0 & 0xffffffffull) | (i2 << 32);
    p1 = i3 + (i2 >> 32);

    *r0 += p0;
    *r1 += (p1 + (*r0 < p0));
    *r2 += ((*r1 < p1) || (*r1 == p1 && *r0 < p0));
#else
    uECC_dword_t p = (uECC_dword_t)a * b;
    uECC_dword_t r01 = ((uECC_dword_t)(*r1) << uECC_WORD_BITS) | *r0;
    r01 += p;
    *r2 += (r01 < p);
    *r1 = r01 >> uECC_WORD_BITS;
    *r0 = (uECC_word_t)r01;
#endif
}

static void vli_mult(uECC_word_t *result,
                     const uECC_word_t *left,
                     const uECC_word_t *right,
                     const wordcount_t num_words) {
    uECC_word_t r0 = 0;
    uECC_word_t r1 = 0;
    uECC_word_t r2 = 0;
    wordcount_t i, k;

    /* Compute each digit of result in sequence, maintaining the carries. */
    for (k = 0; k < num_words; ++k) {
        for (i = 0; i <= k; ++i) {
            muladd(left[i], right[k - i], &r0, &r1, &r2);
        }
        result[k] = r0;
        r0 = r1;
        r1 = r2;
        r2 = 0;
    }
    for (k = num_words; k < num_words * 2 - 1; ++k) {
        for (i = (k + 1) - num_words; i < num_words; ++i) {
            muladd(left[i], right[k - i], &r0, &r1, &r2);
        }
        result[k] = r0;
        r0 = r1;
        r1 = r2;
        r2 = 0;
    }
    result[num_words * 2 - 1] = r0;
}

#if uECC_SQUARE_FUNC

static void mul2add(uECC_word_t a,
                    uECC_word_t b,
                    uECC_word_t *r0,
                    uECC_word_t *r1,
                    uECC_word_t *r2) {
#if uECC_WORD_SIZE == 8 && !SUPPORTS_INT128
    uint64_t a0 = a & 0xffffffffull;
    uint64_t a1 = a >> 32;
    uint64_t b0 = b & 0xffffffffull;
    uint64_t b1 = b >> 32;

    uint64_t i0 = a0 * b0;
    uint64_t i1 = a0 * b1;
    uint64_t i2 = a1 * b0;
    uint64_t i3 = a1 * b1;

    uint64_t p0, p1;

    i2 += (i0 >> 32);
    i2 += i1;
    if (i2 < i1)
    { // overflow
        i3 += 0x100000000ull;
    }

    p0 = (i0 & 0xffffffffull) | (i2 << 32);
    p1 = i3 + (i2 >> 32);

    *r2 += (p1 >> 63);
    p1 = (p1 << 1) | (p0 >> 63);
    p0 <<= 1;

    *r0 += p0;
    *r1 += (p1 + (*r0 < p0));
    *r2 += ((*r1 < p1) || (*r1 == p1 && *r0 < p0));
#else
    uECC_dword_t p = (uECC_dword_t)a * b;
    uECC_dword_t r01 = ((uECC_dword_t)(*r1) << uECC_WORD_BITS) | *r0;
    *r2 += (p >> (uECC_WORD_BITS * 2 - 1));
    p *= 2;
    r01 += p;
    *r2 += (r01 < p);
    *r1 = r01 >> uECC_WORD_BITS;
    *r0 = (uECC_word_t)r01;
#endif
}

static void vli_square(uECC_word_t *result, const uECC_word_t *left, const wordcount_t num_words) {
    uECC_word_t r0 = 0;
    uECC_word_t r1 = 0;
    uECC_word_t r2 = 0;

    wordcount_t i, k;
    
    for (k = 0; k < num_words * 2 - 1; ++k) {
        uECC_word_t min = (k < num_words ? 0 : (k + 1) - num_words);
        for (i = min; i <= k && i <= k - i; ++i) {
            if (i < k-i) {
                mul2add(left[i], left[k - i], &r0, &r1, &r2);
            } else {
                muladd(left[i], left[k - i], &r0, &r1, &r2);
            }
        }
        result[k] = r0;
        r0 = r1;
        r1 = r2;
        r2 = 0;
    }
    
    result[num_words * 2 - 1] = r0;
}

#else /* uECC_SQUARE_FUNC */

#define vli_square(result, left, size) vli_mult((result), (left), (left), (size))

#endif /* uECC_SQUARE_FUNC */

/* Computes result = (left + right) % mod.
   Assumes that left < mod and right < mod, and that result does not overlap mod. */
static void vli_modAdd(uECC_word_t *result,
                       const uECC_word_t *left,
                       const uECC_word_t *right,
                       const uECC_word_t *mod,
                       const wordcount_t num_words) {
    uECC_word_t carry = vli_add(result, left, right, num_words);
    if (carry || vli_cmp(result, mod, num_words) >= 0) {
        /* result > mod (result = mod + remainder), so subtract mod to get remainder. */
        vli_sub(result, result, mod, num_words);
    }
}

/* Computes result = (left - right) % mod.
   Assumes that left < mod and right < mod, and that result does not overlap mod. */
static void vli_modSub(uECC_word_t *result,
                       const uECC_word_t *left,
                       const uECC_word_t *right,
                       const uECC_word_t *mod,
                       const wordcount_t num_words) {
    uECC_word_t l_borrow = vli_sub(result, left, right, num_words);
    if (l_borrow) {
        /* In this case, result == -diff == (max int) - diff. Since -x % d == d - x,
           we can get the correct result from result + mod (with overflow). */
        vli_add(result, result, mod, num_words);
    }
}

/* Computes result = product % mod, where product is 2N words long. */
/* Currently only designed to work for curve_p or curve_n. */
static void vli_mmod(uECC_word_t *result,
                     uECC_word_t *product,
                     const uECC_word_t *mod,
                     const wordcount_t num_words) {
    uECC_word_t mod_multiple[2 * uECC_MAX_WORDS];
    uECC_word_t tmp[2 * uECC_MAX_WORDS];
    uECC_word_t *v[2] = {tmp, product};
    uECC_word_t index;
    
    // Shift mod so its highest set bit is at the maximum position.
    bitcount_t shift = (num_words * 2 * uECC_WORD_BITS) - vli_numBits(mod, num_words);
    wordcount_t word_shift = shift / uECC_WORD_BITS;
    wordcount_t bit_shift = shift % uECC_WORD_BITS;
    uECC_word_t carry = 0;
    vli_clear(mod_multiple, word_shift);
    if (bit_shift > 0) {
        for(index = 0; index < num_words; ++index) {
            mod_multiple[word_shift + index] = (mod[index] << bit_shift) | carry;
            carry = mod[index] >> (uECC_WORD_BITS - bit_shift);
        }
    } else {
        vli_set(mod_multiple + word_shift, mod, num_words);
    }

    for (index = 1; shift >= 0; --shift) {
        uECC_word_t borrow = vli_sub(v[1 - index], v[index], mod_multiple, num_words * 2);
        index = !(index ^ borrow); /* Swap the index if there was no borrow */
        vli_rshift1(mod_multiple, num_words * 2);
    }
    vli_set(result, v[index], num_words);
}

/* Computes result = (left * right) % mod. */
static void vli_modMult(uECC_word_t *result,
                        const uECC_word_t *left,
                        const uECC_word_t *right,
                        const uECC_word_t *mod,
                        const wordcount_t num_words) {
    uECC_word_t product[2 * uECC_MAX_WORDS];
    vli_mult(product, left, right, num_words);
    vli_mmod(result, product, mod, num_words);
}

#if uECC_SQUARE_FUNC

/* Computes result = left^2 % curve_p. */
static void vli_modSquare(uECC_word_t *result,
                          const uECC_word_t *left,
                          const uECC_word_t *mod,
                          const wordcount_t num_words) {
    uECC_word_t product[2 * uECC_MAX_WORDS];
    vli_square(product, left, num_words);
    vli_mmod(result, product, mod, num_words);
}

#else /* uECC_SQUARE_FUNC */

#define vli_modSquare(result, left, mod, num_words) \
    vli_modMult((result), (left), (left), (mod), (num_words))
    
#endif /* uECC_SQUARE_FUNC */

#define EVEN(vli) (!(vli[0] & 1))
/* Computes result = (1 / input) % mod. All VLIs are the same size.
   See "From Euclid's GCD to Montgomery Multiplication to the Great Divide" */
#if !asm_modInv

static void vli_modInv_update(uECC_word_t *uv,
                              const uECC_word_t *mod,
                              const wordcount_t num_words) {
    uECC_word_t carry = 0;
    if (!EVEN(uv)) {
        carry = vli_add(uv, uv, mod, num_words);
    }
    vli_rshift1(uv, num_words);
    if (carry) {
        uv[num_words - 1] |= HIGH_BIT_SET;
    }
}

static void vli_modInv(uECC_word_t *result,
                       const uECC_word_t *input,
                       const uECC_word_t *mod,
                       const wordcount_t num_words) {
    uECC_word_t a[uECC_MAX_WORDS], b[uECC_MAX_WORDS], u[uECC_MAX_WORDS], v[uECC_MAX_WORDS];
    cmpresult_t cmpResult;
    
    if (vli_isZero(input, num_words)) {
        vli_clear(result, num_words);
        return;
    }

    vli_set(a, input, num_words);
    vli_set(b, mod, num_words);
    vli_clear(u, num_words);
    u[0] = 1;
    vli_clear(v, num_words);
    while ((cmpResult = vli_cmp(a, b, num_words)) != 0) {
        if (EVEN(a)) {
            vli_rshift1(a, num_words);
            vli_modInv_update(u, mod, num_words);
        } else if (EVEN(b)) {
            vli_rshift1(b, num_words);
            vli_modInv_update(v, mod, num_words);
        } else if (cmpResult > 0) {
            vli_sub(a, a, b, num_words);
            vli_rshift1(a, num_words);
            if (vli_cmp(u, v, num_words) < 0) {
                vli_add(u, u, mod, num_words);
            }
            vli_sub(u, u, v, num_words);
            vli_modInv_update(u, mod, num_words);
        } else {
            vli_sub(b, b, a, num_words);
            vli_rshift1(b, num_words);
            if (vli_cmp(v, u, num_words) < 0) {
                vli_add(v, v, mod, num_words);
            }
            vli_sub(v, v, u, num_words);
            vli_modInv_update(v, mod, num_words);
        }
    }
    vli_set(result, u, num_words);
}
#endif /* !asm_modInv */

/* ------ Point operations ------ */

/* Returns 1 if 'point' is the point at infinity, 0 otherwise. */
static cmpresult_t EccPoint_isZero(const uECC_word_t *point, uECC_Curve curve) {
    return vli_isZero(point, curve->num_words * 2);
}

/* Point multiplication algorithm using Montgomery's ladder with co-Z coordinates.
From http://eprint.iacr.org/2011/338.pdf
*/

/* Double in place */
static void double_jacobian_secp256k1(uECC_word_t * X1,
                                      uECC_word_t * Y1,
                                      uECC_word_t * Z1,
                                      uECC_Curve curve) {
    /* t1 = X, t2 = Y, t3 = Z */
    uECC_word_t t4[uECC_MAX_WORDS];
    uECC_word_t t5[uECC_MAX_WORDS];
    
    if (vli_isZero(Z1, curve->num_words)) {
        return;
    }
    
    vli_modSquare(t5, Y1, curve->p, curve->num_words);   /* t5 = y1^2 */
    vli_modMult(t4, X1, t5, curve->p, curve->num_words); /* t4 = x1*y1^2 = A */
    vli_modSquare(X1, X1, curve->p, curve->num_words);   /* t1 = x1^2 */
    vli_modSquare(t5, t5, curve->p, curve->num_words);   /* t5 = y1^4 */
    vli_modMult(Z1, Y1, Z1, curve->p, curve->num_words); /* t3 = y1*z1 = z3 */
    
    vli_modAdd(Y1, X1, X1, curve->p, curve->num_words); /* t2 = 2*x1^2 */
    vli_modAdd(Y1, Y1, X1, curve->p, curve->num_words); /* t2 = 3*x1^2 */
    if (vli_testBit(Y1, 0)) {
        uECC_word_t carry = vli_add(Y1, Y1, curve->p, curve->num_words);
        vli_rshift1(Y1, curve->num_words);
        Y1[curve->num_words - 1] |= carry << (uECC_WORD_BITS - 1);
    } else {
        vli_rshift1(Y1, curve->num_words);
    }
    /* t2 = 3/2*(x1^2) = B */
    
    vli_modSquare(X1, Y1, curve->p, curve->num_words);  /* t1 = B^2 */
    vli_modSub(X1, X1, t4, curve->p, curve->num_words); /* t1 = B^2 - A */
    vli_modSub(X1, X1, t4, curve->p, curve->num_words); /* t1 = B^2 - 2A = x3 */
    
    vli_modSub(t4, t4, X1, curve->p, curve->num_words);  /* t4 = A - x3 */
    vli_modMult(Y1, Y1, t4, curve->p, curve->num_words); /* t2 = B * (A - x3) */
    vli_modSub(Y1, Y1, t5, curve->p, curve->num_words);  /* t2 = B * (A - x3) - y1^4 = y3 */
}

static void double_jacobian_default(uECC_word_t * X1,
                                    uECC_word_t * Y1,
                                    uECC_word_t * Z1,
                                    uECC_Curve curve) {
    /* t1 = X, t2 = Y, t3 = Z */
    uECC_word_t t4[uECC_MAX_WORDS];
    uECC_word_t t5[uECC_MAX_WORDS];
    
    if (vli_isZero(Z1, curve->num_words)) {
        return;
    }
    
    vli_modSquare(t4, Y1, curve->p, curve->num_words);   /* t4 = y1^2 */
    vli_modMult(t5, X1, t4, curve->p, curve->num_words); /* t5 = x1*y1^2 = A */
    vli_modSquare(t4, t4, curve->p, curve->num_words);   /* t4 = y1^4 */
    vli_modMult(Y1, Y1, Z1, curve->p, curve->num_words); /* t2 = y1*z1 = z3 */
    vli_modSquare(Z1, Z1, curve->p, curve->num_words);   /* t3 = z1^2 */
    
    vli_modAdd(X1, X1, Z1, curve->p, curve->num_words);  /* t1 = x1 + z1^2 */
    vli_modAdd(Z1, Z1, Z1, curve->p, curve->num_words);  /* t3 = 2*z1^2 */
    vli_modSub(Z1, X1, Z1, curve->p, curve->num_words);  /* t3 = x1 - z1^2 */
    vli_modMult(X1, X1, Z1, curve->p, curve->num_words); /* t1 = x1^2 - z1^4 */
    
    vli_modAdd(Z1, X1, X1, curve->p, curve->num_words); /* t3 = 2*(x1^2 - z1^4) */
    vli_modAdd(X1, X1, Z1, curve->p, curve->num_words); /* t1 = 3*(x1^2 - z1^4) */
    if (vli_testBit(X1, 0)) {
        uECC_word_t l_carry = vli_add(X1, X1, curve->p, curve->num_words);
        vli_rshift1(X1, curve->num_words);
        X1[curve->num_words - 1] |= l_carry << (uECC_WORD_BITS - 1);
    } else {
        vli_rshift1(X1, curve->num_words);
    }
    /* t1 = 3/2*(x1^2 - z1^4) = B */
    
    vli_modSquare(Z1, X1, curve->p, curve->num_words);   /* t3 = B^2 */
    vli_modSub(Z1, Z1, t5, curve->p, curve->num_words);  /* t3 = B^2 - A */
    vli_modSub(Z1, Z1, t5, curve->p, curve->num_words);  /* t3 = B^2 - 2A = x3 */
    vli_modSub(t5, t5, Z1, curve->p, curve->num_words);  /* t5 = A - x3 */
    vli_modMult(X1, X1, t5, curve->p, curve->num_words); /* t1 = B * (A - x3) */
    vli_modSub(t4, X1, t4, curve->p, curve->num_words);  /* t4 = B * (A - x3) - y1^4 = y3 */
    
    vli_set(X1, Z1, curve->num_words);
    vli_set(Z1, Y1, curve->num_words);
    vli_set(Y1, t4, curve->num_words);
}

/* Modify (x1, y1) => (x1 * z^2, y1 * z^3) */
static void apply_z(uECC_word_t * X1,
                    uECC_word_t * Y1,
                    const uECC_word_t * const Z,
                    uECC_Curve curve) {
    uECC_word_t t1[uECC_MAX_WORDS];

    vli_modSquare(t1, Z, curve->p, curve->num_words);    /* z^2 */
    vli_modMult(X1, X1, t1, curve->p, curve->num_words); /* x1 * z^2 */
    vli_modMult(t1, t1, Z, curve->p, curve->num_words);  /* z^3 */
    vli_modMult(Y1, Y1, t1, curve->p, curve->num_words); /* y1 * z^3 */
}

/* P = (x1, y1) => 2P, (x2, y2) => P' */
static void XYcZ_initial_double(uECC_word_t * X1,
                                uECC_word_t * Y1,
                                uECC_word_t * X2,
                                uECC_word_t * Y2,
                                const uECC_word_t * const initial_Z,
                                uECC_Curve curve) {
    uECC_word_t z[uECC_MAX_WORDS];
    if (initial_Z) {
        vli_set(z, initial_Z, curve->num_words);
    } else {
        vli_clear(z, curve->num_words);
        z[0] = 1;
    }

    vli_set(X2, X1, curve->num_words);
    vli_set(Y2, Y1, curve->num_words);

    apply_z(X1, Y1, z, curve);
    curve->double_jacobian(X1, Y1, z, curve);
    apply_z(X2, Y2, z, curve);
}

/* Input P = (x1, y1, Z), Q = (x2, y2, Z)
   Output P' = (x1', y1', Z3), P + Q = (x3, y3, Z3)
   or P => P', Q => P + Q
*/
static void XYcZ_add(uECC_word_t * X1,
                     uECC_word_t * Y1,
                     uECC_word_t * X2,
                     uECC_word_t * Y2,
                     uECC_Curve curve) {
    /* t1 = X1, t2 = Y1, t3 = X2, t4 = Y2 */
    uECC_word_t t5[uECC_MAX_WORDS];
    
    vli_modSub(t5, X2, X1, curve->p, curve->num_words);  /* t5 = x2 - x1 */
    vli_modSquare(t5, t5, curve->p, curve->num_words);   /* t5 = (x2 - x1)^2 = A */
    vli_modMult(X1, X1, t5, curve->p, curve->num_words); /* t1 = x1*A = B */
    vli_modMult(X2, X2, t5, curve->p, curve->num_words); /* t3 = x2*A = C */
    vli_modSub(Y2, Y2, Y1, curve->p, curve->num_words);  /* t4 = y2 - y1 */
    vli_modSquare(t5, Y2, curve->p, curve->num_words);   /* t5 = (y2 - y1)^2 = D */
    
    vli_modSub(t5, t5, X1, curve->p, curve->num_words);  /* t5 = D - B */
    vli_modSub(t5, t5, X2, curve->p, curve->num_words);  /* t5 = D - B - C = x3 */
    vli_modSub(X2, X2, X1, curve->p, curve->num_words);  /* t3 = C - B */
    vli_modMult(Y1, Y1, X2, curve->p, curve->num_words); /* t2 = y1*(C - B) */
    vli_modSub(X2, X1, t5, curve->p, curve->num_words);  /* t3 = B - x3 */
    vli_modMult(Y2, Y2, X2, curve->p, curve->num_words); /* t4 = (y2 - y1)*(B - x3) */
    vli_modSub(Y2, Y2, Y1, curve->p, curve->num_words);  /* t4 = y3 */
    
    vli_set(X2, t5, curve->num_words);
}

/* Input P = (x1, y1, Z), Q = (x2, y2, Z)
   Output P + Q = (x3, y3, Z3), P - Q = (x3', y3', Z3)
   or P => P - Q, Q => P + Q
*/
static void XYcZ_addC(uECC_word_t * X1,
                      uECC_word_t * Y1,
                      uECC_word_t * X2,
                      uECC_word_t * Y2,
                      uECC_Curve curve) {
    /* t1 = X1, t2 = Y1, t3 = X2, t4 = Y2 */
    uECC_word_t t5[uECC_MAX_WORDS];
    uECC_word_t t6[uECC_MAX_WORDS];
    uECC_word_t t7[uECC_MAX_WORDS];
    
    vli_modSub(t5, X2, X1, curve->p, curve->num_words);  /* t5 = x2 - x1 */
    vli_modSquare(t5, t5, curve->p, curve->num_words);   /* t5 = (x2 - x1)^2 = A */
    vli_modMult(X1, X1, t5, curve->p, curve->num_words); /* t1 = x1*A = B */
    vli_modMult(X2, X2, t5, curve->p, curve->num_words); /* t3 = x2*A = C */
    vli_modAdd(t5, Y2, Y1, curve->p, curve->num_words);  /* t5 = y2 + y1 */
    vli_modSub(Y2, Y2, Y1, curve->p, curve->num_words);  /* t4 = y2 - y1 */

    vli_modSub(t6, X2, X1, curve->p, curve->num_words);  /* t6 = C - B */
    vli_modMult(Y1, Y1, t6, curve->p, curve->num_words); /* t2 = y1 * (C - B) = E */
    vli_modAdd(t6, X1, X2, curve->p, curve->num_words);  /* t6 = B + C */
    vli_modSquare(X2, Y2, curve->p, curve->num_words);   /* t3 = (y2 - y1)^2 = D */
    vli_modSub(X2, X2, t6, curve->p, curve->num_words);  /* t3 = D - (B + C) = x3 */
    
    vli_modSub(t7, X1, X2, curve->p, curve->num_words);  /* t7 = B - x3 */
    vli_modMult(Y2, Y2, t7, curve->p, curve->num_words); /* t4 = (y2 - y1)*(B - x3) */
    vli_modSub(Y2, Y2, Y1, curve->p, curve->num_words);  /* t4 = (y2 - y1)*(B - x3) - E = y3 */
    
    vli_modSquare(t7, t5, curve->p, curve->num_words);   /* t7 = (y2 + y1)^2 = F */
    vli_modSub(t7, t7, t6, curve->p, curve->num_words);  /* t7 = F - (B + C) = x3' */
    vli_modSub(t6, t7, X1, curve->p, curve->num_words);  /* t6 = x3' - B */
    vli_modMult(t6, t6, t5, curve->p, curve->num_words); /* t6 = (y2 + y1)*(x3' - B) */
    vli_modSub(Y1, t6, Y1, curve->p, curve->num_words);  /* t2 = (y2 + y1)*(x3' - B) - E = y3' */
    
    vli_set(X1, t7, curve->num_words);
}

/* result may overlap point. */
static void EccPoint_mult(uECC_word_t * result,
                          const uECC_word_t * point,
                          const uECC_word_t * scalar,
                          const uECC_word_t * initial_Z,
                          bitcount_t num_bits,
                          uECC_Curve curve) {
    /* R0 and R1 */
    uECC_word_t Rx[2][uECC_MAX_WORDS];
    uECC_word_t Ry[2][uECC_MAX_WORDS];
    uECC_word_t z[uECC_MAX_WORDS];
    bitcount_t i;
    uECC_word_t nb;
    
    vli_set(Rx[1], point, curve->num_words);
    vli_set(Ry[1], point + curve->num_words, curve->num_words);

    XYcZ_initial_double(Rx[1], Ry[1], Rx[0], Ry[0], initial_Z, curve);

    for (i = num_bits - 2; i > 0; --i) {
        nb = !vli_testBit(scalar, i);
        XYcZ_addC(Rx[1 - nb], Ry[1 - nb], Rx[nb], Ry[nb], curve);
        XYcZ_add(Rx[nb], Ry[nb], Rx[1 - nb], Ry[1 - nb], curve);
    }

    nb = !vli_testBit(scalar, 0);
    XYcZ_addC(Rx[1 - nb], Ry[1 - nb], Rx[nb], Ry[nb], curve);
    
    /* Find final 1/Z value. */
    vli_modSub(z, Rx[1], Rx[0], curve->p, curve->num_words);   /* X1 - X0 */
    vli_modMult(z, z, Ry[1 - nb], curve->p, curve->num_words); /* Yb * (X1 - X0) */
    vli_modMult(z, z, point, curve->p, curve->num_words); /* xP * Yb * (X1 - X0) */
    vli_modInv(z, z, curve->p, curve->num_words);          /* 1 / (xP * Yb * (X1 - X0)) */
    /* yP / (xP * Yb * (X1 - X0)) */
    vli_modMult(z, z, point + curve->num_words, curve->p, curve->num_words); 
    vli_modMult(z, z, Rx[1 - nb], curve->p, curve->num_words); /* Xb * yP / (xP * Yb * (X1 - X0)) */
    /* End 1/Z calculation */

    XYcZ_add(Rx[nb], Ry[nb], Rx[1 - nb], Ry[1 - nb], curve);
    apply_z(Rx[0], Ry[0], z, curve);
    
    vli_set(result, Rx[0], curve->num_words);
    vli_set(result + curve->num_words, Ry[0], curve->num_words);
}

static uECC_word_t regularize_k(const uECC_word_t * const k,
                                uECC_word_t *k0,
                                uECC_word_t *k1,
                                uECC_Curve curve) {
    bitcount_t num_bits = vli_numBits(curve->n, curve->num_n_words);
    uECC_word_t carry = vli_add(k0, k, curve->n, curve->num_n_words) ||
        (num_bits < ((bitcount_t)curve->num_n_words * uECC_WORD_SIZE * 8) &&
         vli_testBit(k0, num_bits));
    vli_add(k1, k0, curve->n, curve->num_n_words);
    return carry;
}

static uECC_word_t EccPoint_compute_public_key(uECC_word_t *result,
                                               uECC_word_t *private,
                                               uECC_Curve curve) {
    uECC_word_t tmp1[uECC_MAX_WORDS];
    uECC_word_t tmp2[uECC_MAX_WORDS];
    uECC_word_t *p2[2] = {tmp1, tmp2};
    uECC_word_t carry;

    /* Make sure the private key is in the range [1, n-1]. */
    if (vli_isZero(private, curve->num_words)) {
        return 0;
    }

    if (vli_cmp(curve->n, private, curve->num_n_words) != 1) {
        return 0;
    }
    
    // Regularize the bitcount for the private key so that attackers cannot use a side channel
    // attack to learn the number of leading zeros.
    carry = regularize_k(private, tmp1, tmp2, curve);

    EccPoint_mult(result, curve->G, p2[!carry], 0,
                  vli_numBits(curve->n, curve->num_n_words) + 1,
                  curve);

    if (EccPoint_isZero(result, curve)) {
        return 0;
    }
    return 1;
}

/* Compute a = sqrt(a) (mod curve_p). */
static void mod_sqrt_default(uECC_word_t *a, uECC_Curve curve) {
    bitcount_t i;
    uECC_word_t p1[uECC_MAX_WORDS] = {1};
    uECC_word_t l_result[uECC_MAX_WORDS] = {1};
    
    /* When curve->p == 3 (mod 4), we can compute
       sqrt(a) = a^((curve->p + 1) / 4) (mod curve->p). */
    vli_add(p1, curve->p, p1, curve->num_words); /* p1 = curve_p + 1 */
    for (i = vli_numBits(p1, curve->num_words) - 1; i > 1; --i) {
        vli_modSquare(l_result, l_result, curve->p, curve->num_words);
        if (vli_testBit(p1, i)) {
            vli_modMult(l_result, l_result, a, curve->p, curve->num_words);
        }
    }
    vli_set(a, l_result, curve->num_words);
}

/* Routine 3.2.4 RS;  from http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void mod_sqrt_secp224r1_rs(uECC_word_t *d1,
                                  uECC_word_t *e1,
                                  uECC_word_t *f1,
                                  const uECC_word_t *d0,
                                  const uECC_word_t *e0,
                                  const uECC_word_t *f0) {
    uECC_word_t t[uECC_MAX_WORDS];

    vli_modSquare(t, d0, curve_secp224r1.p, num_words_secp224r1);    /* t <-- d0 ^ 2 */
    vli_modMult(e1, d0, e0, curve_secp224r1.p, num_words_secp224r1); /* e1 <-- d0 * e0 */
    vli_modAdd(d1, t, f0, curve_secp224r1.p, num_words_secp224r1);   /* d1 <-- t  + f0 */
    vli_modAdd(e1, e1, e1, curve_secp224r1.p, num_words_secp224r1);  /* e1 <-- e1 + e1 */
    vli_modMult(f1, t, f0, curve_secp224r1.p, num_words_secp224r1);  /* f1 <-- t  * f0 */
    vli_modAdd(f1, f1, f1, curve_secp224r1.p, num_words_secp224r1);  /* f1 <-- f1 + f1 */
    vli_modAdd(f1, f1, f1, curve_secp224r1.p, num_words_secp224r1);  /* f1 <-- f1 + f1 */
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

    vli_set(d1, d0, num_words_secp224r1); /* d1 <-- d0 */
    vli_set(e1, e0, num_words_secp224r1); /* e1 <-- e0 */
    vli_set(f1, f0, num_words_secp224r1); /* f1 <-- f0 */
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
    uECC_word_t t1[uECC_MAX_WORDS];
    uECC_word_t t2[uECC_MAX_WORDS];

    vli_modMult(t1, e0, e1, curve_secp224r1.p, num_words_secp224r1);     /* t1 <-- e0 * e1 */
    vli_modMult(t1, t1, c, curve_secp224r1.p, num_words_secp224r1);      /* t1 <-- t1 * c */
    /* t1 <-- p  - t1 */
    vli_modSub(t1, curve_secp224r1.p, t1, curve_secp224r1.p, num_words_secp224r1);
    vli_modMult(t2, d0, d1, curve_secp224r1.p, num_words_secp224r1);     /* t2 <-- d0 * d1 */
    vli_modAdd(t2, t2, t1, curve_secp224r1.p, num_words_secp224r1);      /* t2 <-- t2 + t1 */
    vli_modMult(t1, d0, e1, curve_secp224r1.p, num_words_secp224r1);     /* t1 <-- d0 * e1 */
    vli_modMult(e2, d1, e0, curve_secp224r1.p, num_words_secp224r1);     /* e2 <-- d1 * e0 */
    vli_modAdd(e2, e2, t1, curve_secp224r1.p, num_words_secp224r1);      /* e2 <-- e2 + t1 */
    vli_modSquare(f2, e2, curve_secp224r1.p, num_words_secp224r1);       /* f2 <-- e2^2 */
    vli_modMult(f2, f2, c, curve_secp224r1.p, num_words_secp224r1);      /* f2 <-- f2 * c */
    /* f2 <-- p  - f2 */
    vli_modSub(f2, curve_secp224r1.p, f2, curve_secp224r1.p, num_words_secp224r1);
    vli_set(d2, t2, num_words_secp224r1);                                 /* d2 <-- t2 */
}

/* Routine 3.2.7 RP;  from http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void mod_sqrt_secp224r1_rp(uECC_word_t *d1,
                                  uECC_word_t *e1,
                                  uECC_word_t *f1,
                                  const uECC_word_t *c,
                                  const uECC_word_t *r) {
    wordcount_t i;
    wordcount_t pow2i = 1;
    uECC_word_t d0[uECC_MAX_WORDS];
    uECC_word_t e0[uECC_MAX_WORDS] = {1};          /* e0 <-- 1 */
    uECC_word_t f0[uECC_MAX_WORDS];

    vli_set(d0, r, num_words_secp224r1); /* d0 <-- r */
    /* f0 <-- p  - c */
    vli_modSub(f0, curve_secp224r1.p, c, curve_secp224r1.p, num_words_secp224r1);
    for (i = 0; i <= 6; i++) {
        mod_sqrt_secp224r1_rss(d1, e1, f1, d0, e0, f0, pow2i); /* RSS (d1,e1,f1,d0,e0,f0,2^i) */
        mod_sqrt_secp224r1_rm(d1, e1, f1, c, d1, e1, d0, e0);  /* RM (d1,e1,f1,c,d1,e1,d0,e0) */
        vli_set(d0, d1, num_words_secp224r1); /* d0 <-- d1 */
        vli_set(e0, e1, num_words_secp224r1); /* e0 <-- e1 */
        vli_set(f0, f1, num_words_secp224r1); /* f0 <-- f1 */
        pow2i *= 2;
    }
}

/* Compute a = sqrt(a) (mod curve_p). */
/* Routine 3.2.8 mp_mod_sqrt_224; from http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void mod_sqrt_secp224r1(uECC_word_t *a, uECC_Curve curve) {
    bitcount_t i;
    uECC_word_t e1[uECC_MAX_WORDS];
    uECC_word_t f1[uECC_MAX_WORDS];
    uECC_word_t d0[uECC_MAX_WORDS];
    uECC_word_t e0[uECC_MAX_WORDS];
    uECC_word_t f0[uECC_MAX_WORDS];
    uECC_word_t d1[uECC_MAX_WORDS];

    // s = a; using constant instead of random value
    mod_sqrt_secp224r1_rp(d0, e0, f0, a, a);           /* RP (d0, e0, f0, c, s) */
    mod_sqrt_secp224r1_rs(d1, e1, f1, d0, e0, f0);     /* RS (d1, e1, f1, d0, e0, f0) */
    for (i = 1; i <= 95; i++) {
        vli_set(d0, d1, num_words_secp224r1);          /* d0 <-- d1 */
        vli_set(e0, e1, num_words_secp224r1);          /* e0 <-- e1 */
        vli_set(f0, f1, num_words_secp224r1);          /* f0 <-- f1 */
        mod_sqrt_secp224r1_rs(d1, e1, f1, d0, e0, f0); /* RS (d1, e1, f1, d0, e0, f0) */
        if (vli_isZero(d1, num_words_secp224r1)) {     /* if d1 == 0 */
	        break;
        }
    }
    vli_modInv(f1, e0, curve_secp224r1.p, num_words_secp224r1);     /* f1 <-- 1 / e0 */
    vli_modMult(a, d0, f1, curve_secp224r1.p, num_words_secp224r1); /* a  <-- d0 / e0 */
}

#if uECC_WORD_SIZE == 1

static void vli_nativeToBytes(uint8_t * dest, const uint8_t * src, uECC_Curve curve) {
    uint8_t i;
    for (i = 0; i < curve->num_words; ++i) {
        dest[i] = src[(curve->num_words - 1) - i];
    }
}

#define vli_bytesToNative(dest, src, curve) vli_nativeToBytes((dest), (src), (curve))

#elif uECC_WORD_SIZE == 4

static void vli_nativeToBytes(uint8_t *bytes, const uint32_t *native, uECC_Curve curve) {
    unsigned i;
    for (i = 0; i < curve->num_words; ++i) {
        uint8_t *digit = bytes + 4 * (curve->num_words - 1 - i);
        digit[0] = native[i] >> 24;
        digit[1] = native[i] >> 16;
        digit[2] = native[i] >> 8;
        digit[3] = native[i];
    }
}

static void vli_bytesToNative(uint32_t *native, const uint8_t *bytes, uECC_Curve curve) {
    unsigned i;
    for (i = 0; i < curve->num_words; ++i) {
        const uint8_t *digit = bytes + 4 * (curve->num_words - 1 - i);
        native[i] = ((uint32_t)digit[0] << 24) | ((uint32_t)digit[1] << 16) |
                    ((uint32_t)digit[2] << 8) | (uint32_t)digit[3];
    }
}

#else

static void vli_nativeToBytes(uint8_t *bytes, const uint64_t *native, uECC_Curve curve) {
    unsigned i;
    for (i = 0; i < curve->num_bytes; ++i) {
        unsigned b = curve->num_bytes - 1 - i;
        bytes[i] = native[b / 8] >> (8 * (b % 8));
    }
}

static void vli_bytesToNative(uint64_t *native, const uint8_t *bytes, uECC_Curve curve) {
    unsigned i;
    vli_clear(native, curve->num_words);
    for (i = 0; i < curve->num_bytes; ++i) {
        unsigned b = curve->num_bytes - 1 - i;
        native[b / 8] |= (uint64_t)bytes[i] << (8 * (b % 8));
    }
}

#endif /* uECC_WORD_SIZE */

/* Generate a random integer with num_bits bits. The remaining high bits
   in the buffer (if any) are zeroed. */
static cmpresult_t generate_random_int(uECC_word_t *random,
                                       const wordcount_t num_words,
                                       const wordcount_t num_bits) {
    if (!g_rng_function((uint8_t *)random, num_words * uECC_WORD_SIZE)) {
        return 0;
    }
    if (num_words * uECC_WORD_SIZE * 8 > num_bits) {
        wordcount_t mask = (wordcount_t)-1;
        random[num_words - 1] &= mask >> ((bitcount_t)(num_words * uECC_WORD_SIZE * 8 - num_bits));
    }
    return 1;
}

int uECC_make_key(uint8_t *public_key,
                  uint8_t *private_key,
                  uECC_Curve curve) {
    uECC_word_t private[uECC_MAX_WORDS];
    uECC_word_t public[uECC_MAX_WORDS * 2];
    uECC_word_t tries;
    
    /* Zero out correctly (to compare to curve->n) for secp160r1. */
    private[curve->num_n_words - 1] = 0; 
    
    for (tries = 0; tries < MAX_TRIES; ++tries) {
        if (!generate_random_int(private, curve->num_words, curve->num_bytes * 8)) {
            return 0;
        }
        
        if (EccPoint_compute_public_key(public, private, curve)) {
            vli_nativeToBytes(private_key, private, curve);
            vli_nativeToBytes(public_key, public, curve);
            vli_nativeToBytes(public_key + curve->num_bytes, public + curve->num_words, curve);
            return 1;
        }
    }
    return 0;
}

int uECC_shared_secret(const uint8_t *public_key,
                       const uint8_t *private_key,
                       uint8_t *secret,
                       uECC_Curve curve) {
    uECC_word_t public[uECC_MAX_WORDS * 2];
    uECC_word_t private[uECC_MAX_WORDS];
    uECC_word_t tmp[uECC_MAX_WORDS];
    uECC_word_t *p2[2] = {private, tmp};
    uECC_word_t *initial_Z = 0;
    uECC_word_t tries;
    uECC_word_t carry;
    
    /* Zero out correctly (for addition with curve->n) for secp160r1. */
    private[curve->num_n_words - 1] = 0;
    
    vli_bytesToNative(private, private_key, curve);
    vli_bytesToNative(public, public_key, curve);
    vli_bytesToNative(public + curve->num_words, public_key + curve->num_bytes, curve);
    
    // Regularize the bitcount for the private key so that attackers cannot use a side channel
    // attack to learn the number of leading zeros.
    carry = regularize_k(private, private, tmp, curve);
    
    // If an RNG function was specified, try to get a random initial Z value to improve
    // protection against side-channel attacks.
    if (g_rng_function != &default_RNG) {
        for (tries = 0; tries < MAX_TRIES; ++tries) {
            if (!generate_random_int(p2[carry], curve->num_words, curve->num_bytes * 8)) {
                return 0;
            }
            
            if (!vli_isZero(p2[carry], curve->num_words) &&
                    vli_cmp(curve->p, p2[carry], curve->num_words) == 1) {
                initial_Z = p2[carry];
                break;
            }
        }
    }
    
    EccPoint_mult(public, public, p2[!carry], initial_Z,
                  vli_numBits(curve->n, curve->num_n_words) + 1,
                  curve);
    vli_nativeToBytes(secret, public, curve);
    return !EccPoint_isZero(public, curve);
}

void uECC_compress(const uint8_t *public_key, uint8_t *compressed, uECC_Curve curve) {
    wordcount_t i;
    for (i = 0; i < curve->num_bytes; ++i) {
        compressed[i+1] = public_key[i];
    }
    compressed[0] = 2 + (public_key[curve->num_bytes * 2 - 1] & 0x01);
}

/* Computes result = x^3 + ax + b. result must not overlap x. */
static void x_side_default(uECC_word_t *result, const uECC_word_t *x, uECC_Curve curve) {
    uECC_word_t _3[uECC_MAX_WORDS] = {3}; /* -a = 3 */

    vli_modSquare(result, x, curve->p, curve->num_words);             /* r = x^2 */
    vli_modSub(result, result, _3, curve->p, curve->num_words);       /* r = x^2 - 3 */
    vli_modMult(result, result, x, curve->p, curve->num_words);       /* r = x^3 - 3x */
    vli_modAdd(result, result, curve->b, curve->p, curve->num_words); /* r = x^3 - 3x + b */
}

/* Computes result = x^3 + b. result must not overlap x. */
static void x_side_secp256k1(uECC_word_t *result, const uECC_word_t *x, uECC_Curve curve) {
    vli_modSquare(result, x, curve->p, curve->num_words);             /* r = x^2 */
    vli_modMult(result, result, x, curve->p, curve->num_words);       /* r = x^3 */
    vli_modAdd(result, result, curve->b, curve->p, curve->num_words); /* r = x^3 + b */
}

void uECC_decompress(const uint8_t *compressed, uint8_t *public_key, uECC_Curve curve) {
    uECC_word_t point[uECC_MAX_WORDS * 2];
    uECC_word_t *y = point + curve->num_words;
    vli_bytesToNative(point, compressed + 1, curve);
    curve->x_side(y, point, curve);
    curve->mod_sqrt(y, curve);
    
    if ((y[0] & 0x01) != (compressed[0] & 0x01)) {
        vli_sub(y, curve->p, y, curve->num_words);
    }
    
    vli_nativeToBytes(public_key, point, curve);
    vli_nativeToBytes(public_key + curve->num_bytes, y, curve);
}

int uECC_valid_public_key(const uint8_t *public_key, uECC_Curve curve) {
    uECC_word_t tmp1[uECC_MAX_WORDS];
    uECC_word_t tmp2[uECC_MAX_WORDS];
    uECC_word_t public[uECC_MAX_WORDS * 2];
    
    vli_bytesToNative(public, public_key, curve);
    vli_bytesToNative(public + curve->num_words, public_key + curve->num_bytes, curve);
    
    // The point at infinity is invalid.
    if (EccPoint_isZero(public, curve)) {
        return 0;
    }

    // x and y must be smaller than p.
    if (vli_cmp(curve->p, public, curve->num_words) != 1 ||
            vli_cmp(curve->p, public + curve->num_words, curve->num_words) != 1) {
        return 0;
    }
    
    vli_modSquare(tmp1, public + curve->num_words, curve->p, curve->num_words);
    curve->x_side(tmp2, public, curve); /* tmp2 = x^3 + ax + b */
    
    /* Make sure that y^2 == x^3 + ax + b */
    return (vli_cmp(tmp1, tmp2, curve->num_words) == 0);
}

int uECC_compute_public_key(const uint8_t *private_key, uint8_t *public_key, uECC_Curve curve) {
    uECC_word_t private[uECC_MAX_WORDS];
    uECC_word_t public[uECC_MAX_WORDS * 2];

    vli_bytesToNative(private, private_key, curve);

    if (!EccPoint_compute_public_key(public, private, curve)) {
        return 0;
    }

    vli_nativeToBytes(public_key, public, curve);
    vli_nativeToBytes(public_key + curve->num_bytes, public + curve->num_words, curve);
    return 1;
}


/* -------- ECDSA code -------- */

static int uECC_sign_with_k(const uint8_t *private_key,
                            const uint8_t *message_hash,
                            uECC_word_t *k,
                            uint8_t *signature,
                            uECC_Curve curve) {
    uECC_word_t tmp[uECC_MAX_WORDS];
    uECC_word_t s[uECC_MAX_WORDS];
    uECC_word_t *k2[2] = {tmp, s};
    uECC_word_t p[uECC_MAX_WORDS * 2];
    uECC_word_t carry;
    bitcount_t num_n_bits = vli_numBits(curve->n, curve->num_n_words);
    
    /* Make sure 0 < k < curve_n */
    if (vli_isZero(k, curve->num_words) || vli_cmp(curve->n, k, curve->num_n_words) != 1) {
        return 0;
    }
    
    carry = regularize_k(k, tmp, s, curve);
    EccPoint_mult(p, curve->G, k2[!carry], 0, num_n_bits + 1, curve);
    if (vli_isZero(p, curve->num_words)) {
        return 0;
    }
    
    /* Attempt to get a random number to prevent side channel analysis of k. */
    if (g_rng_function == &default_RNG) {
        vli_clear(tmp, curve->num_n_words);
        tmp[0] = 1;
    } else {
        uECC_word_t tries;
        for (tries = 0; tries < MAX_TRIES; ++tries) {
            if (!generate_random_int(tmp, curve->num_n_words, num_n_bits)) {
                return 0;
            }
            
            if (!vli_isZero(tmp, curve->num_n_words) &&
                    vli_cmp(curve->n, tmp, curve->num_n_words) == 1) {
                goto got_random;
            }
        }
        return 0;
    }
got_random:
    /* Prevent side channel analysis of vli_modInv() to determine
       bits of k / the private key by premultiplying by a random number */
    vli_modMult(k, k, tmp, curve->n, curve->num_n_words);   /* k' = rand * k */
    vli_modInv(k, k, curve->n, curve->num_n_words);       /* k = 1 / k' */
    vli_modMult(k, k, tmp, curve->n, curve->num_n_words); /* k = 1 / k */
    
    vli_nativeToBytes(signature, p, curve); /* store r */
    
    tmp[curve->num_n_words - 1] = 0;
    vli_bytesToNative(tmp, private_key, curve); /* tmp = d */
    s[curve->num_n_words - 1] = 0;
    vli_set(s, p, curve->num_words);
    vli_modMult(s, tmp, s, curve->n, curve->num_n_words); /* s = r*d */

    vli_bytesToNative(tmp, message_hash, curve);
    vli_modAdd(s, tmp, s, curve->n, curve->num_n_words); /* s = e + r*d */
    vli_modMult(s, s, k, curve->n, curve->num_n_words);  /* s = (e + r*d) / k */
    if (vli_numBits(s, curve->num_n_words) > (bitcount_t)curve->num_bytes * 8) {
        return 0;
    }
    vli_nativeToBytes(signature + curve->num_bytes, s, curve);
    return 1;
}

int uECC_sign(const uint8_t *private_key,
              const uint8_t *message_hash,
              uint8_t *signature,
              uECC_Curve curve) {
    uECC_word_t k[uECC_MAX_WORDS];
    uECC_word_t tries;
    bitcount_t num_n_bits = vli_numBits(curve->n, curve->num_n_words);
    
    for (tries = 0; tries < MAX_TRIES; ++tries) {
        if (!generate_random_int(k, curve->num_n_words, num_n_bits)) {
            return 0;
        }

        if (uECC_sign_with_k(private_key, message_hash, k, signature, curve)) {
            return 1;
        }
    }
    return 0;
}

/* Compute an HMAC using K as a key (as in RFC 6979). Note that K is always
   the same size as the hash result size. */
static void HMAC_init(uECC_HashContext *hash_context, const uint8_t *K) {
    uint8_t *pad = hash_context->tmp + 2 * hash_context->result_size;
    unsigned i;
    for (i = 0; i < hash_context->result_size; ++i)
        pad[i] = K[i] ^ 0x36;
    for (; i < hash_context->block_size; ++i)
        pad[i] = 0x36;

    hash_context->init_hash(hash_context);
    hash_context->update_hash(hash_context, pad, hash_context->block_size);
}

static void HMAC_update(uECC_HashContext *hash_context,
                        const uint8_t *message,
                        unsigned message_size) {
    hash_context->update_hash(hash_context, message, message_size);
}

static void HMAC_finish(uECC_HashContext *hash_context, const uint8_t *K, uint8_t *result) {
    uint8_t *pad = hash_context->tmp + 2 * hash_context->result_size;
    unsigned i;
    for (i = 0; i < hash_context->result_size; ++i)
        pad[i] = K[i] ^ 0x5c;
    for (; i < hash_context->block_size; ++i)
        pad[i] = 0x5c;

    hash_context->finish_hash(hash_context, result);

    hash_context->init_hash(hash_context);
    hash_context->update_hash(hash_context, pad, hash_context->block_size);
    hash_context->update_hash(hash_context, result, hash_context->result_size);
    hash_context->finish_hash(hash_context, result);
}

/* V = HMAC_K(V) */
static void update_V(uECC_HashContext *hash_context, uint8_t *K, uint8_t *V) {
    HMAC_init(hash_context, K);
    HMAC_update(hash_context, V, hash_context->result_size);
    HMAC_finish(hash_context, K, V);
}

/* Deterministic signing, similar to RFC 6979. Differences are:
    * We just use (truncated) H(m) directly rather than bits2octets(H(m))
      (it is not reduced modulo curve_n).
    * We generate a value for k (aka T) directly rather than converting endianness.

   Layout of hash_context->tmp: <K> | <V> | (1 byte overlapped 0x00 or 0x01) / <HMAC pad> */
int uECC_sign_deterministic(const uint8_t *private_key,
                            const uint8_t *message_hash,
                            uECC_HashContext *hash_context,
                            uint8_t *signature,
                            uECC_Curve curve) {
    uint8_t *K = hash_context->tmp;
    uint8_t *V = K + hash_context->result_size;
    bitcount_t num_n_bits = vli_numBits(curve->n, curve->num_n_words);
    uECC_word_t tries;
    unsigned i;
    for (i = 0; i < hash_context->result_size; ++i) {
        V[i] = 0x01;
        K[i] = 0;
    }

    // K = HMAC_K(V || 0x00 || int2octets(x) || h(m))
    HMAC_init(hash_context, K);
    V[hash_context->result_size] = 0x00;
    HMAC_update(hash_context, V, hash_context->result_size + 1);
    HMAC_update(hash_context, private_key, curve->num_bytes);
    HMAC_update(hash_context, message_hash, curve->num_bytes);
    HMAC_finish(hash_context, K, K);

    update_V(hash_context, K, V);

    // K = HMAC_K(V || 0x01 || int2octets(x) || h(m))
    HMAC_init(hash_context, K);
    V[hash_context->result_size] = 0x01;
    HMAC_update(hash_context, V, hash_context->result_size + 1);
    HMAC_update(hash_context, private_key, curve->num_bytes);
    HMAC_update(hash_context, message_hash, curve->num_bytes);
    HMAC_finish(hash_context, K, K);

    update_V(hash_context, K, V);

    for (tries = 0; tries < MAX_TRIES; ++tries) {
        uECC_word_t T[uECC_MAX_WORDS];
        uint8_t *T_ptr = (uint8_t *)T;
        unsigned T_bytes = 0;
        while (T_bytes < curve->num_words * uECC_WORD_SIZE) {
            update_V(hash_context, K, V);
            for (i = 0; i < hash_context->result_size && T_bytes < sizeof(T); ++i, ++T_bytes) {
                T_ptr[T_bytes] = V[i];
            }
        }
        if ((bitcount_t)curve->num_words * uECC_WORD_SIZE * 8 > num_n_bits) {
            wordcount_t mask = (wordcount_t)-1;
            T[curve->num_words - 1] &=
                mask >> ((bitcount_t)(curve->num_words * uECC_WORD_SIZE * 8 - num_n_bits));
        }
    
        if (uECC_sign_with_k(private_key, message_hash, T, signature, curve)) {
            return 1;
        }

        // K = HMAC_K(V || 0x00)
        HMAC_init(hash_context, K);
        V[hash_context->result_size] = 0x00;
        HMAC_update(hash_context, V, hash_context->result_size + 1);
        HMAC_finish(hash_context, K, K);

        update_V(hash_context, K, V);
    }
    return 0;
}

static bitcount_t smax(bitcount_t a, bitcount_t b) {
    return (a > b ? a : b);
}

int uECC_verify(const uint8_t *public_key,
                const uint8_t *hash,
                const uint8_t *signature,
                uECC_Curve curve) {
    uECC_word_t u1[uECC_MAX_WORDS], u2[uECC_MAX_WORDS];
    uECC_word_t z[uECC_MAX_WORDS];
    uECC_word_t public[uECC_MAX_WORDS * 2];
    uECC_word_t sum[uECC_MAX_WORDS * 2];
    uECC_word_t rx[uECC_MAX_WORDS];
    uECC_word_t ry[uECC_MAX_WORDS];
    uECC_word_t tx[uECC_MAX_WORDS];
    uECC_word_t ty[uECC_MAX_WORDS];
    uECC_word_t tz[uECC_MAX_WORDS];
    const uECC_word_t *points[4];
    const uECC_word_t *point;
    bitcount_t num_bits;
    bitcount_t i;
    uECC_word_t r[uECC_MAX_WORDS], s[uECC_MAX_WORDS];
    
    
    rx[curve->num_n_words - 1] = 0;
    r[curve->num_n_words - 1] = 0;
    s[curve->num_n_words - 1] = 0;

    vli_bytesToNative(public, public_key, curve);
    vli_bytesToNative(public + curve->num_words, public_key + curve->num_bytes, curve);
    vli_bytesToNative(r, signature, curve);
    vli_bytesToNative(s, signature + curve->num_bytes, curve);
    
    /* r, s must not be 0. */
    if (vli_isZero(r, curve->num_words) || vli_isZero(s, curve->num_words)) {
        return 0;
    }

    /* r, s must be < n. */
    if (vli_cmp(curve->n, r, curve->num_n_words) != 1 ||
            vli_cmp(curve->n, s, curve->num_n_words) != 1) { 
        return 0;
    }

    /* Calculate u1 and u2. */
    vli_modInv(z, s, curve->n, curve->num_n_words); /* z = 1/s */
    u1[curve->num_n_words - 1] = 0;
    vli_bytesToNative(u1, hash, curve);
    vli_modMult(u1, u1, z, curve->n, curve->num_n_words); /* u1 = e/s */
    vli_modMult(u2, r, z, curve->n, curve->num_n_words); /* u2 = r/s */
    
    /* Calculate sum = G + Q. */
    vli_set(sum, public, curve->num_words);
    vli_set(sum + curve->num_words, public + curve->num_words, curve->num_words);
    vli_set(tx, curve->G, curve->num_words);
    vli_set(ty, curve->G + curve->num_words, curve->num_words);
    vli_modSub(z, sum, tx, curve->p, curve->num_words); /* z = x2 - x1 */
    XYcZ_add(tx, ty, sum, sum + curve->num_words, curve);
    vli_modInv(z, z, curve->p, curve->num_words); /* z = 1/z */
    apply_z(sum, sum + curve->num_words, z, curve);
    
    /* Use Shamir's trick to calculate u1*G + u2*Q */
    points[0] = 0;
    points[1] = curve->G;
    points[2] = public;
    points[3] = sum;
    num_bits = smax(vli_numBits(u1, curve->num_n_words), vli_numBits(u2, curve->num_n_words));
    
    point = points[(!!vli_testBit(u1, num_bits - 1)) | ((!!vli_testBit(u2, num_bits - 1)) << 1)];
    vli_set(rx, point, curve->num_words);
    vli_set(ry, point + curve->num_words, curve->num_words);
    vli_clear(z, curve->num_words);
    z[0] = 1;

    for (i = num_bits - 2; i >= 0; --i) {
        uECC_word_t index;
        curve->double_jacobian(rx, ry, z, curve);
        
        index = (!!vli_testBit(u1, i)) | ((!!vli_testBit(u2, i)) << 1);
        point = points[index];
        if (point) {
            vli_set(tx, point, curve->num_words);
            vli_set(ty, point + curve->num_words, curve->num_words);
            apply_z(tx, ty, z, curve);
            vli_modSub(tz, rx, tx, curve->p, curve->num_words); /* Z = x2 - x1 */
            XYcZ_add(tx, ty, rx, ry, curve);
            vli_modMult(z, z, tz, curve->p, curve->num_words);
        }
    }

    vli_modInv(z, z, curve->p, curve->num_words); /* Z = 1/Z */
    apply_z(rx, ry, z, curve);
    
    /* v = x1 (mod n) */
    if (vli_cmp(curve->n, rx, curve->num_n_words) != 1) {
        vli_sub(rx, rx, curve->n, curve->num_n_words);
    }

    /* Accept only if v == r. */
    return (vli_cmp(rx, r, curve->num_words) == 0);
}
