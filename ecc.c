/* Copyright 2013, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#include "ecc.h"

#include <string.h>

typedef unsigned int uint;

#define Curve_P_1 {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFD}
#define Curve_P_2 {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_P_3 {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0x00000000, 0x00000000, 0x00000001, 0xFFFFFFFF}
#define Curve_P_4 {0xFFFFFFFF, 0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, \
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_P_5 {0xFFFFFC2F, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}

#define Curve_B_1 {0x2CEE5ED3, 0xD824993C, 0x1079F43D, 0xE87579C1}
#define Curve_B_2 {0xC146B9B1, 0xFEB8DEEC, 0x72243049, 0x0FA7E9AB, 0xE59C80E7, 0x64210519}
#define Curve_B_3 {0x27D2604B, 0x3BCE3C3E, 0xCC53B0F6, 0x651D06B0, 0x769886BC, 0xB3EBBD55, 0xAA3A93E7, 0x5AC635D8}
#define Curve_B_4 {0xD3EC2AEF, 0x2A85C8ED, 0x8A2ED19D, 0xC656398D, 0x5013875A, 0x0314088F, 0xFE814112, 0x181D9C6E, \
    0xE3F82D19, 0x988E056B, 0xE23EE7E4, 0xB3312FA7}
#define Curve_B_5 {0x00000007, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000}

#define Curve_G_1 { \
    {0xA52C5B86, 0x0C28607C, 0x8B899B2D, 0x161FF752}, \
    {0xDDED7A83, 0xC02DA292, 0x5BAFEB13, 0xCF5AC839}}

#define Curve_G_2 { \
    {0x82FF1012, 0xF4FF0AFD, 0x43A18800, 0x7CBF20EB, 0xB03090F6, 0x188DA80E}, \
    {0x1E794811, 0x73F977A1, 0x6B24CDD5, 0x631011ED, 0xFFC8DA78, 0x07192B95}}
    
#define Curve_G_3 { \
    {0xD898C296, 0xF4A13945, 0x2DEB33A0, 0x77037D81, 0x63A440F2, 0xF8BCE6E5, 0xE12C4247, 0x6B17D1F2}, \
    {0x37BF51F5, 0xCBB64068, 0x6B315ECE, 0x2BCE3357, 0x7C0F9E16, 0x8EE7EB4A, 0xFE1A7F9B, 0x4FE342E2}}

#define Curve_G_4 { \
    {0x72760AB7, 0x3A545E38, 0xBF55296C, 0x5502F25D, 0x82542A38, 0x59F741E0, 0x8BA79B98, 0x6E1D3B62, \
        0xF320AD74, 0x8EB1C71E, 0xBE8B0537, 0xAA87CA22}, \
    {0x90EA0E5F, 0x7A431D7C, 0x1D7E819D, 0x0A60B1CE, 0xB5F0B8C0, 0xE9DA3113, 0x289A147C, 0xF8F41DBD, \
        0x9292DC29, 0x5D9E98BF, 0x96262C6F, 0x3617DE4A}}

#define Curve_G_5 { \
    {0x16F81798, 0x59F2815B, 0x2DCE28D9, 0x029BFCDB, 0xCE870B07, 0x55A06295, 0xF9DCBBAC, 0x79BE667E}, \
    {0xFB10D4B8, 0x9C47D08F, 0xA6855419, 0xFD17B448, 0x0E1108A8, 0x5DA4FBFC, 0x26A3C465, 0x483ADA77}}

#define Curve_N_1 {0x9038A115, 0x75A30D1B, 0x00000000, 0xFFFFFFFE}
#define Curve_N_2 {0xB4D22831, 0x146BC9B1, 0x99DEF836, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_N_3 {0xFC632551, 0xF3B9CAC2, 0xA7179E84, 0xBCE6FAAD, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0xFFFFFFFF}
#define Curve_N_4 {0xCCC52973, 0xECEC196A, 0x48B0A77A, 0x581A0DB2, 0xF4372DDF, 0xC7634D81, 0xFFFFFFFF, 0xFFFFFFFF, \
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_N_5 {0xD0364141, 0xBFD25E8C, 0xAF48A03B, 0xBAAEDCE6, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}

static uint32_t curve_p[NUM_ECC_DIGITS] = ECC_CONCAT(Curve_P_, ECC_CURVE);
static uint32_t curve_b[NUM_ECC_DIGITS] = ECC_CONCAT(Curve_B_, ECC_CURVE);
static EccPoint curve_G = ECC_CONCAT(Curve_G_, ECC_CURVE);
static uint32_t curve_n[NUM_ECC_DIGITS] = ECC_CONCAT(Curve_N_, ECC_CURVE);

static void vli_clear(uint32_t *p_vli)
{
    uint i;
    for(i=0; i<NUM_ECC_DIGITS; ++i)
    {
        p_vli[i] = 0;
    }
}

/* Returns 1 if p_vli == 0, 0 otherwise. */
static int vli_isZero(uint32_t *p_vli)
{
    uint i;
    for(i = 0; i < NUM_ECC_DIGITS; ++i)
    {
        if(p_vli[i])
        {
            return 0;
        }
    }
    return 1;
}

/* Returns nonzero if bit p_bit of p_vli is set. */
static uint32_t vli_testBit(uint32_t *p_vli, uint p_bit)
{
    return (p_vli[p_bit/32] & (1 << (p_bit % 32)));
}

/* Counts the number of 32-bit "digits" in p_vli. */
static uint vli_numDigits(uint32_t *p_vli)
{
    int i;
    /* Search from the end until we find a non-zero digit.
       We do it in reverse because we expect that most digits will be nonzero. */
    for(i = NUM_ECC_DIGITS - 1; i >= 0 && p_vli[i] == 0; --i)
    {
    }

    return (i + 1);
}

/* Counts the number of bits required for p_vli. */
static uint vli_numBits(uint32_t *p_vli)
{
    uint i;
    uint32_t l_digit;
    
    uint l_numDigits = vli_numDigits(p_vli);
    if(l_numDigits == 0)
    {
        return 0;
    }

    l_digit = p_vli[l_numDigits - 1];
    for(i=0; l_digit; ++i)
    {
        l_digit >>= 1;
    }
    
    return ((l_numDigits - 1) * 32 + i);
}

/* Sets p_dest = p_src. */
static void vli_set(uint32_t *p_dest, uint32_t *p_src)
{
    uint i;
    for(i=0; i<NUM_ECC_DIGITS; ++i)
    {
        p_dest[i] = p_src[i];
    }
}

/* Returns sign of p_left - p_right. */
static int vli_cmp(uint32_t *p_left, uint32_t *p_right)
{
    int i;
    for(i = NUM_ECC_DIGITS-1; i >= 0; --i)
    {
        if(p_left[i] > p_right[i])
        {
            return 1;
        }
        else if(p_left[i] < p_right[i])
        {
            return -1;
        }
    }
    return 0;
}

/* Computes p_result = p_in << c, returning carry. Can modify in place (if p_result == p_in). 0 < p_shift < 32. */
static uint32_t vli_lshift(uint32_t *p_result, uint32_t *p_in, uint p_shift)
{
    uint32_t l_carry = 0;
    uint i;
    for(i = 0; i < NUM_ECC_DIGITS; ++i)
    {
        uint32_t l_temp = p_in[i];
        p_result[i] = (l_temp << p_shift) | l_carry;
        l_carry = l_temp >> (32 - p_shift);
    }
    
    return l_carry;
}

/* Computes p_vli = p_vli >> 1. */
static void vli_rshift1(uint32_t *p_vli)
{
    uint32_t *l_end = p_vli;
    uint32_t l_carry = 0;
    
    p_vli += NUM_ECC_DIGITS;
    while(p_vli-- > l_end)
    {
        uint32_t l_temp = *p_vli;
        *p_vli = (l_temp >> 1) | l_carry;
        l_carry = l_temp << 31;
    }
}

/* Computes p_result = p_left + p_right, returning carry. Can modify in place. */
static uint32_t vli_add(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right)
{
#if (ECC_ASM == ecc_asm_thumb || ECC_ASM == ecc_asm_thumb2 || ECC_ASM == ecc_asm_arm)
    uint32_t l_counter = NUM_ECC_DIGITS;
    uint32_t l_carry = 0; /* carry = 0 initially */
    uint32_t l_left;
    uint32_t l_right;
    
    asm volatile (
        ".syntax unified \n\t"
        "1: \n\t"
        "ldmia %[lptr]!, {%[left]} \n\t"  /* Load left word. */
        "ldmia %[rptr]!, {%[right]} \n\t" /* Load right word. */
        "lsrs %[carry], #1 \n\t"          /* Set up carry flag (l_carry = 0 after this). */
        "adcs %[left], %[right] \n\t"     /* Add with carry. */
        "adcs %[carry], %[carry] \n\t"    /* Store carry bit in l_carry. */
        "stmia %[dptr]!, {%[left]} \n\t"  /* Store result word. */
        "subs %[ctr], #1 \n\t"            /* Decrement index. */
        "bne 1b \n\t"                     /* Loop until index == 0. */
    #if (ECC_ASM != ecc_asm_thumb2)
        ".syntax divided \n\t"
    #endif
    #if (ECC_ASM == ecc_asm_thumb)
        : [dptr] "+l" (p_result), [lptr] "+l" (p_left), [rptr] "+l" (p_right), [ctr] "+l" (l_counter), [carry] "+l" (l_carry), [left] "=l" (l_left), [right] "=l" (l_right)
    #else
        : [dptr] "+r" (p_result), [lptr] "+r" (p_left), [rptr] "+r" (p_right), [ctr] "+r" (l_counter), [carry] "+r" (l_carry), [left] "=r" (l_left), [right] "=r" (l_right)
    #endif
        :
        : "cc", "memory"
    );
    return l_carry;
    
#else

    uint32_t l_carry = 0;
    uint i;
    for(i=0; i<NUM_ECC_DIGITS; ++i)
    {
        uint32_t l_sum = p_left[i] + p_right[i] + l_carry;
        if(l_sum != p_left[i])
        {
            l_carry = (l_sum < p_left[i]);
        }
        p_result[i] = l_sum;
    }
    return l_carry;
#endif
}

/* Computes p_result = p_left - p_right, returning borrow. Can modify in place. */
static uint32_t vli_sub(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right)
{
#if (ECC_ASM == ecc_asm_thumb || ECC_ASM == ecc_asm_thumb2 || ECC_ASM == ecc_asm_arm)
    uint32_t l_counter = NUM_ECC_DIGITS;
    uint32_t l_carry = 1; /* carry = 1 initially (means don't borrow) */
    uint32_t l_left;
    uint32_t l_right;
    
    asm volatile (
        ".syntax unified \n\t"
        "1: \n\t"
        "ldmia %[lptr]!, {%[left]} \n\t"  /* Load left word. */
        "ldmia %[rptr]!, {%[right]} \n\t" /* Load right word. */
        "lsrs %[carry], #1 \n\t"          /* Set up carry flag (l_carry = 0 after this). */
        "sbcs %[left], %[right] \n\t"     /* Subtract with borrow. */
        "adcs %[carry], %[carry] \n\t"    /* Store carry bit in l_carry. */
        "stmia %[dptr]!, {%[left]} \n\t"  /* Store result word. */
        "subs %[ctr], #1 \n\t"            /* Decrement index. */
        "bne 1b \n\t"                     /* Loop until index == 0. */
    #if (ECC_ASM != ecc_asm_thumb2)
        ".syntax divided \n\t"
    #endif
    #if (ECC_ASM == ecc_asm_thumb)
        : [dptr] "+l" (p_result), [lptr] "+l" (p_left), [rptr] "+l" (p_right), [ctr] "+l" (l_counter), [carry] "+l" (l_carry), [left] "=l" (l_left), [right] "=l" (l_right)
    #else
        : [dptr] "+r" (p_result), [lptr] "+r" (p_left), [rptr] "+r" (p_right), [ctr] "+r" (l_counter), [carry] "+r" (l_carry), [left] "=r" (l_left), [right] "=r" (l_right)
    #endif
        :
        : "cc", "memory"
    );
    return !l_carry;

#else

    uint32_t l_borrow = 0;
    uint i;
    for(i=0; i<NUM_ECC_DIGITS; ++i)
    {
        uint32_t l_diff = p_left[i] - p_right[i] - l_borrow;
        if(l_diff != p_left[i])
        {
            l_borrow = (l_diff > p_left[i]);
        }
        p_result[i] = l_diff;
    }
    return l_borrow;
#endif
}

/* Computes p_result = p_left * p_right. */
static void vli_mult(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right)
{
#if (ECC_ASM == ecc_asm_thumb2 || ECC_ASM == ecc_asm_arm)
    uint32_t c0 = 0;
    uint32_t c1 = 0;
    uint32_t c2 = 0;
    uint32_t k = 0;
    uint32_t i;
    uint32_t t0, t1;
    
    asm volatile (
        ".syntax unified \n\t"
        
        "1: \n\t" /* outer loop (k < NUM_ECC_DIGITS) */
        "movs %[i], #0 \n\t" /* i = 0 */
        "b 3f \n\t"
        
        "2: \n\t" /* outer loop (k >= NUM_ECC_DIGITS) */
        "movs %[i], %[k] \n\t"      /* i = k */
        "subs %[i], %[eccdm1] \n\t" /* i = k - (NUM_ECC_DIGITS - 1) (times 4) */
        
        "3: \n\t" /* inner loop */
        "subs %[t0], %[k], %[i] \n\t" /* t0 = k-i */
        
        "ldr %[t1], [%[right], %[t0]] \n\t" /* t1 = p_right[k-i] */
        "ldr %[t0], [%[left], %[i]] \n\t"   /* t0 = p_left[i] */
        
        "umull %[t0], %[t1], %[t0], %[t1] \n\t" /* (t0, t1) = p_left[i] * p_right[k-i] */
        
        "adds %[c0], %[t0] \n\t" /* add low word to c0 */
        "adcs %[c1], %[t1] \n\t" /* add high word to c1, including carry */
        "adcs %[c2], #0 \n\t"    /* add carry to c2 */

        "adds %[i], #4 \n\t"     /* i += 4 */
        "cmp %[i], %[eccd] \n\t" /* i < NUM_ECC_DIGITS (times 4)? */
        "bge 4f \n\t" /* if not, exit the loop */
        "cmp %[i], %[k] \n\t"    /* i <= k? */
        "ble 3b \n\t" /* if so, continue looping */
        
        "4: \n\t" /* end inner loop */
        
        "str %[c0], [%[result], %[k]] \n\t" /* p_result[k] = c0 */
        "mov %[c0], %[c1] \n\t"     /* c0 = c1 */
        "mov %[c1], %[c2] \n\t"     /* c1 = c2 */
        "movs %[c2], #0 \n\t"       /* c2 = 0 */
        "adds %[k], #4 \n\t"        /* k += 4 */
        "cmp %[k], %[eccd] \n\t"    /* k < NUM_ECC_DIGITS (times 4) ? */
        "blt 1b \n\t" /* if not, loop back, start with i = 0 */
        "cmp %[k], %[eccd2m1] \n\t" /* k < NUM_ECC_DIGITS * 2 - 1 (times 4) ? */
        "blt 2b \n\t" /* if not, loop back, start with i = (k+1) - NUM_ECC_DIGITS */
        /* end outer loop */
        
        "str %[c0], [%[result], %[k]] \n\t" /* p_result[NUM_ECC_DIGITS * 2 - 1] = c0 */
    #if (ECC_ASM != ecc_asm_thumb2)
        ".syntax divided \n\t"
    #endif
        : [c0] "+r" (c0), [c1] "+r" (c1), [c2] "+r" (c2), [k] "+r" (k), [i] "=&r" (i), [t0] "=&r" (t0), [t1] "=&r" (t1)
        : [result] "r" (p_result), [left] "r" (p_left), [right] "r" (p_right),
          [eccd] "I" (NUM_ECC_DIGITS * 4), [eccdm1] "I" ((NUM_ECC_DIGITS-1) * 4), [eccd2m1] "I" ((NUM_ECC_DIGITS * 2 - 1) * 4)
        : "cc", "memory"
    );
    
#elif (ECC_ASM == ecc_asm_thumb)

    register uint32_t *r0 asm("r0") = p_result;
    register uint32_t *r1 asm("r1") = p_left;
    register uint32_t *r2 asm("r2") = p_right;
    
    asm volatile (
        ".syntax unified \n\t"
        "movs r3, #0 \n\t" /* c0 = 0 */
        "movs r4, #0 \n\t" /* c1 = 0 */
        "movs r5, #0 \n\t" /* c2 = 0 */
        "movs r6, #0 \n\t" /* k = 0 */
        
        "push {r0} \n\t" /* keep p_result on the stack */
        
        "1: \n\t" /* outer loop (k < NUM_ECC_DIGITS) */
        "movs r7, #0 \n\t" /* r7 = i = 0 */
        "b 3f \n\t"
        
        "2: \n\t" /* outer loop (k >= NUM_ECC_DIGITS) */
        "movs r7, r6 \n\t"        /* r7 = k */
        "subs r7, %[eccdm1] \n\t" /* r7 = i = k - (NUM_ECC_DIGITS - 1) (times 4) */
        
        "3: \n\t" /* inner loop */
        "push {r3, r4, r5, r6} \n\t" /* push things, r3 (c0) is at the top of stack. */
        "subs r0, r6, r7 \n\t"       /* r0 = k-i */
        
        "ldr r4, [r2, r0] \n\t" /* r4 = p_right[k-i] */
        "ldr r0, [r1, r7] \n\t" /* r0 = p_left[i] */
        
        "lsrs r3, r0, #16 \n\t" /* r3 = a1 */
        "uxth r0, r0 \n\t"      /* r0 = a0 */
        
        "lsrs r5, r4, #16 \n\t" /* r5 = b1 */
        "uxth r4, r4 \n\t"      /* r4 = b0 */
        
        "movs r6, r3 \n\t"     /* r6 = a1 */
        "muls r6, r5, r6 \n\t" /* r6 = a1*b1 */
        "muls r3, r4, r3 \n\t" /* r3 = b0*a1 */
        "muls r5, r0, r5 \n\t" /* r5 = a0*b1 */
        "muls r0, r4, r0 \n\t" /* r0 = a0*b0 */
        
        "movs r4, #0 \n\t"  /* r4 = 0 */
        "adds r3, r5 \n\t"  /* r3 = b0*a1 + a0*b1 */
        "adcs r4, r4 \n\t"  /* r4 = carry */
        "lsls r4, #16 \n\t" /* r4 = carry << 16 */
        "adds r6, r4 \n\t"  /* r6 = a1*b1 + carry */
        
        "lsls r4, r3, #16 \n\t" /* r4 = (b0*a1 + a0*b1) << 16 */
        "lsrs r3, #16 \n\t"     /* r3 = (b0*a1 + a0*b1) >> 16 */
        "adds r0, r4 \n\t"      /* r0 = low word = a0*b0 + ((b0*a1 + a0*b1) << 16) */
        "adcs r6, r3 \n\t"      /* r6 = high word = a1*b1 + carry + ((b0*a1 + a0*b1) >> 16) */
        
        "pop {r3, r4, r5} \n\t" /* r3 = c0, r4 = c1, r5 = c2 */
        "adds r3, r0 \n\t"      /* add low word to c0 */
        "adcs r4, r6 \n\t"      /* add high word to c1, including carry */
        "movs r0, #0 \n\t"      /* r0 = 0 (does not affect carry bit) */
        "adcs r5, r0 \n\t"      /* add carry to c2 */
        
        "pop {r6} \n\t" /* r6 = k */

        "adds r7, #4 \n\t"     /* i += 4 */
        "cmp r7, %[eccd] \n\t" /* i < NUM_ECC_DIGITS (times 4)? */
        "bge 4f \n\t" /* if not, exit the loop */
        "cmp r7, r6 \n\t"      /* i <= k? */
        "ble 3b \n\t" /* if so, continue looping */
        
        "4: \n\t" /* end inner loop */
        
        "ldr r0, [sp, #0] \n\t" /* r0 = p_result */
        
        "str r3, [r0, r6] \n\t"   /* p_result[k] = c0 */
        "mov r3, r4 \n\t"         /* c0 = c1 */
        "mov r4, r5 \n\t"         /* c1 = c2 */
        "movs r5, #0 \n\t"        /* c2 = 0 */
        "adds r6, #4 \n\t"        /* k += 4 */
        "cmp r6, %[eccd] \n\t"    /* k < NUM_ECC_DIGITS (times 4) ? */
        "blt 1b \n\t" /* if not, loop back, start with i = 0 */
        "cmp r6, %[eccd2m1] \n\t" /* k < NUM_ECC_DIGITS * 2 - 1 (times 4) ? */
        "blt 2b \n\t" /* if not, loop back, start with i = (k+1) - NUM_ECC_DIGITS */
        /* end outer loop */
        
        "str r3, [r0, r6] \n\t" /* p_result[NUM_ECC_DIGITS * 2 - 1] = c0 */
        "pop {r0} \n\t"         /* pop p_result off the stack */
        
        ".syntax divided \n\t"
        : 
        : [r0] "l" (r0), [r1] "l" (r1), [r2] "l" (r2), [eccd] "I" (NUM_ECC_DIGITS * 4), [eccdm1] "I" ((NUM_ECC_DIGITS-1) * 4), [eccd2m1] "I" ((NUM_ECC_DIGITS * 2 - 1) * 4)
        : "r3", "r4", "r5", "r6", "r7", "cc", "memory"
    );

#else

    uint64_t r01 = 0;
    uint32_t r2 = 0;
    
    uint i, k;
    
    /* Compute each digit of p_result in sequence, maintaining the carries. */
    for(k=0; k < NUM_ECC_DIGITS*2 - 1; ++k)
    {
        uint l_min = (k < NUM_ECC_DIGITS ? 0 : (k + 1) - NUM_ECC_DIGITS);
        for(i=l_min; i<=k && i<NUM_ECC_DIGITS; ++i)
        {
            uint64_t l_product = (uint64_t)p_left[i] * p_right[k-i];
            r01 += l_product;
            r2 += (r01 < l_product);
        }
        p_result[k] = (uint32_t)r01;
        r01 = (r01 >> 32) | (((uint64_t)r2) << 32);
        r2 = 0;
    }
    
    p_result[NUM_ECC_DIGITS*2 - 1] = (uint32_t)r01;
#endif
}

/* Computes p_result = (p_left + p_right) % p_mod.
   Assumes that p_left < p_mod and p_right < p_mod, p_result != p_mod. */
static void vli_modAdd(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right, uint32_t *p_mod)
{
    uint32_t l_carry = vli_add(p_result, p_left, p_right);
    if(l_carry || vli_cmp(p_result, p_mod) >= 0)
    { /* p_result > p_mod (p_result = p_mod + remainder), so subtract p_mod to get remainder. */
        vli_sub(p_result, p_result, p_mod);
    }
}

/* Computes p_result = (p_left - p_right) % p_mod.
   Assumes that p_left < p_mod and p_right < p_mod, p_result != p_mod. */
static void vli_modSub(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right, uint32_t *p_mod)
{
    uint32_t l_borrow = vli_sub(p_result, p_left, p_right);
    if(l_borrow)
    { /* In this case, p_result == -diff == (max int) - diff.
         Since -x % d == d - x, we can get the correct result from p_result + p_mod (with overflow). */
        vli_add(p_result, p_result, p_mod);
    }
}

#if (ECC_CURVE == secp128r1)

/* Computes p_result = p_product % curve_p.
   See algorithm 5 and 6 from http://www.isys.uni-klu.ac.at/PDF/2001-0126-MT.pdf */
static void vli_mmod_fast(uint32_t *p_result, uint32_t *p_product)
{
    uint32_t l_tmp[NUM_ECC_DIGITS];
    int l_carry;
    
    vli_set(p_result, p_product);
    
    l_tmp[0] = p_product[4];
    l_tmp[1] = p_product[5];
    l_tmp[2] = p_product[6];
    l_tmp[3] = (p_product[7] & 0x00000001) | (p_product[4] << 1);
    l_carry = vli_add(p_result, p_result, l_tmp);
    
    l_tmp[0] = (p_product[4] >> 31) | (p_product[5] << 1);
    l_tmp[1] = (p_product[5] >> 31) | (p_product[6] << 1);
    l_tmp[2] = (p_product[6] >> 31) | (p_product[7] << 1);
    l_tmp[3] = (p_product[7] >> 31) | ((p_product[4] & 0x80000000) >> 30) | (p_product[5] << 2);
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    l_tmp[0] = (p_product[5] >> 30) | (p_product[6] << 2);
    l_tmp[1] = (p_product[6] >> 30) | (p_product[7] << 2);
    l_tmp[2] = (p_product[7] >> 30);
    l_tmp[3] = ((p_product[5] & 0xC0000000) >> 29) | (p_product[6] << 3);
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    l_tmp[0] = (p_product[6] >> 29) | (p_product[7] << 3);
    l_tmp[1] = (p_product[7] >> 29);
    l_tmp[2] = 0;
    l_tmp[3] = ((p_product[6] & 0xE0000000) >> 28) | (p_product[7] << 4);
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    l_tmp[0] = (p_product[7] >> 28);
    l_tmp[1] = 0;
    l_tmp[2] = 0;
    l_tmp[3] = (p_product[7] & 0xFFFFFFFE);
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    l_tmp[0] = 0;
    l_tmp[1] = 0;
    l_tmp[2] = 0;
    l_tmp[3] = ((p_product[7] & 0xF0000000) >> 27);
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    while(l_carry || vli_cmp(curve_p, p_result) != 1)
    {
        l_carry -= vli_sub(p_result, p_result, curve_p);
    }
}

#elif (ECC_CURVE == secp192r1)

/* Computes p_result = p_product % curve_p.
   See algorithm 5 and 6 from http://www.isys.uni-klu.ac.at/PDF/2001-0126-MT.pdf */
static void vli_mmod_fast(uint32_t *p_result, uint32_t *p_product)
{
    uint32_t l_tmp[NUM_ECC_DIGITS];
    int l_carry;
    
    vli_set(p_result, p_product);
    
    vli_set(l_tmp, &p_product[6]);
    l_carry = vli_add(p_result, p_result, l_tmp);
    
    l_tmp[0] = l_tmp[1] = 0;
    l_tmp[2] = p_product[6];
    l_tmp[3] = p_product[7];
    l_tmp[4] = p_product[8];
    l_tmp[5] = p_product[9];
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    l_tmp[0] = l_tmp[2] = p_product[10];
    l_tmp[1] = l_tmp[3] = p_product[11];
    l_tmp[4] = l_tmp[5] = 0;
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    while(l_carry || vli_cmp(curve_p, p_result) != 1)
    {
        l_carry -= vli_sub(p_result, p_result, curve_p);
    }
}

#elif (ECC_CURVE == secp256r1)

/* Computes p_result = p_product % curve_p
   from http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void vli_mmod_fast(uint32_t *p_result, uint32_t *p_product)
{
    uint32_t l_tmp[NUM_ECC_DIGITS];
    int l_carry;
    
    /* t */
    vli_set(p_result, p_product);
    
    /* s1 */
    l_tmp[0] = l_tmp[1] = l_tmp[2] = 0;
    l_tmp[3] = p_product[11];
    l_tmp[4] = p_product[12];
    l_tmp[5] = p_product[13];
    l_tmp[6] = p_product[14];
    l_tmp[7] = p_product[15];
    l_carry = vli_lshift(l_tmp, l_tmp, 1);
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    /* s2 */
    l_tmp[3] = p_product[12];
    l_tmp[4] = p_product[13];
    l_tmp[5] = p_product[14];
    l_tmp[6] = p_product[15];
    l_tmp[7] = 0;
    l_carry += vli_lshift(l_tmp, l_tmp, 1);
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    /* s3 */
    l_tmp[0] = p_product[8];
    l_tmp[1] = p_product[9];
    l_tmp[2] = p_product[10];
    l_tmp[3] = l_tmp[4] = l_tmp[5] = 0;
    l_tmp[6] = p_product[14];
    l_tmp[7] = p_product[15];
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    /* s4 */
    l_tmp[0] = p_product[9];
    l_tmp[1] = p_product[10];
    l_tmp[2] = p_product[11];
    l_tmp[3] = p_product[13];
    l_tmp[4] = p_product[14];
    l_tmp[5] = p_product[15];
    l_tmp[6] = p_product[13];
    l_tmp[7] = p_product[8];
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    /* d1 */
    l_tmp[0] = p_product[11];
    l_tmp[1] = p_product[12];
    l_tmp[2] = p_product[13];
    l_tmp[3] = l_tmp[4] = l_tmp[5] = 0;
    l_tmp[6] = p_product[8];
    l_tmp[7] = p_product[10];
    l_carry -= vli_sub(p_result, p_result, l_tmp);
    
    /* d2 */
    l_tmp[0] = p_product[12];
    l_tmp[1] = p_product[13];
    l_tmp[2] = p_product[14];
    l_tmp[3] = p_product[15];
    l_tmp[4] = l_tmp[5] = 0;
    l_tmp[6] = p_product[9];
    l_tmp[7] = p_product[11];
    l_carry -= vli_sub(p_result, p_result, l_tmp);
    
    /* d3 */
    l_tmp[0] = p_product[13];
    l_tmp[1] = p_product[14];
    l_tmp[2] = p_product[15];
    l_tmp[3] = p_product[8];
    l_tmp[4] = p_product[9];
    l_tmp[5] = p_product[10];
    l_tmp[6] = 0;
    l_tmp[7] = p_product[12];
    l_carry -= vli_sub(p_result, p_result, l_tmp);
    
    /* d4 */
    l_tmp[0] = p_product[14];
    l_tmp[1] = p_product[15];
    l_tmp[2] = 0;
    l_tmp[3] = p_product[9];
    l_tmp[4] = p_product[10];
    l_tmp[5] = p_product[11];
    l_tmp[6] = 0;
    l_tmp[7] = p_product[13];
    l_carry -= vli_sub(p_result, p_result, l_tmp);
    
    if(l_carry < 0)
    {
        do
        {
            l_carry += vli_add(p_result, p_result, curve_p);
        } while(l_carry < 0);
    }
    else
    {
        while(l_carry || vli_cmp(curve_p, p_result) != 1)
        {
            l_carry -= vli_sub(p_result, p_result, curve_p);
        }
    }
}

#elif (ECC_CURVE == secp384r1)

static void omega_mult(uint32_t *p_result, uint32_t *p_right)
{
    /* Multiply by (2^128 + 2^96 - 2^32 + 1). */
    vli_set(p_result, p_right); /* 1 */
    p_result[3 + NUM_ECC_DIGITS] = vli_add(p_result + 3, p_result + 3, p_right); /* 2^96 + 1 */
    p_result[4 + NUM_ECC_DIGITS] = vli_add(p_result + 4, p_result + 4, p_right); /* 2^128 + 2^96 + 1 */
    if(vli_sub(p_result + 1, p_result + 1, p_right)) /* 2^128 + 2^96 - 2^32 + 1 */
    { /* Propagate borrow if necessary. */
        uint i;
        for(i = 1 + NUM_ECC_DIGITS; ; ++i)
        {
            --p_result[i];
            if(p_result[i] != 0xffffffff)
            {
                break;
            }
        }
    }
}

/* Computes p_result = p_product % curve_p
    see PDF "Comparing Elliptic Curve Cryptography and RSA on 8-bit CPUs"
    section "Curve-Specific Optimizations" */
static void vli_mmod_fast(uint32_t *p_result, uint32_t *p_product)
{
    uint32_t l_tmp[2*NUM_ECC_DIGITS];
     
    while(!vli_isZero(p_product + NUM_ECC_DIGITS)) /* While c1 != 0 */
    {
        uint32_t l_carry = 0;
        uint i;
        
        vli_clear(l_tmp);
        vli_clear(l_tmp + NUM_ECC_DIGITS);
        omega_mult(l_tmp, p_product + NUM_ECC_DIGITS); /* tmp = w * c1 */
        vli_clear(p_product + NUM_ECC_DIGITS); /* p = c0 */
        
        /* (c1, c0) = c0 + w * c1 */
        for(i=0; i<NUM_ECC_DIGITS+5; ++i)
        {
            uint32_t l_sum = p_product[i] + l_tmp[i] + l_carry;
            if(l_sum != p_product[i])
            {
                l_carry = (l_sum < p_product[i]);
            }
            p_product[i] = l_sum;
        }
    }
    
    while(vli_cmp(p_product, curve_p) > 0)
    {
        vli_sub(p_product, p_product, curve_p);
    }
    vli_set(p_result, p_product);
}

#elif (ECC_CURVE == secp256k1)

static void omega_mult(uint32_t *p_result, uint32_t *p_right)
{
    /* Multiply by (2^32 + 2^9 + 2^8 + 2^7 + 2^6 + 2^4 + 1). */
    uint64_t l_mult = 0x3D1; /* everything except 2^32 */
    uint32_t l_carry = 0;
    uint i;
    for(i=0; i<NUM_ECC_DIGITS; ++i)
    {
        uint64_t p = l_mult * p_right[i] + l_carry;
        p_result[i] = (p & 0xffffffff);
        l_carry = p >> 32;
    }
    p_result[NUM_ECC_DIGITS] = l_carry;
    
    p_result[1 + NUM_ECC_DIGITS] = vli_add(p_result + 1, p_result + 1, p_right); /* add the 2^32 multiple */
}

/* Computes p_result = p_product % curve_p  */
static void vli_mmod_fast(uint32_t *p_result, uint32_t *p_product)
{
    uint32_t l_tmp[2*NUM_ECC_DIGITS];
     
    while(!vli_isZero(p_product + NUM_ECC_DIGITS)) /* While c1 != 0 */
    {
        uint32_t l_carry = 0;
        uint i;
        
        vli_clear(l_tmp);
        vli_clear(l_tmp + NUM_ECC_DIGITS);
        omega_mult(l_tmp, p_product + NUM_ECC_DIGITS); /* tmp = w * c1 */
        vli_clear(p_product + NUM_ECC_DIGITS); /* p = c0 */
        
        /* (c1, c0) = c0 + w * c1 */
        for(i=0; i<NUM_ECC_DIGITS+2; ++i)
        {
            uint32_t l_sum = p_product[i] + l_tmp[i] + l_carry;
            if(l_sum != p_product[i])
            {
                l_carry = (l_sum < p_product[i]);
            }
            p_product[i] = l_sum;
        }
    }
    
    while(vli_cmp(p_product, curve_p) > 0)
    {
        vli_sub(p_product, p_product, curve_p);
    }
    vli_set(p_result, p_product);
}

#endif

/* Computes p_result = (p_left * p_right) % curve_p. */
static void vli_modMult_fast(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right)
{
   uint32_t l_product[2 * NUM_ECC_DIGITS];
   vli_mult(l_product, p_left, p_right);
   vli_mmod_fast(p_result, l_product);
}

#if ECC_SQUARE_FUNC

/* Computes p_result = p_left^2. */
static void vli_square(uint32_t *p_result, uint32_t *p_left)
{
#if (ECC_ASM == ecc_asm_thumb2 || ECC_ASM == ecc_asm_arm)
    uint32_t c0 = 0;
    uint32_t c1 = 0;
    uint32_t c2 = 0;
    uint32_t k = 0;
    uint32_t i, tt;
    uint32_t t0, t1;
    
    asm volatile (
        ".syntax unified \n\t"
        
        "1: \n\t" /* outer loop (k < NUM_ECC_DIGITS) */
        "movs %[i], #0 \n\t" /* i = 0 */
        "b 3f \n\t"
        
        "2: \n\t" /* outer loop (k >= NUM_ECC_DIGITS) */
        "movs %[i], %[k] \n\t"      /* i = k */
        "subs %[i], %[eccdm1] \n\t" /* i = k - (NUM_ECC_DIGITS - 1) (times 4) */
        
        "3: \n\t" /* inner loop */
        "subs %[tt], %[k], %[i] \n\t" /* tt = k-i */
        
        "ldr %[t1], [%[left], %[tt]] \n\t" /* t1 = p_left[k-i] */
        "ldr %[t0], [%[left], %[i]] \n\t"  /* t0 = p_left[i] */
        
        "umull %[t0], %[t1], %[t0], %[t1] \n\t" /* (t0, t1) = p_left[i] * p_right[k-i] */
        
        "cmp %[i], %[tt] \n\t"   /* (i < k-i) ? */
        "bge 4f \n\t" /* if i >= k-i, skip */
        "lsls %[t1], #1 \n\t"    /* high word << 1 */
        "adc %[c2], #0 \n\t"     /* add carry bit to c2 */
        "lsls %[t0], #1 \n\t"       /* low word << 1 */
        "adc %[t1], #0 \n\t"     /* add carry bit to high word */
        
        "4: \n\t"

        "adds %[c0], %[t0] \n\t" /* add low word to c0 */
        "adcs %[c1], %[t1] \n\t" /* add high word to c1, including carry */
        "adc %[c2], #0 \n\t"     /* add carry to c2 */
        
        "adds %[i], #4 \n\t"          /* i += 4 */
        "cmp %[i], %[k] \n\t"         /* i <= k? */
        "bge 5f \n\t" /* if not, exit the loop */
        "subs %[tt], %[k], %[i] \n\t" /* tt = k-i */
        "cmp %[i], %[tt] \n\t"        /* i <= k-i? */
        "ble 3b \n\t" /* if so, continue looping */
        
        "5: \n\t" /* end inner loop */
        
        "str %[c0], [%[result], %[k]] \n\t" /* p_result[k] = c0 */
        "mov %[c0], %[c1] \n\t"     /* c0 = c1 */
        "mov %[c1], %[c2] \n\t"     /* c1 = c2 */
        "movs %[c2], #0 \n\t"       /* c2 = 0 */
        "adds %[k], #4 \n\t"        /* k += 4 */
        "cmp %[k], %[eccd] \n\t"    /* k < NUM_ECC_DIGITS (times 4) ? */
        "blt 1b \n\t" /* if not, loop back, start with i = 0 */
        "cmp %[k], %[eccd2m1] \n\t" /* k < NUM_ECC_DIGITS * 2 - 1 (times 4) ? */
        "blt 2b \n\t" /* if not, loop back, start with i = (k+1) - NUM_ECC_DIGITS */
        /* end outer loop */
        
        "str %[c0], [%[result], %[k]] \n\t" /* p_result[NUM_ECC_DIGITS * 2 - 1] = c0 */
    #if (ECC_ASM != ecc_asm_thumb2)
        ".syntax divided \n\t"
    #endif
        : [c0] "+r" (c0), [c1] "+r" (c1), [c2] "+r" (c2), [k] "+r" (k), [i] "=&r" (i), [tt] "=&r" (tt), [t0] "=&r" (t0), [t1] "=&r" (t1)
        : [result] "r" (p_result), [left] "r" (p_left),
          [eccd] "I" (NUM_ECC_DIGITS * 4), [eccdm1] "I" ((NUM_ECC_DIGITS-1) * 4), [eccd2m1] "I" ((NUM_ECC_DIGITS * 2 - 1) * 4)
        : "cc", "memory"
    );
    
#elif (ECC_ASM == ecc_asm_thumb)

    register uint32_t *r0 asm("r0") = p_result;
    register uint32_t *r1 asm("r1") = p_left;
    
    asm volatile (
        ".syntax unified \n\t"
        "movs r2, #0 \n\t" /* c0 = 0 */
        "movs r3, #0 \n\t" /* c1 = 0 */
        "movs r4, #0 \n\t" /* c2 = 0 */
        "movs r5, #0 \n\t" /* k = 0 */
        
        "push {r0} \n\t" /* keep p_result on the stack */
        
        "1: \n\t" /* outer loop (k < NUM_ECC_DIGITS) */
        "movs r6, #0 \n\t" /* r6 = i = 0 */
        "b 3f \n\t"
        
        "2: \n\t" /* outer loop (k >= NUM_ECC_DIGITS) */
        "movs r6, r5 \n\t"        /* r6 = k */
        "subs r6, %[eccdm1] \n\t" /* r6 = i = k - (NUM_ECC_DIGITS - 1) (times 4) */
        
        "3: \n\t" /* inner loop */
        "push {r2, r3, r4, r5} \n\t" /* push things, r2 (c0) is at the top of stack. */
        "subs r7, r5, r6 \n\t"       /* r7 = k-i */
        
        "ldr r3, [r1, r7] \n\t" /* r3 = p_left[k-i] */
        "ldr r0, [r1, r6] \n\t" /* r0 = p_left[i] */
        
        "lsrs r2, r0, #16 \n\t" /* r2 = a1 */
        "uxth r0, r0 \n\t"      /* r0 = a0 */
        
        "lsrs r4, r3, #16 \n\t" /* r4 = b1 */
        "uxth r3, r3 \n\t"      /* r3 = b0 */
        
        "movs r5, r2 \n\t"     /* r5 = a1 */
        "muls r5, r4, r5 \n\t" /* r5 = a1*b1 */
        "muls r2, r3, r2 \n\t" /* r2 = b0*a1 */
        "muls r4, r0, r4 \n\t" /* r4 = a0*b1 */
        "muls r0, r3, r0 \n\t" /* r0 = a0*b0 */
        
        "movs r3, #0 \n\t"  /* r3 = 0 */
        "adds r2, r4 \n\t"  /* r2 = b0*a1 + a0*b1 */
        "adcs r3, r3 \n\t"  /* r3 = carry */
        "lsls r3, #16 \n\t" /* r3 = carry << 16 */
        "adds r5, r3 \n\t"  /* r5 = a1*b1 + carry */
        
        "lsls r3, r2, #16 \n\t" /* r3 = (b0*a1 + a0*b1) << 16 */
        "lsrs r2, #16 \n\t"     /* r2 = (b0*a1 + a0*b1) >> 16 */
        "adds r0, r3 \n\t"      /* r0 = low word = a0*b0 + ((b0*a1 + a0*b1) << 16) */
        "adcs r5, r2 \n\t"      /* r5 = high word = a1*b1 + carry + ((b0*a1 + a0*b1) >> 16) */
    
        "movs r3, #0 \n\t"  /* r3 = 0 */
        "cmp r6, r7 \n\t"   /* (i < k-i) ? */
        "mov r7, r3 \n\t"   /* r7 = 0 (does not affect condition)*/
        "bge 4f \n\t" /* if i >= k-i, skip */
        "lsls r5, #1 \n\t"  /* high word << 1 */
        "adcs r7, r3 \n\t"  /* r7 = carry bit for c2 */
        "lsls r0, #1 \n\t"  /* low word << 1 */
        "adcs r5, r3 \n\t"  /* add carry from shift to high word */
        
        "4: \n\t"
        "pop {r2, r3, r4} \n\t" /* r2 = c0, r3 = c1, r4 = c2 */
        "adds r2, r0 \n\t"      /* add low word to c0 */
        "adcs r3, r5 \n\t"      /* add high word to c1, including carry */
        "movs r0, #0 \n\t"      /* r0 = 0 (does not affect carry bit) */
        "adcs r4, r0 \n\t"      /* add carry to c2 */
        "adds r4, r7 \n\t"      /* add carry from doubling (if any) */
        
        "pop {r5} \n\t" /* r5 = k */
        
        "adds r6, #4 \n\t"     /* i += 4 */
        "cmp r6, r5 \n\t"      /* i <= k? */
        "bge 5f \n\t" /* if not, exit the loop */
        "subs r7, r5, r6 \n\t" /* r7 = k-i */
        "cmp r6, r7 \n\t"      /* i <= k-i? */
        "ble 3b \n\t" /* if so, continue looping */
        
        "5: \n\t" /* end inner loop */
        
        "ldr r0, [sp, #0] \n\t" /* r0 = p_result */
        
        "str r2, [r0, r5] \n\t"   /* p_result[k] = c0 */
        "mov r2, r3 \n\t"         /* c0 = c1 */
        "mov r3, r4 \n\t"         /* c1 = c2 */
        "movs r4, #0 \n\t"        /* c2 = 0 */
        "adds r5, #4 \n\t"        /* k += 4 */
        "cmp r5, %[eccd] \n\t"    /* k < NUM_ECC_DIGITS (times 4) ? */
        "blt 1b \n\t" /* if not, loop back, start with i = 0 */
        "cmp r5, %[eccd2m1] \n\t" /* k < NUM_ECC_DIGITS * 2 - 1 (times 4) ? */
        "blt 2b \n\t" /* if not, loop back, start with i = (k+1) - NUM_ECC_DIGITS */
        /* end outer loop */
        
        "str r2, [r0, r5] \n\t" /* p_result[NUM_ECC_DIGITS * 2 - 1] = c0 */
        "pop {r0} \n\t"         /* pop p_result off the stack */

        ".syntax divided \n\t"
        : [r0] "+l" (r0), [r1] "+l" (r1)
        : [eccd] "I" (NUM_ECC_DIGITS * 4), [eccdm1] "I" ((NUM_ECC_DIGITS-1) * 4), [eccd2m1] "I" ((NUM_ECC_DIGITS * 2 - 1) * 4)
        : "r2", "r3", "r4", "r5", "r6", "r7", "cc", "memory"
    );

#else

    uint64_t r01 = 0;
    uint32_t r2 = 0;
    
    uint i, k;
    for(k=0; k < NUM_ECC_DIGITS*2 - 1; ++k)
    {
        uint l_min = (k < NUM_ECC_DIGITS ? 0 : (k + 1) - NUM_ECC_DIGITS);
        for(i=l_min; i<=k && i<=k-i; ++i)
        {
            uint64_t l_product = (uint64_t)p_left[i] * p_left[k-i];
            if(i < k-i)
            {
                r2 += l_product >> 63;
                l_product *= 2;
            }
            r01 += l_product;
            r2 += (r01 < l_product);
        }
        p_result[k] = (uint32_t)r01;
        r01 = (r01 >> 32) | (((uint64_t)r2) << 32);
        r2 = 0;
    }
    
    p_result[NUM_ECC_DIGITS*2 - 1] = (uint32_t)r01;
#endif
}

/* Computes p_result = p_left^2 % curve_p. */
static void vli_modSquare_fast(uint32_t *p_result, uint32_t *p_left)
{
    uint32_t l_product[2 * NUM_ECC_DIGITS];
    vli_square(l_product, p_left);
    vli_mmod_fast(p_result, l_product);
}

#else /* ECC_SQUARE_FUNC */

#define vli_square(result, left, size) vli_mult((result), (left), (left), (size))
#define vli_modSquare_fast(result, left) vli_modMult_fast((result), (left), (left))
    
#endif /* ECC_SQUARE_FUNC */

#define EVEN(vli) (!(vli[0] & 1))
/* Computes p_result = (1 / p_input) % p_mod. All VLIs are the same size.
   See "From Euclid's GCD to Montgomery Multiplication to the Great Divide"
   https://labs.oracle.com/techrep/2001/smli_tr-2001-95.pdf */
static void vli_modInv(uint32_t *p_result, uint32_t *p_input, uint32_t *p_mod)
{
    uint32_t a[NUM_ECC_DIGITS], b[NUM_ECC_DIGITS], u[NUM_ECC_DIGITS], v[NUM_ECC_DIGITS];
    uint32_t l_carry;
    
    vli_set(a, p_input);
    vli_set(b, p_mod);
    vli_clear(u);
    u[0] = 1;
    vli_clear(v);
    
    int l_cmpResult;
    while((l_cmpResult = vli_cmp(a, b)) != 0)
    {
        l_carry = 0;
        if(EVEN(a))
        {
            vli_rshift1(a);
            if(!EVEN(u))
            {
                l_carry = vli_add(u, u, p_mod);
            }
            vli_rshift1(u);
            u[NUM_ECC_DIGITS-1] |= l_carry << 31;
        }
        else if(EVEN(b))
        {
            vli_rshift1(b);
            if(!EVEN(v))
            {
                l_carry = vli_add(v, v, p_mod);
            }
            vli_rshift1(v);
            v[NUM_ECC_DIGITS-1] |= l_carry << 31;
        }
        else if(l_cmpResult > 0)
        {
            vli_sub(a, a, b);
            vli_rshift1(a);
            if(vli_cmp(u, v) < 0)
            {
                vli_add(u, u, p_mod);
            }
            vli_sub(u, u, v);
            if(!EVEN(u))
            {
                l_carry = vli_add(u, u, p_mod);
            }
            vli_rshift1(u);
            u[NUM_ECC_DIGITS-1] |= l_carry << 31;
        }
        else
        {
            vli_sub(b, b, a);
            vli_rshift1(b);
            if(vli_cmp(v, u) < 0)
            {
                vli_add(v, v, p_mod);
            }
            vli_sub(v, v, u);
            if(!EVEN(v))
            {
                l_carry = vli_add(v, v, p_mod);
            }
            vli_rshift1(v);
            v[NUM_ECC_DIGITS-1] |= l_carry << 31;
        }
    }
    
    vli_set(p_result, u);
}

/* ------ Point operations ------ */

/* Returns 1 if p_point is the point at infinity, 0 otherwise. */
static int EccPoint_isZero(EccPoint *p_point)
{
    return (vli_isZero(p_point->x) && vli_isZero(p_point->y));
}

/* Point multiplication algorithm using Montgomery's ladder with co-Z coordinates.
From http://eprint.iacr.org/2011/338.pdf
*/

/* Double in place */
#if (ECC_CURVE == secp256k1)
static void EccPoint_double_jacobian(uint32_t *X1, uint32_t *Y1, uint32_t *Z1)
{
    /* t1 = X, t2 = Y, t3 = Z */
    uint32_t t4[NUM_ECC_DIGITS];
    uint32_t t5[NUM_ECC_DIGITS];
    
    if(vli_isZero(Z1))
    {
        return;
    }
    
    vli_modSquare_fast(t5, Y1);   /* t5 = y1^2 */
    vli_modMult_fast(t4, X1, t5); /* t4 = x1*y1^2 = A */
    vli_modSquare_fast(X1, X1);   /* t1 = x1^2 */
    vli_modSquare_fast(t5, t5);   /* t5 = y1^4 */
    vli_modMult_fast(Z1, Y1, Z1); /* t3 = y1*z1 = z3 */
    
    vli_modAdd(Y1, X1, X1, curve_p); /* t2 = 2*x1^2 */
    vli_modAdd(Y1, Y1, X1, curve_p); /* t2 = 3*x1^2 */
    if(vli_testBit(Y1, 0))
    {
        uint32_t l_carry = vli_add(Y1, Y1, curve_p);
        vli_rshift1(Y1);
        Y1[NUM_ECC_DIGITS-1] |= l_carry << 31;
    }
    else
    {
        vli_rshift1(Y1);
    }
    /* t2 = 3/2*(x1^2) = B */
    
    vli_modSquare_fast(X1, Y1);   /* t1 = B^2 */
    vli_modSub(X1, X1, t4, curve_p); /* t1 = B^2 - A */
    vli_modSub(X1, X1, t4, curve_p); /* t1 = B^2 - 2A = x3 */
    
    vli_modSub(t4, t4, X1, curve_p); /* t4 = A - x3 */
    vli_modMult_fast(Y1, Y1, t4);    /* t2 = B * (A - x3) */
    vli_modSub(Y1, Y1, t5, curve_p); /* t2 = B * (A - x3) - y1^4 = y3 */
}
#else
static void EccPoint_double_jacobian(uint32_t *X1, uint32_t *Y1, uint32_t *Z1)
{
    /* t1 = X, t2 = Y, t3 = Z */
    uint32_t t4[NUM_ECC_DIGITS];
    uint32_t t5[NUM_ECC_DIGITS];
    
    if(vli_isZero(Z1))
    {
        return;
    }
    
    vli_modSquare_fast(t4, Y1);   /* t4 = y1^2 */
    vli_modMult_fast(t5, X1, t4); /* t5 = x1*y1^2 = A */
    vli_modSquare_fast(t4, t4);   /* t4 = y1^4 */
    vli_modMult_fast(Y1, Y1, Z1); /* t2 = y1*z1 = z3 */
    vli_modSquare_fast(Z1, Z1);   /* t3 = z1^2 */
    
    vli_modAdd(X1, X1, Z1, curve_p); /* t1 = x1 + z1^2 */
    vli_modAdd(Z1, Z1, Z1, curve_p); /* t3 = 2*z1^2 */
    vli_modSub(Z1, X1, Z1, curve_p); /* t3 = x1 - z1^2 */
    vli_modMult_fast(X1, X1, Z1);    /* t1 = x1^2 - z1^4 */
    
    vli_modAdd(Z1, X1, X1, curve_p); /* t3 = 2*(x1^2 - z1^4) */
    vli_modAdd(X1, X1, Z1, curve_p); /* t1 = 3*(x1^2 - z1^4) */
    if(vli_testBit(X1, 0))
    {
        uint32_t l_carry = vli_add(X1, X1, curve_p);
        vli_rshift1(X1);
        X1[NUM_ECC_DIGITS-1] |= l_carry << 31;
    }
    else
    {
        vli_rshift1(X1);
    }
    /* t1 = 3/2*(x1^2 - z1^4) = B */
    
    vli_modSquare_fast(Z1, X1);      /* t3 = B^2 */
    vli_modSub(Z1, Z1, t5, curve_p); /* t3 = B^2 - A */
    vli_modSub(Z1, Z1, t5, curve_p); /* t3 = B^2 - 2A = x3 */
    vli_modSub(t5, t5, Z1, curve_p); /* t5 = A - x3 */
    vli_modMult_fast(X1, X1, t5);    /* t1 = B * (A - x3) */
    vli_modSub(t4, X1, t4, curve_p); /* t4 = B * (A - x3) - y1^4 = y3 */
    
    vli_set(X1, Z1);
    vli_set(Z1, Y1);
    vli_set(Y1, t4);
}
#endif

/* Modify (x1, y1) => (x1 * z^2, y1 * z^3) */
static void apply_z(uint32_t *X1, uint32_t *Y1, uint32_t *Z)
{
    uint32_t t1[NUM_ECC_DIGITS];

    vli_modSquare_fast(t1, Z);    /* z^2 */
    vli_modMult_fast(X1, X1, t1); /* x1 * z^2 */
    vli_modMult_fast(t1, t1, Z);  /* z^3 */
    vli_modMult_fast(Y1, Y1, t1); /* y1 * z^3 */
}

/* P = (x1, y1) => 2P, (x2, y2) => P' */
static void XYcZ_initial_double(uint32_t *X1, uint32_t *Y1, uint32_t *X2, uint32_t *Y2, uint32_t *p_initialZ)
{
    uint32_t z[NUM_ECC_DIGITS];
    
    vli_set(X2, X1);
    vli_set(Y2, Y1);
    
    vli_clear(z);
    z[0] = 1;
    if(p_initialZ)
    {
        vli_set(z, p_initialZ);
    }
    apply_z(X1, Y1, z);
    
    EccPoint_double_jacobian(X1, Y1, z);
    
    apply_z(X2, Y2, z);
}

/* Input P = (x1, y1, Z), Q = (x2, y2, Z)
   Output P' = (x1', y1', Z3), P + Q = (x3, y3, Z3)
   or P => P', Q => P + Q
*/
static void XYcZ_add(uint32_t *X1, uint32_t *Y1, uint32_t *X2, uint32_t *Y2)
{
    /* t1 = X1, t2 = Y1, t3 = X2, t4 = Y2 */
    uint32_t t5[NUM_ECC_DIGITS];
    
    vli_modSub(t5, X2, X1, curve_p); /* t5 = x2 - x1 */
    vli_modSquare_fast(t5, t5);      /* t5 = (x2 - x1)^2 = A */
    vli_modMult_fast(X1, X1, t5);    /* t1 = x1*A = B */
    vli_modMult_fast(X2, X2, t5);    /* t3 = x2*A = C */
    vli_modSub(Y2, Y2, Y1, curve_p); /* t4 = y2 - y1 */
    vli_modSquare_fast(t5, Y2);      /* t5 = (y2 - y1)^2 = D */
    
    vli_modSub(t5, t5, X1, curve_p); /* t5 = D - B */
    vli_modSub(t5, t5, X2, curve_p); /* t5 = D - B - C = x3 */
    vli_modSub(X2, X2, X1, curve_p); /* t3 = C - B */
    vli_modMult_fast(Y1, Y1, X2);    /* t2 = y1*(C - B) */
    vli_modSub(X2, X1, t5, curve_p); /* t3 = B - x3 */
    vli_modMult_fast(Y2, Y2, X2);    /* t4 = (y2 - y1)*(B - x3) */
    vli_modSub(Y2, Y2, Y1, curve_p); /* t4 = y3 */
    
    vli_set(X2, t5);
}

/* Input P = (x1, y1, Z), Q = (x2, y2, Z)
   Output P + Q = (x3, y3, Z3), P - Q = (x3', y3', Z3)
   or P => P - Q, Q => P + Q
*/
static void XYcZ_addC(uint32_t *X1, uint32_t *Y1, uint32_t *X2, uint32_t *Y2)
{
    /* t1 = X1, t2 = Y1, t3 = X2, t4 = Y2 */
    uint32_t t5[NUM_ECC_DIGITS];
    uint32_t t6[NUM_ECC_DIGITS];
    uint32_t t7[NUM_ECC_DIGITS];
    
    vli_modSub(t5, X2, X1, curve_p); /* t5 = x2 - x1 */
    vli_modSquare_fast(t5, t5);      /* t5 = (x2 - x1)^2 = A */
    vli_modMult_fast(X1, X1, t5);    /* t1 = x1*A = B */
    vli_modMult_fast(X2, X2, t5);    /* t3 = x2*A = C */
    vli_modAdd(t5, Y2, Y1, curve_p); /* t4 = y2 + y1 */
    vli_modSub(Y2, Y2, Y1, curve_p); /* t4 = y2 - y1 */

    vli_modSub(t6, X2, X1, curve_p); /* t6 = C - B */
    vli_modMult_fast(Y1, Y1, t6);    /* t2 = y1 * (C - B) */
    vli_modAdd(t6, X1, X2, curve_p); /* t6 = B + C */
    vli_modSquare_fast(X2, Y2);      /* t3 = (y2 - y1)^2 */
    vli_modSub(X2, X2, t6, curve_p); /* t3 = x3 */
    
    vli_modSub(t7, X1, X2, curve_p); /* t7 = B - x3 */
    vli_modMult_fast(Y2, Y2, t7);    /* t4 = (y2 - y1)*(B - x3) */
    vli_modSub(Y2, Y2, Y1, curve_p); /* t4 = y3 */
    
    vli_modSquare_fast(t7, t5);      /* t7 = (y2 + y1)^2 = F */
    vli_modSub(t7, t7, t6, curve_p); /* t7 = x3' */
    vli_modSub(t6, t7, X1, curve_p); /* t6 = x3' - B */
    vli_modMult_fast(t6, t6, t5);    /* t6 = (y2 + y1)*(x3' - B) */
    vli_modSub(Y1, t6, Y1, curve_p); /* t2 = y3' */
    
    vli_set(X1, t7);
}

static void EccPoint_mult(EccPoint *p_result, EccPoint *p_point, uint32_t *p_scalar, uint32_t *p_initialZ, unsigned p_numBits)
{
    /* R0 and R1 */
    uint32_t Rx[2][NUM_ECC_DIGITS];
    uint32_t Ry[2][NUM_ECC_DIGITS];
    uint32_t z[NUM_ECC_DIGITS];
    
    int i, nb;
    
    vli_set(Rx[1], p_point->x);
    vli_set(Ry[1], p_point->y);

    XYcZ_initial_double(Rx[1], Ry[1], Rx[0], Ry[0], p_initialZ);

    for(i = p_numBits - 2; i > 0; --i)
    {
        nb = !vli_testBit(p_scalar, i);
        XYcZ_addC(Rx[1-nb], Ry[1-nb], Rx[nb], Ry[nb]);
        XYcZ_add(Rx[nb], Ry[nb], Rx[1-nb], Ry[1-nb]);
    }

    nb = !vli_testBit(p_scalar, 0);
    XYcZ_addC(Rx[1-nb], Ry[1-nb], Rx[nb], Ry[nb]);
    
    /* Find final 1/Z value. */
    vli_modSub(z, Rx[1], Rx[0], curve_p); /* X1 - X0 */
    vli_modMult_fast(z, z, Ry[1-nb]);     /* Yb * (X1 - X0) */
    vli_modMult_fast(z, z, p_point->x);   /* xP * Yb * (X1 - X0) */
    vli_modInv(z, z, curve_p);            /* 1 / (xP * Yb * (X1 - X0)) */
    vli_modMult_fast(z, z, p_point->y);   /* yP / (xP * Yb * (X1 - X0)) */
    vli_modMult_fast(z, z, Rx[1-nb]);     /* Xb * yP / (xP * Yb * (X1 - X0)) */
    /* End 1/Z calculation */

    XYcZ_add(Rx[nb], Ry[nb], Rx[1-nb], Ry[1-nb]);
    
    apply_z(Rx[0], Ry[0], z);
    
    vli_set(p_result->x, Rx[0]);
    vli_set(p_result->y, Ry[0]);
}

int ecc_make_key(EccPoint *p_publicKey, uint32_t p_privateKey[NUM_ECC_DIGITS], uint32_t p_random[NUM_ECC_DIGITS])
{
    /* Make sure the private key is in the range [1, n-1]. */
    vli_set(p_privateKey, p_random);
    if(vli_cmp(curve_n, p_privateKey) != 1)
    {
        return 0;
    }
    
    if(vli_isZero(p_privateKey))
    {
        return 0; /* The private key cannot be 0 (mod p). */
    }
    
    EccPoint_mult(p_publicKey, &curve_G, p_privateKey, NULL, vli_numBits(p_privateKey));
    return 1;
}

#if (ECC_CURVE == secp256k1)
/* Compute p_result = x^3 + b */
static void curve_x_side(uint32_t p_result[NUM_ECC_DIGITS], uint32_t x[NUM_ECC_DIGITS])
{
    vli_modSquare_fast(p_result, x); /* r = x^2 */
    vli_modMult_fast(p_result, p_result, x); /* r = x^3 */
    vli_modAdd(p_result, p_result, curve_b, curve_p); /* r = x^3 + b */
}
#else
/* Compute p_result = x^3 - 3x + b */
static void curve_x_side(uint32_t p_result[NUM_ECC_DIGITS], uint32_t x[NUM_ECC_DIGITS])
{
    uint32_t _3[NUM_ECC_DIGITS] = {3}; /* -a = 3 */
    
    vli_modSquare_fast(p_result, x); /* r = x^2 */
    vli_modSub(p_result, p_result, _3, curve_p); /* r = x^2 - 3 */
    vli_modMult_fast(p_result, p_result, x); /* r = x^3 - 3x */
    vli_modAdd(p_result, p_result, curve_b, curve_p); /* r = x^3 - 3x + b */
}
#endif

int ecc_valid_public_key(EccPoint *p_publicKey)
{
    uint32_t l_tmp1[NUM_ECC_DIGITS];
    uint32_t l_tmp2[NUM_ECC_DIGITS];
    
    if(EccPoint_isZero(p_publicKey))
    {
        return 0;
    }
    
    if(vli_cmp(curve_p, p_publicKey->x) != 1 || vli_cmp(curve_p, p_publicKey->y) != 1)
    {
        return 0;
    }
    
    vli_modSquare_fast(l_tmp1, p_publicKey->y); /* tmp1 = y^2 */
    
    curve_x_side(l_tmp2, p_publicKey->x); /* tmp2 = x^3 - 3x + b */
    
    /* Make sure that y^2 == x^3 + ax + b */
    if(vli_cmp(l_tmp1, l_tmp2) != 0)
    {
        return 0;
    }
    
    return 1;
}

int ecdh_shared_secret(uint32_t p_secret[NUM_ECC_DIGITS], EccPoint *p_publicKey, uint32_t p_privateKey[NUM_ECC_DIGITS], uint32_t p_random[NUM_ECC_DIGITS])
{
    EccPoint l_product;
    
    EccPoint_mult(&l_product, p_publicKey, p_privateKey, p_random, vli_numBits(p_privateKey));
    if(EccPoint_isZero(&l_product))
    {
        return 0;
    }
    
    vli_set(p_secret, l_product.x);
    
    return 1;
}

/* -------- ECDSA code -------- */

/* Computes p_result = (p_left * p_right) % p_mod. */
static void vli_modMult(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right, uint32_t *p_mod)
{
    uint32_t l_product[2 * NUM_ECC_DIGITS];
    uint32_t l_modMultiple[2 * NUM_ECC_DIGITS];
    uint l_digitShift, l_bitShift;
    uint l_productBits;
    uint l_modBits = vli_numBits(p_mod);
    
    vli_mult(l_product, p_left, p_right);
    l_productBits = vli_numBits(l_product + NUM_ECC_DIGITS);
    if(l_productBits)
    {
        l_productBits += NUM_ECC_DIGITS * 32;
    }
    else
    {
        l_productBits = vli_numBits(l_product);
    }
    
    if(l_productBits < l_modBits)
    { /* l_product < p_mod. */
        vli_set(p_result, l_product);
        return;
    }
    
    /* Shift p_mod by (l_leftBits - l_modBits). This multiplies p_mod by the largest
       power of two possible while still resulting in a number less than p_left. */
    vli_clear(l_modMultiple);
    vli_clear(l_modMultiple + NUM_ECC_DIGITS);
    l_digitShift = (l_productBits - l_modBits) / 32;
    l_bitShift = (l_productBits - l_modBits) % 32;
    if(l_bitShift)
    {
        l_modMultiple[l_digitShift + NUM_ECC_DIGITS] = vli_lshift(l_modMultiple + l_digitShift, p_mod, l_bitShift);
    }
    else
    {
        vli_set(l_modMultiple + l_digitShift, p_mod);
    }

    /* Subtract all multiples of p_mod to get the remainder. */
    vli_clear(p_result);
    p_result[0] = 1; /* Use p_result as a temp var to store 1 (for subtraction) */
    while(l_productBits > NUM_ECC_DIGITS * 32 || vli_cmp(l_modMultiple, p_mod) >= 0)
    {
        int l_cmp = vli_cmp(l_modMultiple + NUM_ECC_DIGITS, l_product + NUM_ECC_DIGITS);
        if(l_cmp < 0 || (l_cmp == 0 && vli_cmp(l_modMultiple, l_product) <= 0))
        {
            if(vli_sub(l_product, l_product, l_modMultiple))
            { /* borrow */
                vli_sub(l_product + NUM_ECC_DIGITS, l_product + NUM_ECC_DIGITS, p_result);
            }
            vli_sub(l_product + NUM_ECC_DIGITS, l_product + NUM_ECC_DIGITS, l_modMultiple + NUM_ECC_DIGITS);
        }
        uint32_t l_carry = (l_modMultiple[NUM_ECC_DIGITS] & 0x01) << 31;
        vli_rshift1(l_modMultiple + NUM_ECC_DIGITS);
        vli_rshift1(l_modMultiple);
        l_modMultiple[NUM_ECC_DIGITS-1] |= l_carry;
        
        --l_productBits;
    }
    vli_set(p_result, l_product);
}

static uint max(uint a, uint b)
{
    return (a > b ? a : b);
}

int ecdsa_sign(uint32_t r[NUM_ECC_DIGITS], uint32_t s[NUM_ECC_DIGITS], uint32_t p_privateKey[NUM_ECC_DIGITS],
    uint32_t p_random[NUM_ECC_DIGITS], uint32_t p_hash[NUM_ECC_DIGITS])
{
    uint32_t k[NUM_ECC_DIGITS];
    uint32_t *k2[2] = {r, s};
    uint32_t l_carry;
    EccPoint p;
    
    if(vli_isZero(p_random))
    { /* The random number must not be 0. */
        return 0;
    }
    
    vli_set(k, p_random);
    if(vli_cmp(curve_n, k) != 1)
    {
        return 0;
    }
    
    /* make sure that we don't leak timing information about k. See http://eprint.iacr.org/2011/232.pdf */
    l_carry = vli_add(r, k, curve_n);
    vli_add(s, r, curve_n);
    
    /* p = k * G */
    EccPoint_mult(&p, &curve_G, k2[!l_carry], NULL, (NUM_ECC_DIGITS * 32) + 1);
    
    /* r = x1 (mod n) */
    vli_set(r, p.x);
    if(vli_cmp(curve_n, r) != 1)
    {
        vli_sub(r, r, curve_n);
    }
    if(vli_isZero(r))
    { /* If r == 0, fail (need a different random number). */
        return 0;
    }
    
    vli_modMult(s, r, p_privateKey, curve_n); /* s = r*d */
    vli_modAdd(s, p_hash, s, curve_n); /* s = e + r*d */
    vli_modInv(k, k, curve_n); /* k = 1 / k */
    vli_modMult(s, s, k, curve_n); /* s = (e + r*d) / k */
    
    return 1;
}

int ecdsa_verify(EccPoint *p_publicKey, uint32_t p_hash[NUM_ECC_DIGITS], uint32_t r[NUM_ECC_DIGITS], uint32_t s[NUM_ECC_DIGITS])
{
    uint32_t u1[NUM_ECC_DIGITS], u2[NUM_ECC_DIGITS];
    uint32_t z[NUM_ECC_DIGITS];
    EccPoint l_sum;
    uint32_t rx[NUM_ECC_DIGITS];
    uint32_t ry[NUM_ECC_DIGITS];
    uint32_t tx[NUM_ECC_DIGITS];
    uint32_t ty[NUM_ECC_DIGITS];
    uint32_t tz[NUM_ECC_DIGITS];
    
    if(vli_isZero(r) || vli_isZero(s))
    { /* r, s must not be 0. */
        return 0;
    }
    
    if(vli_cmp(curve_n, r) != 1 || vli_cmp(curve_n, s) != 1)
    { /* r, s must be < n. */
        return 0;
    }

    /* Calculate u1 and u2. */
    vli_modInv(z, s, curve_n); /* Z = s^-1 */
    vli_modMult(u1, p_hash, z, curve_n); /* u1 = e/s */
    vli_modMult(u2, r, z, curve_n); /* u2 = r/s */
    
    /* Calculate l_sum = G + Q. */
    vli_set(l_sum.x, p_publicKey->x);
    vli_set(l_sum.y, p_publicKey->y);
    vli_set(tx, curve_G.x);
    vli_set(ty, curve_G.y);
    vli_modSub(z, l_sum.x, tx, curve_p); /* Z = x2 - x1 */
    XYcZ_add(tx, ty, l_sum.x, l_sum.y);
    vli_modInv(z, z, curve_p); /* Z = 1/Z */
    apply_z(l_sum.x, l_sum.y, z);
    
    /* Use Shamir's trick to calculate u1*G + u2*Q */
    EccPoint *l_points[4] = {NULL, &curve_G, p_publicKey, &l_sum};
    uint l_numBits = max(vli_numBits(u1), vli_numBits(u2));
    
    EccPoint *l_point = l_points[(!!vli_testBit(u1, l_numBits-1)) | ((!!vli_testBit(u2, l_numBits-1)) << 1)];
    vli_set(rx, l_point->x);
    vli_set(ry, l_point->y);
    vli_clear(z);
    z[0] = 1;

    int i;
    for(i = l_numBits - 2; i >= 0; --i)
    {
        EccPoint_double_jacobian(rx, ry, z);
        
        int l_index = (!!vli_testBit(u1, i)) | ((!!vli_testBit(u2, i)) << 1);
        l_point = l_points[l_index];
        if(l_point)
        {
            vli_set(tx, l_point->x);
            vli_set(ty, l_point->y);
            apply_z(tx, ty, z);
            vli_modSub(tz, rx, tx, curve_p); /* Z = x2 - x1 */
            XYcZ_add(tx, ty, rx, ry);
            vli_modMult_fast(z, z, tz);
        }
    }

    vli_modInv(z, z, curve_p); /* Z = 1/Z */
    apply_z(rx, ry, z);
    
    /* v = x1 (mod n) */
    if(vli_cmp(curve_n, rx) != 1)
    {
        vli_sub(rx, rx, curve_n);
    }

    /* Accept only if v == r. */
    return (vli_cmp(rx, r) == 0);
}

void ecc_bytes2native(uint32_t p_native[NUM_ECC_DIGITS], uint8_t p_bytes[NUM_ECC_DIGITS*4])
{
    unsigned i;
    for(i=0; i<NUM_ECC_DIGITS; ++i)
    {
        uint8_t *p_digit = p_bytes + 4 * (NUM_ECC_DIGITS - 1 - i);
        p_native[i] = ((uint32_t)p_digit[0] << 24) | ((uint32_t)p_digit[1] << 16) | ((uint32_t)p_digit[2] << 8) | (uint32_t)p_digit[3];
    }
}

void ecc_native2bytes(uint8_t p_bytes[NUM_ECC_DIGITS*4], uint32_t p_native[NUM_ECC_DIGITS])
{
    unsigned i;
    for(i=0; i<NUM_ECC_DIGITS; ++i)
    {
        uint8_t *p_digit = p_bytes + 4 * (NUM_ECC_DIGITS - 1 - i);
        p_digit[0] = p_native[i] >> 24;
        p_digit[1] = p_native[i] >> 16;
        p_digit[2] = p_native[i] >> 8;
        p_digit[3] = p_native[i];
    }
}

/* Compute a = sqrt(a) (mod curve_p). */
static void mod_sqrt(uint32_t a[NUM_ECC_DIGITS])
{
    unsigned i;
    uint32_t p1[NUM_ECC_DIGITS] = {1};
    uint32_t l_result[NUM_ECC_DIGITS] = {1};
    
    /* Since curve_p == 3 (mod 4) for all supported curves, we can
       compute sqrt(a) = a^((curve_p + 1) / 4) (mod curve_p). */
    vli_add(p1, curve_p, p1); /* p1 = curve_p + 1 */
    for(i = vli_numBits(p1) - 1; i > 1; --i)
    {
        vli_modSquare_fast(l_result, l_result);
        if(vli_testBit(p1, i))
        {
            vli_modMult_fast(l_result, l_result, a);
        }
    }
    vli_set(a, l_result);
}

void ecc_point_compress(uint8_t p_compressed[NUM_ECC_DIGITS*4 + 1], EccPoint *p_point)
{
    p_compressed[0] = 2 + (p_point->y[0] & 0x01);
    ecc_native2bytes(p_compressed + 1, p_point->x);
}

void ecc_point_decompress(EccPoint *p_point, uint8_t p_compressed[NUM_ECC_DIGITS*4 + 1])
{
    ecc_bytes2native(p_point->x, p_compressed + 1);
    curve_x_side(p_point->y, p_point->x);
    mod_sqrt(p_point->y);
    if((p_point->y[0] & 0x01) != (p_compressed[0] & 0x01))
    {
        vli_sub(p_point->y, curve_p, p_point->y);
    }
}
