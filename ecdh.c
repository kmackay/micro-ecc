#include "ecdh.h"

#include <string.h>

typedef unsigned int uint;

#define CONCAT1(a, b) a##b
#define CONCAT(a, b) CONCAT1(a, b)

#define Curve_P_4 {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFD}
#define Curve_P_5 {0x7FFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_P_6 {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_P_7 {0x00000001, 0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_P_8 {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0x00000000, 0x00000000, 0x00000001, 0xFFFFFFFF}

#define Curve_A_4 {0xFFFFFFFC, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFD}
#define Curve_B_4 {0x2CEE5ED3, 0xD824993C, 0x1079F43D, 0xE87579C1}
#define Curve_A_5 {0x7FFFFFFC, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_B_5 {0xC565FA45, 0x81D4D4AD, 0x65ACF89F, 0x54BD7A8B, 0x1C97BEFC}
#define Curve_A_6 {0xFFFFFFFC, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_B_6 {0xC146B9B1, 0xFEB8DEEC, 0x72243049, 0x0FA7E9AB, 0xE59C80E7, 0x64210519}
#define Curve_A_7 {0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_B_7 {0x2355FFB4, 0x270B3943, 0xD7BFD8BA, 0x5044B0B7, 0xF5413256, 0x0C04B3AB, 0xB4050A85}
#define Curve_A_8 {0xFFFFFFFC, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0x00000000, 0x00000000, 0x00000001, 0xFFFFFFFF}
#define Curve_B_8 {0x27D2604B, 0x3BCE3C3E, 0xCC53B0F6, 0x651D06B0, 0x769886BC, 0xB3EBBD55, 0xAA3A93E7, 0x5AC635D8}

#define Curve_G_4 { \
	{0xA52C5B86, 0x0C28607C, 0x8B899B2D, 0x161FF752}, \
	{0xDDED7A83, 0xC02DA292, 0x5BAFEB13, 0xCF5AC839}}
	
#define Curve_G_5 { \
	{0x13CBFC82, 0x68C38BB9, 0x46646989, 0x8EF57328, 0x4A96B568}, \
	{0x7AC5FB32, 0x04235137, 0x59DCC912, 0x3168947D, 0x23A62855}}

#define Curve_G_6 { \
	{0x82FF1012, 0xF4FF0AFD, 0x43A18800, 0x7CBF20EB, 0xB03090F6, 0x188DA80E}, \
	{0x1E794811, 0x73F977A1, 0x6B24CDD5, 0x631011ED, 0xFFC8DA78, 0x07192B95}}

#define Curve_G_7 { \
	{0x115C1D21, 0x343280D6, 0x56C21122, 0x4A03C1D3, 0x321390B9, 0x6BB4BF7F, 0xB70E0CBD}, \
	{0x85007E34, 0x44D58199, 0x5A074764, 0xCD4375A0, 0x4C22DFE6, 0xB5F723FB, 0xBD376388}}
	
#define Curve_G_8 { \
	{0xD898C296, 0xF4A13945, 0x2DEB33A0, 0x77037D81, 0x63A440F2, 0xF8BCE6E5, 0xE12C4247, 0x6B17D1F2}, \
	{0x37BF51F5, 0xCBB64068, 0x6B315ECE, 0x2BCE3357, 0x7C0F9E16, 0x8EE7EB4A, 0xFE1A7F9B, 0x4FE342E2}}

#define Curve_N_4 {0x9038A115, 0x75A30D1B, 0x00000000, 0xFFFFFFFE}
#define N_size_4 4
#define Curve_N_5 {0xca752257, 0xf927aed3, 0x0001f4c8, 0x00000000, 0x00000000, 0x00000001}
#define N_size_5 6
#define Curve_N_6 {0xB4D22831, 0x146BC9B1, 0x99DEF836, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define N_size_6 6
#define Curve_N_7 {0x5C5C2A3D, 0x13DD2945, 0xE0B8F03E, 0xFFFF16A2, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define N_size_7 7
#define Curve_N_8 {0xD0364141, 0xBFD25E8C, 0xAF48A03B, 0xBAAEDCE6, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define N_size_8 8

#define N_size CONCAT(N_size_, ECC_CURVE)

static uint32_t curve_p[NUM_ECC_DIGITS] = CONCAT(Curve_P_, ECC_CURVE);
static uint32_t curve_a[NUM_ECC_DIGITS] = CONCAT(Curve_A_, ECC_CURVE);
static uint32_t curve_b[NUM_ECC_DIGITS] = CONCAT(Curve_B_, ECC_CURVE);
static EccPoint curve_G = CONCAT(Curve_G_, ECC_CURVE);
static uint32_t curve_n[N_size] = CONCAT(Curve_N_, ECC_CURVE);

static void vli_modMult(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right, uint p_size, uint32_t *p_mod, uint p_modSize);

static void vli_clear(uint32_t *p_vli, uint p_size)
{
    uint i;
    for(i=0; i<p_size; ++i)
    {
        p_vli[i] = 0;
    }
}

/* Returns 1 if p_vli == 0, 0 otherwise. */
static int vli_isZero(uint32_t *p_vli, uint p_size)
{
	uint i;
	for(i = 0; i < p_size; ++i)
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
static uint vli_numDigits(uint32_t *p_vli, uint p_size)
{
	int i;
	/* Search from the end until we find a non-zero digit.
	   We do it in reverse because we expect that most digits will be nonzero. */
	for (i = p_size - 1; i >= 0 && p_vli[i] == 0; --i)
	{
	}

	return (i + 1);
}

/* Counts the number of bits required for p_vli. */
static uint vli_numBits(uint32_t *p_vli, uint p_size)
{
    uint i;
    uint32_t l_digit;
    
	uint l_numDigits = vli_numDigits(p_vli, p_size);
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
static void vli_set(uint32_t *p_dest, uint32_t *p_src, uint p_size)
{
    uint i;
    for(i=0; i<p_size; ++i)
    {
        p_dest[i] = p_src[i];
    }
}

/* Returns sign of p_left - p_right. */
static int vli_cmp(uint32_t *p_left, uint32_t *p_right, uint p_size)
{
	int i;
	for(i = p_size-1; i >= 0; --i)
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
static uint32_t vli_lshift(uint32_t *p_result, uint32_t *p_in, uint p_shift, uint p_size)
{
	uint32_t l_carry = 0;
	uint i;
	for(i = 0; i < p_size; ++i)
	{
		uint32_t l_temp = p_in[i];
		p_result[i] = (l_temp << p_shift) | l_carry;
		l_carry = l_temp >> (32 - p_shift);
	}
	
	return l_carry;
}

/* Computes p_vli = p_vli >> 1. */
static void vli_rshift1(uint32_t *p_vli, uint p_size)
{
	uint32_t *l_end = p_vli;
	uint32_t l_carry = 0;
	
	p_vli += p_size;
	while(p_vli-- > l_end)
	{
		uint32_t l_temp = *p_vli;
		*p_vli = (l_temp >> 1) | l_carry;
		l_carry = l_temp << 31;
	}
}

/* Computes p_result = p_left + p_right, returning carry. Can modify in place.
   Could be much more efficient in asm. */
static uint32_t vli_add(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right, uint p_size)
{
	uint32_t l_carry = 0;
	uint i;
	for(i=0; i<p_size; ++i)
	{
		uint l_sum = p_left[i] + l_carry;
		if(l_sum == 0)
		{ /* Sum was 0, or carry. */
			l_sum = p_right[i];
		}
		else
		{ /* No carry. */
			l_sum += p_right[i];
			l_carry = (l_sum < p_right[i]);
		}
		p_result[i] = l_sum;
	}
	return l_carry;
}

/* Computes p_result = p_left - p_right, returning borrow. Can modify in place. */
static uint32_t vli_sub(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right, uint p_size)
{
	uint32_t l_borrow = 0;
	uint i;
	for(i=0; i<p_size; ++i)
	{
		uint32_t l_diff = p_left[i] - l_borrow;
		if(l_diff == 0xfffffffful)
		{ /* Already borrowed (from 0) (so l_borrow == 1). */
			l_diff -= p_right[i];
		}
		else
		{ /* No borrow yet. */
			l_diff -= p_right[i];
			l_borrow = (l_diff > 0xfffffffful - p_right[i]);
		}
		p_result[i] = l_diff;
	}
	return l_borrow;
}

/* Computes p_result = p_left * p_right. */
static void vli_mult(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right, uint p_size)
{
    uint64_t r01 = 0;
    uint32_t r2 = 0;
    
    uint i, k;
    
    /* Compute each digit of p_result in sequence, maintaining the carries. */
    for(k=0; k < p_size*2 - 1; ++k)
    {
        uint l_min = (k < p_size ? 0 : (k + 1) - p_size);
        for(i=l_min; i<=k && i<p_size; ++i)
        {
        #if ECC_SOFT_MULT64
            uint32_t a0 = p_left[i] & 0xffff;
            uint32_t a1 = p_left[i] >> 16;
            uint32_t b0 = p_right[k-i] & 0xffff;
            uint32_t b1 = p_right[k-i] >> 16;
            
            uint64_t l_product = (a0 * b0) + (((uint64_t)(a0 * b1) + a1 * b0) << 16) + (((uint64_t)(a1 * b1)) << 32);
        #else
            uint64_t l_product = (uint64_t)p_left[i] * p_right[k-i];
        #endif
            r01 += l_product;
            r2 += (r01 < l_product);
        }
        p_result[k] = (uint32_t)r01;
        r01 = (r01 >> 32) | (((uint64_t)r2) << 32);
        r2 = 0;
    }
    
    p_result[p_size*2 - 1] = (uint32_t)r01;
}

/* Computes p_result = (p_left + p_right) % p_mod.
   Assumes that p_left < p_mod and p_right < p_mod, p_result != p_mod. */
static void vli_modAdd(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right, uint32_t *p_mod, uint p_size)
{
	uint32_t l_carry = vli_add(p_result, p_left, p_right, p_size);
	if(l_carry || vli_cmp(p_result, p_mod, p_size) >= 0)
	{ /* p_result > p_mod (p_result = p_mod + remainder), so subtract p_mod to get remainder. */
		vli_sub(p_result, p_result, p_mod, p_size);
	}
}

/* Computes p_result = (p_left - p_right) % p_mod.
   Assumes that p_left < p_mod and p_right < p_mod, p_result != p_mod. */
static void vli_modSub(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right, uint32_t *p_mod, uint p_size)
{
	uint32_t l_borrow = vli_sub(p_result, p_left, p_right, p_size);
	if(l_borrow)
	{ /* In this case, p_result == -diff == (max int) - diff.
		 Since -x % d == d - x, we can get the correct result from p_result + p_mod (with overflow). */
		vli_add(p_result, p_result, p_mod, p_size );
	}
}

#if ECC_CURVE == secp128r1

/* Computes p_result = p_product % curve_p.
   See algorithm 5 and 6 from http://www.isys.uni-klu.ac.at/PDF/2001-0126-MT.pdf */
static void vli_mmod_fast(uint32_t *p_result, uint32_t *p_product)
{
    uint32_t l_tmp[NUM_ECC_DIGITS];
    int l_carry;
	
    vli_set(p_result, p_product, NUM_ECC_DIGITS);
    
    l_tmp[0] = p_product[4];
    l_tmp[1] = p_product[5];
    l_tmp[2] = p_product[6];
    l_tmp[3] = (p_product[7] & 0x00000001) | (p_product[4] << 1);
    l_carry = vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    l_tmp[0] = (p_product[4] >> 31) | (p_product[5] << 1);
    l_tmp[1] = (p_product[5] >> 31) | (p_product[6] << 1);
    l_tmp[2] = (p_product[6] >> 31) | (p_product[7] << 1);
    l_tmp[3] = (p_product[7] >> 31) | ((p_product[4] & 0x80000000) >> 30) | (p_product[5] << 2);
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    l_tmp[0] = (p_product[5] >> 30) | (p_product[6] << 2);
    l_tmp[1] = (p_product[6] >> 30) | (p_product[7] << 2);
    l_tmp[2] = (p_product[7] >> 30);
    l_tmp[3] = ((p_product[5] & 0xC0000000) >> 29) | (p_product[6] << 3);
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    l_tmp[0] = (p_product[6] >> 29) | (p_product[7] << 3);
    l_tmp[1] = (p_product[7] >> 29);
    l_tmp[2] = 0;
    l_tmp[3] = ((p_product[6] & 0xE0000000) >> 28) | (p_product[7] << 4);
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    l_tmp[0] = (p_product[7] >> 28);
    l_tmp[1] = 0;
    l_tmp[2] = 0;
    l_tmp[3] = (p_product[7] & 0xFFFFFFFE);
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    l_tmp[0] = 0;
    l_tmp[1] = 0;
    l_tmp[2] = 0;
    l_tmp[3] = ((p_product[7] & 0xF0000000) >> 27);
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    while(l_carry || vli_cmp(curve_p, p_result, NUM_ECC_DIGITS) != 1)
    {
        l_carry -= vli_sub(p_result, p_result, curve_p, NUM_ECC_DIGITS);
    }
}

#elif ECC_CURVE == secp160r1

/* Computes p_result = p_product % curve_p.
   See algorithm 5 and 6 from http://www.isys.uni-klu.ac.at/PDF/2001-0126-MT.pdf */
static void vli_mmod_fast(uint32_t *p_result, uint32_t *p_product)
{
    uint32_t l_tmp[NUM_ECC_DIGITS];
    int l_carry;
	
    vli_set(p_result, p_product, NUM_ECC_DIGITS);
    
    l_tmp[0] = (p_product[5] & 0x7FFFFFFF) | (p_product[5] << 31);
    l_tmp[1] = (p_product[5] >> 1) | (p_product[6] << 31);
    l_tmp[2] = (p_product[6] >> 1) | (p_product[7] << 31);
    l_tmp[3] = (p_product[7] >> 1) | (p_product[8] << 31);
    l_tmp[4] = (p_product[8] >> 1) | (p_product[9] << 31);
    l_carry = vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    l_tmp[0] = (p_product[9] >> 1) | (p_product[5] & 0x80000000);
    l_tmp[1] = p_product[6];
    l_tmp[2] = p_product[7];
    l_tmp[3] = p_product[8];
    l_tmp[4] = p_product[9];
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    l_tmp[0] = ((p_product[9] & 0x00000002) << 30);
    l_tmp[1] = (p_product[9] >> 2);
    l_tmp[2] = 0;
    l_tmp[3] = 0;
    l_tmp[4] = 0;
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    while(l_carry || vli_cmp(curve_p, p_result, NUM_ECC_DIGITS) != 1)
    {
        l_carry -= vli_sub(p_result, p_result, curve_p, NUM_ECC_DIGITS);
    }
}

#elif ECC_CURVE == secp192r1

/* Computes p_result = p_product % curve_p.
   See algorithm 5 and 6 from http://www.isys.uni-klu.ac.at/PDF/2001-0126-MT.pdf */
static void vli_mmod_fast(uint32_t *p_result, uint32_t *p_product)
{
    uint32_t l_tmp[NUM_ECC_DIGITS];
    int l_carry;
	
    vli_set(p_result, p_product, NUM_ECC_DIGITS);
    
    vli_set(l_tmp, &p_product[6], NUM_ECC_DIGITS);
    l_carry = vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    l_tmp[0] = l_tmp[1] = 0;
    l_tmp[2] = p_product[6];
    l_tmp[3] = p_product[7];
    l_tmp[4] = p_product[8];
    l_tmp[5] = p_product[9];
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    l_tmp[0] = l_tmp[2] = p_product[10];
    l_tmp[1] = l_tmp[3] = p_product[11];
    l_tmp[4] = l_tmp[5] = 0;
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    while(l_carry || vli_cmp(curve_p, p_result, NUM_ECC_DIGITS) != 1)
    {
        l_carry -= vli_sub(p_result, p_result, curve_p, NUM_ECC_DIGITS);
    }
}

#elif ECC_CURVE == secp224r1

/* Computes p_result = p_product % curve_p.
   See http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void vli_mmod_fast(uint32_t *p_result, uint32_t *p_product)
{
    uint32_t l_tmp[NUM_ECC_DIGITS];
    int l_carry;
	
    vli_set(p_result, p_product, NUM_ECC_DIGITS);
    
    l_tmp[0] = l_tmp[1] = l_tmp[2] = 0;
    l_tmp[3] = p_product[7];
    l_tmp[4] = p_product[8];
    l_tmp[5] = p_product[9];
    l_tmp[6] = p_product[10];
    l_carry = vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    l_tmp[3] = p_product[11];
    l_tmp[4] = p_product[12];
    l_tmp[5] = p_product[13];
    l_tmp[6] = 0;
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    vli_set(l_tmp, &p_product[7]);
    l_carry -= vli_sub(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    l_tmp[0] = p_product[11];
    l_tmp[1] = p_product[12];
    l_tmp[2] = p_product[13];
    l_tmp[3] = l_tmp[4] = l_tmp[5] = l_tmp[6] = 0;
    l_carry -= vli_sub(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    if(l_carry < 0)
    {
        do
        {
            l_carry += vli_add(p_result, p_result, curve_p, NUM_ECC_DIGITS);
        } while(l_carry < 0);
    }
    else
    {
        while(l_carry || vli_cmp(curve_p, p_result, NUM_ECC_DIGITS) != 1)
        {
            l_carry -= vli_sub(p_result, p_result, curve_p, NUM_ECC_DIGITS);
        }
    }
}

#elif ECC_CURVE == secp256r1

/* Computes p_result = p_product % curve_p
   from http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void vli_mmod_fast(uint32_t *p_result, uint32_t *p_product)
{
    uint32_t l_tmp[NUM_ECC_DIGITS];
    int l_carry;
	
	/* t */
    vli_set(p_result, p_product, NUM_ECC_DIGITS);
    
    /* s1 */
    l_tmp[0] = l_tmp[1] = l_tmp[2] = 0;
    l_tmp[3] = p_product[11];
    l_tmp[4] = p_product[12];
    l_tmp[5] = p_product[13];
    l_tmp[6] = p_product[14];
    l_tmp[7] = p_product[15];
    l_carry = vli_lshift(l_tmp, l_tmp, 1, NUM_ECC_DIGITS);
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    /* s2 */
    l_tmp[3] = p_product[12];
    l_tmp[4] = p_product[13];
    l_tmp[5] = p_product[14];
    l_tmp[6] = p_product[15];
    l_tmp[7] = 0;
    l_carry += vli_lshift(l_tmp, l_tmp, 1, NUM_ECC_DIGITS);
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    /* s3 */
    l_tmp[0] = p_product[8];
    l_tmp[1] = p_product[9];
    l_tmp[2] = p_product[10];
    l_tmp[3] = l_tmp[4] = l_tmp[5] = 0;
    l_tmp[6] = p_product[14];
    l_tmp[7] = p_product[15];
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    /* s4 */
    l_tmp[0] = p_product[9];
    l_tmp[1] = p_product[10];
    l_tmp[2] = p_product[11];
    l_tmp[3] = p_product[13];
    l_tmp[4] = p_product[14];
    l_tmp[5] = p_product[15];
    l_tmp[6] = p_product[13];
    l_tmp[7] = p_product[8];
    l_carry += vli_add(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    /* d1 */
    l_tmp[0] = p_product[11];
    l_tmp[1] = p_product[12];
    l_tmp[2] = p_product[13];
    l_tmp[3] = l_tmp[4] = l_tmp[5] = 0;
    l_tmp[6] = p_product[8];
    l_tmp[7] = p_product[10];
    l_carry -= vli_sub(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    /* d2 */
    l_tmp[0] = p_product[12];
    l_tmp[1] = p_product[13];
    l_tmp[2] = p_product[14];
    l_tmp[3] = p_product[15];
    l_tmp[4] = l_tmp[5] = 0;
    l_tmp[6] = p_product[9];
    l_tmp[7] = p_product[11];
    l_carry -= vli_sub(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    /* d3 */
    l_tmp[0] = p_product[13];
    l_tmp[1] = p_product[14];
    l_tmp[2] = p_product[15];
    l_tmp[3] = p_product[8];
    l_tmp[4] = p_product[9];
    l_tmp[5] = p_product[10];
    l_tmp[6] = 0;
    l_tmp[7] = p_product[12];
    l_carry -= vli_sub(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    /* d4 */
    l_tmp[0] = p_product[14];
    l_tmp[1] = p_product[15];
    l_tmp[2] = 0;
    l_tmp[3] = p_product[9];
    l_tmp[4] = p_product[10];
    l_tmp[5] = p_product[11];
    l_tmp[6] = 0;
    l_tmp[7] = p_product[13];
    l_carry -= vli_sub(p_result, p_result, l_tmp, NUM_ECC_DIGITS);
    
    if(l_carry < 0)
    {
        do
        {
            l_carry += vli_add(p_result, p_result, curve_p, NUM_ECC_DIGITS);
        } while(l_carry < 0);
    }
    else
    {
        while(l_carry || vli_cmp(curve_p, p_result, NUM_ECC_DIGITS) != 1)
        {
            l_carry -= vli_sub(p_result, p_result, curve_p, NUM_ECC_DIGITS);
        }
    }
}

#endif

/* Computes p_result = (p_left * p_right) % curve_p. */
static void vli_modMult_fast(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right)
{
   uint32_t l_product[2 * NUM_ECC_DIGITS];
   vli_mult(l_product, p_left, p_right, NUM_ECC_DIGITS);
   vli_mmod_fast(p_result, l_product);
}

#if ECC_SQUARE_FUNC

/* Computes p_result = p_left^2. */
static void vli_square(uint32_t *p_result, uint32_t *p_left, uint p_size)
{
    uint64_t r01 = 0;
    uint32_t r2 = 0;
    
    uint i, k;
    for(k=0; k < p_size*2 - 1; ++k)
    {
        uint l_min = (k < p_size ? 0 : (k + 1) - p_size);
        for(i=l_min; i<=k && i<=k-i; ++i)
        {
        #if ECC_SOFT_MULT64
            uint32_t a0 = p_left[i] & 0xffff;
            uint32_t a1 = p_left[i] >> 16;
            uint32_t b0 = p_left[k-i] & 0xffff;
            uint32_t b1 = p_left[k-i] >> 16;

            uint64_t l_product = (a0 * b0) + (((uint64_t)(a0 * b1) + a1 * b0) << 16) + (((uint64_t)(a1 * b1)) << 32);
        #else
            uint64_t l_product = (uint64_t)p_left[i] * p_left[k-i];
        #endif
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
    
    p_result[p_size*2 - 1] = (uint32_t)r01;
}

/* Computes p_result = p_left^2 % curve_p. */
static void vli_modSquare_fast(uint32_t *p_result, uint32_t *p_left)
{
	uint32_t l_product[2 * NUM_ECC_DIGITS];
	vli_square(l_product, p_left, NUM_ECC_DIGITS);
	vli_mmod_fast(p_result, l_product);
}

#else /* ECC_SQUARE_FUNC */

#define vli_square(result, left, size) vli_mult((result), (left), (left), (size))
#define vli_modSquare_fast(result, left) vli_modMult_fast((result), (left), (left))
    
#endif /* ECC_SQUARE_FUNC */

#define EVEN(vli) (!(vli[0] & 1))
/* Computes p_result = (p_left / p_right) % p_mod. All VLIs are the same size.
   See "From Euclid's GCD to Montgomery Multiplication to the Great Divide"
   https://labs.oracle.com/techrep/2001/smli_tr-2001-95.pdf */
static void vli_modDiv(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right, uint32_t *p_mod, uint p_size)
{
	uint32_t a[p_size], b[p_size], u[p_size], v[p_size];
	uint32_t l_carry;
	
	vli_set(a, p_right, p_size);
	vli_set(b, p_mod, p_size);
	vli_set(u, p_left, p_size);
    vli_clear(v, p_size);
	
	int l_cmpResult;
	while ((l_cmpResult = vli_cmp(a, b, p_size)) != 0)
	{
		l_carry = 0;
		if(EVEN(a))
		{
			vli_rshift1(a, p_size);
			if(!EVEN(u))
			{
				l_carry = vli_add(u, u, p_mod, p_size);
			}
			vli_rshift1(u, p_size);
			if(l_carry)
			{
				u[p_size-1] |= 0x80000000;
			}
		}
		else if(EVEN(b))
		{
			vli_rshift1(b, p_size);
			if(!EVEN(v))
			{
				l_carry = vli_add(v, v, p_mod, p_size);
			}
			vli_rshift1(v, p_size);
			if(l_carry)
			{
				v[p_size-1] |= 0x80000000;
			}
		}
		else if(l_cmpResult > 0)
		{
			vli_sub(a, a, b, p_size);
			vli_rshift1(a, p_size);
			if(vli_cmp(u, v, p_size) < 0)
			{
				vli_add(u, u, p_mod, p_size);
			}
			vli_sub(u, u, v, p_size);
			if(!EVEN(u))
			{
				l_carry = vli_add(u, u, p_mod, p_size);
			}
			vli_rshift1(u, p_size);
			if(l_carry)
			{
				u[p_size-1] |= 0x80000000;
			}
		}
		else
		{
			vli_sub(b, b, a, p_size);
			vli_rshift1(b, p_size);
			if(vli_cmp(v, u, p_size) < 0)
			{
				vli_add(v, v, p_mod, p_size);
			}
			vli_sub(v, v, u, p_size);
			if(!EVEN(v))
			{
				l_carry = vli_add(v, v, p_mod, p_size);
			}
			vli_rshift1(v, p_size);
			if(l_carry)
			{
				v[p_size-1] |= 0x80000000;
			}
		}
	}
	
	vli_set(p_result, u, p_size);
}

/* Computes p_result = (1 / p_input) % p_mod. All VLIs are the same size. */
static void vli_modInv(uint32_t *p_result, uint32_t *p_input, uint32_t *p_mod, uint p_size)
{
    uint32_t n[p_size];
    
    vli_clear(n, p_size);
    n[0] = 1;
	
	vli_modDiv(p_result, n, p_input, p_mod, p_size);
}

/* ------ Point operations ------ */

/* Clears a point (set it to the point at infinity). */
static void EccPoint_clear(EccPoint *p_point)
{
    vli_clear(p_point->x, NUM_ECC_DIGITS);
    vli_clear(p_point->y, NUM_ECC_DIGITS);
}

/* Copies a point. */
static void EccPoint_copy(EccPoint *p_dest, EccPoint *p_src)
{
	if(p_dest != p_src)
	{
		vli_set(p_dest->x, p_src->x, NUM_ECC_DIGITS);
		vli_set(p_dest->y, p_src->y, NUM_ECC_DIGITS);
	}
}

/* Returns 1 if p_point is the point at infinity, 0 otherwise. */
static int EccPoint_isZero(EccPoint *p_point)
{
	return (vli_isZero(p_point->x, NUM_ECC_DIGITS) && vli_isZero(p_point->y, NUM_ECC_DIGITS));
}

/* Modified Jacobian point doubling. Note that we use the fact that a is equivalent to -3 (mod p) for the supported
   curves, we can transform M = (3 * x1^2 + a * Z1^2) to M = 3 * (x1 + Z1^2) * (x1 - Z1^2)
*/
static void EccPoint_double_projective(EccPoint *P3, uint32_t *Z3, EccPoint *P1, uint32_t *Z1)
{
    uint32_t l_tmp1[NUM_ECC_DIGITS];
	uint32_t l_tmp2[NUM_ECC_DIGITS];
	uint32_t l_tmp3[NUM_ECC_DIGITS];
	uint32_t l_tmp4[NUM_ECC_DIGITS];
	
	if(vli_isZero(Z1, NUM_ECC_DIGITS))
	{
        vli_clear(Z3, NUM_ECC_DIGITS);
		return;
	}
	
	vli_modSquare_fast(l_tmp1, Z1); /* tmp1 = Z1^2 */
	vli_modAdd(l_tmp2, P1->x, l_tmp1, curve_p, NUM_ECC_DIGITS); /* tmp2 = x1 + Z1^2 */
	vli_modSub(l_tmp1, P1->x, l_tmp1, curve_p, NUM_ECC_DIGITS); /* tmp1 = x1 - Z1^2 */
	vli_modMult_fast(l_tmp1, l_tmp1, l_tmp2); /* tmp1 = (x1 + Z1^2) * (x1 - Z1^2) */
	vli_modAdd(l_tmp2, l_tmp1, l_tmp1, curve_p, NUM_ECC_DIGITS); /* tmp2 = 2 * (x1 + Z1^2) * (x1 - Z1^2) */
	vli_modAdd(l_tmp1, l_tmp2, l_tmp1, curve_p, NUM_ECC_DIGITS); /* tmp1 = M = 3 * (x1 + Z1^2) * (x1 - Z1^2) */
	vli_modMult_fast(Z3, P1->y, Z1); /* Z3 = y1 * Z1 */
	
	vli_modAdd(Z3, Z3, Z3, curve_p, NUM_ECC_DIGITS); /* Z3 = 2 * y1 * Z1 */
	
	vli_modSquare_fast(l_tmp3, P1->y); /* tmp3 = y1^2 */
	vli_modMult_fast(l_tmp2, l_tmp3, P1->x); /* tmp2 = x1 * y1^2 */
	vli_modAdd(l_tmp2, l_tmp2, l_tmp2, curve_p, NUM_ECC_DIGITS); /* tmp2 = 2 * x1 * y1^2 */
	vli_modAdd(l_tmp2, l_tmp2, l_tmp2, curve_p, NUM_ECC_DIGITS); /* tmp2 = S = 4 * x1 * y1^2 */
	/* Now tmp1 = M, tmp2 = S */
	
	vli_modAdd(l_tmp4, l_tmp2, l_tmp2, curve_p, NUM_ECC_DIGITS); /* tmp4 = 2*S */
	vli_modSquare_fast(P3->x, l_tmp1); /* x3 = M^2 */
	vli_modSub(P3->x, P3->x, l_tmp4, curve_p, NUM_ECC_DIGITS); /* x3 = T = M^2 - 2*S */
	/* Now tmp1 = M, tmp2 = S, x3 = T */
	
	/* tmp3 is still y1^2 at this point */
	vli_modSquare_fast(l_tmp3, l_tmp3); /* tmp3 = y1^4 */
	vli_modAdd(l_tmp3, l_tmp3, l_tmp3, curve_p, NUM_ECC_DIGITS); /* tmp3 = 2 * y1^4 */
	vli_modAdd(l_tmp3, l_tmp3, l_tmp3, curve_p, NUM_ECC_DIGITS); /* tmp3 = 4 * y1^4 */
	vli_modAdd(l_tmp3, l_tmp3, l_tmp3, curve_p, NUM_ECC_DIGITS); /* tmp3 = U = 8 * y1^4 */
	/* Now tmp1 = M, tmp2 = S, x3 = T, tmp3 = U */
	
	vli_modSub(l_tmp2, l_tmp2, P3->x, curve_p, NUM_ECC_DIGITS); /* tmp2 = S - T */
	vli_modMult_fast(l_tmp2, l_tmp1, l_tmp2); /* tmp2 = M * (S - T) */
	vli_modSub(P3->y, l_tmp2, l_tmp3, curve_p, NUM_ECC_DIGITS); /* y3 = M * (S - T) - U */
}

/* Modified Jacobian point addition; P3 = P1 + P2. Note that P1 and P3 are in projective coordinates,
   and P2 is in affine coordinates. */
static void EccPoint_add_mixed(EccPoint *P3, uint32_t *Z3, EccPoint *P1, uint32_t *Z1, EccPoint *P2)
{
    uint32_t l_tmp1[NUM_ECC_DIGITS];
	uint32_t l_tmp2[NUM_ECC_DIGITS];
	uint32_t l_tmp3[NUM_ECC_DIGITS];
	uint32_t l_tmp4[NUM_ECC_DIGITS];
	
	if(EccPoint_isZero(P2))
	{
		EccPoint_copy(P3, P1);
		vli_set(Z3, Z1, NUM_ECC_DIGITS);
		return;
	}
	
	if(vli_isZero(Z1, NUM_ECC_DIGITS))
	{
		EccPoint_copy(P3, P2);
        vli_clear(Z3, NUM_ECC_DIGITS);
		Z3[0] = 1;
		return;
	}
	
	vli_modSquare_fast(l_tmp1, Z1); /* tmp1 = Z1^2 */
	vli_modMult_fast(l_tmp2, l_tmp1, Z1); /* tmp2 = Z1^3 */
	vli_modMult_fast(l_tmp1, l_tmp1, P2->x); /* tmp1 = Z1^2 * x2 */
	vli_modSub(l_tmp3, l_tmp1, P1->x, curve_p, NUM_ECC_DIGITS); /* tmp3 = (Z1^2 * x2) - x1 */
	/* tmp2 = Z1^3, tmp3 = H */
	
	vli_modMult_fast(l_tmp2, l_tmp2, P2->y); /* tmp2 = Z1^3 * y2 */
	vli_modSub(l_tmp4, l_tmp2, P1->y, curve_p, NUM_ECC_DIGITS); /* tmp4 = (Z1^3 * y2) - y1 */
	/* tmp4 = r */
	
	if(vli_isZero(l_tmp3, NUM_ECC_DIGITS))
	{
		if(vli_isZero(l_tmp4, NUM_ECC_DIGITS))
		{ /* The points are equal, so we use the doubling formula. */
			EccPoint_double_projective(P3, Z3, P1, Z1);
		}
		else
		{
            vli_clear(Z3, NUM_ECC_DIGITS);
		}
		return;
	}
	
	vli_modMult_fast(Z3, Z1, l_tmp3); /* Z3 = H * Z1 */
	
	vli_modSquare_fast(l_tmp1, l_tmp3); /* tmp1 = H^2 */
	vli_modMult_fast(l_tmp2, l_tmp1, l_tmp3); /* tmp2 = H^3 */
	vli_modMult_fast(l_tmp1, l_tmp1, P1->x); /* tmp1 = H^2 * x1 */
	/* tmp1 = H^2 * x1, tmp2 = H^3, tmp3 = H, tmp4 = r */
	
	vli_modAdd(l_tmp3, l_tmp1, l_tmp1, curve_p, NUM_ECC_DIGITS); /* tmp3 = 2 * H^2 * x1 */
	vli_modSquare_fast(P3->x, l_tmp4); /* x3 = r^2 */
	vli_modSub(P3->x, P3->x, l_tmp3, curve_p, NUM_ECC_DIGITS); /* x3 = r^2 - (2 * H^2 * x1) */
	vli_modSub(P3->x, P3->x, l_tmp2, curve_p, NUM_ECC_DIGITS); /* x3 = r^2 - (2 * H^2 * x1) - H^3 */
	/* tmp1 = H^2 * x1, tmp2 = H^3, tmp4 = r */
	
	vli_modSub(l_tmp3, l_tmp1, P3->x, curve_p, NUM_ECC_DIGITS); /* tmp3 = (H^2 * x1) - x3 */
	vli_modMult_fast(l_tmp3, l_tmp3, l_tmp4); /* tmp3 = r * ((H^2 * x1) - x3) */
	vli_modMult_fast(l_tmp1, l_tmp2, P1->y); /* tmp1 = H^3 * y1 */
	vli_modSub(P3->y, l_tmp3, l_tmp1, curve_p, NUM_ECC_DIGITS); /* y3 = r * ((H^2 * x1) - x3) - (H^3 * y1) */
}

#if ECC_USE_NAF

/* Computes p_result = p_point * p_scalar. p_result must not be the same as p_point.
   Uses modified Jacobian coordinates to reduce divisions, and NAF to reduce point adds. */
static void EccPoint_mult(EccPoint *p_result, EccPoint *p_point, uint32_t *p_scalar, uint p_size)
{
	uint32_t l_tmp[NUM_ECC_DIGITS];
    uint32_t Z1[NUM_ECC_DIGITS];
	uint32_t l_plus[NUM_ECC_DIGITS];
    uint32_t l_minus[NUM_ECC_DIGITS];
    
    int l_numBits = vli_numBits(p_scalar, p_size);
    
    uint l_carry;
	int i;
	
    vli_clear(Z1, NUM_ECC_DIGITS);
    vli_clear(l_plus, NUM_ECC_DIGITS);
    vli_clear(l_minus, NUM_ECC_DIGITS);
	
    EccPoint l_neg;
    vli_set(l_neg.x, p_point->x, NUM_ECC_DIGITS);
    vli_sub(l_neg.y, curve_p, p_point->y, NUM_ECC_DIGITS);
    
	l_carry = 0;
	for(i = 0; i < l_numBits; ++i)
	{
        int l_set = vli_testBit(p_scalar, i);
		if((l_carry && !l_set) || (l_set && !l_carry))
		{
            l_carry = 0;
		    if(i < l_numBits - 1 && vli_testBit(p_scalar, i + 1))
		    {
                l_minus[i/32] |= (1 << (i%32));
                l_carry = 1;
		    }
		    else
		    {
		        l_plus[i/32] |= (1 << (i%32));
		    }
			
		}
	}
    
	EccPoint_clear(p_result);
	
	if(l_carry)
	{
	    EccPoint_double_projective(p_result, Z1, p_result, Z1);
        EccPoint_add_mixed(p_result, Z1, p_result, Z1, p_point);
	}
	for(i = l_numBits - 1; i >= 0; --i)
	{
        unsigned l_mask;
	    EccPoint_double_projective(p_result, Z1, p_result, Z1);
        l_mask = (1 << (i%32));
	    if(l_plus[i/32] & l_mask)
	    {
	        EccPoint_add_mixed(p_result, Z1, p_result, Z1, p_point);
	    }
	    else if(l_minus[i/32] & l_mask)
	    {
	        EccPoint_add_mixed(p_result, Z1, p_result, Z1, &l_neg);
	    }
		
	}
	
	vli_modInv(Z1, Z1, curve_p, NUM_ECC_DIGITS); /* Z1 = 1/Z */
	vli_modSquare_fast(l_tmp, Z1); /* tmp = 1/Z^2 */
	vli_modMult_fast(p_result->x, p_result->x, l_tmp); /* x = x/Z^2 */
	
	vli_modMult_fast(l_tmp, Z1, l_tmp); /* tmp = 1/Z^3 */
	vli_modMult_fast(p_result->y, p_result->y, l_tmp); /* y = y/Z^3 */
}

#else /* ECC_USE_NAF */

/* Computes p_result = p_point * p_scalar. p_result must not be the same as p_point.
   Uses modified Jacobian coordinates to reduce divisions. */
static void EccPoint_mult(EccPoint *p_result, EccPoint *p_point, uint32_t *p_scalar, uint p_size)
{
	uint32_t l_tmp[NUM_ECC_DIGITS];
	uint32_t Z1[NUM_ECC_DIGITS];
	
	uint l_numBits = vli_numBits(p_scalar, p_size);
	int i;
	
    vli_clear(Z1, NUM_ECC_DIGITS);
	EccPoint_clear(p_result);

    for(i = l_numBits - 1; i >= 0; --i)
    {
        EccPoint_double_projective(p_result, Z1, p_result, Z1);
        if(vli_testBit(p_scalar, i))
        {
            EccPoint_add_mixed(p_result, Z1, p_result, Z1, p_point);
        }
    }

	vli_modInv(Z1, Z1, curve_p, NUM_ECC_DIGITS); /* Z1 = 1/Z */
	vli_modSquare_fast(l_tmp, Z1); /* tmp = 1/Z^2 */
	vli_modMult_fast(p_result->x, p_result->x, l_tmp); /* x = x/Z^2 */
	
	vli_modMult_fast(l_tmp, Z1, l_tmp); /* tmp = 1/Z^3 */
	vli_modMult_fast(p_result->y, p_result->y, l_tmp); /* y = y/Z^3 */
}

#endif /* ECC_USE_NAF */

int ecdh_shared_secret(uint32_t p_secret[NUM_ECC_DIGITS], EccPoint *p_publicKey, uint32_t p_privateKey[NUM_ECC_DIGITS])
{
	EccPoint l_product;

	EccPoint_mult(&l_product, p_publicKey, p_privateKey, NUM_ECC_DIGITS);
	if(EccPoint_isZero(&l_product))
	{
		return 0;
	}
	
	vli_set(p_secret, l_product.x, NUM_ECC_DIGITS);

	return 1;
}

int ecdh_make_key(EccPoint *p_publicKey, uint32_t p_privateKey[NUM_ECC_DIGITS], uint32_t p_random[NUM_ECC_DIGITS])
{
    /* Make sure the private key is in the range [1, n-1].
       For the supported curves, n is always large enough that we only need to subtract once at most. */
    vli_set(p_privateKey, p_random, NUM_ECC_DIGITS);
#if (ECC_CURVE != secp160r1)
    if(vli_cmp(curve_n, p_privateKey, NUM_ECC_DIGITS) != 1)
    {
        vli_sub(p_privateKey, p_privateKey, curve_n, NUM_ECC_DIGITS);
    }
#endif
    
    if(vli_isZero(p_privateKey, NUM_ECC_DIGITS))
    {
        return 0; /* The private key cannot be 0 (mod p). */
    }
    
    EccPoint_mult(p_publicKey, &curve_G, p_privateKey, NUM_ECC_DIGITS);
    return 1;
}

int ecc_valid_public_key(EccPoint *p_publicKey)
{
    uint32_t l_tmp1[NUM_ECC_DIGITS];
    uint32_t l_tmp2[NUM_ECC_DIGITS];
    
    if(EccPoint_isZero(p_publicKey))
	{
        return 0;
	}
	
	if(vli_cmp(curve_p, p_publicKey->x, NUM_ECC_DIGITS) != 1 || vli_cmp(curve_p, p_publicKey->y, NUM_ECC_DIGITS) != 1)
    {
        return 0;
    }
    
    vli_modSquare_fast(l_tmp1, p_publicKey->y); /* tmp1 = y^2 */
    
    vli_modSquare_fast(l_tmp2, p_publicKey->x); /* tmp2 = x^2 */
    vli_modAdd(l_tmp2, l_tmp2, curve_a, curve_p, NUM_ECC_DIGITS); /* tmp2 = x^2 + a */
    vli_modMult_fast(l_tmp2, l_tmp2, p_publicKey->x); /* tmp2 = x^3 + ax */
    vli_modAdd(l_tmp2, l_tmp2, curve_b, curve_p, NUM_ECC_DIGITS); /* tmp2 = x^3 + ax + b */
    
    /* Make sure that y^2 == x^3 + ax + b */
    if(vli_cmp(l_tmp1, l_tmp2, NUM_ECC_DIGITS) != 0)
    {
        return 0;
    }
    
    return 1;
}

/* -------- ECDSA code -------- */

/* Computes p_result = p_left % p_mod */
/* Size of p_result == size of p_mod == p_modSize */
static void vli_mod(uint32_t *p_result, uint32_t *p_left, uint p_leftSize, uint32_t *p_mod, uint p_modSize)
{
	uint l_modBits = vli_numBits(p_mod, p_modSize);
	if(l_modBits == 0)
	{ /* Divide by 0. */
		return;
	}
	
	uint l_leftDigits = vli_numDigits(p_left, p_leftSize);
	uint l_leftBits = vli_numBits(p_left, l_leftDigits);
	
	if(l_leftBits < l_modBits)
	{ /* p_left < p_mod, so p_result = p_left. */
		vli_set(p_result, p_left, p_modSize); /* Use p_modSize because that is the size of p_result. */
		return;
	}
	
	/* Shift p_mod by (l_leftBits - l_modBits). This multiplies p_mod by the largest
	   power of two possible while still resulting in a number less than p_left. */
	uint32_t l_multiple[l_leftDigits];
	memset(l_multiple, 0, l_leftDigits * sizeof(uint32_t));
	uint l_digitShift = (l_leftBits - l_modBits) / 32;
	uint l_bitShift = (l_leftBits - l_modBits) % 32;
	vli_set(l_multiple + l_digitShift, p_mod, p_modSize);
	if(l_bitShift)
	{
		vli_lshift(l_multiple, l_multiple, l_bitShift, l_leftDigits);
	}
	
	/* Copy left side into remainder. */
	uint32_t l_remainder[l_leftDigits];
	vli_set(l_remainder, p_left, l_leftDigits);
	
	/* Copy right side into divisor of same size (so we can compare). */
	uint32_t l_divisor[l_leftDigits];
	memset(l_divisor, 0, l_leftDigits * sizeof(uint32_t));
	vli_set(l_divisor, p_mod, p_modSize);
	
	/* Subtract all multiples of l_divisor (= p_mod) to get the remainder. */
	while(vli_cmp(l_multiple, l_divisor, l_leftDigits) >= 0)
	{
		if(vli_cmp(l_multiple, l_remainder, l_leftDigits) <= 0)
		{
			vli_sub(l_remainder, l_remainder, l_multiple, l_leftDigits);
		}
		vli_rshift1(l_multiple, l_leftDigits);
	}
	vli_set(p_result, l_remainder, p_modSize);
}

/* Computes p_result = (p_left * p_right) % p_mod. */
static void vli_modMult(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right, uint p_size, uint32_t *p_mod, uint p_modSize)
{
	uint32_t l_product[2 * p_size];
	vli_mult(l_product, p_left, p_right, p_size);
	vli_mod(p_result, l_product, 2 * p_size, p_mod, p_modSize);
}

int ecdsa_sign(uint32_t p_privateKey[NUM_ECC_DIGITS], uint32_t p_random[NUM_ECC_DIGITS], uint32_t p_hash[NUM_ECC_DIGITS],
    uint32_t r[NUM_ECC_DIGITS], uint32_t s[NUM_ECC_DIGITS])
{
    uint32_t k[N_size], s1[N_size], e[N_size];
    EccPoint l_tmp;
    
    if(vli_isZero(p_random, NUM_ECC_DIGITS))
    { /* The random number must not be 0. */
        return 0;
    }
    
    vli_clear(k, N_size);
    vli_set(k, p_random, NUM_ECC_DIGITS);
#if (ECC_CURVE != secp160r1)
    if(vli_cmp(curve_n, k, NUM_ECC_DIGITS) != 1)
    {
        vli_sub(k, k, curve_n, NUM_ECC_DIGITS);
    }
#endif
    
    /* tmp = k * G */
    EccPoint_mult(&l_tmp, &curve_G, k, NUM_ECC_DIGITS);
    
    /* r = x1 (mod n) */
    vli_set(r, l_tmp.x, NUM_ECC_DIGITS);
#if (ECC_CURVE != secp160r1)
    if(vli_cmp(curve_n, r, NUM_ECC_DIGITS) != 1)
    {
        vli_sub(r, r, curve_n, NUM_ECC_DIGITS);
    }
#endif
    if(vli_isZero(r, NUM_ECC_DIGITS))
    { /* If r == 0, fail (need a different random number). */
        return 0;
    }
    
    vli_clear(e, N_size);
    vli_set(e, p_hash, NUM_ECC_DIGITS);
    
    vli_modMult(s1, r, p_privateKey, NUM_ECC_DIGITS, curve_n, N_size); /* s1 = r*d */
    vli_modAdd(s1, e, s1, curve_n, N_size); /* s1 = e + r*d */
    vli_modDiv(s1, s1, k, curve_n, N_size); /* s1 = (e + r*d) / k */
    
    if(vli_numDigits(s1, N_size) > NUM_ECC_DIGITS)
    { /* s is too big to fit in the result buffer (should be very rare, and only happens when using secp160r1). */
        return 0;
    }
    
    vli_set(s, s1, NUM_ECC_DIGITS);
    
    return 1;
}

int ecdsa_verify(EccPoint *p_publicKey, uint32_t p_hash[NUM_ECC_DIGITS], uint32_t r[NUM_ECC_DIGITS], uint32_t s[NUM_ECC_DIGITS])
{
    uint32_t w[N_size], u1[N_size], u2[N_size];
    EccPoint l_tmp1, l_tmp2, l_tmp3;
    
    if(vli_isZero(r, NUM_ECC_DIGITS) || vli_isZero(s, NUM_ECC_DIGITS))
    { /* r, s must not be 0. */
        return 0;
    }
    
#if (ECC_CURVE != secp160r1)
    if(vli_cmp(curve_n, r, NUM_ECC_DIGITS) != 1 || vli_cmp(curve_n, s, NUM_ECC_DIGITS) != 1)
    { /* r, s must be < n. */
        return 0;
    }
#endif

    vli_clear(w, N_size);
    vli_set(w, s, NUM_ECC_DIGITS);
    vli_modInv(w, w, curve_n, N_size); /* w = s^-1 */
    
    vli_clear(u1, N_size);
    vli_set(u1, p_hash, NUM_ECC_DIGITS);
    vli_modMult(u1, u1, w, N_size, curve_n, N_size); /* u1 = e*w */
    
    vli_clear(u2, N_size);
    vli_set(u2, r, NUM_ECC_DIGITS);
    vli_modMult(u2, u2, w, N_size, curve_n, N_size); /* u2 = r*w */
    
    EccPoint_mult(&l_tmp1, &curve_G, u1, N_size);
    EccPoint_mult(&l_tmp2, p_publicKey, u2, N_size);
    
    
    
	uint32_t Z1[NUM_ECC_DIGITS] = {1};
    uint32_t t[NUM_ECC_DIGITS];
    EccPoint_add_mixed(&l_tmp3, Z1, &l_tmp1, Z1, &l_tmp2);

	vli_modInv(Z1, Z1, curve_p, NUM_ECC_DIGITS); /* Z1 = 1/Z */
	vli_modSquare_fast(t, Z1); /* t = 1/Z^2 */
	vli_modMult_fast(l_tmp3.x, l_tmp3.x, t); /* x = x/Z^2 */
	
	vli_modMult_fast(t, Z1, t); /* t = 1/Z^3 */
	vli_modMult_fast(l_tmp3.y, l_tmp3.y, t); /* y = y/Z^3 */
    
    if(EccPoint_isZero(&l_tmp3))
	{ /* Resulting point == infinity */
        return 0;
	}
	
	/* v = x1 (mod n) */
#if (ECC_CURVE != secp160r1)
    if(vli_cmp(curve_n, l_tmp3.x, NUM_ECC_DIGITS) != 1)
    {
        vli_sub(l_tmp3.x, l_tmp3.x, curve_n, NUM_ECC_DIGITS);
    }
#endif

    /* Accept only if v == r. */
    return (vli_cmp(l_tmp3.x, r, NUM_ECC_DIGITS) == 0);
}
