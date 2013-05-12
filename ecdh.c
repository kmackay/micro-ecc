#include "ecdh.h"

#include <string.h>

typedef unsigned int uint;

#define Curve_P_4 {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFD}
#define Curve_P_5 {0x7FFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_P_6 {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_P_7 {0x00000001, 0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_P_8 {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0x00000000, 0x00000000, 0x00000001, 0xFFFFFFFF}

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
#define Curve_N_5 {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF} /* n for secp160r1 is larger than 160 bits */
#define Curve_N_6 {0xB4D22831, 0x146BC9B1, 0x99DEF836, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_N_7 {0x5C5C2A3D, 0x13DD2945, 0xE0B8F03E, 0xFFFF16A2, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}
#define Curve_N_8 {0xD0364141, 0xBFD25E8C, 0xAF48A03B, 0xBAAEDCE6, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF}

#define CONCAT1(a, b) a##b
#define CONCAT(a, b) CONCAT1(a, b)

static uint32_t curve_p[NUM_ECC_DIGITS] = CONCAT(Curve_P_, ECC_CURVE);
static EccPoint curve_G = CONCAT(Curve_G_, ECC_CURVE);
static uint32_t curve_n[NUM_ECC_DIGITS] = CONCAT(Curve_N_, ECC_CURVE);

/* Returns 1 if p_vli == 0, 0 otherwise. */
static int vli_zero(uint32_t *p_vli)
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
	for (i = NUM_ECC_DIGITS - 1; i >= 0 && p_vli[i] == 0; --i)
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
	if(p_dest != p_src)
	{
		memcpy(p_dest, p_src, NUM_ECC_DIGITS * sizeof(uint32_t));
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

/* Computes p_result = p_left + p_right, returning carry. Can modify in place.
   Could be much more efficient in asm. */
static uint32_t vli_add(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right)
{
	uint32_t l_carry = 0;
	uint i;
	for(i=0; i<NUM_ECC_DIGITS; ++i)
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
static uint32_t vli_sub(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right)
{
	uint32_t l_borrow = 0;
	uint i;
	for(i=0; i<NUM_ECC_DIGITS; ++i)
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
static void vli_mult(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right)
{
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

#if ECC_CURVE == secp128r1

/* Computes p_result = p_product % p_mod.
   See algorithm 5 and 6 from http://www.isys.uni-klu.ac.at/PDF/2001-0126-MT.pdf */
static void vli_mmod(uint32_t *p_result, uint32_t *p_product, uint32_t *p_mod)
{
    uint32_t l_tmp[4];
    int l_carry;
	
    memcpy(p_result, p_product, 4*sizeof(uint32_t));
    
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
    
    while(l_carry || vli_cmp(p_mod, p_result) != 1)
    {
        l_carry -= vli_sub(p_result, p_result, p_mod);
    }
}

#elif ECC_CURVE == secp160r1

/* Computes p_result = p_product % p_mod.
   See algorithm 5 and 6 from http://www.isys.uni-klu.ac.at/PDF/2001-0126-MT.pdf */
static void vli_mmod(uint32_t *p_result, uint32_t *p_product, uint32_t *p_mod)
{
    uint32_t l_tmp[5];
    int l_carry;
	
    memcpy(p_result, p_product, 5*sizeof(uint32_t));
    
    l_tmp[0] = (p_product[5] & 0x7FFFFFFF) | (p_product[5] << 31);
    l_tmp[1] = (p_product[5] >> 1) | (p_product[6] << 31);
    l_tmp[2] = (p_product[6] >> 1) | (p_product[7] << 31);
    l_tmp[3] = (p_product[7] >> 1) | (p_product[8] << 31);
    l_tmp[4] = (p_product[8] >> 1) | (p_product[9] << 31);
    l_carry = vli_add(p_result, p_result, l_tmp);
    
    l_tmp[0] = (p_product[9] >> 1) | (p_product[5] & 0x80000000);
    l_tmp[1] = p_product[6];
    l_tmp[2] = p_product[7];
    l_tmp[3] = p_product[8];
    l_tmp[4] = p_product[9];
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    l_tmp[0] = ((p_product[9] & 0x00000002) << 30);
    l_tmp[1] = (p_product[9] >> 2);
    l_tmp[2] = 0;
    l_tmp[3] = 0;
    l_tmp[4] = 0;
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    while(l_carry || vli_cmp(p_mod, p_result) != 1)
    {
        l_carry -= vli_sub(p_result, p_result, p_mod);
    }
}

#elif ECC_CURVE == secp192r1

/* Computes p_result = p_product % p_mod.
   See algorithm 5 and 6 from http://www.isys.uni-klu.ac.at/PDF/2001-0126-MT.pdf */
static void vli_mmod(uint32_t *p_result, uint32_t *p_product, uint32_t *p_mod)
{
    uint32_t l_tmp[6];
    int l_carry;
	
    memcpy(p_result, p_product, 6*sizeof(uint32_t));
    
    memcpy(l_tmp, &p_product[6], 6*sizeof(uint32_t));
    l_carry = vli_add(p_result, p_result, l_tmp);
    
    l_tmp[0] = l_tmp[1] = 0;
    memcpy(&l_tmp[2], &p_product[6], 4*sizeof(uint32_t));
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    l_tmp[0] = l_tmp[2] = p_product[10];
    l_tmp[1] = l_tmp[3] = p_product[11];
    l_tmp[4] = l_tmp[5] = 0;
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    while(l_carry || vli_cmp(p_mod, p_result) != 1)
    {
        l_carry -= vli_sub(p_result, p_result, p_mod);
    }
}

#elif ECC_CURVE == secp224r1

/* Computes p_result = p_product % p_mod.
   See http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void vli_mmod(uint32_t *p_result, uint32_t *p_product, uint32_t *p_mod)
{
    uint32_t l_tmp[7];
    int l_carry;
	
    memcpy(p_result, p_product, 7*sizeof(uint32_t));
    
    l_tmp[0] = l_tmp[1] = l_tmp[2] = 0;
    memcpy(&l_tmp[3], &p_product[7], 4*sizeof(uint32_t));
    l_carry = vli_add(p_result, p_result, l_tmp);
    
    l_tmp[6] = 0;
    memcpy(&l_tmp[3], &p_product[11], 3*sizeof(uint32_t));
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    memcpy(l_tmp, &p_product[7], 7*sizeof(uint32_t));
    l_carry -= vli_sub(p_result, p_result, l_tmp);
    
    l_tmp[3] = l_tmp[4] = l_tmp[5] = l_tmp[6] = 0;
    memcpy(l_tmp, &p_product[11], 3*sizeof(uint32_t));
    l_carry -= vli_sub(p_result, p_result, l_tmp);
    
    if(l_carry < 0)
    {
        do
        {
            l_carry += vli_add(p_result, p_result, p_mod);
        } while(l_carry < 0);
    }
    else
    {
        while(l_carry || vli_cmp(p_mod, p_result) != 1)
        {
            l_carry -= vli_sub(p_result, p_result, p_mod);
        }
    }
}

#elif ECC_CURVE == secp256r1

/* Computes p_result = p_product % p_mod
   from http://www.nsa.gov/ia/_files/nist-routines.pdf */
static void vli_mmod(uint32_t *p_result, uint32_t *p_product, uint32_t *p_mod)
{
    uint32_t l_tmp[8];
    int l_carry;
	
	/* t */
    memcpy(p_result, p_product, 8*sizeof(uint32_t));
    
    /* s1 */
    l_tmp[0] = l_tmp[1] = l_tmp[2] = 0;
    memcpy(&l_tmp[3], &p_product[11], 5*sizeof(uint32_t));
    l_carry = vli_lshift(l_tmp, l_tmp, 1);
    l_carry += vli_add(p_result, p_result, l_tmp);
    
    /* s2 */
    memcpy(&l_tmp[3], &p_product[12], 4*sizeof(uint32_t));
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
            l_carry += vli_add(p_result, p_result, p_mod);
        } while(l_carry < 0);
    }
    else
    {
        while(l_carry || vli_cmp(p_mod, p_result) != 1)
        {
            l_carry -= vli_sub(p_result, p_result, p_mod);
        }
    }
}

#endif

/* Computes p_result = (p_left * p_right) % p_mod. */
static void vli_modMult(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right, uint32_t *p_mod)
{
	uint32_t l_product[2 * NUM_ECC_DIGITS];
	vli_mult(l_product, p_left, p_right);
	vli_mmod(p_result, l_product, p_mod);
}

#if ECC_SQUARE_FUNC

/* Computes p_result = p_left^2. */
static void vli_square(uint32_t *p_result, uint32_t *p_left)
{
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
                r2 += !!(l_product & ((uint64_t)1 << 63));
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
}

/* Computes p_result = p_left^2 % p_mod. */
static void vli_modSquare(uint32_t *p_result, uint32_t *p_left, uint32_t *p_mod)
{
	uint32_t l_product[2 * NUM_ECC_DIGITS];
	vli_square(l_product, p_left);
	vli_mmod(p_result, l_product, p_mod);
}

#else /* ECC_SQUARE_FUNC */

#define vli_modSquare(result, left, mod) vli_modMult((result), (left), (left), (mod))
    
#endif /* ECC_SQUARE_FUNC */

#define EVEN(vli) (!(vli[0] & 1))
/* Computes p_result = (p_left / p_right) % p_mod. All VLIs are the same size.
   See "From Euclid's GCD to Montgomery Multiplication to the Great Divide"
   https://labs.oracle.com/techrep/2001/smli_tr-2001-95.pdf */
static void vli_modDiv(uint32_t *p_result, uint32_t *p_left, uint32_t *p_right, uint32_t *p_mod)
{
	uint32_t a[NUM_ECC_DIGITS], b[NUM_ECC_DIGITS], u[NUM_ECC_DIGITS], v[NUM_ECC_DIGITS];
	uint32_t l_carry;
	int l_cmpResult;
	
	vli_set(a, p_right);
	vli_set(b, p_mod);
	vli_set(u, p_left);
	memset(v, 0, NUM_ECC_DIGITS*sizeof(uint32_t));
	
	while ((l_cmpResult = vli_cmp(a, b)) != 0)
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
			if(l_carry)
			{
				u[NUM_ECC_DIGITS-1] |= 0x80000000;
			}
		}
		else if(EVEN(b))
		{
			vli_rshift1(b);
			if(!EVEN(v))
			{
				l_carry = vli_add(v, v, p_mod);
			}
			vli_rshift1(v);
			if(l_carry)
			{
				v[NUM_ECC_DIGITS-1] |= 0x80000000;
			}
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
			if(l_carry)
			{
				u[NUM_ECC_DIGITS-1] |= 0x80000000;
			}
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
			if(l_carry)
			{
				v[NUM_ECC_DIGITS-1] |= 0x80000000;
			}
		}
	}
	
	vli_set(p_result, u);
}

/* Computes p_result = (1 / p_input) % p_mod. All VLIs are the same size. */
static void vli_modInv(uint32_t *p_result, uint32_t *p_input, uint32_t *p_mod)
{
	uint32_t n[NUM_ECC_DIGITS];
    memset(n, 0, NUM_ECC_DIGITS*sizeof(uint32_t));
	n[0] = 1;
	
	vli_modDiv(p_result, n, p_input, p_mod);
}

/* ------ Point operations ------ */

/* Clears a point (set it to the point at infinity). */
static void EccPoint_clear(EccPoint *p_point)
{
	memset(p_point->x, 0, NUM_ECC_DIGITS*sizeof(uint32_t));
	memset(p_point->y, 0, NUM_ECC_DIGITS*sizeof(uint32_t));
}

/* Copies a point. */
static void EccPoint_copy(EccPoint *p_dest, EccPoint *p_src)
{
	if(p_dest != p_src)
	{
		vli_set(p_dest->x, p_src->x);
		vli_set(p_dest->y, p_src->y);
	}
}

/* Returns 1 if p_point is the point at infinity, 0 otherwise. */
static int EccPoint_isZero(EccPoint *p_point)
{
	return (vli_zero(p_point->x) && vli_zero(p_point->y));
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
	
	if(vli_zero(Z1))
	{
		memset(Z3, 0, sizeof(uint32_t) * NUM_ECC_DIGITS);
		return;
	}
	
	vli_modSquare(l_tmp1, Z1, curve_p); /* tmp1 = Z1^2 */
	vli_modAdd(l_tmp2, P1->x, l_tmp1, curve_p); /* tmp2 = x1 + Z1^2 */
	vli_modSub(l_tmp1, P1->x, l_tmp1, curve_p); /* tmp1 = x1 - Z1^2 */
	vli_modMult(l_tmp1, l_tmp1, l_tmp2, curve_p); /* tmp1 = (x1 + Z1^2) * (x1 - Z1^2) */
	vli_modAdd(l_tmp2, l_tmp1, l_tmp1, curve_p); /* tmp2 = 2 * (x1 + Z1^2) * (x1 - Z1^2) */
	vli_modAdd(l_tmp1, l_tmp2, l_tmp1, curve_p); /* tmp1 = M = 3 * (x1 + Z1^2) * (x1 - Z1^2) */
	vli_modMult(Z3, P1->y, Z1, curve_p); /* Z3 = y1 * Z1 */
	
	vli_modAdd(Z3, Z3, Z3, curve_p); /* Z3 = 2 * y1 * Z1 */
	
	vli_modSquare(l_tmp3, P1->y, curve_p); /* tmp3 = y1^2 */
	vli_modMult(l_tmp2, l_tmp3, P1->x, curve_p); /* tmp2 = x1 * y1^2 */
	vli_modAdd(l_tmp2, l_tmp2, l_tmp2, curve_p); /* tmp2 = 2 * x1 * y1^2 */
	vli_modAdd(l_tmp2, l_tmp2, l_tmp2, curve_p); /* tmp2 = S = 4 * x1 * y1^2 */
	/* Now tmp1 = M, tmp2 = S */
	
	vli_modAdd(l_tmp4, l_tmp2, l_tmp2, curve_p); /* tmp4 = 2*S */
	vli_modSquare(P3->x, l_tmp1, curve_p); /* x3 = M^2 */
	vli_modSub(P3->x, P3->x, l_tmp4, curve_p); /* x3 = T = M^2 - 2*S */
	/* Now tmp1 = M, tmp2 = S, x3 = T */
	
	/* tmp3 is still y1^2 at this point */
	vli_modSquare(l_tmp3, l_tmp3, curve_p); /* tmp3 = y1^4 */
	vli_modAdd(l_tmp3, l_tmp3, l_tmp3, curve_p); /* tmp3 = 2 * y1^4 */
	vli_modAdd(l_tmp3, l_tmp3, l_tmp3, curve_p); /* tmp3 = 4 * y1^4 */
	vli_modAdd(l_tmp3, l_tmp3, l_tmp3, curve_p); /* tmp3 = U = 8 * y1^4 */
	/* Now tmp1 = M, tmp2 = S, x3 = T, tmp3 = U */
	
	vli_modSub(l_tmp2, l_tmp2, P3->x, curve_p); /* tmp2 = S - T */
	vli_modMult(l_tmp2, l_tmp1, l_tmp2, curve_p); /* tmp2 = M * (S - T) */
	vli_modSub(P3->y, l_tmp2, l_tmp3, curve_p); /* y3 = M * (S - T) - U */
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
		vli_set(Z3, Z1);
		return;
	}
	
	if(vli_zero(Z1))
	{
		EccPoint_copy(P3, P2);
		memset(Z3, 0, sizeof(uint32_t) * NUM_ECC_DIGITS);
		Z3[0] = 1;
		return;
	}
	
	vli_modSquare(l_tmp1, Z1, curve_p); /* tmp1 = Z1^2 */
	vli_modMult(l_tmp2, l_tmp1, Z1, curve_p); /* tmp2 = Z1^3 */
	vli_modMult(l_tmp1, l_tmp1, P2->x, curve_p); /* tmp1 = Z1^2 * x2 */
	vli_modSub(l_tmp3, l_tmp1, P1->x, curve_p); /* tmp3 = (Z1^2 * x2) - x1 */
	/* tmp2 = Z1^3, tmp3 = H */
	
	vli_modMult(l_tmp2, l_tmp2, P2->y, curve_p); /* tmp2 = Z1^3 * y2 */
	vli_modSub(l_tmp4, l_tmp2, P1->y, curve_p); /* tmp4 = (Z1^3 * y2) - y1 */
	/* tmp4 = r */
	
	if(vli_zero(l_tmp3))
	{
		if(vli_zero(l_tmp4))
		{ /* The points are equal, so we use the doubling formula. */
			EccPoint_double_projective(P3, Z3, P1, Z1);
		}
		else
		{
			memset(Z3, 0, sizeof(uint32_t) * NUM_ECC_DIGITS);
		}
		return;
	}
	
	vli_modMult(Z3, Z1, l_tmp3, curve_p); /* Z3 = H * Z1 */
	
	vli_modSquare(l_tmp1, l_tmp3, curve_p); /* tmp1 = H^2 */
	vli_modMult(l_tmp2, l_tmp1, l_tmp3, curve_p); /* tmp2 = H^3 */
	vli_modMult(l_tmp1, l_tmp1, P1->x, curve_p); /* tmp1 = H^2 * x1 */
	/* tmp1 = H^2 * x1, tmp2 = H^3, tmp3 = H, tmp4 = r */
	
	vli_modAdd(l_tmp3, l_tmp1, l_tmp1, curve_p); /* tmp3 = 2 * H^2 * x1 */
	vli_modSquare(P3->x, l_tmp4, curve_p); /* x3 = r^2 */
	vli_modSub(P3->x, P3->x, l_tmp3, curve_p); /* x3 = r^2 - (2 * H^2 * x1) */
	vli_modSub(P3->x, P3->x, l_tmp2, curve_p); /* x3 = r^2 - (2 * H^2 * x1) - H^3 */
	/* tmp1 = H^2 * x1, tmp2 = H^3, tmp4 = r */
	
	vli_modSub(l_tmp3, l_tmp1, P3->x, curve_p); /* tmp3 = (H^2 * x1) - x3 */
	vli_modMult(l_tmp3, l_tmp3, l_tmp4, curve_p); /* tmp3 = r * ((H^2 * x1) - x3) */
	vli_modMult(l_tmp1, l_tmp2, P1->y, curve_p); /* tmp1 = H^3 * y1 */
	vli_modSub(P3->y, l_tmp3, l_tmp1, curve_p); /* y3 = r * ((H^2 * x1) - x3) - (H^3 * y1) */
}

#if ECC_USE_NAF

/* Computes p_result = p_point * p_scalar. p_result must not be the same as p_point.
   Uses modified Jacobian coordinates to reduce divisions, and NAF to reduce point adds. */
void EccPoint_mult(EccPoint *p_result, EccPoint *p_point, uint32_t *p_scalar)
{
	uint32_t l_tmp[NUM_ECC_DIGITS];
	uint32_t Z1[NUM_ECC_DIGITS] = {0};
	uint32_t l_plus[NUM_ECC_DIGITS] = {0};
    uint32_t l_minus[NUM_ECC_DIGITS] = {0};
    
    int l_numBits = vli_numBits(p_scalar);
    
    uint l_carry;
	int i;
	
    EccPoint l_neg;
    memcpy(l_neg.x, p_point->x, NUM_ECC_DIGITS*sizeof(uint32_t));
    vli_sub(l_neg.y, curve_p, p_point->y);
    
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
	
	vli_modInv(Z1, Z1, curve_p); /* Z1 = 1/Z */
	vli_modSquare(l_tmp, Z1, curve_p); /* tmp = 1/Z^2 */
	vli_modMult(p_result->x, p_result->x, l_tmp, curve_p); /* x = x/Z^2 */
	
	vli_modMult(l_tmp, Z1, l_tmp, curve_p); /* tmp = 1/Z^3 */
	vli_modMult(p_result->y, p_result->y, l_tmp, curve_p); /* y = y/Z^3 */
}

#else /* ECC_USE_NAF */

/* Computes p_result = p_point * p_scalar. p_result must not be the same as p_point.
   Uses modified Jacobian coordinates to reduce divisions. */
void EccPoint_mult(EccPoint *p_result, EccPoint *p_point, uint32_t *p_scalar)
{
	uint32_t l_tmp[NUM_ECC_DIGITS];
	uint32_t Z1[NUM_ECC_DIGITS] = {0};
	
	uint l_numBits = vli_numBits(p_scalar);
	int i;
	
	EccPoint_clear(p_result);

    for(i = l_numBits - 1; i >= 0; --i)
    {
        EccPoint_double_projective(p_result, Z1, p_result, Z1);
        if(vli_testBit(p_scalar, i))
        {
            EccPoint_add_mixed(p_result, Z1, p_result, Z1, p_point);
        }
    }

	vli_modInv(Z1, Z1, curve_p); /* Z1 = 1/Z */
	vli_modSquare(l_tmp, Z1, curve_p); /* tmp = 1/Z^2 */
	vli_modMult(p_result->x, p_result->x, l_tmp, curve_p); /* x = x/Z^2 */
	
	vli_modMult(l_tmp, Z1, l_tmp, curve_p); /* tmp = 1/Z^3 */
	vli_modMult(p_result->y, p_result->y, l_tmp, curve_p); /* y = y/Z^3 */
}

#endif /* ECC_USE_NAF */

int ecdh_shared_secret(uint32_t p_secret[NUM_ECC_DIGITS], EccPoint *p_publicKey, uint32_t p_privateKey[NUM_ECC_DIGITS])
{
	EccPoint l_product;

	EccPoint_mult(&l_product, p_publicKey, p_privateKey);
	if(EccPoint_isZero(&l_product))
	{
		return 0;
	}
	
	memcpy(p_secret, l_product.x, NUM_ECC_DIGITS * sizeof(uint32_t));

	return 1;
}

int ecdh_make_key(EccPoint *p_publicKey, uint32_t p_privateKey[NUM_ECC_DIGITS], uint32_t p_random[NUM_ECC_DIGITS])
{
    /* Make sure the private key is in the range [1, n-1].
       For the supported curves, n is always large enough that we only need to subtract once at most. */
    vli_set(p_privateKey, p_random);
    if(vli_cmp(curve_n, p_privateKey) != 1)
    {
        vli_sub(p_privateKey, p_privateKey, curve_n);
    }
    
    if(vli_zero(p_privateKey))
    {
        return 0; /* The private key cannot be 0 (mod p). */
    }
    
    EccPoint_mult(p_publicKey, &curve_G, p_privateKey);
    return 1;
}
