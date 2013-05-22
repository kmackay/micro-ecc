#include "ecdh.h"

#if (ECC_CURVE == secp160r1)

#include <string.h>
#include <stdio.h>

void vli_print(uint32_t *p_vli, unsigned int p_size)
{
	while(p_size)
	{
		printf("%08X ", (unsigned)p_vli[p_size - 1]);
		--p_size;
	}
}

/* Test data from http://www.secg.org/collateral/gec2.pdf page 2
   Requires that ECC_CURVE be set to secp160r1. */

int main()
{
    uint32_t l_privateKey[NUM_ECC_DIGITS] = {0xB2A4E982, 0x72CB6D57, 0xB0733079, 0x3CE144E6, 0xAA374FFC};
    EccPoint l_publicKey = {
        {0x51419DC0, 0x3C032062, 0x0E75A24A, 0xECC406ED, 0x51B4496F},
        {0x1756AA6C, 0x4F381CCC, 0x68D79389, 0x73A514B4, 0xC28DCB4B}
    };
    uint32_t l_random[NUM_ECC_DIGITS] = {0xdecd52da, 0x2ac5d528, 0xb9185c8b, 0x681a3f28, 0x7b012db7};
    uint32_t l_hash[NUM_ECC_DIGITS] = {0x9cd0d89d, 0x7850c26c, 0xba3e2571, 0x4706816a, 0xa9993e36};
    
    uint32_t r[NUM_ECC_DIGITS];
    uint32_t s[NUM_ECC_DIGITS];
    
    if(!ecdsa_sign(l_privateKey, l_random, l_hash, r, s))
    {
        printf("ecdsa_sign() failed\n");
    }
    
    printf("r: ");
    vli_print(r, NUM_ECC_DIGITS);
    printf("\n");
    printf("s: ");
    vli_print(s, NUM_ECC_DIGITS);
    printf("\n");
    
    if(!ecdsa_verify(&l_publicKey, l_hash, r, s))
    {
        printf("ecdsa_verify() failed\n");
    }
	
	return 0;
}

#endif


