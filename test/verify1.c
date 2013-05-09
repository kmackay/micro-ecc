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

extern void EccPoint_mult(EccPoint *p_result, EccPoint *p_point, uint32_t *p_scalar);
extern Curve curve;
extern EccPoint curve_G;

// Test data from http://www.secg.org/collateral/gec2.pdf page 44
// Requires that ECC_CURVE be set to secp160r1

int main(int argc, char **argv)
{
    uint32_t l_expectedSecret[NUM_ECC_DIGITS] = {0x347BB40A, 0x8E6AF594, 0x6E1B74AC, 0x3FFA87A9, 0xCA7C0F8C};
    uint32_t l_privateKey1[NUM_ECC_DIGITS] = {0xB2A4E982, 0x72CB6D57, 0xB0733079, 0x3CE144E6, 0xAA374FFC};
    uint32_t l_privateKey2[NUM_ECC_DIGITS] = {0x2B460866, 0xE74F277E, 0x15101C66, 0x2A17AD4B, 0x45FB58A9};
    EccPoint l_publicKey1, l_publicKey2;
    EccPoint_mult(&l_publicKey1, &curve_G, l_privateKey1);
	EccPoint_mult(&l_publicKey2, &curve_G, l_privateKey2);
	
    printf("Public key 1:\n");
    vli_print(l_publicKey1.x, NUM_ECC_DIGITS);
    printf("\n");
    vli_print(l_publicKey1.y, NUM_ECC_DIGITS);
    printf("\n\n");
    
    printf("Public key 2:\n");
    vli_print(l_publicKey2.x, NUM_ECC_DIGITS);
    printf("\n");
    vli_print(l_publicKey2.y, NUM_ECC_DIGITS);
    printf("\n\n");
	
	uint32_t l_shared1[NUM_ECC_DIGITS];
	if(!ecdh_shared_secret((uint8_t *)l_shared1, sizeof(l_shared1), &l_publicKey1, l_privateKey2))
	{
		printf("shared_secret() failed (1)\n");
		return 1;
	}

	uint32_t l_shared2[NUM_ECC_DIGITS];
	if(!ecdh_shared_secret((uint8_t *)l_shared2, sizeof(l_shared2), &l_publicKey2, l_privateKey1))
	{
		printf("shared_secret() failed (2)\n");
		return 1;
	}
	
	if(memcmp(l_shared1, l_shared2, sizeof(l_shared1)) != 0)
	{
		printf("Shared secrets are not identical!\n");
	}
	else if(memcmp(l_shared1, l_expectedSecret, sizeof(l_shared1)) != 0)
	{
		printf("Shared secret is not the expected value!\n");
	}
	else
	{
        printf("Everything worked as expected.\n");
	}
	
	printf("Shared secret 1 = ");
	vli_print(l_shared1, NUM_ECC_DIGITS);
	printf("\n");
	printf("Shared secret 2 = ");
	vli_print(l_shared2, NUM_ECC_DIGITS);
	printf("\n");
	
	return 0;
}

#endif
