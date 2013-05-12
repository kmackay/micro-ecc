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

/* Test data from http://www.secg.org/collateral/gec2.pdf page 44
   Requires that ECC_CURVE be set to secp160r1. */

int main()
{
    uint32_t l_expectedSecret[NUM_ECC_DIGITS] = {0x347BB40A, 0x8E6AF594, 0x6E1B74AC, 0x3FFA87A9, 0xCA7C0F8C};
    uint32_t l_privateKey1[NUM_ECC_DIGITS] = {0xB2A4E982, 0x72CB6D57, 0xB0733079, 0x3CE144E6, 0xAA374FFC};
    uint32_t l_privateKey2[NUM_ECC_DIGITS] = {0x2B460866, 0xE74F277E, 0x15101C66, 0x2A17AD4B, 0x45FB58A9};
    EccPoint l_publicKey1 = {
        {0x51419DC0, 0x3C032062, 0x0E75A24A, 0xECC406ED, 0x51B4496F},
        {0x1756AA6C, 0x4F381CCC, 0x68D79389, 0x73A514B4, 0xC28DCB4B}
    };
    EccPoint l_publicKey2 = {
        {0x07C6E5BC, 0x0F63D567, 0x328739D9, 0x9C0369C2, 0x49B41E0E},
        {0x0E9C8F83, 0x111C3EDC, 0x6D232A03, 0x67015ED9, 0x26E008B5}
    };
	
	uint32_t l_shared1[NUM_ECC_DIGITS];
	uint32_t l_shared2[NUM_ECC_DIGITS];
	
	if(!ecdh_shared_secret(l_shared1, &l_publicKey1, l_privateKey2))
	{
		printf("shared_secret() failed (1)\n");
		return 1;
	}

	if(!ecdh_shared_secret(l_shared2, &l_publicKey2, l_privateKey1))
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
