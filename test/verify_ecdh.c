#if !TARGET_LPC11XX

#include "ecc.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

extern void EccPoint_mult(EccPoint *p_result, EccPoint *p_point, uint32_t *p_scalar);

void vli_print(uint32_t *p_vli, unsigned int p_size)
{
	while(p_size)
	{
		printf("%08X ", (unsigned)p_vli[p_size - 1]);
		--p_size;
	}
}

int randfd;

void getRandomBytes(void *p_dest, unsigned p_size)
{
    if(read(randfd, p_dest, p_size) != (int)p_size)
    {
        printf("Failed to get random bytes.\n");
    }
}

int main()
{
    EccPoint l_Q1, l_Q2; /* public keys */
	uint32_t l_secret1[NUM_ECC_DIGITS];
	uint32_t l_secret2[NUM_ECC_DIGITS];
	uint64_t l_total;
	int i;
	
	uint32_t l_shared1[NUM_ECC_DIGITS];
	uint32_t l_shared2[NUM_ECC_DIGITS];
	
    randfd = open("/dev/urandom", O_RDONLY);
	if(randfd == -1)
	{
        printf("No access to urandom\n");
        return -1;
	}
	
	printf("Testing 256 random private key pairs\n");
	
    l_total = 0;
	for(i=0; i<256; ++i)
	{
		printf(".");
		fflush(stdout);
		getRandomBytes((char *)l_secret1, NUM_ECC_DIGITS * sizeof(uint32_t));
		getRandomBytes((char *)l_secret2, NUM_ECC_DIGITS * sizeof(uint32_t));
		
        ecc_make_key(&l_Q1, l_secret1, l_secret1);
        ecc_make_key(&l_Q2, l_secret2, l_secret2);

		if(!ecdh_shared_secret(l_shared1, &l_Q1, l_secret2))
		{
			printf("shared_secret() failed (1)\n");
			return 1;
		}

		if(!ecdh_shared_secret(l_shared2, &l_Q2, l_secret1))
		{
			printf("shared_secret() failed (2)\n");
			return 1;
		}
		
		if(memcmp(l_shared1, l_shared2, sizeof(l_shared1)) != 0)
		{
			printf("Shared secrets are not identical!\n");
			printf("Shared secret 1 = ");
			vli_print(l_shared1, NUM_ECC_DIGITS);
			printf("\n");
			printf("Shared secret 2 = ");
			vli_print(l_shared2, NUM_ECC_DIGITS);
			printf("\n");
			printf("Private key 1 = ");
			vli_print(l_secret1, NUM_ECC_DIGITS);
			printf("\n");
			printf("Private key 2 = ");
			vli_print(l_secret2, NUM_ECC_DIGITS);
			printf("\n");
		}
	}
	printf("\n");
	
	return 0;
}

#endif /* !TARGET_LPC11XX */
