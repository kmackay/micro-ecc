/* Copyright 2014, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#include "uECC.h"

#include <stdio.h>
#include <string.h>

void vli_print(uint8_t *p_vli, unsigned int p_size)
{
    while(p_size)
    {
        printf("%02X ", (unsigned)p_vli[p_size - 1]);
        --p_size;
    }
}

int main()
{
    int i;
    
    uint8_t l_private1[uECC_BYTES];
    uint8_t l_private2[uECC_BYTES];
    
    uint8_t l_public1[uECC_BYTES * 2];
    uint8_t l_public2[uECC_BYTES * 2];
    
    uint8_t l_secret1[uECC_BYTES];
    uint8_t l_secret2[uECC_BYTES];
    
    printf("Testing 256 random private key pairs\n");

    for(i=0; i<256; ++i)
    {
        printf(".");
        fflush(stdout);

        if(!uECC_make_key(l_public1, l_private1) || !uECC_make_key(l_public2, l_private2))
        {
            printf("uECC_make_key() failed\n");
            return 1;
        }

        if(!uECC_shared_secret(l_public2, l_private1, l_secret1))
        {
            printf("shared_secret() failed (1)\n");
            return 1;
        }

        if(!uECC_shared_secret(l_public1, l_private2, l_secret2))
        {
            printf("shared_secret() failed (2)\n");
            return 1;
        }
        
        if(memcmp(l_secret1, l_secret2, sizeof(l_secret1)) != 0)
        {
            printf("Shared secrets are not identical!\n");
            printf("Shared secret 1 = ");
            vli_print(l_secret1, uECC_BYTES);
            printf("\n");
            printf("Shared secret 2 = ");
            vli_print(l_secret2, uECC_BYTES);
            printf("\n");
            printf("Private key 1 = ");
            vli_print(l_private1, uECC_BYTES);
            printf("\n");
            printf("Private key 2 = ");
            vli_print(l_private2, uECC_BYTES);
            printf("\n");
        }
    }
    printf("\n");
    
    return 0;
}
