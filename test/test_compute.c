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
    int success;

    uint8_t l_private[uECC_BYTES];

    uint8_t l_public[uECC_BYTES * 2];
    uint8_t l_public_computed[uECC_BYTES * 2];

    printf("Testing 256 random private key pairs\n");

    for(i=0; i<256; ++i)
    {
        printf(".");
        fflush(stdout);

        int success = uECC_make_key(l_public, l_private);
        if (!success) {
            printf("uECC_make_key() failed\n");
            return 1;
        }

        success = uECC_compute_public_key(l_private, l_public_computed);
        if (!success) {
            printf("uECC_compute_public_key() failed\n");
        }

        if(memcmp(l_public, l_public_computed, sizeof(l_public)) != 0)
        {
            printf("Computed and provided public keys are not identical!\n");
            printf("Computed public key = ");
            vli_print(l_public_computed, uECC_BYTES);
            printf("\n");
            printf("Provided public key = ");
            vli_print(l_public, uECC_BYTES);
            printf("\n");
            printf("Private key = ");
            vli_print(l_private, uECC_BYTES);
            printf("\n");
        }
    }

    printf("\n");

    printf("Testing private key = 0\n");

    memset(l_private, 0, uECC_BYTES);
    success = uECC_compute_public_key(l_private, l_public_computed);
    if (success) {
        printf("uECC_compute_public_key() should have failed\n");
    }

    return 0;
}
