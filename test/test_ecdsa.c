#include "ecc.h"

#include <stdio.h>
#include <string.h>

int main()
{
    uint8_t l_public[ECC_BYTES*2];
    uint8_t l_private[ECC_BYTES];

    uint8_t l_hash[ECC_BYTES];
    
    uint8_t l_sig[ECC_BYTES*2];
    
    int i;
    
    printf("Testing 256 signatures\n");
    
    for(i=0; i<256; ++i)
    {
        printf(".");
        fflush(stdout);
        
        ecc_make_key(l_public, l_private);
        memcpy(l_hash, l_public, ECC_BYTES);
        
        if(!ecdsa_sign(l_private, l_hash, l_sig))
        {
            printf("ecdsa_sign() failed\n");
            continue;
        }
        
        if(!ecdsa_verify(l_public, l_hash, l_sig))
        {
            printf("ecdsa_verify() failed\n");
        }
    }
    printf("\n");
    
    return 0;
}
