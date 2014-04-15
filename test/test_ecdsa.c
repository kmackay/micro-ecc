#include "uECC.h"

#include <stdio.h>
#include <string.h>

int main()
{
    uint8_t l_public[uECC_BYTES*2];
    uint8_t l_private[uECC_BYTES];

    uint8_t l_hash[uECC_BYTES];
    
    uint8_t l_sig[uECC_BYTES*2];
    
    int i;
    
    printf("Testing 256 signatures\n");
    
    for(i=0; i<256; ++i)
    {
        printf(".");
        fflush(stdout);
        
        uECC_make_key(l_public, l_private);
        memcpy(l_hash, l_public, uECC_BYTES);
        
        if(!uECC_sign(l_private, l_hash, l_sig))
        {
            printf("uECC_sign() failed\n");
            continue;
        }
        
        if(!uECC_verify(l_public, l_hash, l_sig))
        {
            printf("uECC_verify() failed\n");
        }
    }
    printf("\n");
    
    return 0;
}
