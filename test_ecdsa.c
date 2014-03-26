#include "ecc.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

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
    uint8_t l_public[ECC_BYTES*2];
    uint8_t l_private[ECC_BYTES];

    uint8_t l_hash[ECC_BYTES];
    
    uint8_t l_sig[ECC_BYTES*2];
    
    int i;
    
    randfd = open("/dev/urandom", O_RDONLY);
    if(randfd == -1)
    {
        printf("No access to urandom\n");
        return -1;
    }
    
    printf("Testing 256 signatures\n");
    
    for(i=0; i<256; ++i)
    {
        printf(".");
        fflush(stdout);
        
        ecc_make_key(l_public, l_private);
        
        getRandomBytes((char *)l_hash, ECC_BYTES);
        
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
