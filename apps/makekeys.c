#if !TARGET_LPC11XX

#include "ecc.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>

extern void EccPoint_mult(EccPoint *p_result, EccPoint *p_point, uint32_t *p_scalar);

void vli_print(uint32_t *p_vli)
{
    unsigned i;
    for(i=0; i<NUM_ECC_DIGITS-1; ++i)
    {
        printf("0x%08X, ", (unsigned)p_vli[i]);
    }
    printf("0x%08X", (unsigned)p_vli[i]);
}

int randfd;

void getRandomBytes(void *p_dest, unsigned p_size)
{
    if(read(randfd, p_dest, p_size) != (int)p_size)
    {
        printf("Failed to get random bytes.\n");
    }
}

int main(int argc, char **argv)
{
    unsigned l_num = 1;
    unsigned i, j;
    
    if(argc > 1)
    {
        l_num = strtoul(argv[1], NULL, 10);
    }
    
    randfd = open("/dev/urandom", O_RDONLY);
    if(randfd == -1)
    {
        printf("No access to urandom\n");
        return -1;
    }
    
    uint32_t l_private[NUM_ECC_DIGITS];
    EccPoint l_public;
    
    for(i=0; i<l_num; ++i)
    {
        getRandomBytes((char *)l_private, NUM_ECC_DIGITS * sizeof(uint32_t));
        ecc_make_key(&l_public, l_private, l_private);
        
        printf("uint32_t private_%u[NUM_ECC_DIGITS] = {", i);
        vli_print(l_private);
        printf("};\n");

        printf("EccPoint public_%u = {\n", i);
        printf("    {");
        vli_print(l_public.x);
        printf("},\n");
        printf("    {");
        vli_print(l_public.y);
        printf("}};\n\n");
    }
    
    return 0;
}

#endif /* !TARGET_LPC11XX */
