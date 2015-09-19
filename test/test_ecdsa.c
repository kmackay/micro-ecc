/* Copyright 2014, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#include "uECC.h"

#include <stdio.h>
#include <string.h>

int main() {
    int i, c;
    uint8_t private[32] = {0};
    uint8_t public[64] = {0};
    uint8_t hash[32] = {0};
    uint8_t sig[64] = {0};

    const struct uECC_Curve_t * curves[5];
    curves[0] = uECC_secp160r1();
    curves[1] = uECC_secp192r1();
    curves[2] = uECC_secp224r1();
    curves[3] = uECC_secp256r1();
    curves[4] = uECC_secp256k1();
    
    printf("Testing 256 signatures\n");
    for (c = 0; c < 5; ++c) {
        for (i = 0; i < 256; ++i) {
            printf(".");
            fflush(stdout);

            if (!uECC_make_key(public, private, curves[c])) {
                printf("uECC_make_key() failed\n");
                return 1;
            }
            memcpy(hash, public, sizeof(hash));
            
            if (!uECC_sign(private, hash, sizeof(hash), sig, curves[c])) {
                printf("uECC_sign() failed\n");
                return 1;
            }

            if (!uECC_verify(public, hash, sizeof(hash), sig, curves[c])) {
                printf("uECC_verify() failed\n");
                return 1;
            }
        }
        printf("\n");
    }
    
    return 0;
}
