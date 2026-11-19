/* Copyright 2014, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#ifndef _UECC_TEST_H_
#define _UECC_TEST_H_

#include "uECC.h"

#ifdef __cplusplus
extern "C"
{
#endif

int uECC_sign_with_k(const uint8_t *private_key,
                     const uint8_t *message_hash,
                     unsigned hash_size,
                     const uint8_t *k,
                     uint8_t *signature,
                     uECC_Curve curve);

#ifdef __cplusplus
} /* end of extern "C" */
#endif

#endif /* _UECC_TEST_H_ */
