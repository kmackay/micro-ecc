#ifndef _GOLANG_WRAP_H_
#define _GOLANG_WRAP_H_

#include "../uECC.h"

#ifdef _NEED_INIT
const struct uECC_Curve_t *curve = NULL;

void init_ecc() { curve = uECC_secp256r1(); }
#endif

typedef struct {
  uint64_t ptr;
  uint64_t len;
  uint64_t cap;
} __attribute__((packed)) SliceHeader;

#ifdef _NEED_SIGN_VERRIFY
extern const struct uECC_Curve_t *curve;

void sign(SliceHeader *out, const SliceHeader *private_key,
          const SliceHeader *message_hash) {
  uECC_sign((const uint8_t *)private_key->ptr,
            (const uint8_t *)message_hash->ptr, message_hash->len,
            (uint8_t *)out->ptr, curve);
}

void verify(const SliceHeader *public_key, const SliceHeader *message_hash,
            const SliceHeader *signature, int64_t *ret) {
  *ret = uECC_verify((const uint8_t *)public_key->ptr,
                     (const uint8_t *)message_hash->ptr, message_hash->len,
                     (const uint8_t *)signature->ptr, curve);
}
#endif

#endif
