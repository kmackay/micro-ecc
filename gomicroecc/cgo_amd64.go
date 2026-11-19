package gomicroecc

/*
#define _NEED_SIGN_VERRIFY
#include "go_wrap.h"
*/
import "C"
import (
	"unsafe"

	"github.com/petermattis/fastcgo"
)

func Sign(out []byte, privateKey []byte, hash []byte) {
	fastcgo.UnsafeCall4(C.sign,
		uint64(uintptr(unsafe.Pointer(&out))),
		uint64(uintptr(unsafe.Pointer(&privateKey))),
		uint64(uintptr(unsafe.Pointer(&hash))),
		0,
	)
}

func Verify(publicKey []byte, hash []byte, signature []byte) bool {
	var ret int64
	fastcgo.UnsafeCall4(C.verify,
		uint64(uintptr(unsafe.Pointer(&publicKey))),
		uint64(uintptr(unsafe.Pointer(&hash))),
		uint64(uintptr(unsafe.Pointer(&signature))),
		uint64(uintptr(unsafe.Pointer(&ret))),
	)
	return ret != 0
}
