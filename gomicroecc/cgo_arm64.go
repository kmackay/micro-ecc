package gomicroecc

/*
#define _NEED_SIGN_VERRIFY
#include "go_wrap.h"
*/
import "C"
import (
	"unsafe"
)

func Sign(out []byte, privateKey []byte, hash []byte) {
	C.sign(
		(*C.SliceHeader)(unsafe.Pointer(&out)),
		(*C.SliceHeader)(unsafe.Pointer(&privateKey)),
		(*C.SliceHeader)(unsafe.Pointer(&hash)),
	)
}

func Verify(publicKey []byte, hash []byte, signature []byte) bool {
	var ret C.longlong
	C.verify(
		(*C.SliceHeader)(unsafe.Pointer(&publicKey)),
		(*C.SliceHeader)(unsafe.Pointer(&hash)),
		(*C.SliceHeader)(unsafe.Pointer(&signature)),
		&ret,
	)
	return ret != 0
}
