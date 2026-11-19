package gomicroecc

/*
#define _NEED_INIT
#include "go_wrap.h"
#include "../uECC.c"
*/
import "C"
import (
	"errors"
	"fmt"

	"crypto/ecdsa"
	"crypto/elliptic"
	"crypto/rand"
	"crypto/x509"
	"encoding/pem"

	"golang.org/x/crypto/cryptobyte"
	"golang.org/x/crypto/cryptobyte/asn1"
)

func init() {
	C.init_ecc()
}

// GenerateECDSAP256KeyPair generates an ECDSA P-256 key pair and returns them as PEM-encoded strings.
func GenerateECDSAP256KeyPair() (string, string, error) {
	// Generate a new ECDSA P-256 private key
	privateKey, err := ecdsa.GenerateKey(elliptic.P256(), rand.Reader)
	if err != nil {
		return "", "", err
	}

	// Encode the private key as PKCS#8 PEM
	privateKeyBytes, err := x509.MarshalPKCS8PrivateKey(privateKey)
	if err != nil {
		return "", "", err
	}
	privateKeyPEM := pem.EncodeToMemory(&pem.Block{Type: "PRIVATE KEY", Bytes: privateKeyBytes})

	// Encode the public key as PEM
	publicKeyBytes, err := x509.MarshalPKIXPublicKey(&privateKey.PublicKey)
	if err != nil {
		return "", "", err
	}
	publicKeyPEM := pem.EncodeToMemory(&pem.Block{Type: "PUBLIC KEY", Bytes: publicKeyBytes})

	return string(privateKeyPEM), string(publicKeyPEM), nil
}

func ParsePrivateKey(s string) (*ecdsa.PrivateKey, error) {
	privateKeyPEMBlock, _ := pem.Decode([]byte(s))
	if privateKeyPEMBlock == nil {
		return nil, errors.New("decode error")
	}
	var err error
	var privateKeyAny any
	privateKeyAny, err = x509.ParsePKCS8PrivateKey(privateKeyPEMBlock.Bytes)
	if err != nil {
		return nil, fmt.Errorf("x509.ParsePKCS8PrivateKey error, err=%s", err.Error())
	}
	return privateKeyAny.(*ecdsa.PrivateKey), nil
}

// addASN1IntBytes encodes in ASN.1 a positive integer represented as
// a big-endian byte slice with zero or more leading zeroes.
// copy from crypto/ecdsa
func addASN1IntBytes(b *cryptobyte.Builder, bytes []byte) {
	for len(bytes) > 0 && bytes[0] == 0 {
		bytes = bytes[1:]
	}
	if len(bytes) == 0 {
		b.SetError(errors.New("invalid integer"))
		return
	}
	b.AddASN1(asn1.INTEGER, func(c *cryptobyte.Builder) {
		if bytes[0]&0x80 != 0 {
			c.AddUint8(0)
		}
		c.AddBytes(bytes)
	})
}

// EncodeSignature encode signature as golang format
// copy from crypto/ecdsa
func EncodeSignature(signature []byte) ([]byte, error) {
	if len(signature) != 64 {
		return nil, errors.New("signature len must be 64")
	}
	var b cryptobyte.Builder
	b.AddASN1(asn1.SEQUENCE, func(b *cryptobyte.Builder) {
		addASN1IntBytes(b, signature[:32])
		addASN1IntBytes(b, signature[32:])
	})
	return b.Bytes()
}
