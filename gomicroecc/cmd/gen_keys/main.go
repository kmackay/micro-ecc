package main

import (
	"fmt"

	"github.com/jestan/easy-ecc/gomicroecc"
)

func main() {
	privateKeyPEM, publicKeyPEM, err := gomicroecc.GenerateECDSAP256KeyPair()
	if err != nil {
		fmt.Println("Error:", err)
		return
	}

	fmt.Println("Private Key (PKCS#8 PEM):")
	fmt.Println(privateKeyPEM)

	fmt.Println("\nPublic Key (PEM):")
	fmt.Println(publicKeyPEM)
}
