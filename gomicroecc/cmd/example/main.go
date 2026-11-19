package main

import (
	"encoding/hex"
	"log"
	"os"

	"github.com/jestan/easy-ecc/gomicroecc"
)

func main() {
	s, err := os.ReadFile("./private_key.txt")
	if err != nil {
		log.Fatalln(err.Error())
	}
	privateKey, err := gomicroecc.ParsePrivateKey(string(s))
	if err != nil {
		log.Fatalln(err.Error())
	}
	sha256Value, err := hex.DecodeString("758a18de504f9aed6e23a203cdb1d207cb86571d84c41e1f66c1deec48c01de5")
	if err != nil {
		log.Fatalln(err.Error())
	}
	//
	outSign := make([]byte, 64)
	gomicroecc.Sign(outSign, privateKey.D.Bytes(), sha256Value)
	golangSign, err := gomicroecc.EncodeSignature(outSign)
	if err != nil {
		log.Fatalln(err.Error())
	}
	log.Println("sign:", hex.EncodeToString(golangSign))
	//
	publicKeyBytes := make([]byte, 0, 64)
	publicKeyBytes = append(publicKeyBytes, privateKey.PublicKey.X.Bytes()...)
	publicKeyBytes = append(publicKeyBytes, privateKey.PublicKey.Y.Bytes()...)
	ret := gomicroecc.Verify(publicKeyBytes, sha256Value, outSign)
	if !ret {
		log.Println("Verify fail")
		return
	}
	log.Println("Verify ok")
}
