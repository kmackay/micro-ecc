#include <uECC.h>

#include <j0g.h>
#include <js0n.h>

#include <lwm.h>

#include <bitlash.h>

#include <GS.h>

#include <SPI.h>
#include <Wire.h>
#include <Scout.h>
#include <Shell.h>


#include <uECC.h>

extern "C" {

static int RNG(uint8_t *p_dest, unsigned p_size)
{
  while(p_size) {
    long v = random();
    unsigned l_amount = min(p_size, sizeof(long));
    memcpy(p_dest, &v, l_amount);
    p_size -= l_amount;
    p_dest += l_amount;
  }
  return 1;
}

void px(uint8_t *v, uint8_t num)
{
  uint8_t i;
  for(i=0; i<num; ++i)
  {
    Serial.print(v[i]); Serial.print(" ");
  }
  Serial.println();
}

}

void setup() {
  Scout.setup();
  uint8_t l_private[uECC_BYTES];
  
  uint8_t l_public[uECC_BYTES * 2];

  uint8_t l_hash[uECC_BYTES];
  uint8_t l_sig[uECC_BYTES*2];
  
  Serial.print("Testing ECDSA\n");
  
  uECC_set_rng(&RNG);
  
  for(;;) {
    unsigned long a = millis();
    if(!uECC_make_key(l_public, l_private))
    {
      Serial.println("uECC_make_key() failed");
      continue;
    }
    unsigned long b = millis();
    
    Serial.print("Made key 1 in "); Serial.println(b-a);
    memcpy(l_hash, l_public, uECC_BYTES);
    
    a = millis();
    if(!uECC_sign(l_private, l_hash, l_sig))
    {
      Serial.println("uECC_sign() failed\n");
      continue;
    }
    b = millis();
    Serial.print("ECDSA sign in "); Serial.println(b-a);
    
    a = millis();
    if(!uECC_verify(l_public, l_hash, l_sig))
    {
      Serial.println("uECC_verify() failed\n");
      continue;
    }
    b = millis();
    Serial.print("ECDSA verify in "); Serial.println(b-a);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
