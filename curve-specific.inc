#define num_bytes_secp160r1 20
#define num_bytes_secp192r1 24
#define num_bytes_secp224r1 28
#define num_bytes_secp256r1 32
#define num_bytes_secp256k1 32

#if (uECC_WORD_SIZE == 1)

#define num_words_secp160r1 20
#define num_words_secp192r1 24
#define num_words_secp224r1 28
#define num_words_secp256r1 32
#define num_words_secp256k1 32

#define num_n_words_secp160r1 21

#define BYTES_TO_WORDS_8(a, b, c, d, e, f, g, h) \
    0x##a, 0x##b, 0x##c, 0x##d, 0x##e, 0x##f, 0x##g, 0x##h
#define BYTES_TO_WORDS_4(a, b, c, d) 0x##a, 0x##b, 0x##c, 0x##d

#elif (uECC_WORD_SIZE == 4)

#define num_words_secp160r1 5
#define num_words_secp192r1 6
#define num_words_secp224r1 7
#define num_words_secp256r1 8
#define num_words_secp256k1 8

#define num_n_words_secp160r1 6

#define BYTES_TO_WORDS_8(a, b, c, d, e, f, g, h) 0x##d##c##b##a, 0x##h##g##f##e
#define BYTES_TO_WORDS_4(a, b, c, d) 0x##d##c##b##a

#elif (uECC_WORD_SIZE == 8)

#define num_words_secp160r1 3
#define num_words_secp192r1 3
#define num_words_secp224r1 4
#define num_words_secp256r1 4
#define num_words_secp256k1 4

#define num_n_words_secp160r1 3

#define BYTES_TO_WORDS_8(a, b, c, d, e, f, g, h) 0x##h##g##f##e##d##c##b##a##ull
#define BYTES_TO_WORDS_4(a, b, c, d) 0x##d##c##b##a##ull

#endif /* uECC_WORD_SIZE */

static const struct uECC_Curve_t curve_secp160r1 = {
    num_words_secp160r1,
    num_n_words_secp160r1,
    num_bytes_secp160r1,
    { BYTES_TO_WORDS_8(FF, FF, FF, 7F, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_4(FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(57, 22, 75, CA, D3, AE, 27, F9),
        BYTES_TO_WORDS_8(C8, F4, 01, 00, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(00, 00, 00, 00, 01, 00, 00, 00) },
    { BYTES_TO_WORDS_8(82, FC, CB, 13, B9, 8B, C3, 68),
        BYTES_TO_WORDS_8(89, 69, 64, 46, 28, 73, F5, 8E),
        BYTES_TO_WORDS_4(68, B5, 96, 4A),

        BYTES_TO_WORDS_8(32, FB, C5, 7A, 37, 51, 23, 04),
        BYTES_TO_WORDS_8(12, C9, DC, 59, 7D, 94, 68, 31),
        BYTES_TO_WORDS_4(55, 28, A6, 23) },
    { BYTES_TO_WORDS_8(45, FA, 65, C5, AD, D4, D4, 81),
        BYTES_TO_WORDS_8(9F, F8, AC, 65, 8B, 7A, BD, 54),
        BYTES_TO_WORDS_4(FC, BE, 97, 1C) },
    &double_jacobian_default,
    &mod_sqrt_default,
    &x_side_default
};

static const struct uECC_Curve_t curve_secp192r1 = {
    num_words_secp192r1,
    num_words_secp192r1,
    num_bytes_secp192r1,
    { BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FE, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(31, 28, D2, B4, B1, C9, 6B, 14),
        BYTES_TO_WORDS_8(36, F8, DE, 99, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(12, 10, FF, 82, FD, 0A, FF, F4),
        BYTES_TO_WORDS_8(00, 88, A1, 43, EB, 20, BF, 7C),
        BYTES_TO_WORDS_8(F6, 90, 30, B0, 0E, A8, 8D, 18),

        BYTES_TO_WORDS_8(11, 48, 79, 1E, A1, 77, F9, 73),
        BYTES_TO_WORDS_8(D5, CD, 24, 6B, ED, 11, 10, 63),
        BYTES_TO_WORDS_8(78, DA, C8, FF, 95, 2B, 19, 07) },
    { BYTES_TO_WORDS_8(B1, B9, 46, C1, EC, DE, B8, FE),
        BYTES_TO_WORDS_8(49, 30, 24, 72, AB, E9, A7, 0F),
        BYTES_TO_WORDS_8(E7, 80, 9C, E5, 19, 05, 21, 64) },
    &double_jacobian_default,
    &mod_sqrt_default,
    &x_side_default
};

static const struct uECC_Curve_t curve_secp224r1 = {
    num_words_secp224r1,
    num_words_secp224r1,
    num_bytes_secp224r1,
    { BYTES_TO_WORDS_8(01, 00, 00, 00, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(00, 00, 00, 00, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_4(FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(3D, 2A, 5C, 5C, 45, 29, DD, 13),
        BYTES_TO_WORDS_8(3E, F0, B8, E0, A2, 16, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_4(FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(21, 1D, 5C, 11, D6, 80, 32, 34),
        BYTES_TO_WORDS_8(22, 11, C2, 56, D3, C1, 03, 4A),
        BYTES_TO_WORDS_8(B9, 90, 13, 32, 7F, BF, B4, 6B),
        BYTES_TO_WORDS_4(BD, 0C, 0E, B7),

        BYTES_TO_WORDS_8(34, 7E, 00, 85, 99, 81, D5, 44),
        BYTES_TO_WORDS_8(64, 47, 07, 5A, A0, 75, 43, CD),
        BYTES_TO_WORDS_8(E6, DF, 22, 4C, FB, 23, F7, B5),
        BYTES_TO_WORDS_4(88, 63, 37, BD) },
    { BYTES_TO_WORDS_8(B4, FF, 55, 23, 43, 39, 0B, 27),
        BYTES_TO_WORDS_8(BA, D8, BF, D7, B7, B0, 44, 50),
        BYTES_TO_WORDS_8(56, 32, 41, F5, AB, B3, 04, 0C),
        BYTES_TO_WORDS_4(85, 0A, 05, B4) },
    &double_jacobian_default,
    &mod_sqrt_secp224r1,
    &x_side_default
};

static const struct uECC_Curve_t curve_secp256r1 = {
    num_words_secp256r1,
    num_words_secp256r1,
    num_bytes_secp256r1,
    { BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(00, 00, 00, 00, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(01, 00, 00, 00, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(51, 25, 63, FC, C2, CA, B9, F3),
        BYTES_TO_WORDS_8(84, 9E, 17, A7, AD, FA, E6, BC),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(00, 00, 00, 00, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(96, C2, 98, D8, 45, 39, A1, F4),
        BYTES_TO_WORDS_8(A0, 33, EB, 2D, 81, 7D, 03, 77),
        BYTES_TO_WORDS_8(F2, 40, A4, 63, E5, E6, BC, F8),
        BYTES_TO_WORDS_8(47, 42, 2C, E1, F2, D1, 17, 6B),

        BYTES_TO_WORDS_8(F5, 51, BF, 37, 68, 40, B6, CB),
        BYTES_TO_WORDS_8(CE, 5E, 31, 6B, 57, 33, CE, 2B),
        BYTES_TO_WORDS_8(16, 9E, 0F, 7C, 4A, EB, E7, 8E),
        BYTES_TO_WORDS_8(9B, 7F, 1A, FE, E2, 42, E3, 4F) },
    { BYTES_TO_WORDS_8(4B, 60, D2, 27, 3E, 3C, CE, 3B),
        BYTES_TO_WORDS_8(F6, B0, 53, CC, B0, 06, 1D, 65),
        BYTES_TO_WORDS_8(BC, 86, 98, 76, 55, BD, EB, B3),
        BYTES_TO_WORDS_8(E7, 93, 3A, AA, D8, 35, C6, 5A) },
    &double_jacobian_default,
    &mod_sqrt_default,
    &x_side_default
};

static const struct uECC_Curve_t curve_secp256k1 = {
    num_words_secp256k1,
    num_words_secp256k1,
    num_bytes_secp256k1,
    { BYTES_TO_WORDS_8(2F, FC, FF, FF, FE, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(41, 41, 36, D0, 8C, 5E, D2, BF),
        BYTES_TO_WORDS_8(3B, A0, 48, AF, E6, DC, AE, BA),
        BYTES_TO_WORDS_8(FE, FF, FF, FF, FF, FF, FF, FF),
        BYTES_TO_WORDS_8(FF, FF, FF, FF, FF, FF, FF, FF) },
    { BYTES_TO_WORDS_8(98, 17, F8, 16, 5B, 81, F2, 59),
        BYTES_TO_WORDS_8(D9, 28, CE, 2D, DB, FC, 9B, 02),
        BYTES_TO_WORDS_8(07, 0B, 87, CE, 95, 62, A0, 55),
        BYTES_TO_WORDS_8(AC, BB, DC, F9, 7E, 66, BE, 79),

        BYTES_TO_WORDS_8(B8, D4, 10, FB, 8F, D0, 47, 9C),
        BYTES_TO_WORDS_8(19, 54, 85, A6, 48, B4, 17, FD),
        BYTES_TO_WORDS_8(A8, 08, 11, 0E, FC, FB, A4, 5D),
        BYTES_TO_WORDS_8(65, C4, A3, 26, 77, DA, 3A, 48) },
    { BYTES_TO_WORDS_8(07, 00, 00, 00, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(00, 00, 00, 00, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(00, 00, 00, 00, 00, 00, 00, 00),
        BYTES_TO_WORDS_8(00, 00, 00, 00, 00, 00, 00, 00) },
    &double_jacobian_secp256k1,
    &mod_sqrt_default,
    &x_side_secp256k1
};

uECC_Curve uECC_secp160r1(void) { return &curve_secp160r1; }
uECC_Curve uECC_secp192r1(void) { return &curve_secp192r1; }
uECC_Curve uECC_secp224r1(void) { return &curve_secp224r1; }
uECC_Curve uECC_secp256r1(void) { return &curve_secp256r1; }
uECC_Curve uECC_secp256k1(void) { return &curve_secp256k1; }

// #if !asm_mmod_fast
// 
// #if (uECC_CURVE == uECC_secp160r1 || uECC_CURVE == uECC_secp256k1)
// /* omega_mult() is defined farther below for the different curves / word sizes */
// static void omega_mult(uECC_word_t * RESTRICT result, const uECC_word_t * RESTRICT right);
// 
// /* Computes result = product % curve_p
//     see http://www.isys.uni-klu.ac.at/PDF/2001-0126-MT.pdf page 354
//     
//     Note that this only works if log2(omega) < log2(p) / 2 */
// static void vli_mmod_fast(uECC_word_t *RESTRICT result, uECC_word_t *RESTRICT product) {
//     uECC_word_t tmp[2 * uECC_WORDS];
//     uECC_word_t carry;
//     
//     vli_clear(tmp);
//     vli_clear(tmp + uECC_WORDS);
//     
//     omega_mult(tmp, product + uECC_WORDS); /* (Rq, q) = q * c */
//     
//     carry = vli_add(result, product, tmp); /* (C, r) = r + q       */
//     vli_clear(product);
//     omega_mult(product, tmp + uECC_WORDS); /* Rq*c */
//     carry += vli_add(result, result, product); /* (C1, r) = r + Rq*c */
//     
//     while (carry > 0) {
//         --carry;
//         vli_sub(result, result, curve_p);
//     }
//     if (vli_cmp(result, curve_p) > 0) {
//         vli_sub(result, result, curve_p);
//     }
// }
// 
// #endif
// 
// #if uECC_CURVE == uECC_secp160r1
// 
// #if uECC_WORD_SIZE == 1
// static void omega_mult(uint8_t * RESTRICT result, const uint8_t * RESTRICT right) {
//     uint8_t carry;
//     uint8_t i;
//     
//     /* Multiply by (2^31 + 1). */
//     vli_set(result + 4, right); /* 2^32 */
//     vli_rshift1(result + 4); /* 2^31 */
//     result[3] = right[0] << 7; /* get last bit from shift */
//     
//     carry = vli_add(result, result, right); /* 2^31 + 1 */
//     for (i = uECC_WORDS; carry; ++i) {
//         uint16_t sum = (uint16_t)result[i] + carry;
//         result[i] = (uint8_t)sum;
//         carry = sum >> 8;
//     }
// }
// #elif uECC_WORD_SIZE == 4
// static void omega_mult(uint32_t * RESTRICT result, const uint32_t * RESTRICT right) {
//     uint32_t carry;
//     unsigned i;
//     
//     /* Multiply by (2^31 + 1). */
//     vli_set(result + 1, right); /* 2^32 */
//     vli_rshift1(result + 1); /* 2^31 */
//     result[0] = right[0] << 31; /* get last bit from shift */
//     
//     carry = vli_add(result, result, right); /* 2^31 + 1 */
//     for (i = uECC_WORDS; carry; ++i) {
//         uint64_t sum = (uint64_t)result[i] + carry;
//         result[i] = (uint32_t)sum;
//         carry = sum >> 32;
//     }
// }
// #endif /* uECC_WORD_SIZE */
// 
// #elif uECC_CURVE == uECC_secp192r1
// 
// /* Computes result = product % curve_p.
//    See algorithm 5 and 6 from http://www.isys.uni-klu.ac.at/PDF/2001-0126-MT.pdf */
// #if uECC_WORD_SIZE == 1
// static void vli_mmod_fast(uint8_t *RESTRICT result, uint8_t *RESTRICT product) {
//     uint8_t tmp[uECC_WORDS];
//     uint8_t carry;
//     
//     vli_set(result, product);
//     
//     vli_set(tmp, &product[24]);
//     carry = vli_add(result, result, tmp);
//     
//     tmp[0] = tmp[1] = tmp[2] = tmp[3] = tmp[4] = tmp[5] = tmp[6] = tmp[7] = 0;
//     tmp[8] = product[24]; tmp[9] = product[25]; tmp[10] = product[26]; tmp[11] = product[27];
//     tmp[12] = product[28]; tmp[13] = product[29]; tmp[14] = product[30]; tmp[15] = product[31];
//     tmp[16] = product[32]; tmp[17] = product[33]; tmp[18] = product[34]; tmp[19] = product[35];
//     tmp[20] = product[36]; tmp[21] = product[37]; tmp[22] = product[38]; tmp[23] = product[39];
//     carry += vli_add(result, result, tmp);
//     
//     tmp[0] = tmp[8] = product[40];
//     tmp[1] = tmp[9] = product[41];
//     tmp[2] = tmp[10] = product[42];
//     tmp[3] = tmp[11] = product[43];
//     tmp[4] = tmp[12] = product[44];
//     tmp[5] = tmp[13] = product[45];
//     tmp[6] = tmp[14] = product[46];
//     tmp[7] = tmp[15] = product[47];
//     tmp[16] = tmp[17] = tmp[18] = tmp[19] = tmp[20] = tmp[21] = tmp[22] = tmp[23] = 0;
//     carry += vli_add(result, result, tmp);
//     
//     while (carry || vli_cmp(curve_p, result) != 1) {
//         carry -= vli_sub(result, result, curve_p);
//     }
// }
// #elif uECC_WORD_SIZE == 4
// static void vli_mmod_fast(uint32_t *RESTRICT result, uint32_t *RESTRICT product) {
//     uint32_t tmp[uECC_WORDS];
//     int carry;
//     
//     vli_set(result, product);
//     
//     vli_set(tmp, &product[6]);
//     carry = vli_add(result, result, tmp);
//     
//     tmp[0] = tmp[1] = 0;
//     tmp[2] = product[6];
//     tmp[3] = product[7];
//     tmp[4] = product[8];
//     tmp[5] = product[9];
//     carry += vli_add(result, result, tmp);
//     
//     tmp[0] = tmp[2] = product[10];
//     tmp[1] = tmp[3] = product[11];
//     tmp[4] = tmp[5] = 0;
//     carry += vli_add(result, result, tmp);
//     
//     while (carry || vli_cmp(curve_p, result) != 1) {
//         carry -= vli_sub(result, result, curve_p);
//     }
// }
// #else
// static void vli_mmod_fast(uint64_t *RESTRICT result, uint64_t *RESTRICT product) {
//     uint64_t tmp[uECC_WORDS];
//     int carry;
//     
//     vli_set(result, product);
//     
//     vli_set(tmp, &product[3]);
//     carry = vli_add(result, result, tmp);
//     
//     tmp[0] = 0;
//     tmp[1] = product[3];
//     tmp[2] = product[4];
//     carry += vli_add(result, result, tmp);
//     
//     tmp[0] = tmp[1] = product[5];
//     tmp[2] = 0;
//     carry += vli_add(result, result, tmp);
//     
//     while (carry || vli_cmp(curve_p, result) != 1) {
//         carry -= vli_sub(result, result, curve_p);
//     }
// }
// #endif /* uECC_WORD_SIZE */
// 
// #elif uECC_CURVE == uECC_secp256r1
// 
// /* Computes result = product % curve_p
//    from http://www.nsa.gov/ia/_files/nist-routines.pdf */
// #if uECC_WORD_SIZE == 1
// static void vli_mmod_fast(uint8_t *RESTRICT result, uint8_t *RESTRICT product) {
//     uint8_t tmp[uECC_BYTES];
//     int8_t carry;
//     
//     /* t */
//     vli_set(result, product);
//     
//     /* s1 */
//     tmp[0] = tmp[1] = tmp[2] = tmp[3] = 0;
//     tmp[4] = tmp[5] = tmp[6] = tmp[7] = 0;
//     tmp[8] = tmp[9] = tmp[10] = tmp[11] = 0;
//     tmp[12] = product[44]; tmp[13] = product[45]; tmp[14] = product[46]; tmp[15] = product[47];
//     tmp[16] = product[48]; tmp[17] = product[49]; tmp[18] = product[50]; tmp[19] = product[51];
//     tmp[20] = product[52]; tmp[21] = product[53]; tmp[22] = product[54]; tmp[23] = product[55];
//     tmp[24] = product[56]; tmp[25] = product[57]; tmp[26] = product[58]; tmp[27] = product[59];
//     tmp[28] = product[60]; tmp[29] = product[61]; tmp[30] = product[62]; tmp[31] = product[63];
//     carry = vli_add(tmp, tmp, tmp);
//     carry += vli_add(result, result, tmp);
//     
//     /* s2 */
//     tmp[12] = product[48]; tmp[13] = product[49]; tmp[14] = product[50]; tmp[15] = product[51];
//     tmp[16] = product[52]; tmp[17] = product[53]; tmp[18] = product[54]; tmp[19] = product[55];
//     tmp[20] = product[56]; tmp[21] = product[57]; tmp[22] = product[58]; tmp[23] = product[59];
//     tmp[24] = product[60]; tmp[25] = product[61]; tmp[26] = product[62]; tmp[27] = product[63];
//     tmp[28] = tmp[29] = tmp[30] = tmp[31] = 0;
//     carry += vli_add(tmp, tmp, tmp);
//     carry += vli_add(result, result, tmp);
//     
//     /* s3 */
//     tmp[0] = product[32]; tmp[1] = product[33]; tmp[2] = product[34]; tmp[3] = product[35];
//     tmp[4] = product[36]; tmp[5] = product[37]; tmp[6] = product[38]; tmp[7] = product[39];
//     tmp[8] = product[40]; tmp[9] = product[41]; tmp[10] = product[42]; tmp[11] = product[43];
//     tmp[12] = tmp[13] = tmp[14] = tmp[15] = 0;
//     tmp[16] = tmp[17] = tmp[18] = tmp[19] = 0;
//     tmp[20] = tmp[21] = tmp[22] = tmp[23] = 0;
//     tmp[24] = product[56]; tmp[25] = product[57]; tmp[26] = product[58]; tmp[27] = product[59];
//     tmp[28] = product[60]; tmp[29] = product[61]; tmp[30] = product[62]; tmp[31] = product[63];
//     carry += vli_add(result, result, tmp);
//     
//     /* s4 */
//     tmp[0] = product[36]; tmp[1] = product[37]; tmp[2] = product[38]; tmp[3] = product[39];
//     tmp[4] = product[40]; tmp[5] = product[41]; tmp[6] = product[42]; tmp[7] = product[43];
//     tmp[8] = product[44]; tmp[9] = product[45]; tmp[10] = product[46]; tmp[11] = product[47];
//     tmp[12] = product[52]; tmp[13] = product[53]; tmp[14] = product[54]; tmp[15] = product[55];
//     tmp[16] = product[56]; tmp[17] = product[57]; tmp[18] = product[58]; tmp[19] = product[59];
//     tmp[20] = product[60]; tmp[21] = product[61]; tmp[22] = product[62]; tmp[23] = product[63];
//     tmp[24] = product[52]; tmp[25] = product[53]; tmp[26] = product[54]; tmp[27] = product[55];
//     tmp[28] = product[32]; tmp[29] = product[33]; tmp[30] = product[34]; tmp[31] = product[35];
//     carry += vli_add(result, result, tmp);
//     
//     /* d1 */
//     tmp[0] = product[44]; tmp[1] = product[45]; tmp[2] = product[46]; tmp[3] = product[47];
//     tmp[4] = product[48]; tmp[5] = product[49]; tmp[6] = product[50]; tmp[7] = product[51];
//     tmp[8] = product[52]; tmp[9] = product[53]; tmp[10] = product[54]; tmp[11] = product[55];
//     tmp[12] = tmp[13] = tmp[14] = tmp[15] = 0;
//     tmp[16] = tmp[17] = tmp[18] = tmp[19] = 0;
//     tmp[20] = tmp[21] = tmp[22] = tmp[23] = 0;
//     tmp[24] = product[32]; tmp[25] = product[33]; tmp[26] = product[34]; tmp[27] = product[35];
//     tmp[28] = product[40]; tmp[29] = product[41]; tmp[30] = product[42]; tmp[31] = product[43];
//     carry -= vli_sub(result, result, tmp);
//     
//     /* d2 */
//     tmp[0] = product[48]; tmp[1] = product[49]; tmp[2] = product[50]; tmp[3] = product[51];
//     tmp[4] = product[52]; tmp[5] = product[53]; tmp[6] = product[54]; tmp[7] = product[55];
//     tmp[8] = product[56]; tmp[9] = product[57]; tmp[10] = product[58]; tmp[11] = product[59];
//     tmp[12] = product[60]; tmp[13] = product[61]; tmp[14] = product[62]; tmp[15] = product[63];
//     tmp[16] = tmp[17] = tmp[18] = tmp[19] = 0;
//     tmp[20] = tmp[21] = tmp[22] = tmp[23] = 0;
//     tmp[24] = product[36]; tmp[25] = product[37]; tmp[26] = product[38]; tmp[27] = product[39];
//     tmp[28] = product[44]; tmp[29] = product[45]; tmp[30] = product[46]; tmp[31] = product[47];
//     carry -= vli_sub(result, result, tmp);
//     
//     /* d3 */
//     tmp[0] = product[52]; tmp[1] = product[53]; tmp[2] = product[54]; tmp[3] = product[55];
//     tmp[4] = product[56]; tmp[5] = product[57]; tmp[6] = product[58]; tmp[7] = product[59];
//     tmp[8] = product[60]; tmp[9] = product[61]; tmp[10] = product[62]; tmp[11] = product[63];
//     tmp[12] = product[32]; tmp[13] = product[33]; tmp[14] = product[34]; tmp[15] = product[35];
//     tmp[16] = product[36]; tmp[17] = product[37]; tmp[18] = product[38]; tmp[19] = product[39];
//     tmp[20] = product[40]; tmp[21] = product[41]; tmp[22] = product[42]; tmp[23] = product[43];
//     tmp[24] = tmp[25] = tmp[26] = tmp[27] = 0;
//     tmp[28] = product[48]; tmp[29] = product[49]; tmp[30] = product[50]; tmp[31] = product[51];
//     carry -= vli_sub(result, result, tmp);
//     
//     /* d4 */
//     tmp[0] = product[56]; tmp[1] = product[57]; tmp[2] = product[58]; tmp[3] = product[59];
//     tmp[4] = product[60]; tmp[5] = product[61]; tmp[6] = product[62]; tmp[7] = product[63];
//     tmp[8] = tmp[9] = tmp[10] = tmp[11] = 0;
//     tmp[12] = product[36]; tmp[13] = product[37]; tmp[14] = product[38]; tmp[15] = product[39];
//     tmp[16] = product[40]; tmp[17] = product[41]; tmp[18] = product[42]; tmp[19] = product[43];
//     tmp[20] = product[44]; tmp[21] = product[45]; tmp[22] = product[46]; tmp[23] = product[47];
//     tmp[24] = tmp[25] = tmp[26] = tmp[27] = 0;
//     tmp[28] = product[52]; tmp[29] = product[53]; tmp[30] = product[54]; tmp[31] = product[55];
//     carry -= vli_sub(result, result, tmp);
//     
//     if (carry < 0) {
//         do {
//             carry += vli_add(result, result, curve_p);
//         } while (carry < 0);
//     } else {
//         while (carry || vli_cmp(curve_p, result) != 1) {
//             carry -= vli_sub(result, result, curve_p);
//         }
//     }
// }
// #elif uECC_WORD_SIZE == 4
// static void vli_mmod_fast(uint32_t *RESTRICT result, uint32_t *RESTRICT product) {
//     uint32_t tmp[uECC_WORDS];
//     int carry;
//     
//     /* t */
//     vli_set(result, product);
//     
//     /* s1 */
//     tmp[0] = tmp[1] = tmp[2] = 0;
//     tmp[3] = product[11];
//     tmp[4] = product[12];
//     tmp[5] = product[13];
//     tmp[6] = product[14];
//     tmp[7] = product[15];
//     carry = vli_add(tmp, tmp, tmp);
//     carry += vli_add(result, result, tmp);
//     
//     /* s2 */
//     tmp[3] = product[12];
//     tmp[4] = product[13];
//     tmp[5] = product[14];
//     tmp[6] = product[15];
//     tmp[7] = 0;
//     carry += vli_add(tmp, tmp, tmp);
//     carry += vli_add(result, result, tmp);
//     
//     /* s3 */
//     tmp[0] = product[8];
//     tmp[1] = product[9];
//     tmp[2] = product[10];
//     tmp[3] = tmp[4] = tmp[5] = 0;
//     tmp[6] = product[14];
//     tmp[7] = product[15];
//     carry += vli_add(result, result, tmp);
//     
//     /* s4 */
//     tmp[0] = product[9];
//     tmp[1] = product[10];
//     tmp[2] = product[11];
//     tmp[3] = product[13];
//     tmp[4] = product[14];
//     tmp[5] = product[15];
//     tmp[6] = product[13];
//     tmp[7] = product[8];
//     carry += vli_add(result, result, tmp);
//     
//     /* d1 */
//     tmp[0] = product[11];
//     tmp[1] = product[12];
//     tmp[2] = product[13];
//     tmp[3] = tmp[4] = tmp[5] = 0;
//     tmp[6] = product[8];
//     tmp[7] = product[10];
//     carry -= vli_sub(result, result, tmp);
//     
//     /* d2 */
//     tmp[0] = product[12];
//     tmp[1] = product[13];
//     tmp[2] = product[14];
//     tmp[3] = product[15];
//     tmp[4] = tmp[5] = 0;
//     tmp[6] = product[9];
//     tmp[7] = product[11];
//     carry -= vli_sub(result, result, tmp);
//     
//     /* d3 */
//     tmp[0] = product[13];
//     tmp[1] = product[14];
//     tmp[2] = product[15];
//     tmp[3] = product[8];
//     tmp[4] = product[9];
//     tmp[5] = product[10];
//     tmp[6] = 0;
//     tmp[7] = product[12];
//     carry -= vli_sub(result, result, tmp);
//     
//     /* d4 */
//     tmp[0] = product[14];
//     tmp[1] = product[15];
//     tmp[2] = 0;
//     tmp[3] = product[9];
//     tmp[4] = product[10];
//     tmp[5] = product[11];
//     tmp[6] = 0;
//     tmp[7] = product[13];
//     carry -= vli_sub(result, result, tmp);
//     
//     if (carry < 0) {
//         do {
//             carry += vli_add(result, result, curve_p);
//         } while (carry < 0);
//     } else {
//         while (carry || vli_cmp(curve_p, result) != 1) {
//             carry -= vli_sub(result, result, curve_p);
//         }
//     }
// }
// #else
// static void vli_mmod_fast(uint64_t *RESTRICT result, uint64_t *RESTRICT product) {
//     uint64_t tmp[uECC_WORDS];
//     int carry;
//     
//     /* t */
//     vli_set(result, product);
//     
//     /* s1 */
//     tmp[0] = 0;
//     tmp[1] = product[5] & 0xffffffff00000000ull;
//     tmp[2] = product[6];
//     tmp[3] = product[7];
//     carry = vli_add(tmp, tmp, tmp);
//     carry += vli_add(result, result, tmp);
//     
//     /* s2 */
//     tmp[1] = product[6] << 32;
//     tmp[2] = (product[6] >> 32) | (product[7] << 32);
//     tmp[3] = product[7] >> 32;
//     carry += vli_add(tmp, tmp, tmp);
//     carry += vli_add(result, result, tmp);
//     
//     /* s3 */
//     tmp[0] = product[4];
//     tmp[1] = product[5] & 0xffffffff;
//     tmp[2] = 0;
//     tmp[3] = product[7];
//     carry += vli_add(result, result, tmp);
//     
//     /* s4 */
//     tmp[0] = (product[4] >> 32) | (product[5] << 32);
//     tmp[1] = (product[5] >> 32) | (product[6] & 0xffffffff00000000ull);
//     tmp[2] = product[7];
//     tmp[3] = (product[6] >> 32) | (product[4] << 32);
//     carry += vli_add(result, result, tmp);
//     
//     /* d1 */
//     tmp[0] = (product[5] >> 32) | (product[6] << 32);
//     tmp[1] = (product[6] >> 32);
//     tmp[2] = 0;
//     tmp[3] = (product[4] & 0xffffffff) | (product[5] << 32);
//     carry -= vli_sub(result, result, tmp);
//     
//     /* d2 */
//     tmp[0] = product[6];
//     tmp[1] = product[7];
//     tmp[2] = 0;
//     tmp[3] = (product[4] >> 32) | (product[5] & 0xffffffff00000000ull);
//     carry -= vli_sub(result, result, tmp);
//     
//     /* d3 */
//     tmp[0] = (product[6] >> 32) | (product[7] << 32);
//     tmp[1] = (product[7] >> 32) | (product[4] << 32);
//     tmp[2] = (product[4] >> 32) | (product[5] << 32);
//     tmp[3] = (product[6] << 32);
//     carry -= vli_sub(result, result, tmp);
//     
//     /* d4 */
//     tmp[0] = product[7];
//     tmp[1] = product[4] & 0xffffffff00000000ull;
//     tmp[2] = product[5];
//     tmp[3] = product[6] & 0xffffffff00000000ull;
//     carry -= vli_sub(result, result, tmp);
//     
//     if (carry < 0) {
//         do {
//             carry += vli_add(result, result, curve_p);
//         } while (carry < 0);
//     } else {
//         while (carry || vli_cmp(curve_p, result) != 1) {
//             carry -= vli_sub(result, result, curve_p);
//         }
//     }
// }
// #endif /* uECC_WORD_SIZE */
// 
// #elif uECC_CURVE == uECC_secp256k1
// 
// #if uECC_WORD_SIZE == 1
// static void omega_mult(uint8_t * RESTRICT result, const uint8_t * RESTRICT right) {
//     /* Multiply by (2^32 + 2^9 + 2^8 + 2^7 + 2^6 + 2^4 + 1). */
//     uECC_word_t r0 = 0;
//     uECC_word_t r1 = 0;
//     uECC_word_t r2 = 0;
//     wordcount_t k;
//     
//     /* Multiply by (2^9 + 2^8 + 2^7 + 2^6 + 2^4 + 1). */
//     muladd(0xD1, right[0], &r0, &r1, &r2);
//     result[0] = r0;
//     r0 = r1;
//     r1 = r2;
//     /* r2 is still 0 */
//     
//     for (k = 1; k < uECC_WORDS; ++k) {
//         muladd(0x03, right[k - 1], &r0, &r1, &r2);
//         muladd(0xD1, right[k], &r0, &r1, &r2);
//         result[k] = r0;
//         r0 = r1;
//         r1 = r2;
//         r2 = 0;
//     }
//     muladd(0x03, right[uECC_WORDS - 1], &r0, &r1, &r2);
//     result[uECC_WORDS] = r0;
//     result[uECC_WORDS + 1] = r1;
// 
//     result[4 + uECC_WORDS] = vli_add(result + 4, result + 4, right); /* add the 2^32 multiple */
// }
// #elif uECC_WORD_SIZE == 4
// static void omega_mult(uint32_t * RESTRICT result, const uint32_t * RESTRICT right) {
//     /* Multiply by (2^9 + 2^8 + 2^7 + 2^6 + 2^4 + 1). */
//     uint32_t carry = 0;
//     wordcount_t k;
//     
//     for (k = 0; k < uECC_WORDS; ++k) {
//         uint64_t p = (uint64_t)0x3D1 * right[k] + carry;
//         result[k] = (p & 0xffffffff);
//         carry = p >> 32;
//     }
//     result[uECC_WORDS] = carry;
//     
//     result[1 + uECC_WORDS] = vli_add(result + 1, result + 1, right); /* add the 2^32 multiple */
// }
// #else
// static void omega_mult(uint64_t * RESTRICT result, const uint64_t * RESTRICT right) {
//     uECC_word_t r0 = 0;
//     uECC_word_t r1 = 0;
//     uECC_word_t r2 = 0;
//     wordcount_t k;
//     
//     /* Multiply by (2^32 + 2^9 + 2^8 + 2^7 + 2^6 + 2^4 + 1). */
//     for (k = 0; k < uECC_WORDS; ++k) {
//         muladd(0x1000003D1ull, right[k], &r0, &r1, &r2);
//         result[k] = r0;
//         r0 = r1;
//         r1 = r2;
//         r2 = 0;
//     }
//     result[uECC_WORDS] = r0;
// }
// #endif /* uECC_WORD_SIZE */
// 
// #elif uECC_CURVE == uECC_secp224r1
// 
// /* Computes result = product % curve_p
//    from http://www.nsa.gov/ia/_files/nist-routines.pdf */
// #if uECC_WORD_SIZE == 1
// // TODO it may be faster to use the omega_mult method when fully asm optimized.
// void vli_mmod_fast(uint8_t *RESTRICT result, uint8_t *RESTRICT product) {
//     uint8_t tmp[uECC_WORDS];
//     int8_t carry;
// 
//     /* t */
//     vli_set(result, product);
// 
//     /* s1 */
//     tmp[0] = tmp[1] = tmp[2] = tmp[3] = 0;
//     tmp[4] = tmp[5] = tmp[6] = tmp[7] = 0;
//     tmp[8] = tmp[9] = tmp[10] = tmp[11] = 0;
//     tmp[12] = product[28]; tmp[13] = product[29]; tmp[14] = product[30]; tmp[15] = product[31];
//     tmp[16] = product[32]; tmp[17] = product[33]; tmp[18] = product[34]; tmp[19] = product[35];
//     tmp[20] = product[36]; tmp[21] = product[37]; tmp[22] = product[38]; tmp[23] = product[39];
//     tmp[24] = product[40]; tmp[25] = product[41]; tmp[26] = product[42]; tmp[27] = product[43];
//     carry = vli_add(result, result, tmp);
// 
//     /* s2 */
//     tmp[12] = product[44]; tmp[13] = product[45]; tmp[14] = product[46]; tmp[15] = product[47];
//     tmp[16] = product[48]; tmp[17] = product[49]; tmp[18] = product[50]; tmp[19] = product[51];
//     tmp[20] = product[52]; tmp[21] = product[53]; tmp[22] = product[54]; tmp[23] = product[55];
//     tmp[24] = tmp[25] = tmp[26] = tmp[27] = 0;
//     carry += vli_add(result, result, tmp);
// 
//     /* d1 */
//     tmp[0]  = product[28]; tmp[1]  = product[29]; tmp[2]  = product[30]; tmp[3]  = product[31];
//     tmp[4]  = product[32]; tmp[5]  = product[33]; tmp[6]  = product[34]; tmp[7]  = product[35];
//     tmp[8]  = product[36]; tmp[9]  = product[37]; tmp[10] = product[38]; tmp[11] = product[39];
//     tmp[12] = product[40]; tmp[13] = product[41]; tmp[14] = product[42]; tmp[15] = product[43];
//     tmp[16] = product[44]; tmp[17] = product[45]; tmp[18] = product[46]; tmp[19] = product[47];
//     tmp[20] = product[48]; tmp[21] = product[49]; tmp[22] = product[50]; tmp[23] = product[51];
//     tmp[24] = product[52]; tmp[25] = product[53]; tmp[26] = product[54]; tmp[27] = product[55];
//     carry -= vli_sub(result, result, tmp);
// 
//     /* d2 */
//     tmp[0]  = product[44]; tmp[1]  = product[45]; tmp[2]  = product[46]; tmp[3]  = product[47];
//     tmp[4]  = product[48]; tmp[5]  = product[49]; tmp[6]  = product[50]; tmp[7]  = product[51];
//     tmp[8]  = product[52]; tmp[9]  = product[53]; tmp[10] = product[54]; tmp[11] = product[55];
//     tmp[12] = tmp[13] = tmp[14] = tmp[15] = 0;
//     tmp[16] = tmp[17] = tmp[18] = tmp[19] = 0;
//     tmp[20] = tmp[21] = tmp[22] = tmp[23] = 0;
//     tmp[24] = tmp[25] = tmp[26] = tmp[27] = 0;
//     carry -= vli_sub(result, result, tmp);
// 
//     if (carry < 0) {
//         do {
//             carry += vli_add(result, result, curve_p);
//         } while (carry < 0);
//     } else {
//         while (carry || vli_cmp(curve_p, result) != 1) {
//             carry -= vli_sub(result, result, curve_p);
//         }
//     }
// }
// #elif uECC_WORD_SIZE == 4
// void vli_mmod_fast(uint32_t *RESTRICT result, uint32_t *RESTRICT product)
// {
//     uint32_t tmp[uECC_WORDS];
//     int carry;
// 
//     /* t */
//     vli_set(result, product);
// 
//     /* s1 */
//     tmp[0] = tmp[1] = tmp[2] = 0;
//     tmp[3] = product[7];
//     tmp[4] = product[8];
//     tmp[5] = product[9];
//     tmp[6] = product[10];
//     carry = vli_add(result, result, tmp);
// 
//     /* s2 */
//     tmp[3] = product[11];
//     tmp[4] = product[12];
//     tmp[5] = product[13];
//     tmp[6] = 0;
//     carry += vli_add(result, result, tmp);
// 
//     /* d1 */
//     tmp[0] = product[7];
//     tmp[1] = product[8];
//     tmp[2] = product[9];
//     tmp[3] = product[10];
//     tmp[4] = product[11];
//     tmp[5] = product[12];
//     tmp[6] = product[13];
//     carry -= vli_sub(result, result, tmp);
// 
//     /* d2 */
//     tmp[0] = product[11];
//     tmp[1] = product[12];
//     tmp[2] = product[13];
//     tmp[3] = tmp[4] = tmp[5] = tmp[6] = 0;
//     carry -= vli_sub(result, result, tmp);
// 
//     if (carry < 0) {
//         do {
//             carry += vli_add(result, result, curve_p);
//         } while (carry < 0);
//     } else {
//         while (carry || vli_cmp(curve_p, result) != 1) {
//             carry -= vli_sub(result, result, curve_p);
//         }
//     }
// }
// #endif /* uECC_WORD_SIZE */
// 
// #endif /* uECC_CURVE */
// #endif /* !asm_mmod_fast */
