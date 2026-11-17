/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#ifndef _UECC_ASM_AVR_H_
#define _UECC_ASM_AVR_H_

#if (uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1)
    #define uECC_MIN_WORDS 32
#endif
#if uECC_SUPPORTS_secp224r1
    #undef uECC_MIN_WORDS
    #define uECC_MIN_WORDS 28
#endif
#if uECC_SUPPORTS_secp192r1
    #undef uECC_MIN_WORDS
    #define uECC_MIN_WORDS 24
#endif
#if uECC_SUPPORTS_secp160r1
    #undef uECC_MIN_WORDS
    #define uECC_MIN_WORDS 20
#endif

#if __AVR_HAVE_EIJMP_EICALL__
    #define IJMP "eijmp \n\t"
#else
    #define IJMP "ijmp \n\t"
#endif

/* Multi-curve builds at optimization level >= 3 exceed rjmp's +/-4 KB reach
   across curve blocks; use 4-byte jmp when supported by the target MCU. */
#if __AVR_HAVE_JMP_CALL__
    #define uECC_JMP "jmp "
#else
    #define uECC_JMP "rjmp "
#endif

#if (uECC_OPTIMIZATION_LEVEL >= 2)

uECC_VLI_API void uECC_vli_clear(uECC_word_t *vli, wordcount_t num_words) {
    __asm__ volatile (
    #if (uECC_MAX_WORDS != uECC_MIN_WORDS)
        "ldi r30, pm_lo8(1f) \n\t"
        "ldi r31, pm_hi8(1f) \n\t"
        "sub r30, %[num] \n\t"
        "sbc r31, __zero_reg__ \n\t"
        IJMP
    #endif

        REPEAT(uECC_MAX_WORDS, "st x+, __zero_reg__ \n\t")
        "1: \n\t"
        : "+x" (vli)
        : [num] "r" (num_words)
        : "memory"
    #if (uECC_MAX_WORDS != uECC_MIN_WORDS)
          , "r30", "r31", "cc"
    #endif
    );
}
#define asm_clear 1

/* noinline: the "+y" operand needs Y (r28:r29), which a caller with a stack
   frame uses as its frame pointer, so inlining can fail to allocate it. */
__attribute__((noinline))
uECC_VLI_API void uECC_vli_set(uECC_word_t *dest, const uECC_word_t *src, wordcount_t num_words) {
    __asm__ volatile (
    #if (uECC_MAX_WORDS != uECC_MIN_WORDS)
        "ldi r30, pm_lo8(1f) \n\t"
        "ldi r31, pm_hi8(1f) \n\t"
        "sub r30, %[num] \n\t"
        "sbc r31, __zero_reg__ \n\t"
        IJMP
    #endif

        REPEAT(uECC_MAX_WORDS,
            "ld r0, y+ \n\t"
            "st x+, r0 \n\t")
        "1: \n\t"
        : "+x" (dest), "+y" (src)
        : [num] "r" ((uint8_t)(num_words * 2))
        : "r0", "memory"
    #if (uECC_MAX_WORDS != uECC_MIN_WORDS)
          , "r30", "r31", "cc"
    #endif
    );
}
#define asm_set 1

uECC_VLI_API void uECC_vli_rshift1(uECC_word_t *vli, wordcount_t num_words) {
    __asm__ volatile (
    #if (uECC_MAX_WORDS != uECC_MIN_WORDS)
        "ldi r30, pm_lo8(1f) \n\t"
        "ldi r31, pm_hi8(1f) \n\t"
        "sub r30, %[jump] \n\t"
        "sbc r31, __zero_reg__ \n\t"
    #endif

        "add r26, %[num] \n\t"
        "adc r27, __zero_reg__ \n\t"
        "ld r0, -x \n\t"
        "lsr r0 \n\t"
        "st x, r0 \n\t"
    #if (uECC_MAX_WORDS != uECC_MIN_WORDS)
        IJMP
    #endif

        REPEAT(DEC(uECC_MAX_WORDS),
            "ld r0, -x \n\t"
            "ror r0 \n\t"
            "st x, r0 \n\t")
        "1: \n\t"
        : "+x" (vli)
    #if (uECC_MAX_WORDS != uECC_MIN_WORDS)
        : [num] "r" (num_words), [jump] "r" ((uint8_t)(3 * (num_words - 1)))
        : "r0", "r30", "r31", "cc", "memory"
    #else
        : [num] "r" (num_words)
        : "r0", "cc", "memory"
    #endif
    );
}
#define asm_rshift1 1

/* One byte of result = left + right (first byte uses add, the rest adc). */
#define AVR_ADD_FIRST_BYTE \
    "ld %[clb], x+ \n\t"   \
    "ld %[rb], y+ \n\t"    \
    "add %[clb], %[rb] \n\t" \
    "st z+, %[clb] \n\t"

#define AVR_ADC_BYTE \
    "ld %[clb], x+ \n\t"   \
    "ld %[rb], y+ \n\t"    \
    "adc %[clb], %[rb] \n\t" \
    "st z+, %[clb] \n\t"

/* Computes result = left + right, returning carry. Can modify in place.
   Unrolled; when several sizes are supported, the first uECC_MIN_WORDS bytes are
   always added and the tail is selected by testing bits of num_words with
   sbrs/sbrc + rjmp, which do not affect the carry flag. */
__attribute__((noinline))
uECC_VLI_API uECC_word_t uECC_vli_add(uECC_word_t *result,
                                      const uECC_word_t *left,
                                      const uECC_word_t *right,
                                      wordcount_t num_words) {
    uint8_t carry;
    uint8_t right_byte;

    __asm__ volatile (
    #if (uECC_MAX_WORDS == uECC_MIN_WORDS)
        AVR_ADD_FIRST_BYTE
        REPEAT(DEC(uECC_MAX_WORDS), AVR_ADC_BYTE)
    #elif (uECC_MIN_WORDS == 20) && (uECC_MAX_WORDS == 21)
        /* secp160r1 only: 20 bytes, plus 1 more if num_words == 21. */
        AVR_ADD_FIRST_BYTE
        REPEAT(19, AVR_ADC_BYTE)
        "sbrs %[num], 0 \n\t"
        "rjmp done_%= \n\t"
        AVR_ADC_BYTE
        "done_%=: \n\t"
    #elif (uECC_MIN_WORDS == 20) && (uECC_MAX_WORDS == 32)
        /* All curves: num_words is 20, 21, 24, 28 or 32. Label at_K enters a
           32-byte chain at byte K (so 32 - K more bytes are added): 32 -> at_20,
           28 -> at_24, 24 -> at_28, 21 -> at_31. */
        AVR_ADD_FIRST_BYTE
        REPEAT(19, AVR_ADC_BYTE)
        "sbrc %[num], 5 \n\t"
        "rjmp at_20_%= \n\t"
        "sbrs %[num], 3 \n\t"
        "rjmp disp_20_21_%= \n\t"
        "sbrc %[num], 2 \n\t"
        "rjmp at_24_%= \n\t"
        "rjmp at_28_%= \n\t"
        "disp_20_21_%=: \n\t"
        "sbrs %[num], 0 \n\t"
        "rjmp done_%= \n\t"
        "rjmp at_31_%= \n\t"
        "at_20_%=: \n\t"
        REPEAT(4, AVR_ADC_BYTE)
        "at_24_%=: \n\t"
        REPEAT(4, AVR_ADC_BYTE)
        "at_28_%=: \n\t"
        REPEAT(3, AVR_ADC_BYTE)
        "at_31_%=: \n\t"
        AVR_ADC_BYTE
        "done_%=: \n\t"
    #else
        /* Other size sets: add the remaining num_words - uECC_MIN_WORDS bytes
           in chunks of 8/4/2/1 selected by its bits. */
        "subi %[num], " STR(uECC_MIN_WORDS) " \n\t"
        AVR_ADD_FIRST_BYTE
        REPEAT(DEC(uECC_MIN_WORDS), AVR_ADC_BYTE)
        #if (uECC_MAX_WORDS - uECC_MIN_WORDS) >= 8
        "sbrs %[num], 3 \n\t"
        "rjmp add_skip8_%= \n\t"
        REPEAT(8, AVR_ADC_BYTE)
        "add_skip8_%=: \n\t"
        #endif
        #if (uECC_MAX_WORDS - uECC_MIN_WORDS) >= 4
        "sbrs %[num], 2 \n\t"
        "rjmp add_skip4_%= \n\t"
        REPEAT(4, AVR_ADC_BYTE)
        "add_skip4_%=: \n\t"
        #endif
        #if (uECC_MAX_WORDS - uECC_MIN_WORDS) >= 2
        "sbrs %[num], 1 \n\t"
        "rjmp add_skip2_%= \n\t"
        REPEAT(2, AVR_ADC_BYTE)
        "add_skip2_%=: \n\t"
        #endif
        #if (uECC_MAX_WORDS - uECC_MIN_WORDS) >= 1
        "sbrs %[num], 0 \n\t"
        "rjmp add_skip1_%= \n\t"
        AVR_ADC_BYTE
        "add_skip1_%=: \n\t"
        #endif
    #endif

        "mov %[clb], __zero_reg__ \n\t"
        "adc %[clb], %[clb] \n\t" /* Store carry bit. */

        : "+z" (result), "+x" (left), "+y" (right),
          [clb] "=&r" (carry), [rb] "=&r" (right_byte)
    #if (uECC_MAX_WORDS == uECC_MIN_WORDS) || \
        ((uECC_MIN_WORDS == 20) && ((uECC_MAX_WORDS == 21) || (uECC_MAX_WORDS == 32)))
        : [num] "r" (num_words)
    #else
        , [num] "+d" (num_words)
        :
    #endif
        : "cc", "memory"
    );
    return carry;
}
#define asm_add 1

/* One byte of result = left - right (first byte uses sub, the rest sbc). */
#define AVR_SUB_FIRST_BYTE \
    "ld %[clb], x+ \n\t"   \
    "ld %[rb], y+ \n\t"    \
    "sub %[clb], %[rb] \n\t" \
    "st z+, %[clb] \n\t"

#define AVR_SBC_BYTE \
    "ld %[clb], x+ \n\t"   \
    "ld %[rb], y+ \n\t"    \
    "sbc %[clb], %[rb] \n\t" \
    "st z+, %[clb] \n\t"

/* Computes result = left - right, returning borrow. Can modify in place.
   Same size dispatch as uECC_vli_add. */
__attribute__((noinline))
uECC_VLI_API uECC_word_t uECC_vli_sub(uECC_word_t *result,
                                      const uECC_word_t *left,
                                      const uECC_word_t *right,
                                      wordcount_t num_words) {
    uint8_t carry;
    uint8_t right_byte;

    __asm__ volatile (
    #if (uECC_MAX_WORDS == uECC_MIN_WORDS)
        AVR_SUB_FIRST_BYTE
        REPEAT(DEC(uECC_MAX_WORDS), AVR_SBC_BYTE)
    #elif (uECC_MIN_WORDS == 20) && (uECC_MAX_WORDS == 21)
        AVR_SUB_FIRST_BYTE
        REPEAT(19, AVR_SBC_BYTE)
        "sbrs %[num], 0 \n\t"
        "rjmp done_%= \n\t"
        AVR_SBC_BYTE
        "done_%=: \n\t"
    #elif (uECC_MIN_WORDS == 20) && (uECC_MAX_WORDS == 32)
        AVR_SUB_FIRST_BYTE
        REPEAT(19, AVR_SBC_BYTE)
        "sbrc %[num], 5 \n\t"
        "rjmp at_20_%= \n\t"
        "sbrs %[num], 3 \n\t"
        "rjmp disp_20_21_%= \n\t"
        "sbrc %[num], 2 \n\t"
        "rjmp at_24_%= \n\t"
        "rjmp at_28_%= \n\t"
        "disp_20_21_%=: \n\t"
        "sbrs %[num], 0 \n\t"
        "rjmp done_%= \n\t"
        "rjmp at_31_%= \n\t"
        "at_20_%=: \n\t"
        REPEAT(4, AVR_SBC_BYTE)
        "at_24_%=: \n\t"
        REPEAT(4, AVR_SBC_BYTE)
        "at_28_%=: \n\t"
        REPEAT(3, AVR_SBC_BYTE)
        "at_31_%=: \n\t"
        AVR_SBC_BYTE
        "done_%=: \n\t"
    #else
        "subi %[num], " STR(uECC_MIN_WORDS) " \n\t"
        AVR_SUB_FIRST_BYTE
        REPEAT(DEC(uECC_MIN_WORDS), AVR_SBC_BYTE)
        #if (uECC_MAX_WORDS - uECC_MIN_WORDS) >= 8
        "sbrs %[num], 3 \n\t"
        "rjmp sub_skip8_%= \n\t"
        REPEAT(8, AVR_SBC_BYTE)
        "sub_skip8_%=: \n\t"
        #endif
        #if (uECC_MAX_WORDS - uECC_MIN_WORDS) >= 4
        "sbrs %[num], 2 \n\t"
        "rjmp sub_skip4_%= \n\t"
        REPEAT(4, AVR_SBC_BYTE)
        "sub_skip4_%=: \n\t"
        #endif
        #if (uECC_MAX_WORDS - uECC_MIN_WORDS) >= 2
        "sbrs %[num], 1 \n\t"
        "rjmp sub_skip2_%= \n\t"
        REPEAT(2, AVR_SBC_BYTE)
        "sub_skip2_%=: \n\t"
        #endif
        #if (uECC_MAX_WORDS - uECC_MIN_WORDS) >= 1
        "sbrs %[num], 0 \n\t"
        "rjmp sub_skip1_%= \n\t"
        AVR_SBC_BYTE
        "sub_skip1_%=: \n\t"
        #endif
    #endif

        "mov %[clb], __zero_reg__ \n\t"
        "adc %[clb], %[clb] \n\t" /* Store carry bit. */

        : "+z" (result), "+x" (left), "+y" (right),
          [clb] "=&r" (carry), [rb] "=&r" (right_byte)
    #if (uECC_MAX_WORDS == uECC_MIN_WORDS) || \
        ((uECC_MIN_WORDS == 20) && ((uECC_MAX_WORDS == 21) || (uECC_MAX_WORDS == 32)))
        : [num] "r" (num_words)
    #else
        , [num] "+d" (num_words)
        :
    #endif
        : "cc", "memory"
    );
    return carry;
}
#define asm_sub 1

#if (uECC_OPTIMIZATION_LEVEL >= 3)

/* Constant-time multiply and square (asm_avr_mult_square.inc):
 *   mult 20:     2-level subtractive Karatsuba, fully unrolled; num_words == 21
 *                (secp160r1 order n) adds a border row/column (..._EXT_21).
 *   mult 24/28/32: 1-level subtractive Karatsuba with 3 rcalls to a shared
 *                half-size leaf (much smaller, ~2% slower than fully unrolled).
 *                At level 4, 24 and 32 use fully unrolled 2/3-level Karatsuba.
 *   square 20/24: MOVW-windowed product scanning: cross products once, doubled,
 *                plus diagonal squares (20 also has an ..._EXT_21 border).
 *   square 28/32: 1-level Karatsuba, A^2 = L^2 + 2^8k (L^2 + H^2 - (L - H)^2)
 *                + 2^16k H^2, with 3 rcalls to a half-size leaf squarer.
 */
#include "asm_avr_mult_square.inc"

/* Computes result = left * right. Jumps to the fixed-size multiplier for num_words. */
__attribute((noinline))
uECC_VLI_API void uECC_vli_mult(uECC_word_t *result,
                                const uECC_word_t *left,
                                const uECC_word_t *right,
                                wordcount_t num_words) {
    /* num_words should already be in r18. */
    register wordcount_t r18 __asm__("r18") = num_words;

    __asm__ volatile (
        /* Dispatch on num_words. secp160r1 alone gives MAX_WORDS == 21 (for n),
           so other curves are present iff uECC_MAX_WORDS > 21. Short branches
           stay in this header; uECC_JMP reaches the multi-KB curve blocks. */
#if uECC_SUPPORTS_secp160r1 && (uECC_MAX_WORDS > 21)
        "cpi r18, 22 \n\t"
        "brlo 20f \n\t"
#endif
#if uECC_SUPPORTS_secp192r1 && ((uECC_MIN_WORDS < 24) || (uECC_MAX_WORDS > 24))
    #if (uECC_MIN_WORDS == 24)
        "cpi r18, 24 \n\t"
        "breq 24f \n\t"
    #else
        #if (uECC_MAX_WORDS > 24)
        "cpi r18, 24 \n\t"
        "brne 19f \n\t"
        #endif
        uECC_JMP "24f \n\t"
        "19: \n\t"
    #endif
#endif
#if uECC_SUPPORTS_secp224r1 && ((uECC_MIN_WORDS < 28) || (uECC_MAX_WORDS > 28))
    #if (uECC_MIN_WORDS == 28)
        "cpi r18, 28 \n\t"
        "breq 28f \n\t"
    #else
        #if (uECC_MAX_WORDS > 28)
        "cpi r18, 28 \n\t"
        "brne 27f \n\t"
        #endif
        uECC_JMP "28f \n\t"
        "27: \n\t"
    #endif
#endif
#if (uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1) && (uECC_MIN_WORDS < 32)
        uECC_JMP "32f \n\t"
#endif

#if uECC_SUPPORTS_secp160r1
        "20: \n\t"
        /* num_words is 20 for field ops, 21 for ECDSA ops mod n (161 bits).
           For 21, save X/Y/Z (restored by _EXT_21), do the 20x20 product,
           then add the 21st border row/column. */
        "cpi r18, 21 \n\t"
        "brne 80f \n\t"
        "push r26 \n\t"
        "push r27 \n\t"
        "push r28 \n\t"
        "push r29 \n\t"
        "push r30 \n\t"
        "push r31 \n\t"
        "80: \n\t"
        "push r18 \n\t" /* FAST_MULT_ASM_20 clobbers r18. */
        FAST_MULT_ASM_20
        "pop r18 \n\t"
        "cpi r18, 21 \n\t"
        "breq 84f \n\t"
        uECC_JMP "99f \n\t"
        "84: \n\t"
        FAST_MULT_ASM_20_EXT_21
    #if (uECC_MAX_WORDS > 21)
        uECC_JMP "99f \n\t"
    #endif
#endif
        /* Each 1-level block's leaf subroutine sits right after its exit jump,
           keeping its rcalls in range. Level 4 unrolled code has no leaf. */
#if uECC_SUPPORTS_secp192r1
        "24: \n\t"
    #if (uECC_OPTIMIZATION_LEVEL >= 4)
        FAST_MULT_ASM_24_UNROLLED
        uECC_JMP "99f \n\t"
    #else
        FAST_MULT_ASM_24
        uECC_JMP "99f \n\t"
        FAST_MULT_LEAF_24
    #endif
#endif
#if uECC_SUPPORTS_secp224r1
        "28: \n\t"
        FAST_MULT_ASM_28
        uECC_JMP "99f \n\t"
        FAST_MULT_LEAF_28
#endif
#if (uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1)
        "32: \n\t"
    #if (uECC_OPTIMIZATION_LEVEL >= 4)
        FAST_MULT_ASM_32_UNROLLED
    #else
        FAST_MULT_ASM_32
        uECC_JMP "99f \n\t"
        FAST_MULT_LEAF_32
    #endif
#endif
        "99: \n\t"
        "eor r1, r1 \n\t" /* Restore __zero_reg__. */
        : "+x" (left), "+y" (right), "+z" (result), "+r" (r18)
        :
        : "r0", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10",
          "r11", "r12", "r13", "r14", "r15", "r16", "r17", "r19", "r20",
          "r21", "r22", "r23", "r24", "r25", "cc", "memory"
    );
}
#define asm_mult 1

#if uECC_SQUARE_FUNC
/* Computes result = left^2. Jumps to the fixed-size squarer for num_words. */
__attribute((noinline))
uECC_VLI_API void uECC_vli_square(uECC_word_t *result,
                                  const uECC_word_t *left,
                                  wordcount_t num_words) {
    /* num_words should already be in r20. */
    register wordcount_t r20 __asm__("r20") = num_words;

    __asm__ volatile (
        /* Multi-curve size dispatch (same structure as uECC_vli_mult). */
#if uECC_SUPPORTS_secp160r1 && (uECC_MAX_WORDS > 21)
        "cpi r20, 22 \n\t"
        "brlo 20f \n\t"
#endif
#if uECC_SUPPORTS_secp192r1 && ((uECC_MIN_WORDS < 24) || (uECC_MAX_WORDS > 24))
    #if (uECC_MIN_WORDS == 24)
        "cpi r20, 24 \n\t"
        "breq 24f \n\t"
    #else
        #if (uECC_MAX_WORDS > 24)
        "cpi r20, 24 \n\t"
        "brne 19f \n\t"
        #endif
        uECC_JMP "24f \n\t"
        "19: \n\t"
    #endif
#endif
#if uECC_SUPPORTS_secp224r1 && ((uECC_MIN_WORDS < 28) || (uECC_MAX_WORDS > 28))
    #if (uECC_MIN_WORDS == 28)
        "cpi r20, 28 \n\t"
        "breq 28f \n\t"
    #else
        #if (uECC_MAX_WORDS > 28)
        "cpi r20, 28 \n\t"
        "brne 27f \n\t"
        #endif
        uECC_JMP "28f \n\t"
        "27: \n\t"
    #endif
#endif
#if (uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1) && (uECC_MIN_WORDS < 32)
        uECC_JMP "32f \n\t"
#endif

#if uECC_SUPPORTS_secp160r1
        "20: \n\t"
        /* Same as mult: for 21 words, save X/Z, square 20 bytes, then add the
           21st border via FAST_SQUARE_ASM_20_EXT_21. */
        "cpi r20, 21 \n\t"
        "brne 80f \n\t"
        "push r26 \n\t"
        "push r27 \n\t"
        "push r30 \n\t"
        "push r31 \n\t"
        "80: \n\t"
        "push r20 \n\t" /* FAST_SQUARE_ASM_20 clobbers r20. */
        FAST_SQUARE_ASM_20
        "pop r20 \n\t"
        "cpi r20, 21 \n\t"
        "breq 84f \n\t"
        uECC_JMP "99f \n\t"
        "84: \n\t"
        FAST_SQUARE_ASM_20_EXT_21
    #if (uECC_MAX_WORDS > 21)
        uECC_JMP "99f \n\t"
    #endif
#endif
#if uECC_SUPPORTS_secp192r1
        "24: \n\t"
        FAST_SQUARE_ASM_24
    #if (uECC_MAX_WORDS > 24)
        uECC_JMP "99f \n\t"
    #endif
#endif
        /* Leaf squarers sit right after their block's exit jump. */
#if uECC_SUPPORTS_secp224r1
        "28: \n\t"
        FAST_SQUARE_ASM_28
        uECC_JMP "99f \n\t"
        FAST_SQUARE_LEAF_28
#endif
#if (uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1)
        "32: \n\t"
        FAST_SQUARE_ASM_32
        uECC_JMP "99f \n\t"
        FAST_SQUARE_LEAF_32
#endif
        "99: \n\t"
        "eor r1, r1 \n\t" /* Restore __zero_reg__. */
        : "+x" (left), "+z" (result), "+r" (r20)
        :
        : "r0", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10",
          "r11", "r12", "r13", "r14", "r15",
    #if (uECC_SUPPORTS_secp160r1 || uECC_SUPPORTS_secp192r1 || \
         uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1)
          "r16", "r17", /* Not used by the secp224r1 squarer. */
    #endif
          "r18", "r19", "r21", "r22", "r23", "r24", "r25", "r28", "r29", "cc", "memory"
    );
}
#define asm_square 1
#endif /* uECC_SQUARE_FUNC */

#endif /* (uECC_OPTIMIZATION_LEVEL >= 3) */

#include "asm_avr_mmod.inc"

#endif /* (uECC_OPTIMIZATION_LEVEL >= 2) */

/* ---- "Small" implementations ---- */

#if !asm_add
__attribute__((noinline))
uECC_VLI_API uECC_word_t uECC_vli_add(uECC_word_t *result,
                                      const uECC_word_t *left,
                                      const uECC_word_t *right,
                                      wordcount_t num_words) {
    uint8_t left_byte;
    uint8_t right_byte;

    __asm__ volatile (
        "clc \n\t"

        "1: \n\t"
        "ld %[left], x+ \n\t"  /* Load left byte. */
        "ld %[right], y+ \n\t" /* Load right byte. */
        "adc %[left], %[right] \n\t" /* Add. */
        "st z+, %[left] \n\t"  /* Store the result. */
        "dec %[i] \n\t"
        "brne 1b \n\t"

        "mov %[left], __zero_reg__ \n\t"
        "adc %[left], %[left] \n\t" /* Store carry bit. */

        : "+z" (result), "+x" (left), "+y" (right), [i] "+r" (num_words),
          [left] "=&r" (left_byte), [right] "=&r" (right_byte)
        :
        : "cc", "memory"
    );
    return left_byte;
}
#define asm_add 1
#endif

#if !asm_sub
__attribute__((noinline))
uECC_VLI_API uECC_word_t uECC_vli_sub(uECC_word_t *result,
                                      const uECC_word_t *left,
                                      const uECC_word_t *right,
                                      wordcount_t num_words) {
    uint8_t left_byte;
    uint8_t right_byte;

    __asm__ volatile (
        "clc \n\t"

        "1: \n\t"
        "ld %[left], x+ \n\t"  /* Load left byte. */
        "ld %[right], y+ \n\t" /* Load right byte. */
        "sbc %[left], %[right] \n\t" /* Subtract. */
        "st z+, %[left] \n\t"  /* Store the result. */
        "dec %[i] \n\t"
        "brne 1b \n\t"

        "mov %[left], __zero_reg__ \n\t"
        "adc %[left], %[left] \n\t" /* Store carry bit in borrow. */

        : "+z" (result), "+x" (left), "+y" (right), [i] "+r" (num_words),
          [left] "=&r" (left_byte), [right] "=&r" (right_byte)
        :
        : "cc", "memory"
    );
    return left_byte;
}
#define asm_sub 1
#endif

#if !asm_mult
__attribute((noinline))
uECC_VLI_API void uECC_vli_mult(uECC_word_t *result,
                                const uECC_word_t *left,
                                const uECC_word_t *right,
                                wordcount_t num_words) {
    uint8_t r0 = 0;
    uint8_t r1 = 0;
    uint8_t r2 = 0;
    uint8_t zero = 0;
    uint8_t k, i;

    __asm__ volatile (
        "ldi %[k], 1 \n\t" /* k = 1; k < num_words; ++k */

        "1: \n\t"
        "ldi %[i], 0 \n\t"  /* i = 0; i < k; ++i */

        "add r28, %[k] \n\t" /* pre-add right ptr */
        "adc r29, %[zero] \n\t"

        "2: \n\t"
        "ld r0, x+ \n\t"
        "ld r1, -y \n\t"
        "mul r0, r1 \n\t"

        "add %[r0], r0 \n\t"
        "adc %[r1], r1 \n\t"
        "adc %[r2], %[zero] \n\t"

        "inc %[i] \n\t"
        "cp %[i], %[k] \n\t"
        "brlo 2b \n\t" /* loop if i < k */

        "sub r26, %[k] \n\t" /* fix up left ptr */
        "sbc r27, %[zero] \n\t"

        "st z+, %[r0] \n\t"  /* Store the result. */
        "mov %[r0], %[r1] \n\t"
        "mov %[r1], %[r2] \n\t"
        "mov %[r2], %[zero] \n\t"

        "inc %[k] \n\t"
        "cp %[k], %[num] \n\t"
        "brlo 1b \n\t" /* loop if k < num_words */

        /* second half */
        "mov %[k], %[num] \n\t" /* k = num_words; k > 0; --k */
        "add r28, %[num] \n\t" /* move right ptr to point at the end of right */
        "adc r29, %[zero] \n\t"

        "1: \n\t"
        "ldi %[i], 0 \n\t" /* i = 0; i < k; ++i */

        "2: \n\t"
        "ld r0, x+ \n\t"
        "ld r1, -y \n\t"
        "mul r0, r1 \n\t"

        "add %[r0], r0 \n\t"
        "adc %[r1], r1 \n\t"
        "adc %[r2], %[zero] \n\t"

        "inc %[i] \n\t"
        "cp %[i], %[k] \n\t"
        "brlo 2b \n\t" /* loop if i < k */

        "add r28, %[k] \n\t" /* fix up right ptr */
        "adc r29, %[zero] \n\t"

        "st z+, %[r0] \n\t"  /* Store the result. */
        "mov %[r0], %[r1] \n\t"
        "mov %[r1], %[r2] \n\t"
        "mov %[r2], %[zero] \n\t"

        "dec %[k] \n\t"
        "sub r26, %[k] \n\t" /* fix up left ptr (after k is decremented, so next time
                                we start 1 higher) */
        "sbc r27, %[zero] \n\t"

        "cp %[k], %[zero] \n\t"
        "brne 1b \n\t" /* loop if k > 0 */

        "st z+, %[r0] \n\t"  /* Store last result byte. */
        "eor r1, r1 \n\t" /* fix r1 to be 0 again */

        : "+z" (result), "+x" (left), "+y" (right),
          [r0] "+r" (r0), [r1] "+r" (r1), [r2] "+r" (r2),
          [zero] "+r" (zero), [num] "+r" (num_words),
          [k] "=&d" (k), [i] "=&d" (i)
        :
        : "r0", "cc", "memory"
    );
}
#define asm_mult 1
#endif

#if (uECC_SQUARE_FUNC && !asm_square)
uECC_VLI_API void uECC_vli_square(uECC_word_t *result,
                                  const uECC_word_t *left,
                                  wordcount_t num_words) {
    uint8_t r0 = 0;
    uint8_t r1 = 0;
    uint8_t r2 = 0;
    uint8_t zero = 0;
    uint8_t k;

    __asm__ volatile (
        "ldi %[k], 1 \n\t" /* k = 1; k < num_words * 2; ++k */

        "1: \n\t"

        "movw r26, %[orig] \n\t"  /* copy orig ptr to 'left' ptr */
        "movw r30, %[orig] \n\t"  /* copy orig ptr to 'right' ptr */
        "cp %[k], %[num] \n\t"
        "brlo 2f \n\t"
        "breq 2f \n\t"

        /* when k > num_words, we start from (k - num_words) on the 'left' ptr */
        "add r26, %[k] \n\t"
        "adc r27, %[zero] \n\t"
        "sub r26, %[num] \n\t"
        "sbc r27, %[zero] \n\t"
        "add r30, %[num] \n\t" /* move right ptr to point at the end */
        "adc r31, %[zero] \n\t"
        "rjmp 3f \n\t"

        "2: \n\t" /* when k <= num_words, we add k to the 'right' ptr */
        "add r30, %[k] \n\t" /* pre-add 'right' ptr */
        "adc r31, %[zero] \n\t"

        "3: \n\t"
        "ld r0, x+ \n\t"
        "cp r26, r30 \n\t" /* if left == right here, then we are done after this mult
                              (and we don't need to double) */
        "breq 4f \n\t"
        "ld r1, -z \n\t"
        "mul r0, r1 \n\t"

        /* add twice since it costs the same as doubling */
        "add %[r0], r0 \n\t"
        "adc %[r1], r1 \n\t"
        "adc %[r2], %[zero] \n\t"
        "add %[r0], r0 \n\t"
        "adc %[r1], r1 \n\t"
        "adc %[r2], %[zero] \n\t"

        "cpse r26, r30 \n\t" /* if left == right here, then we are done */
        "rjmp 3b \n\t"
        "rjmp 5f \n\t" /* skip code for non-doubled mult */

        "4: \n\t"
        "ld r1, -z \n\t"
        "mul r0, r1 \n\t"
        "add %[r0], r0 \n\t"
        "adc %[r1], r1 \n\t"
        "adc %[r2], %[zero] \n\t"

        "5: \n\t"
        "movw r30, %[result] \n\t" /* make z point to result */
        "st z+, %[r0] \n\t"        /* Store the result. */
        "movw %[result], r30 \n\t" /* update result ptr*/
        "mov %[r0], %[r1] \n\t"
        "mov %[r1], %[r2] \n\t"
        "mov %[r2], %[zero] \n\t"

        "inc %[k] \n\t"
        "cp %[k], %[max] \n\t"
        "brlo 1b \n\t" /* loop if k < num_words * 2 */

        "st z+, %[r0] \n\t"  /* Store last result byte. */
        "eor r1, r1 \n\t" /* fix r1 to be 0 again */

        : [result] "+r" (result),
          [r0] "+r" (r0), [r1] "+r" (r1), [r2] "+r" (r2), [zero] "+r" (zero),
          [k] "=&a" (k)
        : [orig] "r" (left), [max] "r" ((uint8_t)(2 * num_words)),
          [num] "r" (num_words)
        : "r0", "r26", "r27", "r30", "r31", "cc", "memory"
    );
}
#define asm_square 1
#endif /* uECC_SQUARE_FUNC && !asm_square */


#if (uECC_SAFEGCD == 2)
/* Helpers for the 14-bit safegcd inversion (see uECC_vli_modInv in uECC.c). */

/* acc += a * b, for int32_t acc and int16_t a, b (signed 16x16->32-bit product
   from four 8x8 multiplies). mulsu / muls set C to bit 15 of their product, which
   sign-extends the cross products into the top byte. */
#define S14_MAC(acc, a, b) \
    do { \
        uint8_t zero_; \
        __asm__ ( \
            "clr %1 \n\t"  /* r1 holds product bytes, so use our own zero. */ \
            "mul %A2, %A3 \n\t"  /* lo * lo (unsigned) at byte 0. */ \
            "add %A0, r0 \n\t" \
            "adc %B0, r1 \n\t" \
            "adc %C0, %1 \n\t" \
            "adc %D0, %1 \n\t" \
            "mulsu %B2, %A3 \n\t"  /* hi(a) * lo(b) at byte 1. */ \
            "sbc %D0, %1 \n\t" \
            "add %B0, r0 \n\t" \
            "adc %C0, r1 \n\t" \
            "adc %D0, %1 \n\t" \
            "mulsu %B3, %A2 \n\t"  /* hi(b) * lo(a) at byte 1. */ \
            "sbc %D0, %1 \n\t" \
            "add %B0, r0 \n\t" \
            "adc %C0, r1 \n\t" \
            "adc %D0, %1 \n\t" \
            "muls %B2, %B3 \n\t"  /* hi * hi (signed) at byte 2. */ \
            "add %C0, r0 \n\t" \
            "adc %D0, r1 \n\t" \
            "clr __zero_reg__ \n\t"  /* Restore r1. */ \
            : "+r" (acc), "=&r" (zero_) \
            : "a" ((int16_t)(a)), "a" ((int16_t)(b)) \
            : "r0", "cc"); \
    } while (0)

/* acc >>= 14 (arithmetic), via a 2-bit left shift and a 2-byte move. */
#define S14_SHR(acc) \
    do { \
        uint8_t sign_; \
        __asm__ ( \
            "mov %1, %D0 \n\t"  /* sign_ = 0x00 or 0xFF (sign of acc). */ \
            "lsl %1 \n\t" \
            "sbc %1, %1 \n\t" \
            "lsl %B0 \n\t" \
            "rol %C0 \n\t" \
            "rol %D0 \n\t" \
            "rol %1 \n\t" \
            "lsl %B0 \n\t" \
            "rol %C0 \n\t" \
            "rol %D0 \n\t" \
            "rol %1 \n\t" \
            "mov %A0, %C0 \n\t"  /* Bytes 2..4 of (acc << 2) -> acc. */ \
            "mov %B0, %D0 \n\t" \
            "mov %C0, %1 \n\t" \
            "lsl %1 \n\t"  /* Sign-extend into the top byte. */ \
            "sbc %D0, %D0 \n\t" \
            : "+r" (acc), "=&r" (sign_) \
            : \
            : "cc"); \
    } while (0)
#endif /* uECC_SAFEGCD == 2 */

#endif /* _UECC_ASM_AVR_H_ */
