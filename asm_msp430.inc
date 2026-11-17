/* Copyright 2026, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#ifndef _UECC_ASM_MSP430_H_
#define _UECC_ASM_MSP430_H_

/* Hardware multiplier selection (see uECC_MSP430_HWMULT in uECC.h): 16 uses the MPY16
   peripheral (also present, register-compatible, on MPY32 devices); 0 uses software
   multiplication. By default it is detected from the device header. */
#ifndef uECC_MSP430_HWMULT
    /* Generic CPU builds (-mcpu without -mmcu) have no device header. */
    #if defined(__has_include) && !defined(__MSP430GENERIC__) && \
        !defined(__MSP430I2XXGENERIC__) && !defined(__MSP430i2XXGENERIC__)
        #if __has_include(<msp430.h>)
            #include <msp430.h>
            /* The device headers define short status-register bit names. */
            #undef C
            #undef Z
            #undef N
            #undef V
        #endif
    #endif
    #if defined(__MSP430_HAS_MPY__) || defined(__MSP430_HAS_MPY32__)
        #define uECC_MSP430_HWMULT 16
    #else
        #define uECC_MSP430_HWMULT 0
    #endif
#endif

/* Disable interrupts while the asm uses the hardware multiplier (see uECC.h). */
#ifndef uECC_MSP430_DISABLE_INTERRUPTS
    #define uECC_MSP430_DISABLE_INTERRUPTS 1
#endif

/* Constant-time software multiplication (uECC_MSP430_HWMULT == 0). */

/* One step of the constant-time software 16x16 multiply: with C = the inverted multiplier
   bit, adds the multiplicand a to h if the bit is set, then shifts (C:h:l) right. Starting
   from l = ~b shifted right once (with rra) and h = h0, 16 steps give (h:l) = a * b + h0. */
#define uECC_MSP430_SW_STEP(m, a, h, l) \
    "subc " m ", " m " \n\t" /* m = C - 1: 0xFFFF if the bit is set. */ \
    "and " a ", " m " \n\t"  \
    "add " m ", " h " \n\t"  \
    "rrc " h " \n\t"         \
    "rrc " l " \n\t"         /* C = next inverted multiplier bit. */
#define uECC_MSP430_SW_STEP4(m, a, h, l) \
    uECC_MSP430_SW_STEP(m, a, h, l) uECC_MSP430_SW_STEP(m, a, h, l) \
    uECC_MSP430_SW_STEP(m, a, h, l) uECC_MSP430_SW_STEP(m, a, h, l)
/* (h:l) = a * l + h (l is the multiplier on input). */
#define uECC_MSP430_SW_MUL16(m, a, h, l) \
    "inv " l " \n\t"   \
    "rra " l " \n\t"   \
    uECC_MSP430_SW_STEP4(m, a, h, l) uECC_MSP430_SW_STEP4(m, a, h, l) \
    uECC_MSP430_SW_STEP4(m, a, h, l) uECC_MSP430_SW_STEP4(m, a, h, l)

#if (uECC_MSP430_HWMULT == 0)
/* Returns a * b. The C code's word multiplications use this instead of libgcc's software
   multiply, whose time depends on the operands. (With the hardware multiplier, the libgcc
   helpers are constant time.) */
static inline uint32_t uECC_msp430_mul16(uint16_t a, uint16_t b) {
    uint16_t hi, m, lo = b;
    __asm__ (
        "clr %0 \n\t"
        uECC_MSP430_SW_MUL16("%1", "%3", "%0", "%2")
        : "=&r" (hi), "=&r" (m), "+r" (lo)
        : "r" (a)
        : "cc"
    );
    return ((uint32_t)hi << 16) | lo;
}
#define uECC_MUL_WORD(a, b) uECC_msp430_mul16((a), (b))
#define uECC_MUL_WORD_LO(a, b) ((uECC_word_t)uECC_msp430_mul16((a), (b)))
#define S14_MUL_LO(a, b) ((uint16_t)uECC_msp430_mul16((a), (b)))
#endif

#if (uECC_OPTIMIZATION_LEVEL >= 1)

/* MPY16 registers (the same addresses on every classic device with MPY or MPY32). */
#define uECC_MSP430_MPY    "&0x0130"
#define uECC_MSP430_MAC    "&0x0134"
#define uECC_MSP430_OP2    "&0x0138"
#define uECC_MSP430_RESLO  "&0x013A"
#define uECC_MSP430_RESHI  "&0x013C"
#define uECC_MSP430_SUMEXT "&0x013E"

/* Brackets for asm that uses the hardware multiplier: save SR and disable interrupts (the
   nop covers the instruction after dint, which can still be interrupted), then restore SR,
   which returns GIE to its previous state. */
#if (uECC_MSP430_HWMULT == 16) && uECC_MSP430_DISABLE_INTERRUPTS
    #define uECC_MSP430_DINT "push r2 \n\t" "dint \n\t" "nop \n\t"
    #define uECC_MSP430_RINT "pop r2 \n\t"
#else
    #define uECC_MSP430_DINT
    #define uECC_MSP430_RINT
#endif

/* ---- Looped implementations ----
   Add/sub/rshift are only used at level 1. Multiply/square are used at levels 1 and 2,
   and at level 3 and up as the fallback for sizes without unrolled code (which only the
   uECC_ENABLE_VLI_API functions use, e.g. 11 words for the secp160r1 order). */

#if (uECC_OPTIMIZATION_LEVEL >= 3) && (uECC_MSP430_HWMULT == 16)
    #define uECC_MSP430_LOOPED(name) static void vli_##name##_loop
    #define uECC_MSP430_LOOPED_SQUARE uECC_ENABLE_VLI_API
#else
    #define uECC_MSP430_LOOPED(name) uECC_VLI_API void uECC_vli_##name
    #define uECC_MSP430_LOOPED_SQUARE 1
#endif
#if (uECC_OPTIMIZATION_LEVEL >= 3)
    #define uECC_MSP430_LOOPED_MULT uECC_ENABLE_VLI_API
    #define uECC_MSP430_LOOPED_MULT_NAME static void vli_mult_loop
#else
    #define uECC_MSP430_LOOPED_MULT 1
    #define uECC_MSP430_LOOPED_MULT_NAME uECC_VLI_API void uECC_vli_mult
#endif

#if (uECC_OPTIMIZATION_LEVEL < 2)
/* Computes result = left + right, returning carry. Can modify in place. The carry is
   kept in r11 across the loop control. */
__attribute__((noinline))
uECC_VLI_API uECC_word_t uECC_vli_add(uECC_word_t *result,
                                      const uECC_word_t *left,
                                      const uECC_word_t *right,
                                      wordcount_t num_words) {
    register uint16_t r12 __asm__("r12") = (uint16_t)result;
    register const uECC_word_t *r13 __asm__("r13") = left;
    register const uECC_word_t *r14 __asm__("r14") = right;
    register uint16_t r15 __asm__("r15") = (uint16_t)num_words;
    __asm__ volatile (
        "clr r11 \n\t"
        "1: \n\t"
        "rra r11 \n\t"          /* C = carry, r11 = 0. */
        "mov @r13+, r10 \n\t"
        "addc @r14+, r10 \n\t"
        "mov r10, 0(r12) \n\t"
        "adc r11 \n\t"          /* Save carry. */
        "incd r12 \n\t"
        "dec r15 \n\t"
        "jnz 1b \n\t"
        "mov r11, r12 \n\t"
        : "+r" (r12), "+r" (r13), "+r" (r14), "+r" (r15)
        :
        : "r10", "r11", "cc", "memory"
    );
    return r12;
}
#define asm_add 1

/* Computes result = left - right, returning borrow. Can modify in place. */
__attribute__((noinline))
uECC_VLI_API uECC_word_t uECC_vli_sub(uECC_word_t *result,
                                      const uECC_word_t *left,
                                      const uECC_word_t *right,
                                      wordcount_t num_words) {
    register uint16_t r12 __asm__("r12") = (uint16_t)result;
    register const uECC_word_t *r13 __asm__("r13") = left;
    register const uECC_word_t *r14 __asm__("r14") = right;
    register uint16_t r15 __asm__("r15") = (uint16_t)num_words;
    __asm__ volatile (
        "mov #1, r11 \n\t"      /* r11 = C (1 = no borrow). */
        "1: \n\t"
        "rra r11 \n\t"
        "mov @r13+, r10 \n\t"
        "subc @r14+, r10 \n\t"
        "mov r10, 0(r12) \n\t"
        "adc r11 \n\t"
        "incd r12 \n\t"
        "dec r15 \n\t"
        "jnz 1b \n\t"
        "mov #1, r12 \n\t"
        "xor r11, r12 \n\t"     /* borrow = !C */
        : "+r" (r12), "+r" (r13), "+r" (r14), "+r" (r15)
        :
        : "r10", "r11", "cc", "memory"
    );
    return r12;
}
#define asm_sub 1

/* Computes vli = vli >> 1. From the bottom: each word is rotated right with C = the low
   bit of the next word (bit sets C when the result is nonzero), so no carry is kept
   across the loop control. */
__attribute__((noinline))
uECC_VLI_API void uECC_vli_rshift1(uECC_word_t *vli, wordcount_t num_words) {
    register uECC_word_t *r12 __asm__("r12") = vli;
    register uint16_t r13 __asm__("r13") = (uint16_t)num_words;
    __asm__ volatile (
        "jmp 2f \n\t"
        "1: \n\t"
        "bit #1, 2(r12) \n\t"
        "rrc @r12+ \n\t"
        "2: \n\t"
        "dec r13 \n\t"
        "jnz 1b \n\t"
        "clrc \n\t"
        "rrc @r12 \n\t"
        : "+r" (r12), "+r" (r13)
        :
        : "cc", "memory"
    );
}
#define asm_rshift1 1
#endif /* (uECC_OPTIMIZATION_LEVEL < 2) */

#if (uECC_MSP430_HWMULT == 16)

#if uECC_MSP430_LOOPED_MULT
/* Computes result = left * right. Product scanning: column k accumulates the products
   left[i] * right[k - i] in RESHI:RESLO with MAC, and the carries (SUMEXT) in r11; then
   RESLO is stored and the window shifts down one word. r13/r8 are the first left/right
   words of the column, r14 its number of products. */
__attribute__((noinline))
uECC_MSP430_LOOPED_MULT_NAME(uECC_word_t *result,
                             const uECC_word_t *left,
                             const uECC_word_t *right,
                             wordcount_t num_words) {
    register uECC_word_t *r12 __asm__("r12") = result;
    register const uECC_word_t *r13 __asm__("r13") = left;
    register const uECC_word_t *r14 __asm__("r14") = right;
    register uint16_t r15 __asm__("r15") = (uint16_t)num_words;
    __asm__ volatile (
        uECC_MSP430_DINT
        "mov r15, r9 \n\t"
        "rla r9 \n\t"
        "mov r9, r10 \n\t"
        "dec r10 \n\t"          /* r10 = 2n - 1 columns */
        "add r14, r9 \n\t"
        "decd r9 \n\t"          /* r9 = &right[n - 1] */
        "mov r14, r8 \n\t"
        "mov #1, r14 \n\t"
        "clr r11 \n\t"
        "mov #0x013E, r5 \n\t"   /* SUMEXT */
        "mov #0, " uECC_MSP430_RESLO " \n\t"
        "mov #0, " uECC_MSP430_RESHI " \n\t"
        "1: \n\t"
        "mov r13, r7 \n\t"
        "mov r8, r6 \n\t"
        "mov r14, r15 \n\t"
        "2: \n\t"
        "mov @r7+, " uECC_MSP430_MAC " \n\t"
        "mov @r6, " uECC_MSP430_OP2 " \n\t"
        "decd r6 \n\t"        /* (not @r5 right after the OP2 write) */
        "add @r5, r11 \n\t"
        "dec r15 \n\t"
        "jnz 2b \n\t"
        "mov " uECC_MSP430_RESLO ", 0(r12) \n\t"
        "incd r12 \n\t"
        "mov " uECC_MSP430_RESHI ", " uECC_MSP430_RESLO " \n\t"
        "mov r11, " uECC_MSP430_RESHI " \n\t"
        "clr r11 \n\t"
        /* Next column: one more product until right[n - 1] is reached, then one fewer. */
        "cmp r9, r8 \n\t"
        "jeq 3f \n\t"
        "incd r8 \n\t"
        "inc r14 \n\t"
        "jmp 4f \n\t"
        "3: \n\t"
        "incd r13 \n\t"
        "dec r14 \n\t"
        "4: \n\t"
        "dec r10 \n\t"
        "jnz 1b \n\t"
        "mov " uECC_MSP430_RESLO ", 0(r12) \n\t"
        uECC_MSP430_RINT
        : "+r" (r12), "+r" (r13), "+r" (r14), "+r" (r15)
        :
        : "r5", "r6", "r7", "r8", "r9", "r10", "r11", "cc", "memory"
    );
}
#endif /* uECC_MSP430_LOOPED_MULT */

#if uECC_SQUARE_FUNC && uECC_MSP430_LOOPED_SQUARE
/* Computes result = left^2. Product scanning as in uECC_vli_mult; each cross product
   left[i] * left[j] (i < j) is accumulated twice by writing OP2 twice. */
__attribute__((noinline))
uECC_MSP430_LOOPED(square)(uECC_word_t *result,
                           const uECC_word_t *left,
                           wordcount_t num_words) {
    register uECC_word_t *r12 __asm__("r12") = result;
    register const uECC_word_t *r13 __asm__("r13") = left;
    register uint16_t r14 __asm__("r14") = (uint16_t)num_words;
    __asm__ volatile (
        uECC_MSP430_DINT
        "mov r14, r9 \n\t"
        "rla r9 \n\t"
        "mov r9, r10 \n\t"
        "dec r10 \n\t"          /* r10 = 2n - 1 columns */
        "add r13, r9 \n\t"
        "decd r9 \n\t"          /* r9 = &left[n - 1] */
        "mov r13, r8 \n\t"      /* r13/r8 = lowest/highest word of the column */
        "clr r11 \n\t"
        "mov #0x013E, r5 \n\t"   /* SUMEXT */
        "mov #0, " uECC_MSP430_RESLO " \n\t"
        "mov #0, " uECC_MSP430_RESHI " \n\t"
        "1: \n\t"
        "mov r13, r7 \n\t"
        "mov r8, r6 \n\t"
        "cmp r6, r7 \n\t"
        "jhs 3f \n\t"
        "2: \n\t"               /* Cross products, while r7 < r6. */
        "mov @r7+, " uECC_MSP430_MAC " \n\t"
        "mov @r6, r15 \n\t"
        "mov r15, " uECC_MSP430_OP2 " \n\t"
        "decd r6 \n\t"
        "add @r5, r11 \n\t"
        "mov r15, " uECC_MSP430_OP2 " \n\t"
        "add " uECC_MSP430_SUMEXT ", r11 \n\t"
        "cmp r6, r7 \n\t"
        "jlo 2b \n\t"
        "3: \n\t"
        "jne 4f \n\t"           /* Even column: add the square term. */
        "mov @r7, " uECC_MSP430_MAC " \n\t"
        "mov @r7, " uECC_MSP430_OP2 " \n\t"
        "add " uECC_MSP430_SUMEXT ", r11 \n\t"
        "4: \n\t"
        "mov " uECC_MSP430_RESLO ", 0(r12) \n\t"
        "incd r12 \n\t"
        "mov " uECC_MSP430_RESHI ", " uECC_MSP430_RESLO " \n\t"
        "mov r11, " uECC_MSP430_RESHI " \n\t"
        "clr r11 \n\t"
        "cmp r9, r8 \n\t"
        "jeq 5f \n\t"
        "incd r8 \n\t"
        "jmp 6f \n\t"
        "5: \n\t"
        "incd r13 \n\t"
        "6: \n\t"
        "dec r10 \n\t"
        "jnz 1b \n\t"
        "mov " uECC_MSP430_RESLO ", 0(r12) \n\t"
        uECC_MSP430_RINT
        : "+r" (r12), "+r" (r13), "+r" (r14)
        :
        : "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r15", "cc", "memory"
    );
}
#endif /* uECC_SQUARE_FUNC && uECC_MSP430_LOOPED_SQUARE */

#else /* uECC_MSP430_HWMULT == 0 */

#if uECC_MSP430_LOOPED_MULT
/* Computes result = left * right with software 16x16 multiplies. Operand scanning: row i
   adds left[i] * right to result[i..i+n]; the row carry stays in the high product word r8,
   which is also the addend h0 of the next multiply. The row count is kept on the stack. */
__attribute__((noinline))
uECC_MSP430_LOOPED_MULT_NAME(uECC_word_t *result,
                             const uECC_word_t *left,
                             const uECC_word_t *right,
                             wordcount_t num_words) {
    register uECC_word_t *r12 __asm__("r12") = result;
    register const uECC_word_t *r13 __asm__("r13") = left;
    register const uECC_word_t *r14 __asm__("r14") = right;
    register uint16_t r15 __asm__("r15") = (uint16_t)num_words;
    __asm__ volatile (
        "push r15 \n\t"         /* row count */
        "mov r12, r11 \n\t"     /* Clear result[0..n-1]. */
        "mov r15, r10 \n\t"
        "1: \n\t"
        "mov #0, 0(r11) \n\t"
        "incd r11 \n\t"
        "dec r10 \n\t"
        "jnz 1b \n\t"
        "rla r15 \n\t"
        "add r14, r15 \n\t"     /* r15 = end of right */
        "2: \n\t"
        "mov @r13+, r6 \n\t"    /* a = left[i] */
        "mov r14, r10 \n\t"
        "mov r12, r11 \n\t"
        "clr r8 \n\t"
        "3: \n\t"
        "mov @r10+, r7 \n\t"
        uECC_MSP430_SW_MUL16("r9", "r6", "r8", "r7")
        "add 0(r11), r7 \n\t"
        "adc r8 \n\t"
        "mov r7, 0(r11) \n\t"
        "incd r11 \n\t"
        "cmp r15, r10 \n\t"
        "jne 3b \n\t"
        "mov r8, 0(r11) \n\t"
        "incd r12 \n\t"
        "dec 0(r1) \n\t"
        "jnz 2b \n\t"
        "incd r1 \n\t"
        : "+r" (r12), "+r" (r13), "+r" (r14), "+r" (r15)
        :
        : "r6", "r7", "r8", "r9", "r10", "r11", "cc", "memory"
    );
}
#endif /* uECC_MSP430_LOOPED_MULT */

#if uECC_SQUARE_FUNC
/* Computes result = left^2 with software multiplies: the cross products left[i] * left[j]
   (i < j) by operand scanning, then result = 2 * result + the square terms, with the
   doubling carry in r12 and the addition carry in r14. */
__attribute__((noinline))
uECC_VLI_API void uECC_vli_square(uECC_word_t *result,
                                  const uECC_word_t *left,
                                  wordcount_t num_words) {
    register uECC_word_t *r12 __asm__("r12") = result;
    register const uECC_word_t *r13 __asm__("r13") = left;
    register uint16_t r14 __asm__("r14") = (uint16_t)num_words;
    __asm__ volatile (
        "mov r12, r11 \n\t"     /* Clear result. */
        "mov r14, r10 \n\t"
        "rla r10 \n\t"
        "1: \n\t"
        "mov #0, 0(r11) \n\t"
        "incd r11 \n\t"
        "dec r10 \n\t"
        "jnz 1b \n\t"
        "mov r14, r15 \n\t"
        "rla r15 \n\t"
        "add r13, r15 \n\t"     /* r15 = end of left */
        "push r13 \n\t"
        "push r12 \n\t"
        "incd r12 \n\t"         /* row i starts at result[2i + 1] */
        "mov r13, r9 \n\t"
        "incd r9 \n\t"
        "cmp r15, r9 \n\t"
        "jeq 4f \n\t"           /* n == 1: no cross products */
        "2: \n\t"
        "mov @r13+, r6 \n\t"    /* a = left[i] */
        "mov r13, r10 \n\t"
        "mov r12, r11 \n\t"
        "clr r8 \n\t"
        "3: \n\t"
        "mov @r10+, r7 \n\t"
        uECC_MSP430_SW_MUL16("r9", "r6", "r8", "r7")
        "add 0(r11), r7 \n\t"
        "adc r8 \n\t"
        "mov r7, 0(r11) \n\t"
        "incd r11 \n\t"
        "cmp r15, r10 \n\t"
        "jne 3b \n\t"
        "mov r8, 0(r11) \n\t"
        "add #4, r12 \n\t"
        "mov r13, r9 \n\t"
        "incd r9 \n\t"
        "cmp r15, r9 \n\t"
        "jne 2b \n\t"
        "4: \n\t"
        "pop r11 \n\t"          /* result */
        "pop r13 \n\t"          /* left */
        "clr r12 \n\t"
        "clr r14 \n\t"
        "5: \n\t"
        "mov @r13+, r6 \n\t"
        "mov r6, r7 \n\t"
        "clr r8 \n\t"
        uECC_MSP430_SW_MUL16("r9", "r6", "r8", "r7")
        "mov 0(r11), r10 \n\t"
        "mov 2(r11), r5 \n\t"
        "rra r12 \n\t"          /* Double, with the doubling carry. */
        "rlc r10 \n\t"
        "rlc r5 \n\t"
        "adc r12 \n\t"
        "rra r14 \n\t"          /* Add the square, with the addition carry. */
        "addc r7, r10 \n\t"
        "addc r8, r5 \n\t"
        "adc r14 \n\t"
        "mov r10, 0(r11) \n\t"
        "mov r5, 2(r11) \n\t"
        "add #4, r11 \n\t"
        "cmp r15, r13 \n\t"
        "jne 5b \n\t"
        : "+r" (r12), "+r" (r13), "+r" (r14)
        :
        : "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r15", "cc", "memory"
    );
}
#define asm_square 1
#endif /* uECC_SQUARE_FUNC */

#endif /* uECC_MSP430_HWMULT */

#if (uECC_OPTIMIZATION_LEVEL < 3)
    #define asm_mult 1
    #if uECC_SQUARE_FUNC
        #define asm_square 1
    #endif
#endif


/* ---- Unrolled add/sub/rshift (level 2 and up) ---- */

#if (uECC_OPTIMIZATION_LEVEL >= 2)

#if (uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1)
    #define uECC_MIN_WORDS 16
#endif
#if uECC_SUPPORTS_secp224r1
    #undef uECC_MIN_WORDS
    #define uECC_MIN_WORDS 14
#endif
#if uECC_SUPPORTS_secp192r1
    #undef uECC_MIN_WORDS
    #define uECC_MIN_WORDS 12
#endif
#if uECC_SUPPORTS_secp160r1
    #undef uECC_MIN_WORDS
    #define uECC_MIN_WORDS 10
#endif

/* The chains below have uECC_MAX_WORDS elements. With several sizes (always with
   secp160r1, whose order n has 11 words), they are entered at the element for num_words
   by a computed branch. */
#define uECC_MSP430_STR2(a) #a
#define uECC_MSP430_STR(a) uECC_MSP430_STR2(a)
#if (uECC_MIN_WORDS == uECC_MAX_WORDS)
    #define uECC_MSP430_ONE_SIZE 1
#else
    #define uECC_MSP430_ONE_SIZE 0
#endif

/* result[i] = left[i] op right[i] for every word: 8 bytes, 8 cycles per element. The
   result offsets are fixed, so for num_words < uECC_MAX_WORDS r12 is moved down by
   2 * (uECC_MAX_WORDS - num_words). */
#define uECC_MSP430_ADDSUB_CHAIN(op) \
    ".set .Luecc_off, 0 \n\t" \
    ".rept " uECC_MSP430_STR(uECC_MAX_WORDS) " \n\t" \
    "mov @r13+, r11 \n\t" \
    op " @r14+, r11 \n\t" \
    "mov r11, .Luecc_off(r12) \n\t" \
    ".set .Luecc_off, .Luecc_off + 2 \n\t" \
    ".endr \n\t"

#if uECC_MSP430_ONE_SIZE
    #define uECC_MSP430_ADDSUB_ENTER(flag) flag " \n\t"
#else
    /* r12 -= 2 * (MAX - num_words), then branch to element MAX - num_words. */
    #define uECC_MSP430_ADDSUB_ENTER(flag) \
        "mov #" uECC_MSP430_STR(uECC_MAX_WORDS) ", r11 \n\t" \
        "sub r15, r11 \n\t" \
        "rla r11 \n\t" \
        "sub r11, r12 \n\t" \
        "rla r11 \n\t" \
        "rla r11 \n\t" \
        "add #2f, r11 \n\t" \
        flag " \n\t" \
        "br r11 \n\t" \
        "2: \n\t"
#endif

/* Computes result = left + right, returning carry. Can modify in place. */
__attribute__((noinline))
uECC_VLI_API uECC_word_t uECC_vli_add(uECC_word_t *result,
                                      const uECC_word_t *left,
                                      const uECC_word_t *right,
                                      wordcount_t num_words) {
    register uint16_t r12 __asm__("r12") = (uint16_t)result;
    register const uECC_word_t *r13 __asm__("r13") = left;
    register const uECC_word_t *r14 __asm__("r14") = right;
    register uint16_t r15 __asm__("r15") = (uint16_t)num_words;
    __asm__ volatile (
        uECC_MSP430_ADDSUB_ENTER("clrc")
        uECC_MSP430_ADDSUB_CHAIN("addc")
        "mov #0, r12 \n\t"
        "adc r12 \n\t"
        : "+r" (r12), "+r" (r13), "+r" (r14), "+r" (r15)
        :
        : "r11", "cc", "memory"
    );
    return r12;
}
#define asm_add 1

/* Computes result = left - right, returning borrow. Can modify in place. */
__attribute__((noinline))
uECC_VLI_API uECC_word_t uECC_vli_sub(uECC_word_t *result,
                                      const uECC_word_t *left,
                                      const uECC_word_t *right,
                                      wordcount_t num_words) {
    register uint16_t r12 __asm__("r12") = (uint16_t)result;
    register const uECC_word_t *r13 __asm__("r13") = left;
    register const uECC_word_t *r14 __asm__("r14") = right;
    register uint16_t r15 __asm__("r15") = (uint16_t)num_words;
    __asm__ volatile (
        uECC_MSP430_ADDSUB_ENTER("setc")    /* C = 1: no borrow */
        uECC_MSP430_ADDSUB_CHAIN("subc")
        "subc r12, r12 \n\t"                /* r12 = C - 1 */
        "and #1, r12 \n\t"                  /* borrow = !C */
        : "+r" (r12), "+r" (r13), "+r" (r14), "+r" (r15)
        :
        : "r11", "cc", "memory"
    );
    return r12;
}
#define asm_sub 1

/* Computes vli = vli >> 1, from the top word down (rrc of each word in memory, 4 cycles
   per element). The chain is entered at the element for word num_words - 1 (all elements
   are 4 bytes except the last, rrc @r12). */
__attribute__((noinline))
uECC_VLI_API void uECC_vli_rshift1(uECC_word_t *vli, wordcount_t num_words) {
    register uECC_word_t *r12 __asm__("r12") = vli;
    register uint16_t r13 __asm__("r13") = (uint16_t)num_words;
    __asm__ volatile (
    #if !uECC_MSP430_ONE_SIZE
        "rla r13 \n\t"
        "rla r13 \n\t"
        "sub #3f + 4 * " uECC_MSP430_STR(uECC_MAX_WORDS) ", r13 \n\t"
        "inv r13 \n\t"
        "inc r13 \n\t"    /* r13 = 3: + 4 * (MAX - num_words) */
    #endif
        "clrc \n\t"
    #if !uECC_MSP430_ONE_SIZE
        "br r13 \n\t"
    #endif
        "3: \n\t"
        ".set .Luecc_off, " uECC_MSP430_STR(uECC_MAX_WORDS) " * 2 - 2 \n\t"
        ".rept " uECC_MSP430_STR(uECC_MAX_WORDS) " \n\t"
        "rrc .Luecc_off(r12) \n\t"
        ".set .Luecc_off, .Luecc_off - 2 \n\t"
        ".endr \n\t"
        "2: \n\t"
        : "+r" (r12), "+r" (r13)
        :
        : "cc", "memory"
    );
}
#define asm_rshift1 1

#endif /* (uECC_OPTIMIZATION_LEVEL >= 2) */

/* ---- Unrolled multiplication and squaring (level 3 and up) ---- */

#if (uECC_OPTIMIZATION_LEVEL >= 3)

/* Hardware multiplier: product scanning, unrolled for each curve size.
   Software multiplication: Karatsuba down to 2..4-word schoolbook leaves (see
   asm_msp430_mult_square.inc); squaring keeps the looped symmetric version. */
#include "asm_msp430_mult_square.inc"

#if (uECC_MSP430_HWMULT == 16)
    #define uECC_MSP430_MULT(N) uECC_MSP430_MULT_##N
    #define uECC_MSP430_SQUARE(N) uECC_MSP430_SQUARE_##N
#elif (uECC_OPTIMIZATION_LEVEL >= 4)
    #define uECC_MSP430_MULT(N) uECC_MSP430_SWMULT_UNROLLED_##N
#else
    #define uECC_MSP430_MULT(N) uECC_MSP430_SWMULT_##N
#endif

/* r4 is saved by hand, since GCC may use it as the frame pointer. */
#define uECC_MSP430_MULT_FN(N) \
__attribute__((noinline)) \
static void vli_mult_##N(uECC_word_t *result, const uECC_word_t *left, \
                         const uECC_word_t *right) { \
    register uECC_word_t *r12 __asm__("r12") = result; \
    register const uECC_word_t *r13 __asm__("r13") = left; \
    register const uECC_word_t *r14 __asm__("r14") = right; \
    __asm__ volatile ( \
        "push r4 \n\t" \
        uECC_MSP430_DINT \
        uECC_MSP430_MULT(N) \
        uECC_MSP430_RINT \
        "pop r4 \n\t" \
        : "+r" (r12), "+r" (r13), "+r" (r14) \
        : \
        : "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r15", "cc", "memory" \
    ); \
}

#define uECC_MSP430_SQUARE_FN(N) \
__attribute__((noinline)) \
static void vli_square_##N(uECC_word_t *result, const uECC_word_t *left) { \
    register uECC_word_t *r12 __asm__("r12") = result; \
    register const uECC_word_t *r13 __asm__("r13") = left; \
    __asm__ volatile ( \
        "push r4 \n\t" \
        uECC_MSP430_DINT \
        uECC_MSP430_SQUARE(N) \
        uECC_MSP430_RINT \
        "pop r4 \n\t" \
        : "+r" (r12), "+r" (r13) \
        : \
        : "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r14", "r15", "cc", "memory" \
    ); \
}

#if uECC_SUPPORTS_secp160r1
uECC_MSP430_MULT_FN(10)
#endif
#if uECC_SUPPORTS_secp192r1
uECC_MSP430_MULT_FN(12)
#endif
#if uECC_SUPPORTS_secp224r1
uECC_MSP430_MULT_FN(14)
#endif
#if (uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1)
uECC_MSP430_MULT_FN(16)
#endif

/* Computes result = left * right, with the fixed-size code for num_words. */
uECC_VLI_API void uECC_vli_mult(uECC_word_t *result,
                                const uECC_word_t *left,
                                const uECC_word_t *right,
                                wordcount_t num_words) {
#if uECC_SUPPORTS_secp160r1
    if (num_words == 10) {
        vli_mult_10(result, left, right);
        return;
    }
#endif
#if uECC_SUPPORTS_secp192r1
    if (num_words == 12) {
        vli_mult_12(result, left, right);
        return;
    }
#endif
#if uECC_SUPPORTS_secp224r1
    if (num_words == 14) {
        vli_mult_14(result, left, right);
        return;
    }
#endif
#if (uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1)
    if (num_words == 16) {
        vli_mult_16(result, left, right);
        return;
    }
#endif
#if uECC_ENABLE_VLI_API
    vli_mult_loop(result, left, right, num_words);
#endif
}
#define asm_mult 1

#if uECC_SQUARE_FUNC && (uECC_MSP430_HWMULT == 16)
#if uECC_SUPPORTS_secp160r1
uECC_MSP430_SQUARE_FN(10)
#endif
#if uECC_SUPPORTS_secp192r1
uECC_MSP430_SQUARE_FN(12)
#endif
#if uECC_SUPPORTS_secp224r1
uECC_MSP430_SQUARE_FN(14)
#endif
#if (uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1)
uECC_MSP430_SQUARE_FN(16)
#endif

/* Computes result = left^2, with the fixed-size code for num_words. */
uECC_VLI_API void uECC_vli_square(uECC_word_t *result,
                                  const uECC_word_t *left,
                                  wordcount_t num_words) {
#if uECC_SUPPORTS_secp160r1
    if (num_words == 10) {
        vli_square_10(result, left);
        return;
    }
#endif
#if uECC_SUPPORTS_secp192r1
    if (num_words == 12) {
        vli_square_12(result, left);
        return;
    }
#endif
#if uECC_SUPPORTS_secp224r1
    if (num_words == 14) {
        vli_square_14(result, left);
        return;
    }
#endif
#if (uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1)
    if (num_words == 16) {
        vli_square_16(result, left);
        return;
    }
#endif
#if uECC_ENABLE_VLI_API
    vli_square_loop(result, left, num_words);
#endif
}
#define asm_square 1
#endif /* uECC_SQUARE_FUNC && (uECC_MSP430_HWMULT == 16) */

#endif /* (uECC_OPTIMIZATION_LEVEL >= 3) */

/* ---- safegcd helpers (from msp_mmod; used from level 1) ---- */

/* S14_MAC / S14_SHR for the safegcd inversion (uECC_SAFEGCD == 2; see uECC.c):
   the C versions call slow libgcc helpers (and, without a multiplier, a
   variable-time software multiply). */
#if (uECC_SAFEGCD == 2) && !defined(S14_MAC)
/* acc >>= 14 (arithmetic, exact): (sign:hi:lo) << 2, then take the top two words. */
#define S14_SHR(acc) \
    do { \
        uint16_t lo_ = (uint16_t)(acc), hi_ = (uint16_t)((uint32_t)(acc) >> 16), s_; \
        __asm__ ( \
            "mov %1, %2 \n\t" \
            "rla %2 \n\t" \
            "subc %2, %2 \n\t" \
            "inv %2 \n\t"          /* s_ = sign of acc */ \
            "rla %0 \n\t" \
            "rlc %1 \n\t" \
            "rlc %2 \n\t" \
            "rla %0 \n\t" \
            "rlc %1 \n\t" \
            "rlc %2 \n\t" \
            : "+r" (lo_), "+r" (hi_), "=&r" (s_) \
            : \
            : "cc"); \
        (acc) = (int32_t)(((uint32_t)s_ << 16) | hi_); \
    } while (0)

#if (uECC_MSP430_HWMULT == 16)
#if uECC_MSP430_DISABLE_INTERRUPTS
    #define S14_GIE_OFF "push r2 \n\t dint \n\t nop \n\t"
    #define S14_GIE_ON "pop r2 \n\t nop \n\t"
#else
    #define S14_GIE_OFF
    #define S14_GIE_ON
#endif
/* acc += a * b (signed 16x16 -> 32 bits) with the MPYS multiplier. */
#define S14_MAC(acc, a, b) \
    do { \
        uint16_t lo_ = (uint16_t)(acc), hi_ = (uint16_t)((uint32_t)(acc) >> 16); \
        __asm__ volatile ( \
            S14_GIE_OFF \
            "mov %2, &0x0132 \n\t" \
            "mov %3, &0x0138 \n\t" \
            "add &0x013A, %0 \n\t" \
            "addc &0x013C, %1 \n\t" \
            S14_GIE_ON \
            : "+r" (lo_), "+r" (hi_) \
            : "r" ((int16_t)(a)), "r" ((int16_t)(b)) \
            : "cc"); \
        (acc) = (int32_t)(((uint32_t)hi_ << 16) | lo_); \
    } while (0)
#else
/* a * b (signed 16x16 -> 32 bits) in software, constant time: for each bit i
   of b, adds (a << i) & -bit to the product; bit 15 has weight -2^15. */
__attribute__((noinline))
static int32_t s14_mul_msp430(int16_t a, int16_t b) {
    register uint16_t r12 __asm__("r12") = (uint16_t)a;
    register uint16_t r13 __asm__("r13") = (uint16_t)b;
    register uint16_t r14 __asm__("r14");
    register uint16_t r15 __asm__("r15");
    __asm__ (
        "push r9 \n\t"
        "push r10 \n\t"
        /* r11 = sign of a; product = 0. */
        "mov r12, r11 \n\t"
        "rla r11 \n\t"
        "subc r11, r11 \n\t"
        "inv r11 \n\t"
        "mov #0, r14 \n\t"
        "mov #0, r15 \n\t"
        /* Bit 0: product += (a << 0) & -(b bit 0). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 1: product += (a << 1) & -(b bit 1). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 2: product += (a << 2) & -(b bit 2). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 3: product += (a << 3) & -(b bit 3). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 4: product += (a << 4) & -(b bit 4). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 5: product += (a << 5) & -(b bit 5). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 6: product += (a << 6) & -(b bit 6). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 7: product += (a << 7) & -(b bit 7). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 8: product += (a << 8) & -(b bit 8). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 9: product += (a << 9) & -(b bit 9). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 10: product += (a << 10) & -(b bit 10). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 11: product += (a << 11) & -(b bit 11). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 12: product += (a << 12) & -(b bit 12). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 13: product += (a << 13) & -(b bit 13). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 14: product += (a << 14) & -(b bit 14). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "add r9, r14 \n\t"
        "addc r10, r15 \n\t"
        "rla r12 \n\t"
        "rlc r11 \n\t"
        /* Bit 15: product -= (a << 15) & -(b bit 15). */
        "rra r13 \n\t"
        "subc r10, r10 \n\t"
        "inv r10 \n\t"
        "mov r10, r9 \n\t"
        "and r12, r9 \n\t"
        "and r11, r10 \n\t"
        "sub r9, r14 \n\t"
        "subc r10, r15 \n\t"
        "pop r10 \n\t"
        "pop r9 \n\t"
        : "+r" (r12), "+r" (r13), "=&r" (r14), "=&r" (r15)
        :
        : "r11", "cc"
    );
    return (int32_t)(((uint32_t)r15 << 16) | r14);
}
#define S14_MAC(acc, a, b) ((acc) += s14_mul_msp430((int16_t)(a), (int16_t)(b)))
#endif
#endif /* (uECC_SAFEGCD == 2) && !defined(S14_MAC) */

#endif /* (uECC_OPTIMIZATION_LEVEL >= 1) */

#if (uECC_OPTIMIZATION_LEVEL >= 2)
    #include "asm_msp430_mmod.inc"
#endif

#endif /* _UECC_ASM_MSP430_H_ */
