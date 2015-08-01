/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#ifndef _UECC_ASM_ARM_FAST_H_
#define _UECC_ASM_ARM_FAST_H_

#if (uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1)
    #define uECC_MIN_WORDS 8
#endif
#if uECC_SUPPORTS_secp224r1
    #undef uECC_MIN_WORDS
    #define uECC_MIN_WORDS 7
#endif
#if uECC_SUPPORTS_secp192r1
    #undef uECC_MIN_WORDS
    #define uECC_MIN_WORDS 6
#endif
#if uECC_SUPPORTS_secp160r1
    #undef uECC_MIN_WORDS
    #define uECC_MIN_WORDS 5
#endif

#if (uECC_PLATFORM == uECC_arm_thumb)
    #define REG_RW "+l"
    #define REG_WRITE "=l"
#else
    #define REG_RW "+r"
    #define REG_WRITE "=r"
#endif

#if (uECC_PLATFORM == uECC_arm_thumb2)
    #define RESUME_SYNTAX
#else
    #define RESUME_SYNTAX ".syntax divided \n\t"
#endif

static uECC_word_t vli_add(uECC_word_t *result,
                           const uECC_word_t *left,
                           const uECC_word_t *right,
                           wordcount_t num_words) {
#if (uECC_PLATFORM == uECC_arm_thumb) || (uECC_PLATFORM == uECC_arm_thumb2)
    uint32_t jump = ((uECC_MAX_WORDS - num_words) * 4 + 5) * 2 + 1;
#else /* ARM */
    uint32_t jump = ((uECC_MAX_WORDS - num_words) * 4 + 5) * 4;
#endif
    uint32_t carry;
    uint32_t left_word;
    uint32_t right_word;
    
    __asm__ volatile (
        ".syntax unified \n\t"
        "movs %[carry], #0 \n\t"
        "mov %[left], pc \n\t"
        "adds %[jump], %[left] \n\t"
        
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "adds %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
        
        "bx %[jump] \n\t"

        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "adcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
        
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "adcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
        
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "adcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
        
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "adcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
        
    #if (uECC_MAX_WORDS >= 6)
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "adcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
    #endif
    #if (uECC_MAX_WORDS >= 7)
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "adcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
    #endif
    #if (uECC_MAX_WORDS >= 8)
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "adcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
    #endif
        "adcs %[carry], %[carry] \n\t"
        RESUME_SYNTAX
        : [dptr] REG_RW (result), [lptr] REG_RW (left), [rptr] REG_RW (right),
          [carry] REG_WRITE (carry), [left] REG_WRITE (left_word), [right] REG_WRITE (right_word),
          [jump] REG_RW (jump)
        :
        : "cc", "memory"
    );
    return carry;
}
#define asm_add 1

static uECC_word_t vli_sub(uECC_word_t *result,
                           const uECC_word_t *left,
                           const uECC_word_t *right,
                           wordcount_t num_words) {
#if (uECC_PLATFORM == uECC_arm_thumb) || (uECC_PLATFORM == uECC_arm_thumb2)
    uint32_t jump = ((uECC_MAX_WORDS - num_words) * 4 + 5) * 2 + 1;
#else /* ARM */
    uint32_t jump = ((uECC_MAX_WORDS - num_words) * 4 + 5) * 4;
#endif
    uint32_t carry;
    uint32_t left_word;
    uint32_t right_word;
    
    __asm__ volatile (
        ".syntax unified \n\t"
        "movs %[carry], #0 \n\t"
        "mov %[left], pc \n\t"
        "adds %[jump], %[left] \n\t"
        
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "subs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
        
        "bx %[jump] \n\t"
        
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "sbcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
        
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "sbcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
        
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "sbcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
        
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "sbcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
        
    #if (uECC_MAX_WORDS >= 6)
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "sbcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
    #endif
    #if (uECC_MAX_WORDS >= 7)
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "sbcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
    #endif
    #if (uECC_MAX_WORDS >= 8)
        "ldmia %[lptr]!, {%[left]} \n\t"
        "ldmia %[rptr]!, {%[right]} \n\t"
        "sbcs %[left], %[right] \n\t"
        "stmia %[dptr]!, {%[left]} \n\t"
    #endif
        "adcs %[carry], %[carry] \n\t"
        RESUME_SYNTAX
        : [dptr] REG_RW (result), [lptr] REG_RW (left), [rptr] REG_RW (right),
          [carry] REG_WRITE (carry), [left] REG_WRITE (left_word), [right] REG_WRITE (right_word),
          [jump] REG_RW (jump)
        :
        : "cc", "memory"
    );
    return !carry; /* Note that on ARM, carry flag set means "no borrow" when subtracting
                      (for some reason...) */
}
#define asm_sub 1

#define FAST_MULT_ASM_5_TO_6                 \
    "cmp r3, #5 \n\t"                        \
    "beq 1f \n\t"                            \
                                             \
    /* r4 = left high, r5 = right high */    \
    "ldr r4, [r1] \n\t"                      \
    "ldr r5, [r2] \n\t"                      \
                                             \
    "sub r0, #20 \n\t"                       \
    "sub r1, #20 \n\t"                       \
    "sub r2, #20 \n\t"                       \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r14, #0 \n\t"                       \
    "umull r9, r10, r4, r8 \n\t"             \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r9, r6 \n\t"                       \
    "adc r10, #0 \n\t"                       \
    "adds r9, r11 \n\t"                      \
    "adcs r10, r12 \n\t"                     \
    "adc r14, #0 \n\t"                       \
    "str r9, [r0], #4 \n\t"                  \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r10, r6 \n\t"                      \
    "adcs r14, #0 \n\t"                      \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r9, #0 \n\t"                        \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r10, r11 \n\t"                     \
    "adcs r14, r12 \n\t"                     \
    "adc r9, #0 \n\t"                        \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r10, r11 \n\t"                     \
    "adcs r14, r12 \n\t"                     \
    "adc r9, #0 \n\t"                        \
    "str r10, [r0], #4 \n\t"                 \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r14, r6 \n\t"                      \
    "adcs r9, #0 \n\t"                       \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r10, #0 \n\t"                       \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r14, r11 \n\t"                     \
    "adcs r9, r12 \n\t"                      \
    "adc r10, #0 \n\t"                       \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r14, r11 \n\t"                     \
    "adcs r9, r12 \n\t"                      \
    "adc r10, #0 \n\t"                       \
    "str r14, [r0], #4 \n\t"                 \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r9, r6 \n\t"                       \
    "adcs r10, #0 \n\t"                      \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r14, #0 \n\t"                       \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r9, r11 \n\t"                      \
    "adcs r10, r12 \n\t"                     \
    "adc r14, #0 \n\t"                       \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r9, r11 \n\t"                      \
    "adcs r10, r12 \n\t"                     \
    "adc r14, #0 \n\t"                       \
    "str r9, [r0], #4 \n\t"                  \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r10, r6 \n\t"                      \
    "adcs r14, #0 \n\t"                      \
    /* skip past already-loaded (r4, r5) */  \
    "ldr r7, [r1], #8 \n\t"                  \
    "ldr r8, [r2], #8 \n\t"                  \
    "mov r9, #0 \n\t"                        \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r10, r11 \n\t"                     \
    "adcs r14, r12 \n\t"                     \
    "adc r9, #0 \n\t"                        \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r10, r11 \n\t"                     \
    "adcs r14, r12 \n\t"                     \
    "adc r9, #0 \n\t"                        \
    "str r10, [r0], #4 \n\t"                 \
                                             \
    "umull r11, r12, r4, r5 \n\t"            \
    "adds r11, r14 \n\t"                     \
    "adc r12, r9 \n\t"                       \
    "stmia r0!, {r11, r12} \n\t"

#define FAST_MULT_ASM_6_TO_7                 \
    "cmp r3, #6 \n\t"                        \
    "beq 1f \n\t"                            \
                                             \
    /* r4 = left high, r5 = right high */    \
    "ldr r4, [r1] \n\t"                      \
    "ldr r5, [r2] \n\t"                      \
                                             \
    "sub r0, #24 \n\t"                       \
    "sub r1, #24 \n\t"                       \
    "sub r2, #24 \n\t"                       \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r14, #0 \n\t"                       \
    "umull r9, r10, r4, r8 \n\t"             \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r9, r6 \n\t"                       \
    "adc r10, #0 \n\t"                       \
    "adds r9, r11 \n\t"                      \
    "adcs r10, r12 \n\t"                     \
    "adc r14, #0 \n\t"                       \
    "str r9, [r0], #4 \n\t"                  \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r10, r6 \n\t"                      \
    "adcs r14, #0 \n\t"                      \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r9, #0 \n\t"                        \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r10, r11 \n\t"                     \
    "adcs r14, r12 \n\t"                     \
    "adc r9, #0 \n\t"                        \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r10, r11 \n\t"                     \
    "adcs r14, r12 \n\t"                     \
    "adc r9, #0 \n\t"                        \
    "str r10, [r0], #4 \n\t"                 \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r14, r6 \n\t"                      \
    "adcs r9, #0 \n\t"                       \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r10, #0 \n\t"                       \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r14, r11 \n\t"                     \
    "adcs r9, r12 \n\t"                      \
    "adc r10, #0 \n\t"                       \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r14, r11 \n\t"                     \
    "adcs r9, r12 \n\t"                      \
    "adc r10, #0 \n\t"                       \
    "str r14, [r0], #4 \n\t"                 \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r9, r6 \n\t"                       \
    "adcs r10, #0 \n\t"                      \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r14, #0 \n\t"                       \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r9, r11 \n\t"                      \
    "adcs r10, r12 \n\t"                     \
    "adc r14, #0 \n\t"                       \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r9, r11 \n\t"                      \
    "adcs r10, r12 \n\t"                     \
    "adc r14, #0 \n\t"                       \
    "str r9, [r0], #4 \n\t"                  \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r10, r6 \n\t"                      \
    "adcs r14, #0 \n\t"                      \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r9, #0 \n\t"                        \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r10, r11 \n\t"                     \
    "adcs r14, r12 \n\t"                     \
    "adc r9, #0 \n\t"                        \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r10, r11 \n\t"                     \
    "adcs r14, r12 \n\t"                     \
    "adc r9, #0 \n\t"                        \
    "str r10, [r0], #4 \n\t"                 \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r14, r6 \n\t"                      \
    "adcs r9, #0 \n\t"                       \
    /* skip past already-loaded (r4, r5) */  \
    "ldr r7, [r1], #8 \n\t"                  \
    "ldr r8, [r2], #8 \n\t"                  \
    "mov r10, #0 \n\t"                       \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r14, r11 \n\t"                     \
    "adcs r9, r12 \n\t"                      \
    "adc r10, #0 \n\t"                       \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r14, r11 \n\t"                     \
    "adcs r9, r12 \n\t"                      \
    "adc r10, #0 \n\t"                       \
    "str r14, [r0], #4 \n\t"                 \
                                             \
    "umull r11, r12, r4, r5 \n\t"            \
    "adds r11, r9 \n\t"                      \
    "adc r12, r10 \n\t"                      \
    "stmia r0!, {r11, r12} \n\t"

#define FAST_MULT_ASM_7_TO_8                 \
    "cmp r3, #7 \n\t"                        \
    "beq 1f \n\t"                            \
                                             \
    /* r4 = left high, r5 = right high */    \
    "ldr r4, [r1] \n\t"                      \
    "ldr r5, [r2] \n\t"                      \
                                             \
    "sub r0, #28 \n\t"                       \
    "sub r1, #28 \n\t"                       \
    "sub r2, #28 \n\t"                       \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r14, #0 \n\t"                       \
    "umull r9, r10, r4, r8 \n\t"             \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r9, r6 \n\t"                       \
    "adc r10, #0 \n\t"                       \
    "adds r9, r11 \n\t"                      \
    "adcs r10, r12 \n\t"                     \
    "adc r14, #0 \n\t"                       \
    "str r9, [r0], #4 \n\t"                  \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r10, r6 \n\t"                      \
    "adcs r14, #0 \n\t"                      \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r9, #0 \n\t"                        \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r10, r11 \n\t"                     \
    "adcs r14, r12 \n\t"                     \
    "adc r9, #0 \n\t"                        \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r10, r11 \n\t"                     \
    "adcs r14, r12 \n\t"                     \
    "adc r9, #0 \n\t"                        \
    "str r10, [r0], #4 \n\t"                 \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r14, r6 \n\t"                      \
    "adcs r9, #0 \n\t"                       \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r10, #0 \n\t"                       \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r14, r11 \n\t"                     \
    "adcs r9, r12 \n\t"                      \
    "adc r10, #0 \n\t"                       \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r14, r11 \n\t"                     \
    "adcs r9, r12 \n\t"                      \
    "adc r10, #0 \n\t"                       \
    "str r14, [r0], #4 \n\t"                 \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r9, r6 \n\t"                       \
    "adcs r10, #0 \n\t"                      \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r14, #0 \n\t"                       \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r9, r11 \n\t"                      \
    "adcs r10, r12 \n\t"                     \
    "adc r14, #0 \n\t"                       \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r9, r11 \n\t"                      \
    "adcs r10, r12 \n\t"                     \
    "adc r14, #0 \n\t"                       \
    "str r9, [r0], #4 \n\t"                  \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r10, r6 \n\t"                      \
    "adcs r14, #0 \n\t"                      \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r9, #0 \n\t"                        \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r10, r11 \n\t"                     \
    "adcs r14, r12 \n\t"                     \
    "adc r9, #0 \n\t"                        \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r10, r11 \n\t"                     \
    "adcs r14, r12 \n\t"                     \
    "adc r9, #0 \n\t"                        \
    "str r10, [r0], #4 \n\t"                 \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r14, r6 \n\t"                      \
    "adcs r9, #0 \n\t"                       \
    "ldr r7, [r1], #4 \n\t"                  \
    "ldr r8, [r2], #4 \n\t"                  \
    "mov r10, #0 \n\t"                       \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r14, r11 \n\t"                     \
    "adcs r9, r12 \n\t"                      \
    "adc r10, #0 \n\t"                       \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r14, r11 \n\t"                     \
    "adcs r9, r12 \n\t"                      \
    "adc r10, #0 \n\t"                       \
    "str r14, [r0], #4 \n\t"                 \
                                             \
    "ldr r6, [r0] \n\t"                      \
    "adds r9, r6 \n\t"                       \
    "adcs r10, #0 \n\t"                      \
    /* skip past already-loaded (r4, r5) */  \
    "ldr r7, [r1], #8 \n\t"                  \
    "ldr r8, [r2], #8 \n\t"                  \
    "mov r14, #0 \n\t"                       \
    "umull r11, r12, r4, r8 \n\t"            \
    "adds r9, r11 \n\t"                      \
    "adcs r10, r12 \n\t"                     \
    "adc r14, #0 \n\t"                       \
    "umull r11, r12, r5, r7 \n\t"            \
    "adds r9, r11 \n\t"                      \
    "adcs r10, r12 \n\t"                     \
    "adc r14, #0 \n\t"                       \
    "str r9, [r0], #4 \n\t"                  \
                                             \
    "umull r11, r12, r4, r5 \n\t"            \
    "adds r11, r10 \n\t"                     \
    "adc r12, r14 \n\t"                      \
    "stmia r0!, {r11, r12} \n\t"

#if (uECC_PLATFORM != uECC_arm_thumb)
static void vli_mult(uint32_t *result,
                     const uint32_t *left,
                     const uint32_t *right,
                     wordcount_t num_words) {
    register uint32_t *r0 __asm__("r0") = result;
    register const uint32_t *r1 __asm__("r1") = left;
    register const uint32_t *r2 __asm__("r2") = right;
    register uint32_t r3 __asm__("r3") = num_words;
    
    __asm__ volatile (
        ".syntax unified \n\t"
        "push {r3} \n\t"
    
#if (uECC_MIN_WORDS == 5)
        FAST_MULT_ASM_5
        "pop {r3} \n\t"
    #if (uECC_MAX_WORDS > 5)
        FAST_MULT_ASM_5_TO_6
    #endif
    #if (uECC_MAX_WORDS > 6)
        FAST_MULT_ASM_6_TO_7
    #endif
    #if (uECC_MAX_WORDS > 7)
        FAST_MULT_ASM_7_TO_8
    #endif
#elif (uECC_MIN_WORDS == 6)
        FAST_MULT_ASM_6
        "pop {r3} \n\t"
    #if (uECC_MAX_WORDS > 6)
        FAST_MULT_ASM_6_TO_7
    #endif
    #if (uECC_MAX_WORDS > 7)
        FAST_MULT_ASM_7_TO_8
    #endif
#elif (uECC_MIN_WORDS == 7)
        FAST_MULT_ASM_7
        "pop {r3} \n\t"
    #if (uECC_MAX_WORDS > 7)
        FAST_MULT_ASM_7_TO_8
    #endif
#elif (uECC_MIN_WORDS == 8)
        FAST_MULT_ASM_8
        "pop {r3} \n\t"
#endif

        "1: \n\t"
        RESUME_SYNTAX
        : "+r" (r0), "+r" (r1), "+r" (r2)
        : "r" (r3)
        : "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r14", "cc", "memory"
    );
}
#define asm_mult 1

#if uECC_SQUARE_FUNC

#define FAST_SQUARE_ASM_5_TO_6           \
    "cmp r2, #5 \n\t"                    \
    "beq 1f \n\t"                        \
                                         \
    /* r3 = high */                      \
    "ldr r3, [r1] \n\t"                  \
                                         \
    "sub r0, #20 \n\t"                   \
    "sub r1, #20 \n\t"                   \
                                         \
    /* Do off-center multiplication */   \
    "ldr r14, [r1], #4 \n\t"             \
    "umull r4, r5, r3, r14 \n\t"         \
    "ldr r14, [r1], #4 \n\t"             \
    "umull r7, r6, r3, r14 \n\t"         \
    "adds r5, r7 \n\t"                   \
    "ldr r14, [r1], #4 \n\t"             \
    "umull r8, r7, r3, r14 \n\t"         \
    "adcs r6, r8 \n\t"                   \
    "ldr r14, [r1], #4 \n\t"             \
    "umull r9, r8, r3, r14 \n\t"         \
    "adcs r7, r9 \n\t"                   \
    /* Skip already-loaded r3 */         \
    "ldr r14, [r1], #8 \n\t"             \
    "umull r10, r9, r3, r14 \n\t"        \
    "adcs r8, r10 \n\t"                  \
    "adcs r9, #0 \n\t"                   \
                                         \
    /* Multiply by 2 */                  \
    "mov r10, #0 \n\t"                   \
    "adds r4, r4 \n\t"                   \
    "adcs r5, r5 \n\t"                   \
    "adcs r6, r6 \n\t"                   \
    "adcs r7, r7 \n\t"                   \
    "adcs r8, r8 \n\t"                   \
    "adcs r9, r9 \n\t"                   \
    "adcs r10, #0 \n\t"                  \
                                         \
    /* Add into previous */              \
    "ldr r14, [r0] \n\t"                 \
    "adds r4, r14 \n\t"                  \
    "str r4, [r0], #4 \n\t"              \
    "ldr r14, [r0] \n\t"                 \
    "adcs r5, r14 \n\t"                  \
    "str r5, [r0], #4 \n\t"              \
    "ldr r14, [r0] \n\t"                 \
    "adcs r6, r14 \n\t"                  \
    "str r6, [r0], #4 \n\t"              \
    "ldr r14, [r0] \n\t"                 \
    "adcs r7, r14 \n\t"                  \
    "str r7, [r0], #4 \n\t"              \
    "ldr r14, [r0] \n\t"                 \
    "adcs r8, r14 \n\t"                  \
    "str r8, [r0], #4 \n\t"              \
    "adcs r9, #0 \n\t"                   \
    "adcs r10, #0 \n\t"                  \
                                         \
    /* Perform center multiplication */  \
    "umull r4, r5, r3, r3 \n\t"          \
    "adds r4, r9 \n\t"                   \
    "adc r5, r10 \n\t"                   \
    "stmia r0!, {r4, r5} \n\t"           

#define FAST_SQUARE_ASM_6_TO_7               \
    "cmp r2, #6 \n\t"                        \
    "beq 1f \n\t"                            \
                                             \
    /* r3 = high */                          \
    "ldr r3, [r1] \n\t"                      \
                                             \
    "sub r0, #24 \n\t"                       \
    "sub r1, #24 \n\t"                       \
                                             \
    /* Do off-center multiplication */       \
    "ldr r14, [r1], #4 \n\t"                 \
    "umull r4, r5, r3, r14 \n\t"             \
    "ldr r14, [r1], #4 \n\t"                 \
    "umull r7, r6, r3, r14 \n\t"             \
    "adds r5, r7 \n\t"                       \
    "ldr r14, [r1], #4 \n\t"                 \
    "umull r8, r7, r3, r14 \n\t"             \
    "adcs r6, r8 \n\t"                       \
    "ldr r14, [r1], #4 \n\t"                 \
    "umull r9, r8, r3, r14 \n\t"             \
    "adcs r7, r9 \n\t"                       \
    "ldr r14, [r1], #4 \n\t"                 \
    "umull r10, r9, r3, r14 \n\t"            \
    "adcs r8, r10 \n\t"                      \
    /* Skip already-loaded r3 */             \
    "ldr r14, [r1], #8 \n\t"                 \
    "umull r11, r10, r3, r14 \n\t"           \
    "adcs r9, r11 \n\t"                      \
    "adcs r10, #0 \n\t"                      \
                                             \
    /* Multiply by 2 */                      \
    "mov r11, #0 \n\t"                       \
    "adds r4, r4 \n\t"                       \
    "adcs r5, r5 \n\t"                       \
    "adcs r6, r6 \n\t"                       \
    "adcs r7, r7 \n\t"                       \
    "adcs r8, r8 \n\t"                       \
    "adcs r9, r9 \n\t"                       \
    "adcs r10, r10 \n\t"                     \
    "adcs r11, #0 \n\t"                      \
                                             \
    /* Add into previous */                  \
    "ldr r14, [r0] \n\t"                     \
    "adds r4, r14 \n\t"                      \
    "str r4, [r0], #4 \n\t"                  \
    "ldr r14, [r0] \n\t"                     \
    "adcs r5, r14 \n\t"                      \
    "str r5, [r0], #4 \n\t"                  \
    "ldr r14, [r0] \n\t"                     \
    "adcs r6, r14 \n\t"                      \
    "str r6, [r0], #4 \n\t"                  \
    "ldr r14, [r0] \n\t"                     \
    "adcs r7, r14 \n\t"                      \
    "str r7, [r0], #4 \n\t"                  \
    "ldr r14, [r0] \n\t"                     \
    "adcs r8, r14 \n\t"                      \
    "str r8, [r0], #4 \n\t"                  \
    "ldr r14, [r0] \n\t"                     \
    "adcs r9, r14 \n\t"                      \
    "str r9, [r0], #4 \n\t"                  \
    "adcs r10, #0 \n\t"                      \
    "adcs r11, #0 \n\t"                      \
                                             \
    /* Perform center multiplication */      \
    "umull r4, r5, r3, r3 \n\t"              \
    "adds r4, r10 \n\t"                      \
    "adc r5, r11 \n\t"                       \
    "stmia r0!, {r4, r5} \n\t"

#define FAST_SQUARE_ASM_7_TO_8           \
    "cmp r2, #7 \n\t"                    \
    "beq 1f \n\t"                        \
                                         \
    /* r3 = high */                      \
    "ldr r3, [r1] \n\t"                  \
                                         \
    "sub r0, #28 \n\t"                   \
    "sub r1, #28 \n\t"                   \
                                         \
    /* Do off-center multiplication */   \
    "ldr r14, [r1], #4 \n\t"             \
    "umull r4, r5, r3, r14 \n\t"         \
    "ldr r14, [r1], #4 \n\t"             \
    "umull r7, r6, r3, r14 \n\t"         \
    "adds r5, r7 \n\t"                   \
    "ldr r14, [r1], #4 \n\t"             \
    "umull r8, r7, r3, r14 \n\t"         \
    "adcs r6, r8 \n\t"                   \
    "ldr r14, [r1], #4 \n\t"             \
    "umull r9, r8, r3, r14 \n\t"         \
    "adcs r7, r9 \n\t"                   \
    "ldr r14, [r1], #4 \n\t"             \
    "umull r10, r9, r3, r14 \n\t"        \
    "adcs r8, r10 \n\t"                  \
    "ldr r14, [r1], #4 \n\t"             \
    "umull r11, r10, r3, r14 \n\t"       \
    "adcs r9, r11 \n\t"                  \
    /* Skip already-loaded r3 */         \
    "ldr r14, [r1], #8 \n\t"             \
    "umull r12, r11, r3, r14 \n\t"       \
    "adcs r10, r12 \n\t"                 \
    "adcs r11, #0 \n\t"                  \
                                         \
    /* Multiply by 2 */                  \
    "mov r12, #0 \n\t"                   \
    "adds r4, r4 \n\t"                   \
    "adcs r5, r5 \n\t"                   \
    "adcs r6, r6 \n\t"                   \
    "adcs r7, r7 \n\t"                   \
    "adcs r8, r8 \n\t"                   \
    "adcs r9, r9 \n\t"                   \
    "adcs r10, r10 \n\t"                 \
    "adcs r11, r11 \n\t"                 \
    "adcs r12, #0 \n\t"                  \
                                         \
    /* Add into previous */              \
    "ldr r14, [r0] \n\t"                 \
    "adds r4, r14 \n\t"                  \
    "str r4, [r0], #4 \n\t"              \
    "ldr r14, [r0] \n\t"                 \
    "adcs r5, r14 \n\t"                  \
    "str r5, [r0], #4 \n\t"              \
    "ldr r14, [r0] \n\t"                 \
    "adcs r6, r14 \n\t"                  \
    "str r6, [r0], #4 \n\t"              \
    "ldr r14, [r0] \n\t"                 \
    "adcs r7, r14 \n\t"                  \
    "str r7, [r0], #4 \n\t"              \
    "ldr r14, [r0] \n\t"                 \
    "adcs r8, r14 \n\t"                  \
    "str r8, [r0], #4 \n\t"              \
    "ldr r14, [r0] \n\t"                 \
    "adcs r9, r14 \n\t"                  \
    "str r9, [r0], #4 \n\t"              \
    "ldr r14, [r0] \n\t"                 \
    "adcs r10, r14 \n\t"                 \
    "str r10, [r0], #4 \n\t"             \
    "adcs r11, #0 \n\t"                  \
    "adcs r12, #0 \n\t"                  \
                                         \
    /* Perform center multiplication */  \
    "umull r4, r5, r3, r3 \n\t"          \
    "adds r4, r11 \n\t"                  \
    "adc r5, r12 \n\t"                   \
    "stmia r0!, {r4, r5} \n\t"           

static void vli_square(uECC_word_t *result, const uECC_word_t *left, wordcount_t num_words) {
    register uint32_t *r0 __asm__("r0") = result;
    register const uint32_t *r1 __asm__("r1") = left;
    register uint32_t r2 __asm__("r2") = num_words;
    
    __asm__ volatile (
        ".syntax unified \n\t"
        "push {r1, r2} \n\t"

#if (uECC_MIN_WORDS == 5)
        FAST_SQUARE_ASM_5
        "pop {r1, r2} \n\t"
    #if (uECC_MAX_WORDS > 5)
        "add r1, #20 \n\t"
        FAST_SQUARE_ASM_5_TO_6
    #endif
    #if (uECC_MAX_WORDS > 6)
        FAST_SQUARE_ASM_6_TO_7
    #endif
    #if (uECC_MAX_WORDS > 7)
        FAST_SQUARE_ASM_7_TO_8
    #endif
#elif (uECC_MIN_WORDS == 6)
        FAST_SQUARE_ASM_6
        "pop {r1, r2} \n\t"
    #if (uECC_MAX_WORDS > 6)
        "add r1, #24 \n\t"
        FAST_SQUARE_ASM_6_TO_7
    #endif
    #if (uECC_MAX_WORDS > 7)
        FAST_SQUARE_ASM_7_TO_8
    #endif
#elif (uECC_MIN_WORDS == 7)
        FAST_SQUARE_ASM_7
        "pop {r1, r2} \n\t"
    #if (uECC_MAX_WORDS > 7)
        "add r1, #28 \n\t"
        FAST_SQUARE_ASM_7_TO_8
    #endif
#elif (uECC_MIN_WORDS == 8)
        FAST_SQUARE_ASM_8
        "pop {r1, r2} \n\t"
#endif

        "1: \n\t"
        RESUME_SYNTAX
        : "+r" (r0), "+r" (r1)
        : "r" (r2)
        : "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r14", "cc", "memory"
    );
}
#define asm_square 1
#endif /* uECC_SQUARE_FUNC */

#endif /* uECC_PLATFORM != uECC_arm_thumb */

#endif /* _UECC_ASM_ARM_FAST_H_ */
