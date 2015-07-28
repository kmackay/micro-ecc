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
    uint32_t carry = 0;
    uint32_t left_word;
    uint32_t right_word;
    
    __asm__ volatile (
        ".syntax unified \n\t"
        "1: \n\t"
        "ldmia %[lptr]!, {%[left]} \n\t"  /* Load left word. */
        "ldmia %[rptr]!, {%[right]} \n\t" /* Load right word. */
        "lsrs %[carry], #1 \n\t"          /* Set up carry flag (carry = 0 after this). */
        "adcs %[left], %[right] \n\t"     /* Add with carry. */
        "adcs %[carry], %[carry] \n\t"    /* Store carry bit. */
        "stmia %[dptr]!, {%[left]} \n\t"  /* Store result word. */
        "subs %[ctr], #1 \n\t"            /* Decrement counter. */
        "bne 1b \n\t"                     /* Loop until counter == 0. */
        RESUME_SYNTAX
        : [dptr] REG_RW (result), [lptr] REG_RW (left), [rptr] REG_RW (right),
          [ctr] REG_RW (num_words), [carry] REG_RW (carry),
          [left] REG_WRITE (left_word), [right] REG_WRITE (right_word)
        :
        : "cc", "memory"
    );
    return carry;
}
#define asm_add 1

static uint32_t vli_sub(uECC_word_t *result,
                        const uECC_word_t *left,
                        const uECC_word_t *right,
                        wordcount_t num_words) {
    uint32_t carry = 1; /* carry = 1 initially (means don't borrow) */
    uint32_t left_word;
    uint32_t right_word;
    
    __asm__ volatile (
        ".syntax unified \n\t"
        "1: \n\t"
        "ldmia %[lptr]!, {%[left]} \n\t"  /* Load left word. */
        "ldmia %[rptr]!, {%[right]} \n\t" /* Load right word. */
        "lsrs %[carry], #1 \n\t"          /* Set up carry flag (carry = 0 after this). */
        "sbcs %[left], %[right] \n\t"     /* Subtract with borrow. */
        "adcs %[carry], %[carry] \n\t"    /* Store carry bit. */
        "stmia %[dptr]!, {%[left]} \n\t"  /* Store result word. */
        "subs %[ctr], #1 \n\t"            /* Decrement counter. */
        "bne 1b \n\t"                     /* Loop until counter == 0. */
        RESUME_SYNTAX
        : [dptr] REG_RW (result), [lptr] REG_RW (left), [rptr] REG_RW (right),
          [ctr] REG_RW (num_words), [carry] REG_RW (carry),
          [left] REG_WRITE (left_word), [right] REG_WRITE (right_word)
        :
        : "cc", "memory"
    );
    return !carry;
}
#define asm_sub 1

static void vli_mult(uECC_word_t *result,
                     const uECC_word_t *left,
                     const uECC_word_t *right,
                     wordcount_t num_words) {
#if (uECC_PLATFORM != uECC_arm_thumb)
    uint32_t c0 = 0;
    uint32_t c1 = 0;
    uint32_t c2 = 0;
    uint32_t k = 0;
    uint32_t i;
    uint32_t t0, t1;
    
    __asm__ volatile (
        ".syntax unified \n\t"
        
        "1: \n\t" /* outer loop (k < num_words) */
        "movs %[i], #0 \n\t" /* i = 0 */
        "b 3f \n\t"
        
        "2: \n\t" /* outer loop (k >= num_words) */
        "movs %[i], %[k] \n\t"         /* i = k */
        "subs %[i], %[last_word] \n\t" /* i = k - (num_words - 1) (times 4) */
        
        "3: \n\t" /* inner loop */
        "subs %[t0], %[k], %[i] \n\t" /* t0 = k-i */
        
        "ldr %[t1], [%[right], %[t0]] \n\t" /* t1 = right[k - i] */
        "ldr %[t0], [%[left], %[i]] \n\t"   /* t0 = left[i] */
        
        "umull %[t0], %[t1], %[t0], %[t1] \n\t" /* (t0, t1) = left[i] * right[k - i] */
        
        "adds %[c0], %[t0] \n\t" /* add low word to c0 */
        "adcs %[c1], %[t1] \n\t" /* add high word to c1, including carry */
        "adcs %[c2], #0 \n\t"    /* add carry to c2 */

        "adds %[i], #4 \n\t"          /* i += 4 */
        "cmp %[i], %[last_word] \n\t" /* i > (num_words - 1) (times 4)? */
        "bgt 4f \n\t"                 /*   if so, exit the loop */
        "cmp %[i], %[k] \n\t"         /* i <= k? */
        "ble 3b \n\t"                 /*   if so, continue looping */
        
        "4: \n\t" /* end inner loop */
        
        "str %[c0], [%[result], %[k]] \n\t" /* result[k] = c0 */
        "mov %[c0], %[c1] \n\t"       /* c0 = c1 */
        "mov %[c1], %[c2] \n\t"       /* c1 = c2 */
        "movs %[c2], #0 \n\t"         /* c2 = 0 */
        "adds %[k], #4 \n\t"          /* k += 4 */
        "cmp %[k], %[last_word] \n\t" /* k <= (num_words - 1) (times 4) ? */
        "ble 1b \n\t"                 /*   if so, loop back, start with i = 0 */
        "cmp %[k], %[last_word], lsl #1 \n\t" /* k <= (num_words * 2 - 2) (times 4) ? */
        "ble 2b \n\t"                 /*   if so, loop back, start with i = (k + 1) - num_words */
        /* end outer loop */
        
        "str %[c0], [%[result], %[k]] \n\t" /* result[num_words * 2 - 1] = c0 */
        RESUME_SYNTAX
        : [c0] "+r" (c0), [c1] "+r" (c1), [c2] "+r" (c2),
          [k] "+r" (k), [i] "=&r" (i), [t0] "=&r" (t0), [t1] "=&r" (t1)
        : [result] "r" (result), [left] "r" (left), [right] "r" (right),
          [last_word] "r" ((num_words - 1) * 4)
        : "cc", "memory"
    );
    
#else /* Thumb-1 */

    register uint32_t *r0 __asm__("r0") = result;
    register const uint32_t *r1 __asm__("r1") = left;
    register const uint32_t *r2 __asm__("r2") = right;
    
    __asm__ volatile (
        ".syntax unified \n\t"
        "movs r3, #0 \n\t" /* c0 = 0 */
        "movs r4, #0 \n\t" /* c1 = 0 */
        "movs r5, #0 \n\t" /* c2 = 0 */
        "movs r6, #0 \n\t" /* k = 0 */
        
        "push {r0} \n\t" /* keep result on the stack */
        
        "1: \n\t" /* outer loop (k < num_words) */
        "movs r7, #0 \n\t" /* r7 = i = 0 */
        "b 3f \n\t"
        
        "2: \n\t" /* outer loop (k >= num_words) */
        "movs r7, r6 \n\t"           /* r7 = k */
        "subs r7, %[last_word] \n\t" /* r7 = i = k - (num_words - 1) (times 4) */
        
        "3: \n\t" /* inner loop */
        "push {r3, r4, r5, r6} \n\t" /* push things, r3 (c0) is at the top of stack. */
        "subs r0, r6, r7 \n\t"       /* r0 = k - i */
        
        "ldr r4, [r2, r0] \n\t" /* r4 = right[k - i] */
        "ldr r0, [r1, r7] \n\t" /* r0 = left[i] */
        
        "lsrs r3, r0, #16 \n\t" /* r3 = a1 */
        "uxth r0, r0 \n\t"      /* r0 = a0 */
        
        "lsrs r5, r4, #16 \n\t" /* r5 = b1 */
        "uxth r4, r4 \n\t"      /* r4 = b0 */
        
        "movs r6, r3 \n\t"     /* r6 = a1 */
        "muls r6, r5, r6 \n\t" /* r6 = a1 * b1 */
        "muls r3, r4, r3 \n\t" /* r3 = b0 * a1 */
        "muls r5, r0, r5 \n\t" /* r5 = a0 * b1 */
        "muls r0, r4, r0 \n\t" /* r0 = a0 * b0 */
        
        "movs r4, #0 \n\t"  /* r4 = 0 */
        "adds r3, r5 \n\t"  /* r3 = b0 * a1 + a0 * b1 */
        "adcs r4, r4 \n\t"  /* r4 = carry */
        "lsls r4, #16 \n\t" /* r4 = carry << 16 */
        "adds r6, r4 \n\t"  /* r6 = a1 * b1 + carry */
        
        "lsls r4, r3, #16 \n\t" /* r4 = (b0 * a1 + a0 * b1) << 16 */
        "lsrs r3, #16 \n\t"     /* r3 = (b0 * a1 + a0 * b1) >> 16 */
        "adds r0, r4 \n\t"      /* r0 = low word = a0 * b0 + ((b0 * a1 + a0 * b1) << 16) */
        "adcs r6, r3 \n\t"      /* r6 = high word = a1 * b1 + carry + ((b0 * a1 + a0 * b1) >> 16) */
        
        "pop {r3, r4, r5} \n\t" /* r3 = c0, r4 = c1, r5 = c2 */
        "adds r3, r0 \n\t"      /* add low word to c0 */
        "adcs r4, r6 \n\t"      /* add high word to c1, including carry */
        "movs r0, #0 \n\t"      /* r0 = 0 (does not affect carry bit) */
        "adcs r5, r0 \n\t"      /* add carry to c2 */
        
        "pop {r6} \n\t" /* r6 = k */

        "adds r7, #4 \n\t"          /* i += 4 */
        "cmp r7, %[last_word] \n\t" /* i > (num_words - 1) (times 4)? */
        "bgt 4f \n\t"               /*   if so, exit the loop */
        "cmp r7, r6 \n\t"           /* i <= k? */
        "ble 3b \n\t"               /*   if so, continue looping */
        
        "4: \n\t" /* end inner loop */
        
        "ldr r0, [sp, #0] \n\t" /* r0 = result */
        
        "str r3, [r0, r6] \n\t"     /* result[k] = c0 */
        "mov r3, r4 \n\t"           /* c0 = c1 */
        "mov r4, r5 \n\t"           /* c1 = c2 */
        "movs r5, #0 \n\t"          /* c2 = 0 */
        "adds r6, #4 \n\t"          /* k += 4 */
        "cmp r6, %[last_word] \n\t" /* k <= (num_words - 1) (times 4) ? */
        "ble 1b \n\t"               /*   if so, loop back, start with i = 0 */
        "cmp r6, %[lw2] \n\t"       /* k <= (num_words * 2 - 2) (times 4) ? */
        "ble 2b \n\t"               /*   if so, loop back, start with i = (k + 1) - num_words */
        /* end outer loop */
        
        "str r3, [r0, r6] \n\t" /* result[num_words * 2 - 1] = c0 */
        "pop {r0} \n\t"         /* pop result off the stack */
        
        ".syntax divided \n\t"
        : 
        : [r0] "l" (r0), [r1] "l" (r1), [r2] "l" (r2),
          [last_word] "r" ((num_words - 1) * 4), [lw2] "r" ((num_words - 1) * 4 * 2)
        : "r3", "r4", "r5", "r6", "r7", "cc", "memory"
    );
#endif
}
#define asm_mult 1

#if uECC_SQUARE_FUNC
static void vli_square(uECC_word_t *result, const uECC_word_t *left, wordcount_t num_words) {
#if (uECC_PLATFORM != uECC_arm_thumb)
    uint32_t c0 = 0;
    uint32_t c1 = 0;
    uint32_t c2 = 0;
    uint32_t k = 0;
    uint32_t i, tt;
    uint32_t t0, t1;
    
    __asm__ volatile (
        ".syntax unified \n\t"
        
        "1: \n\t" /* outer loop (k < num_words) */
        "movs %[i], #0 \n\t" /* i = 0 */
        "b 3f \n\t"
        
        "2: \n\t" /* outer loop (k >= num_words) */
        "movs %[i], %[k] \n\t"         /* i = k */
        "subs %[i], %[last_word] \n\t" /* i = k - (num_words - 1) (times 4) */
        
        "3: \n\t" /* inner loop */
        "subs %[tt], %[k], %[i] \n\t" /* tt = k-i */
        
        "ldr %[t1], [%[left], %[tt]] \n\t" /* t1 = left[k - i] */
        "ldr %[t0], [%[left], %[i]] \n\t"  /* t0 = left[i] */
        
        "umull %[t0], %[t1], %[t0], %[t1] \n\t" /* (t0, t1) = left[i] * right[k - i] */
        
        "cmp %[i], %[tt] \n\t" /* (i < k - i) ? */
        "bge 4f \n\t"          /*   if i >= k - i, skip */
        "lsls %[t1], #1 \n\t"  /* high word << 1 */
        "adc %[c2], #0 \n\t"   /* add carry bit to c2 */
        "lsls %[t0], #1 \n\t"  /* low word << 1 */
        "adc %[t1], #0 \n\t"   /* add carry bit to high word */
        
        "4: \n\t"

        "adds %[c0], %[t0] \n\t" /* add low word to c0 */
        "adcs %[c1], %[t1] \n\t" /* add high word to c1, including carry */
        "adc %[c2], #0 \n\t"     /* add carry to c2 */
        
        "adds %[i], #4 \n\t"          /* i += 4 */
        "cmp %[i], %[k] \n\t"         /* i >= k? */
        "bge 5f \n\t"                 /*   if so, exit the loop */
        "subs %[tt], %[k], %[i] \n\t" /* tt = k - i */
        "cmp %[i], %[tt] \n\t"        /* i <= k - i? */
        "ble 3b \n\t"                 /*   if so, continue looping */
        
        "5: \n\t" /* end inner loop */
        
        "str %[c0], [%[result], %[k]] \n\t" /* result[k] = c0 */
        "mov %[c0], %[c1] \n\t"       /* c0 = c1 */
        "mov %[c1], %[c2] \n\t"       /* c1 = c2 */
        "movs %[c2], #0 \n\t"         /* c2 = 0 */
        "adds %[k], #4 \n\t"          /* k += 4 */
        "cmp %[k], %[last_word] \n\t" /* k <= (num_words - 1) (times 4) ? */
        "ble 1b \n\t"                 /*   if so, loop back, start with i = 0 */
        "cmp %[k], %[last_word], lsl #1 \n\t" /* k <= (num_words * 2 - 2) (times 4) ? */
        "ble 2b \n\t"                 /*   if so, loop back, start with i = (k + 1) - num_words */
        /* end outer loop */
        
        "str %[c0], [%[result], %[k]] \n\t" /* result[num_words * 2 - 1] = c0 */
        RESUME_SYNTAX
        : [c0] "+r" (c0), [c1] "+r" (c1), [c2] "+r" (c2),
          [k] "+r" (k), [i] "=&r" (i), [tt] "=&r" (tt), [t0] "=&r" (t0), [t1] "=&r" (t1)
        : [result] "r" (result), [left] "r" (left), [last_word] "r" ((num_words - 1) * 4)
        : "cc", "memory"
    );
    
#else

    register uint32_t *r0 __asm__("r0") = result;
    register const uint32_t *r1 __asm__("r1") = left;
    
    __asm__ volatile (
        ".syntax unified \n\t"
        "movs r2, #0 \n\t" /* c0 = 0 */
        "movs r3, #0 \n\t" /* c1 = 0 */
        "movs r4, #0 \n\t" /* c2 = 0 */
        "movs r5, #0 \n\t" /* k = 0 */
        
        "push {r0} \n\t" /* keep result on the stack */
        
        "1: \n\t" /* outer loop (k < num_words) */
        "movs r6, #0 \n\t" /* r6 = i = 0 */
        "b 3f \n\t"
        
        "2: \n\t" /* outer loop (k >= num_words) */
        "movs r6, r5 \n\t"        /* r6 = k */
        "subs r6, %[last_word] \n\t" /* r6 = i = k - (num_words - 1) (times 4) */
        
        "3: \n\t" /* inner loop */
        "push {r2, r3, r4, r5} \n\t" /* push things, r2 (c0) is at the top of stack. */
        "subs r7, r5, r6 \n\t"       /* r7 = k - i */
        
        "ldr r3, [r1, r7] \n\t" /* r3 = left[k - i] */
        "ldr r0, [r1, r6] \n\t" /* r0 = left[i] */
        
        "lsrs r2, r0, #16 \n\t" /* r2 = a1 */
        "uxth r0, r0 \n\t"      /* r0 = a0 */
        
        "lsrs r4, r3, #16 \n\t" /* r4 = b1 */
        "uxth r3, r3 \n\t"      /* r3 = b0 */
        
        "movs r5, r2 \n\t"     /* r5 = a1 */
        "muls r5, r4, r5 \n\t" /* r5 = a1 * b1 */
        "muls r2, r3, r2 \n\t" /* r2 = b0 * a1 */
        "muls r4, r0, r4 \n\t" /* r4 = a0 * b1 */
        "muls r0, r3, r0 \n\t" /* r0 = a0 * b0 */
        
        "movs r3, #0 \n\t"  /* r3 = 0 */
        "adds r2, r4 \n\t"  /* r2 = b0 * a1 + a0 * b1 */
        "adcs r3, r3 \n\t"  /* r3 = carry */
        "lsls r3, #16 \n\t" /* r3 = carry << 16 */
        "adds r5, r3 \n\t"  /* r5 = a1 * b1 + carry */
        
        "lsls r3, r2, #16 \n\t" /* r3 = (b0 * a1 + a0 * b1) << 16 */
        "lsrs r2, #16 \n\t"     /* r2 = (b0 * a1 + a0 * b1) >> 16 */
        "adds r0, r3 \n\t"      /* r0 = low word = a0 * b0 + ((b0 * a1 + a0 * b1) << 16) */
        "adcs r5, r2 \n\t"      /* r5 = high word = a1 * b1 + carry + ((b0 * a1 + a0 * b1) >> 16) */
    
        "movs r3, #0 \n\t"  /* r3 = 0 */
        "cmp r6, r7 \n\t"   /* (i < k - i) ? */
        "mov r7, r3 \n\t"   /* r7 = 0 (does not affect condition)*/
        "bge 4f \n\t"       /*   if i >= k - i, skip */
        "lsls r5, #1 \n\t"  /* high word << 1 */
        "adcs r7, r3 \n\t"  /* r7 = carry bit for c2 */
        "lsls r0, #1 \n\t"  /* low word << 1 */
        "adcs r5, r3 \n\t"  /* add carry from shift to high word */
        
        "4: \n\t"
        "pop {r2, r3, r4} \n\t" /* r2 = c0, r3 = c1, r4 = c2 */
        "adds r2, r0 \n\t"      /* add low word to c0 */
        "adcs r3, r5 \n\t"      /* add high word to c1, including carry */
        "movs r0, #0 \n\t"      /* r0 = 0 (does not affect carry bit) */
        "adcs r4, r0 \n\t"      /* add carry to c2 */
        "adds r4, r7 \n\t"      /* add carry from doubling (if any) */
        
        "pop {r5} \n\t" /* r5 = k */
        
        "adds r6, #4 \n\t"     /* i += 4 */
        "cmp r6, r5 \n\t"      /* i >= k? */
        "bge 5f \n\t"          /*   if so, exit the loop */
        "subs r7, r5, r6 \n\t" /* r7 = k - i */
        "cmp r6, r7 \n\t"      /* i <= k - i? */
        "ble 3b \n\t"          /*   if so, continue looping */
        
        "5: \n\t" /* end inner loop */
        
        "ldr r0, [sp, #0] \n\t" /* r0 = result */
        
        "str r2, [r0, r5] \n\t"     /* result[k] = c0 */
        "mov r2, r3 \n\t"           /* c0 = c1 */
        "mov r3, r4 \n\t"           /* c1 = c2 */
        "movs r4, #0 \n\t"          /* c2 = 0 */
        "adds r5, #4 \n\t"          /* k += 4 */
        "cmp r5, %[last_word] \n\t" /* k <= (num_words - 1) (times 4) ? */
        "ble 1b \n\t"               /*   if so, loop back, start with i = 0 */
        "cmp r5, %[lw2] \n\t"       /* k <= (num_words * 2 - 2) (times 4) ? */
        "ble 2b \n\t"               /*   if so, loop back, start with i = (k + 1) - num_words */
        /* end outer loop */
        
        "str r2, [r0, r5] \n\t" /* result[num_words * 2 - 1] = c0 */
        "pop {r0} \n\t"        /* pop result off the stack */

        ".syntax divided \n\t"
        : [r0] "+l" (r0), [r1] "+l" (r1)
        : [last_word] "r" ((num_words - 1) * 4), [lw2] "r" ((num_words - 1) * 4 * 2)
        : "r2", "r3", "r4", "r5", "r6", "r7", "cc", "memory"
    );
#endif
}
#define asm_square 1
#endif /* uECC_SQUARE_FUNC */
