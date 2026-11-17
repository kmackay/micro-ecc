/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#ifndef _UECC_ASM_ARM_MULT_SQUARE_H_
#define _UECC_ASM_ARM_MULT_SQUARE_H_

#if (uECC_PLATFORM == uECC_arm_thumb)

/* ============================================================================
 * Thumb-1 (ARMv6-M / Cortex-M0/M0+) Level 4 unrolled multiplication.
 *
 * muls only returns the low 32 bits, so each word is split into 16-bit halves:
 *   A = A_E + (A_O << 16),  B = B_E + (B_O << 16).
 * The 16x16-bit products are summed on 32-bit word grids, with no per-product
 * shifts:  E = A_E * B_E + 2^32 * A_O * B_O  and  O = A_O * B_E + A_E * B_O.
 * A final merge pass computes result = E + (O << 16).
 * - 5 words: direct E/O multiplication.
 * - 6, 7, 8 words: one level of subtractive Karatsuba on top of the shared
 *   3x3-word (mul3) and 4x4-word (mul4) E/O leaf subroutines.
 * ============================================================================ */

#if (uECC_SUPPORTS_secp160r1 || uECC_SUPPORTS_secp192r1 || uECC_SUPPORTS_secp224r1)
/* Leaf subroutine (called with bl): result[0..5] = left[0..2] * right[0..2],
   with r0 = result, r1 = left, r2 = right. Clobbers r0-r10 and r12. E and O
   are accumulated column by column on the stack (E at sp[0], O at sp[24]),
   then merged as result = E + (O << 16). */
#define FAST_MULT_THUMB1_MUL3_BODY \
    /* Split right: B_E in r8-r10, B_O to sp[48..56]. */ \
    "sub   sp, #84 \n\t" \
    "mov   r12, r0 \n\t" \
    "ldm   r2, {r2, r3, r4} \n\t" \
    "uxth  r6, r2 \n\t" \
    "mov   r8, r6 \n\t" \
    "lsrs  r2, r2, #16 \n\t" \
    "uxth  r6, r3 \n\t" \
    "mov   r9, r6 \n\t" \
    "lsrs  r3, r3, #16 \n\t" \
    "uxth  r6, r4 \n\t" \
    "mov   r10, r6 \n\t" \
    "lsrs  r4, r4, #16 \n\t" \
    "add   r6, sp, #48 \n\t" \
    "stmia r6!, {r2, r3, r4} \n\t" \
    /* Split left: A_E in r1-r3 and sp[72..80], A_O to sp[60..68]. */ \
    "ldm   r1, {r1, r2, r3} \n\t" \
    "lsrs  r5, r1, #16 \n\t" \
    "uxth  r1, r1 \n\t" \
    "str   r5, [sp, #60] \n\t" \
    "lsrs  r5, r2, #16 \n\t" \
    "uxth  r2, r2 \n\t" \
    "str   r5, [sp, #64] \n\t" \
    "lsrs  r5, r3, #16 \n\t" \
    "uxth  r3, r3 \n\t" \
    "str   r5, [sp, #68] \n\t" \
    "add   r0, sp, #72 \n\t" \
    "stmia r0!, {r1, r2, r3} \n\t" \
    /* E[0..4] = A_E * B_E by columns: r5 = column sum, r6 = its carries, r7 = 0. */ \
    "movs  r7, #0 \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "str   r5, [sp, #0] \n\t" \
    "mov   r5, r9 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #4] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #8] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #12] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "str   r5, [sp, #16] \n\t" \
    /* O[0..4] = A_O * B_E. */ \
    "add   r0, sp, #60 \n\t" \
    "ldmia r0!, {r1, r2, r3} \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "str   r5, [sp, #24] \n\t" \
    "mov   r5, r9 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #28] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #32] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #36] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "str   r5, [sp, #40] \n\t" \
    /* Load B_O; E[1..5] += A_O * B_O (weight 2^32, so one word up). */ \
    "ldr   r0, [sp, #48] \n\t" \
    "mov   r8, r0 \n\t" \
    "ldr   r0, [sp, #52] \n\t" \
    "mov   r9, r0 \n\t" \
    "ldr   r0, [sp, #56] \n\t" \
    "mov   r10, r0 \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #4] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #4] \n\t" \
    "mov   r5, r9 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #8] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #8] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #12] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #12] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #16] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #16] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "str   r5, [sp, #20] \n\t" \
    /* O[0..5] += A_E * B_O. */ \
    "add   r0, sp, #72 \n\t" \
    "ldmia r0!, {r1, r2, r3} \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #24] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #24] \n\t" \
    "mov   r5, r9 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #28] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #28] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #32] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #32] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #36] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #36] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #40] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #40] \n\t" \
    "str   r6, [sp, #44] \n\t" \
    /* Merge low 4 words: result[0..3] = E[0..3] + (O << 16). */ \
    "add   r0, sp, #24 \n\t" \
    "ldm   r0, {r0, r1, r2, r3} \n\t" \
    "lsls  r4, r0, #16 \n\t" \
    "lsrs  r0, r0, #16 \n\t" \
    "lsls  r5, r1, #16 \n\t" \
    "orrs  r5, r5, r0 \n\t" \
    "lsrs  r1, r1, #16 \n\t" \
    "lsls  r6, r2, #16 \n\t" \
    "orrs  r6, r6, r1 \n\t" \
    "lsrs  r2, r2, #16 \n\t" \
    "lsls  r7, r3, #16 \n\t" \
    "orrs  r7, r7, r2 \n\t" \
    "lsrs  r3, r3, #16 \n\t" \
    "mov   r8, r3 \n\t" \
    "mov   r0, sp \n\t" \
    "ldm   r0, {r0, r1, r2, r3} \n\t" \
    "adds  r0, r0, r4 \n\t" \
    "adcs  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r6 \n\t" \
    "adcs  r3, r3, r7 \n\t" \
    "mov   r4, r12 \n\t" \
    "stmia r4!, {r0, r1, r2, r3} \n\t" \
    "mov   r12, r4 \n\t" \
    /* Save the carry in r9 (lsrs below shifts it back into C); merge result[4..5]. */ \
    "movs  r0, #0 \n\t" \
    "adcs  r0, r0, r0 \n\t" \
    "mov   r9, r0 \n\t" \
    "add   r0, sp, #40 \n\t" \
    "ldm   r0, {r0, r1} \n\t" \
    "mov   r4, r8 \n\t" \
    "lsls  r5, r0, #16 \n\t" \
    "orrs  r4, r4, r5 \n\t" \
    "lsrs  r0, r0, #16 \n\t" \
    "lsls  r5, r1, #16 \n\t" \
    "orrs  r5, r5, r0 \n\t" \
    "add   r0, sp, #16 \n\t" \
    "ldm   r0, {r0, r1} \n\t" \
    "mov   r8, r0 \n\t" \
    "mov   r0, r9 \n\t" \
    "lsrs  r0, r0, #1 \n\t" \
    "mov   r0, r8 \n\t" \
    "adcs  r0, r0, r4 \n\t" \
    "adcs  r1, r1, r5 \n\t" \
    "mov   r4, r12 \n\t" \
    "stmia r4!, {r0, r1} \n\t" \
    "add   sp, #84 \n\t" \
    "bx    lr \n\t"
#endif

#if (uECC_SUPPORTS_secp224r1 || uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1)
/* Leaf subroutine (called with bl): result[0..7] = left[0..3] * right[0..3],
   with r0 = result, r1 = left, r2 = right. Clobbers r0-r12. Same E/O scheme
   as mul3, with E at sp[0] and O at sp[32]. */
#define FAST_MULT_THUMB1_MUL4_BODY \
    /* Split right: B_E in r8-r11, B_O to sp[64..76]. */ \
    "sub   sp, #112 \n\t" \
    "mov   r12, r0 \n\t" \
    "ldm   r2, {r2, r3, r4, r5} \n\t" \
    "uxth  r6, r2 \n\t" \
    "mov   r8, r6 \n\t" \
    "lsrs  r2, r2, #16 \n\t" \
    "uxth  r6, r3 \n\t" \
    "mov   r9, r6 \n\t" \
    "lsrs  r3, r3, #16 \n\t" \
    "uxth  r6, r4 \n\t" \
    "mov   r10, r6 \n\t" \
    "lsrs  r4, r4, #16 \n\t" \
    "uxth  r6, r5 \n\t" \
    "mov   r11, r6 \n\t" \
    "lsrs  r5, r5, #16 \n\t" \
    "add   r6, sp, #64 \n\t" \
    "stmia r6!, {r2, r3, r4, r5} \n\t" \
    /* Split left: A_E in r1-r4 and sp[96..108], A_O to sp[80..92]. */ \
    "ldm   r1, {r1, r2, r3, r4} \n\t" \
    "lsrs  r5, r1, #16 \n\t" \
    "uxth  r1, r1 \n\t" \
    "str   r5, [sp, #80] \n\t" \
    "lsrs  r5, r2, #16 \n\t" \
    "uxth  r2, r2 \n\t" \
    "str   r5, [sp, #84] \n\t" \
    "lsrs  r5, r3, #16 \n\t" \
    "uxth  r3, r3 \n\t" \
    "str   r5, [sp, #88] \n\t" \
    "lsrs  r5, r4, #16 \n\t" \
    "uxth  r4, r4 \n\t" \
    "str   r5, [sp, #92] \n\t" \
    "add   r0, sp, #96 \n\t" \
    "stmia r0!, {r1, r2, r3, r4} \n\t" \
    /* E[0..6] = A_E * B_E by columns: r5 = column sum, r6 = its carries, r7 = 0. */ \
    "movs  r7, #0 \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "str   r5, [sp, #0] \n\t" \
    "mov   r5, r9 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #4] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #8] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #12] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #16] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #20] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r4, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "str   r5, [sp, #24] \n\t" \
    /* O[0..6] = A_O * B_E. */ \
    "add   r0, sp, #80 \n\t" \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "str   r5, [sp, #32] \n\t" \
    "mov   r5, r9 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #36] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #40] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #44] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #48] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #52] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r4, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "str   r5, [sp, #56] \n\t" \
    /* Load B_O; E[1..7] += A_O * B_O (weight 2^32, so one word up). */ \
    "ldr   r0, [sp, #64] \n\t" \
    "mov   r8, r0 \n\t" \
    "ldr   r0, [sp, #68] \n\t" \
    "mov   r9, r0 \n\t" \
    "ldr   r0, [sp, #72] \n\t" \
    "mov   r10, r0 \n\t" \
    "ldr   r0, [sp, #76] \n\t" \
    "mov   r11, r0 \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #4] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #4] \n\t" \
    "mov   r5, r9 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #8] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #8] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #12] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #12] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #16] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #16] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #20] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #20] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #24] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #24] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r4, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "str   r5, [sp, #28] \n\t" \
    /* O[0..7] += A_E * B_O. */ \
    "add   r0, sp, #96 \n\t" \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #32] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #32] \n\t" \
    "mov   r5, r9 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #36] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #36] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #40] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #40] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #44] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #44] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #48] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #48] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #52] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #52] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r4, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #56] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #56] \n\t" \
    "str   r6, [sp, #60] \n\t" \
    /* Merge low 4 words: result[0..3] = E[0..3] + (O << 16). */ \
    "add   r0, sp, #32 \n\t" \
    "ldm   r0, {r0, r1, r2, r3} \n\t" \
    "lsls  r4, r0, #16 \n\t" \
    "lsrs  r0, r0, #16 \n\t" \
    "lsls  r5, r1, #16 \n\t" \
    "orrs  r5, r5, r0 \n\t" \
    "lsrs  r1, r1, #16 \n\t" \
    "lsls  r6, r2, #16 \n\t" \
    "orrs  r6, r6, r1 \n\t" \
    "lsrs  r2, r2, #16 \n\t" \
    "lsls  r7, r3, #16 \n\t" \
    "orrs  r7, r7, r2 \n\t" \
    "lsrs  r3, r3, #16 \n\t" \
    "mov   r8, r3 \n\t" \
    "mov   r0, sp \n\t" \
    "ldm   r0, {r0, r1, r2, r3} \n\t" \
    "adds  r0, r0, r4 \n\t" \
    "adcs  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r6 \n\t" \
    "adcs  r3, r3, r7 \n\t" \
    "mov   r4, r12 \n\t" \
    "stmia r4!, {r0, r1, r2, r3} \n\t" \
    "mov   r12, r4 \n\t" \
    /* Save the carry in r9 (lsrs below shifts it back into C); merge result[4..7]. */ \
    "movs  r0, #0 \n\t" \
    "adcs  r0, r0, r0 \n\t" \
    "mov   r9, r0 \n\t" \
    "add   r0, sp, #48 \n\t" \
    "ldm   r0, {r0, r1, r2, r3} \n\t" \
    "mov   r4, r8 \n\t" \
    "lsls  r5, r0, #16 \n\t" \
    "orrs  r4, r4, r5 \n\t" \
    "lsrs  r0, r0, #16 \n\t" \
    "lsls  r5, r1, #16 \n\t" \
    "orrs  r5, r5, r0 \n\t" \
    "lsrs  r1, r1, #16 \n\t" \
    "lsls  r6, r2, #16 \n\t" \
    "orrs  r6, r6, r1 \n\t" \
    "lsrs  r2, r2, #16 \n\t" \
    "lsls  r7, r3, #16 \n\t" \
    "orrs  r7, r7, r2 \n\t" \
    "add   r0, sp, #16 \n\t" \
    "ldm   r0, {r0, r1, r2, r3} \n\t" \
    "mov   r8, r0 \n\t" \
    "mov   r0, r9 \n\t" \
    "lsrs  r0, r0, #1 \n\t" \
    "mov   r0, r8 \n\t" \
    "adcs  r0, r0, r4 \n\t" \
    "adcs  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r6 \n\t" \
    "adcs  r3, r3, r7 \n\t" \
    "mov   r4, r12 \n\t" \
    "stmia r4!, {r0, r1, r2, r3} \n\t" \
    "add   sp, #112 \n\t" \
    "bx    lr \n\t"
#endif

#if uECC_SUPPORTS_secp160r1
/* result[0..9] = left[0..4] * right[0..4] using the E/O split. Each pass
   multiplies by B[0..3] (held in r8-r11) column by column, then adds a border
   row for B[4]. Stack: E at sp[0..36], O at sp[40..76], A_E at sp[80..96],
   A_O at sp[100..116], B_E at sp[120..136], B_O at sp[140..156].
   r4-r7 are pushed here rather than declared as clobbers (also in the 6-8 word
   versions below). */
#define FAST_MULT_THUMB1_ASM_5 \
    "push  {r4-r7} \n\t" \
    "sub   sp, #160 \n\t" \
    /* Split right into halves (B_E[0..3] also in r8-r11). */ \
    "mov   r12, r0 \n\t" \
    "movs  r0, r2 \n\t" \
    "adds  r0, #16 \n\t" \
    "ldmia r0!, {r3} \n\t" \
    "lsrs  r7, r3, #16 \n\t" \
    "str   r7, [sp, #156] \n\t" \
    "uxth  r3, r3 \n\t" \
    "str   r3, [sp, #136] \n\t" \
    "ldm   r2, {r2, r3, r4, r5} \n\t" \
    "uxth  r6, r2 \n\t" \
    "mov   r8, r6 \n\t" \
    "str   r6, [sp, #120] \n\t" \
    "lsrs  r2, r2, #16 \n\t" \
    "uxth  r6, r3 \n\t" \
    "mov   r9, r6 \n\t" \
    "str   r6, [sp, #124] \n\t" \
    "lsrs  r3, r3, #16 \n\t" \
    "uxth  r6, r4 \n\t" \
    "mov   r10, r6 \n\t" \
    "str   r6, [sp, #128] \n\t" \
    "lsrs  r4, r4, #16 \n\t" \
    "uxth  r6, r5 \n\t" \
    "mov   r11, r6 \n\t" \
    "str   r6, [sp, #132] \n\t" \
    "lsrs  r5, r5, #16 \n\t" \
    "add   r6, sp, #140 \n\t" \
    "stmia r6!, {r2, r3, r4, r5} \n\t" \
    /* Split left into halves (A_E[0..3] in r1-r4). */ \
    "movs  r0, r1 \n\t" \
    "adds  r0, #16 \n\t" \
    "ldmia r0!, {r2} \n\t" \
    "lsrs  r6, r2, #16 \n\t" \
    "str   r6, [sp, #116] \n\t" \
    "uxth  r2, r2 \n\t" \
    "str   r2, [sp, #96] \n\t" \
    "ldm   r1, {r1, r2, r3, r4} \n\t" \
    "lsrs  r5, r1, #16 \n\t" \
    "str   r5, [sp, #100] \n\t" \
    "uxth  r1, r1 \n\t" \
    "lsrs  r5, r2, #16 \n\t" \
    "str   r5, [sp, #104] \n\t" \
    "uxth  r2, r2 \n\t" \
    "lsrs  r5, r3, #16 \n\t" \
    "str   r5, [sp, #108] \n\t" \
    "uxth  r3, r3 \n\t" \
    "lsrs  r5, r4, #16 \n\t" \
    "str   r5, [sp, #112] \n\t" \
    "uxth  r4, r4 \n\t" \
    "add   r0, sp, #80 \n\t" \
    "stmia r0!, {r1, r2, r3, r4} \n\t" \
    /* E[0..7] = A_E * B_E[0..3] by columns: r5 = column sum, r6 = its carries, r7 = 0. */ \
    "movs  r7, #0 \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "str   r5, [sp, #0] \n\t" \
    "mov   r5, r9 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #4] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #8] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #12] \n\t" \
    "ldr   r1, [sp, #96] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r1, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #16] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r1, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #20] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r4, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r1, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #24] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "str   r5, [sp, #28] \n\t" \
    /* O[0..7] = A_O * B_E[0..3]. */ \
    "add   r0, sp, #100 \n\t" \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "str   r5, [sp, #40] \n\t" \
    "mov   r5, r9 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #44] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #48] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #52] \n\t" \
    "ldr   r1, [sp, #116] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r1, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #56] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r1, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #60] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r4, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r1, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #64] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "str   r5, [sp, #68] \n\t" \
    /* Border row: E[4..8] += A_E * B_E[4]. */ \
    "ldr   r0, [sp, #136] \n\t" \
    "mov   r8, r0 \n\t" \
    "add   r0, sp, #80 \n\t" \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #16] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #16] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #20] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #20] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #24] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #24] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r4, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #28] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #28] \n\t" \
    "ldr   r1, [sp, #96] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "str   r5, [sp, #32] \n\t" \
    /* O[4..8] += A_O * B_E[4]. */ \
    "add   r0, sp, #100 \n\t" \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #56] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #56] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #60] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #60] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #64] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #64] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r4, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #68] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #68] \n\t" \
    "ldr   r1, [sp, #116] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "str   r5, [sp, #72] \n\t" \
    /* Load B_O[0..3]; E[1..9] += A_O * B_O[0..3]. */ \
    "ldr   r0, [sp, #140] \n\t" \
    "mov   r8, r0 \n\t" \
    "ldr   r0, [sp, #144] \n\t" \
    "mov   r9, r0 \n\t" \
    "ldr   r0, [sp, #148] \n\t" \
    "mov   r10, r0 \n\t" \
    "ldr   r0, [sp, #152] \n\t" \
    "mov   r11, r0 \n\t" \
    "add   r0, sp, #100 \n\t" \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #4] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #4] \n\t" \
    "mov   r5, r9 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #8] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #8] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #12] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #12] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #16] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #16] \n\t" \
    "ldr   r1, [sp, #116] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r1, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #20] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #20] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r1, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #24] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #24] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r4, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r1, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #28] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #28] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #32] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #32] \n\t" \
    "str   r6, [sp, #36] \n\t" \
    /* O[0..7] += A_E * B_O[0..3]; the carry out is parked in sp[76], added to O[8] below. */ \
    "add   r0, sp, #80 \n\t" \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #40] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #40] \n\t" \
    "mov   r5, r9 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #44] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #44] \n\t" \
    "mov   r5, r10 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #48] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #48] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r2, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #52] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #52] \n\t" \
    "ldr   r1, [sp, #96] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r3, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r8 \n\t" \
    "muls  r0, r1, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #56] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #56] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r4, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r0, r9 \n\t" \
    "muls  r0, r1, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #60] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #60] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r4, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, r10 \n\t" \
    "muls  r0, r1, r0 \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #64] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #64] \n\t" \
    "mov   r5, r11 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #68] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #68] \n\t" \
    "str   r6, [sp, #76] \n\t" \
    /* Border row: E[5..9] += A_O * B_O[4] (old E[9] held in lr). */ \
    "ldr   r0, [sp, #156] \n\t" \
    "mov   r8, r0 \n\t" \
    "ldr   r0, [sp, #36] \n\t" \
    "mov   lr, r0 \n\t" \
    "add   r0, sp, #100 \n\t" \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #20] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #20] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #24] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #24] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #28] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #28] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r4, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #32] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #32] \n\t" \
    "ldr   r1, [sp, #116] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    /* E[9] is the top word: no carry out. */ \
    "mov   r0, lr \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "str   r5, [sp, #36] \n\t" \
    /* O[4..9] += A_E * B_O[4], adding the parked carry (lr) into O[8]. */ \
    "ldr   r0, [sp, #76] \n\t" \
    "mov   lr, r0 \n\t" \
    "add   r0, sp, #80 \n\t" \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #56] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #56] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r2, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #60] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #60] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r3, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #64] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #64] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r4, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "ldr   r0, [sp, #68] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #68] \n\t" \
    "ldr   r1, [sp, #96] \n\t" \
    "mov   r5, r8 \n\t" \
    "muls  r5, r1, r5 \n\t" \
    "adds  r5, r5, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "mov   r0, lr \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "ldr   r0, [sp, #72] \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #72] \n\t" \
    "str   r6, [sp, #76] \n\t" \
    /* Merge result = E + (O << 16) in groups of 4, 4, 2 words (carry kept in r9). */ \
    "add   r0, sp, #40 \n\t" \
    "ldm   r0, {r0, r1, r2, r3} \n\t" \
    "lsls  r4, r0, #16 \n\t" \
    "lsrs  r0, r0, #16 \n\t" \
    "lsls  r5, r1, #16 \n\t" \
    "orrs  r5, r5, r0 \n\t" \
    "lsrs  r1, r1, #16 \n\t" \
    "lsls  r6, r2, #16 \n\t" \
    "orrs  r6, r6, r1 \n\t" \
    "lsrs  r2, r2, #16 \n\t" \
    "lsls  r7, r3, #16 \n\t" \
    "orrs  r7, r7, r2 \n\t" \
    "lsrs  r3, r3, #16 \n\t" \
    "mov   r8, r3 \n\t" \
    "mov   r0, sp \n\t" \
    "ldm   r0, {r0, r1, r2, r3} \n\t" \
    "adds  r0, r0, r4 \n\t" \
    "adcs  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r6 \n\t" \
    "adcs  r3, r3, r7 \n\t" \
    "mov   r4, r12 \n\t" \
    "stmia r4!, {r0, r1, r2, r3} \n\t" \
    "mov   r12, r4 \n\t" \
    "movs  r0, #0 \n\t" \
    "adcs  r0, r0, r0 \n\t" \
    "mov   r9, r0 \n\t" \
    "add   r0, sp, #56 \n\t" \
    "ldm   r0, {r0, r1, r2, r3} \n\t" \
    "mov   r4, r8 \n\t" \
    "lsls  r5, r0, #16 \n\t" \
    "orrs  r4, r4, r5 \n\t" \
    "lsrs  r0, r0, #16 \n\t" \
    "lsls  r5, r1, #16 \n\t" \
    "orrs  r5, r5, r0 \n\t" \
    "lsrs  r1, r1, #16 \n\t" \
    "lsls  r6, r2, #16 \n\t" \
    "orrs  r6, r6, r1 \n\t" \
    "lsrs  r2, r2, #16 \n\t" \
    "lsls  r7, r3, #16 \n\t" \
    "orrs  r7, r7, r2 \n\t" \
    "lsrs  r3, r3, #16 \n\t" \
    "mov   r8, r3 \n\t" \
    "add   r0, sp, #16 \n\t" \
    "ldm   r0, {r0, r1, r2, r3} \n\t" \
    "mov   r10, r0 \n\t" \
    "mov   r0, r9 \n\t" \
    "lsrs  r0, r0, #1 \n\t" \
    "mov   r0, r10 \n\t" \
    "adcs  r0, r0, r4 \n\t" \
    "adcs  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r6 \n\t" \
    "adcs  r3, r3, r7 \n\t" \
    "mov   r4, r12 \n\t" \
    "stmia r4!, {r0, r1, r2, r3} \n\t" \
    "mov   r12, r4 \n\t" \
    "movs  r0, #0 \n\t" \
    "adcs  r0, r0, r0 \n\t" \
    "mov   r9, r0 \n\t" \
    "add   r0, sp, #72 \n\t" \
    "ldm   r0, {r0, r1} \n\t" \
    "mov   r4, r8 \n\t" \
    "lsls  r5, r0, #16 \n\t" \
    "orrs  r4, r4, r5 \n\t" \
    "lsrs  r0, r0, #16 \n\t" \
    "lsls  r5, r1, #16 \n\t" \
    "orrs  r5, r5, r0 \n\t" \
    "add   r0, sp, #32 \n\t" \
    "ldm   r0, {r0, r1} \n\t" \
    "mov   r8, r0 \n\t" \
    "mov   r0, r9 \n\t" \
    "lsrs  r0, r0, #1 \n\t" \
    "mov   r0, r8 \n\t" \
    "adcs  r0, r0, r4 \n\t" \
    "adcs  r1, r1, r5 \n\t" \
    "mov   r4, r12 \n\t" \
    "stmia r4!, {r0, r1} \n\t" \
    "add   sp, #160 \n\t" \
    "pop   {r4-r7} \n\t"
#endif

#if (uECC_SUPPORTS_secp160r1 || uECC_SUPPORTS_secp192r1)
/* result[0..11] = left[0..5] * right[0..5] using subtractive Karatsuba with
   3x3 leaves. With L = a_lo * b_lo, H = a_hi * b_hi, M = |a_lo - a_hi| * |b_lo - b_hi|:
     result = L + (L + H -/+ M) * 2^96 + H * 2^192,
   subtracting M when (a_lo - a_hi) and (b_lo - b_hi) have the same sign.
   Stack: |a_lo - a_hi| at sp[0], |b_lo - b_hi| at sp[16] (M overwrites both),
   combined sign mask at sp[32], saved r0-r2 at sp[36..44]. */
#define FAST_MULT_THUMB1_ASM_6 \
    "push  {r4-r7} \n\t" \
    "sub   sp, #48 \n\t" \
    "str   r0, [sp, #36] \n\t" \
    "str   r1, [sp, #40] \n\t" \
    "str   r2, [sp, #44] \n\t" \
    "mov   r9, r2 \n\t" \
    /* sp[0..2] = |a_lo - a_hi|: sbcs r0, r0, r0 turns the borrow into a 0/-1 mask, */ \
    /* which then conditionally negates via eor/sub. r10 = mask. */ \
    "ldmia r1!, {r4, r5, r6} \n\t" \
    "ldm   r1, {r0, r1, r2} \n\t" \
    "subs  r4, r4, r0 \n\t" \
    "sbcs  r5, r5, r1 \n\t" \
    "sbcs  r6, r6, r2 \n\t" \
    "sbcs  r0, r0, r0 \n\t" \
    "eors  r4, r0 \n\t" \
    "eors  r5, r0 \n\t" \
    "eors  r6, r0 \n\t" \
    "subs  r4, r4, r0 \n\t" \
    "sbcs  r5, r5, r0 \n\t" \
    "sbcs  r6, r6, r0 \n\t" \
    "mov   r1, sp \n\t" \
    "stmia r1!, {r4, r5, r6} \n\t" \
    "mov   r10, r0 \n\t" \
    /* sp[16..18] = |b_lo - b_hi|; sp[32] = xor of the two sign masks. */ \
    "mov   r2, r9 \n\t" \
    "ldmia r2!, {r4, r5, r6} \n\t" \
    "ldm   r2, {r0, r1, r2} \n\t" \
    "subs  r4, r4, r0 \n\t" \
    "sbcs  r5, r5, r1 \n\t" \
    "sbcs  r6, r6, r2 \n\t" \
    "sbcs  r0, r0, r0 \n\t" \
    "eors  r4, r0 \n\t" \
    "eors  r5, r0 \n\t" \
    "eors  r6, r0 \n\t" \
    "subs  r4, r4, r0 \n\t" \
    "sbcs  r5, r5, r0 \n\t" \
    "sbcs  r6, r6, r0 \n\t" \
    "add   r1, sp, #16 \n\t" \
    "stmia r1!, {r4, r5, r6} \n\t" \
    "mov   r1, r10 \n\t" \
    "eors  r0, r1 \n\t" \
    "str   r0, [sp, #32] \n\t" \
    /* M = sp[0..5] (in place), L = result[0..5], H = result[6..11]. */ \
    "mov   r0, sp \n\t" \
    "mov   r1, sp \n\t" \
    "add   r2, sp, #16 \n\t" \
    "bl    uECC_vli_mul3_thumb1 \n\t" \
    "ldr   r0, [sp, #36] \n\t" \
    "ldr   r1, [sp, #40] \n\t" \
    "ldr   r2, [sp, #44] \n\t" \
    "bl    uECC_vli_mul3_thumb1 \n\t" \
    "ldr   r0, [sp, #36] \n\t" \
    "adds  r0, #24 \n\t" \
    "ldr   r1, [sp, #40] \n\t" \
    "adds  r1, #12 \n\t" \
    "ldr   r2, [sp, #44] \n\t" \
    "adds  r2, #12 \n\t" \
    "bl    uECC_vli_mul3_thumb1 \n\t" \
    /* T = L_hi + H_lo (carry in r9); result[3..5] = L_lo + T. */ \
    "ldr   r0, [sp, #36] \n\t" \
    "movs  r7, r0 \n\t" \
    "adds  r7, #12 \n\t" \
    "ldmia r7!, {r1, r2, r3} \n\t" \
    "ldmia r7!, {r4, r5, r6} \n\t" \
    "adds  r1, r1, r4 \n\t" \
    "adcs  r2, r2, r5 \n\t" \
    "adcs  r3, r3, r6 \n\t" \
    "movs  r4, #0 \n\t" \
    "adcs  r4, r4, r4 \n\t" \
    "mov   r9, r4 \n\t" \
    "ldmia r0!, {r4, r5, r6} \n\t" \
    "adds  r4, r4, r1 \n\t" \
    "adcs  r5, r5, r2 \n\t" \
    "adcs  r6, r6, r3 \n\t" \
    "stmia r0!, {r4, r5, r6} \n\t" \
    /* result[6..8] = T + H_hi + carries; r9 = carry into result[9..11]. */ \
    "ldmia r7!, {r4, r5, r6} \n\t" \
    "adcs  r1, r1, r4 \n\t" \
    "adcs  r2, r2, r5 \n\t" \
    "adcs  r3, r3, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "adcs  r6, r6, r6 \n\t" \
    "mov   r5, r9 \n\t" \
    "adds  r6, r6, r5 \n\t" \
    "movs  r7, #0 \n\t" \
    "adds  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "adcs  r3, r3, r7 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r9, r6 \n\t" \
    "stmia r0!, {r1, r2, r3} \n\t" \
    /* result[3..8] -/+= M: add ~M + 1 if the mask is 0, else add M (lsrs sets the carry-in). */ \
    "subs  r0, #24 \n\t" \
    "mov   r8, r0 \n\t" \
    "mov   r5, sp \n\t" \
    "ldr   r7, [sp, #32] \n\t" \
    "mvns  r7, r7 \n\t" \
    "mov   r10, r7 \n\t" \
    "lsrs  r6, r7, #1 \n\t" \
    "ldmia r0!, {r1, r2, r3} \n\t" \
    "mov   r0, r10 \n\t" \
    "ldmia r5!, {r4, r6, r7} \n\t" \
    "eors  r4, r0 \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "adcs  r1, r1, r4 \n\t" \
    "adcs  r2, r2, r6 \n\t" \
    "adcs  r3, r3, r7 \n\t" \
    "mov   r6, r8 \n\t" \
    "stmia r6!, {r1, r2, r3} \n\t" \
    "mov   r8, r6 \n\t" \
    "ldmia r6!, {r1, r2, r3} \n\t" \
    "ldmia r5!, {r4, r6, r7} \n\t" \
    "eors  r4, r0 \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "adcs  r1, r1, r4 \n\t" \
    "adcs  r2, r2, r6 \n\t" \
    "adcs  r3, r3, r7 \n\t" \
    "mov   r6, r8 \n\t" \
    "stmia r6!, {r1, r2, r3} \n\t" \
    /* Add the carry and the sign extension of +/-M (r0) into result[9..11]. */ \
    "mov   r5, r9 \n\t" \
    "adcs  r5, r5, r0 \n\t" \
    "mov   r0, r6 \n\t" \
    "ldmia r0!, {r1, r2, r3} \n\t" \
    "movs  r7, #0 \n\t" \
    "adds  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "adcs  r3, r3, r7 \n\t" \
    "stmia r6!, {r1, r2, r3} \n\t" \
    "add   sp, #48 \n\t" \
    "pop   {r4-r7} \n\t"
#endif

#if uECC_SUPPORTS_secp224r1
/* result[0..13] = left[0..6] * right[0..6]. Same Karatsuba as
   FAST_MULT_THUMB1_ASM_6 with a 4 + 3 word split (a_hi, b_hi zero-extended):
   M and L use the 4x4 leaf and H uses the 3x3 leaf. */
#define FAST_MULT_THUMB1_ASM_7 \
    "push  {r4-r7} \n\t" \
    "sub   sp, #48 \n\t" \
    "str   r0, [sp, #36] \n\t" \
    "str   r1, [sp, #40] \n\t" \
    "str   r2, [sp, #44] \n\t" \
    "mov   r9, r2 \n\t" \
    /* sp[0..3] = |a_lo - a_hi|; r10 = sign mask. */ \
    "ldmia r1!, {r4, r5, r6, r7} \n\t" \
    "ldm   r1, {r0, r1, r2} \n\t" \
    "movs  r3, #0 \n\t" \
    "subs  r4, r4, r0 \n\t" \
    "sbcs  r5, r5, r1 \n\t" \
    "sbcs  r6, r6, r2 \n\t" \
    "sbcs  r7, r7, r3 \n\t" \
    "sbcs  r0, r0, r0 \n\t" \
    "eors  r4, r0 \n\t" \
    "eors  r5, r0 \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "subs  r4, r4, r0 \n\t" \
    "sbcs  r5, r5, r0 \n\t" \
    "sbcs  r6, r6, r0 \n\t" \
    "sbcs  r7, r7, r0 \n\t" \
    "mov   r1, sp \n\t" \
    "stmia r1!, {r4, r5, r6, r7} \n\t" \
    "mov   r10, r0 \n\t" \
    /* sp[16..19] = |b_lo - b_hi| (r3 is still 0); sp[32] = xor of the two sign masks. */ \
    "mov   r2, r9 \n\t" \
    "ldmia r2!, {r4, r5, r6, r7} \n\t" \
    "ldm   r2, {r0, r1, r2} \n\t" \
    "subs  r4, r4, r0 \n\t" \
    "sbcs  r5, r5, r1 \n\t" \
    "sbcs  r6, r6, r2 \n\t" \
    "sbcs  r7, r7, r3 \n\t" \
    "sbcs  r0, r0, r0 \n\t" \
    "eors  r4, r0 \n\t" \
    "eors  r5, r0 \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "subs  r4, r4, r0 \n\t" \
    "sbcs  r5, r5, r0 \n\t" \
    "sbcs  r6, r6, r0 \n\t" \
    "sbcs  r7, r7, r0 \n\t" \
    "add   r1, sp, #16 \n\t" \
    "stmia r1!, {r4, r5, r6, r7} \n\t" \
    "mov   r1, r10 \n\t" \
    "eors  r0, r1 \n\t" \
    "str   r0, [sp, #32] \n\t" \
    /* M = sp[0..7] (in place), L = result[0..7], H = result[8..13]. */ \
    "mov   r0, sp \n\t" \
    "mov   r1, sp \n\t" \
    "add   r2, sp, #16 \n\t" \
    "bl    uECC_vli_mul4_thumb1 \n\t" \
    "ldr   r0, [sp, #36] \n\t" \
    "ldr   r1, [sp, #40] \n\t" \
    "ldr   r2, [sp, #44] \n\t" \
    "bl    uECC_vli_mul4_thumb1 \n\t" \
    "ldr   r0, [sp, #36] \n\t" \
    "adds  r0, #32 \n\t" \
    "ldr   r1, [sp, #40] \n\t" \
    "adds  r1, #16 \n\t" \
    "ldr   r2, [sp, #44] \n\t" \
    "adds  r2, #16 \n\t" \
    "bl    uECC_vli_mul3_thumb1 \n\t" \
    /* T = L_hi + H_lo (carry in r9), written to result[8..11]; result[4..7] = L_lo + T. */ \
    "ldr   r0, [sp, #36] \n\t" \
    "movs  r5, r0 \n\t" \
    "adds  r5, #16 \n\t" \
    "ldmia r5!, {r1, r2, r3, r4} \n\t" \
    "ldmia r5!, {r6, r7} \n\t" \
    "adds  r1, r1, r6 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "ldmia r5!, {r6, r7} \n\t" \
    "adcs  r3, r3, r6 \n\t" \
    "adcs  r4, r4, r7 \n\t" \
    "movs  r6, #0 \n\t" \
    "adcs  r6, r6, r6 \n\t" \
    "mov   r9, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "movs  r7, r0 \n\t" \
    "adds  r7, #32 \n\t" \
    "stmia r7!, {r1, r2, r3, r4} \n\t" \
    "ldmia r0!, {r5, r7} \n\t" \
    "adds  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "ldmia r0!, {r5, r7} \n\t" \
    "adcs  r3, r3, r5 \n\t" \
    "adcs  r4, r4, r7 \n\t" \
    "stmia r0!, {r1, r2, r3, r4} \n\t" \
    /* result[8..11] = T + H_hi + carries; r9 = carry into result[12..13]. */ \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "ldmia r0!, {r5, r7} \n\t" \
    "adcs  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "adcs  r3, r3, r6 \n\t" \
    "adcs  r4, r4, r6 \n\t" \
    "adcs  r6, r6, r6 \n\t" \
    "mov   r5, r9 \n\t" \
    "adds  r6, r6, r5 \n\t" \
    "movs  r7, #0 \n\t" \
    "adds  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "adcs  r3, r3, r7 \n\t" \
    "adcs  r4, r4, r7 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r9, r6 \n\t" \
    "subs  r0, #24 \n\t" \
    "stmia r0!, {r1, r2, r3, r4} \n\t" \
    /* result[4..11] -/+= M, as in FAST_MULT_THUMB1_ASM_6. */ \
    "subs  r0, #32 \n\t" \
    "mov   r8, r0 \n\t" \
    "mov   r5, sp \n\t" \
    "ldr   r7, [sp, #32] \n\t" \
    "mvns  r7, r7 \n\t" \
    "mov   r10, r7 \n\t" \
    "lsrs  r6, r7, #1 \n\t" \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "mov   r0, r10 \n\t" \
    "ldmia r5!, {r6, r7} \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "adcs  r1, r1, r6 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "ldmia r5!, {r6, r7} \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "adcs  r3, r3, r6 \n\t" \
    "adcs  r4, r4, r7 \n\t" \
    "mov   r6, r8 \n\t" \
    "stmia r6!, {r1, r2, r3, r4} \n\t" \
    "mov   r8, r6 \n\t" \
    "ldmia r6!, {r1, r2, r3, r4} \n\t" \
    "ldmia r5!, {r6, r7} \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "adcs  r1, r1, r6 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "ldmia r5!, {r6, r7} \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "adcs  r3, r3, r6 \n\t" \
    "adcs  r4, r4, r7 \n\t" \
    "mov   r6, r8 \n\t" \
    "stmia r6!, {r1, r2, r3, r4} \n\t" \
    /* Add the carry and the sign extension of +/-M (r0) into result[12..13]. */ \
    "mov   r5, r9 \n\t" \
    "adcs  r5, r5, r0 \n\t" \
    "mov   r0, r6 \n\t" \
    "ldmia r0!, {r1, r2} \n\t" \
    "movs  r7, #0 \n\t" \
    "adds  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "stmia r6!, {r1, r2} \n\t" \
    "add   sp, #48 \n\t" \
    "pop   {r4-r7} \n\t"
#endif

#if (uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1)
/* result[0..15] = left[0..7] * right[0..7]. Same Karatsuba as
   FAST_MULT_THUMB1_ASM_6 with a 4 + 4 word split and 4x4 leaves. */
#define FAST_MULT_THUMB1_ASM_8 \
    "push  {r4-r7} \n\t" \
    "sub   sp, #48 \n\t" \
    "str   r0, [sp, #36] \n\t" \
    "str   r1, [sp, #40] \n\t" \
    "str   r2, [sp, #44] \n\t" \
    "mov   r9, r2 \n\t" \
    /* sp[0..3] = |a_lo - a_hi|; r10 = sign mask. */ \
    "ldmia r1!, {r4, r5, r6, r7} \n\t" \
    "ldm   r1, {r0, r1, r2, r3} \n\t" \
    "subs  r4, r4, r0 \n\t" \
    "sbcs  r5, r5, r1 \n\t" \
    "sbcs  r6, r6, r2 \n\t" \
    "sbcs  r7, r7, r3 \n\t" \
    "sbcs  r0, r0, r0 \n\t" \
    "eors  r4, r0 \n\t" \
    "eors  r5, r0 \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "subs  r4, r4, r0 \n\t" \
    "sbcs  r5, r5, r0 \n\t" \
    "sbcs  r6, r6, r0 \n\t" \
    "sbcs  r7, r7, r0 \n\t" \
    "mov   r1, sp \n\t" \
    "stmia r1!, {r4, r5, r6, r7} \n\t" \
    "mov   r10, r0 \n\t" \
    /* sp[16..19] = |b_lo - b_hi|; sp[32] = xor of the two sign masks. */ \
    "mov   r2, r9 \n\t" \
    "ldmia r2!, {r4, r5, r6, r7} \n\t" \
    "ldm   r2, {r0, r1, r2, r3} \n\t" \
    "subs  r4, r4, r0 \n\t" \
    "sbcs  r5, r5, r1 \n\t" \
    "sbcs  r6, r6, r2 \n\t" \
    "sbcs  r7, r7, r3 \n\t" \
    "sbcs  r0, r0, r0 \n\t" \
    "eors  r4, r0 \n\t" \
    "eors  r5, r0 \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "subs  r4, r4, r0 \n\t" \
    "sbcs  r5, r5, r0 \n\t" \
    "sbcs  r6, r6, r0 \n\t" \
    "sbcs  r7, r7, r0 \n\t" \
    "add   r1, sp, #16 \n\t" \
    "stmia r1!, {r4, r5, r6, r7} \n\t" \
    "mov   r1, r10 \n\t" \
    "eors  r0, r1 \n\t" \
    "str   r0, [sp, #32] \n\t" \
    /* M = sp[0..7] (in place), L = result[0..7], H = result[8..15]. */ \
    "mov   r0, sp \n\t" \
    "mov   r1, sp \n\t" \
    "add   r2, sp, #16 \n\t" \
    "bl    uECC_vli_mul4_thumb1 \n\t" \
    "ldr   r0, [sp, #36] \n\t" \
    "ldr   r1, [sp, #40] \n\t" \
    "ldr   r2, [sp, #44] \n\t" \
    "bl    uECC_vli_mul4_thumb1 \n\t" \
    "ldr   r0, [sp, #36] \n\t" \
    "adds  r0, #32 \n\t" \
    "ldr   r1, [sp, #40] \n\t" \
    "adds  r1, #16 \n\t" \
    "ldr   r2, [sp, #44] \n\t" \
    "adds  r2, #16 \n\t" \
    "bl    uECC_vli_mul4_thumb1 \n\t" \
    /* T = L_hi + H_lo (carry in r9), written to result[8..11]; result[4..7] = L_lo + T. */ \
    "ldr   r0, [sp, #36] \n\t" \
    "movs  r5, r0 \n\t" \
    "adds  r5, #16 \n\t" \
    "ldmia r5!, {r1, r2, r3, r4} \n\t" \
    "ldmia r5!, {r6, r7} \n\t" \
    "adds  r1, r1, r6 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "ldmia r5!, {r6, r7} \n\t" \
    "adcs  r3, r3, r6 \n\t" \
    "adcs  r4, r4, r7 \n\t" \
    "movs  r6, #0 \n\t" \
    "adcs  r6, r6, r6 \n\t" \
    "mov   r9, r6 \n\t" \
    "movs  r6, #0 \n\t" \
    "movs  r7, r0 \n\t" \
    "adds  r7, #32 \n\t" \
    "stmia r7!, {r1, r2, r3, r4} \n\t" \
    "ldmia r0!, {r5, r7} \n\t" \
    "adds  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "ldmia r0!, {r5, r7} \n\t" \
    "adcs  r3, r3, r5 \n\t" \
    "adcs  r4, r4, r7 \n\t" \
    "stmia r0!, {r1, r2, r3, r4} \n\t" \
    /* result[8..11] = T + H_hi + carries; r9 = carry into result[12..15]. */ \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "ldmia r0!, {r5, r7} \n\t" \
    "adcs  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "ldmia r0!, {r5, r7} \n\t" \
    "adcs  r3, r3, r5 \n\t" \
    "adcs  r4, r4, r7 \n\t" \
    "adcs  r6, r6, r6 \n\t" \
    "mov   r5, r9 \n\t" \
    "adds  r6, r6, r5 \n\t" \
    "movs  r7, #0 \n\t" \
    "adds  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "adcs  r3, r3, r7 \n\t" \
    "adcs  r4, r4, r7 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "mov   r9, r6 \n\t" \
    "subs  r0, #32 \n\t" \
    "stmia r0!, {r1, r2, r3, r4} \n\t" \
    /* result[4..11] -/+= M, as in FAST_MULT_THUMB1_ASM_6. */ \
    "subs  r0, #32 \n\t" \
    "mov   r8, r0 \n\t" \
    "mov   r5, sp \n\t" \
    "ldr   r7, [sp, #32] \n\t" \
    "mvns  r7, r7 \n\t" \
    "mov   r10, r7 \n\t" \
    "lsrs  r6, r7, #1 \n\t" \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "mov   r0, r10 \n\t" \
    "ldmia r5!, {r6, r7} \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "adcs  r1, r1, r6 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "ldmia r5!, {r6, r7} \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "adcs  r3, r3, r6 \n\t" \
    "adcs  r4, r4, r7 \n\t" \
    "mov   r6, r8 \n\t" \
    "stmia r6!, {r1, r2, r3, r4} \n\t" \
    "mov   r8, r6 \n\t" \
    "ldmia r6!, {r1, r2, r3, r4} \n\t" \
    "ldmia r5!, {r6, r7} \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "adcs  r1, r1, r6 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "ldmia r5!, {r6, r7} \n\t" \
    "eors  r6, r0 \n\t" \
    "eors  r7, r0 \n\t" \
    "adcs  r3, r3, r6 \n\t" \
    "adcs  r4, r4, r7 \n\t" \
    "mov   r6, r8 \n\t" \
    "stmia r6!, {r1, r2, r3, r4} \n\t" \
    /* Add the carry and the sign extension of +/-M (r0) into result[12..15]. */ \
    "mov   r5, r9 \n\t" \
    "adcs  r5, r5, r0 \n\t" \
    "mov   r0, r6 \n\t" \
    "ldmia r0!, {r1, r2, r3, r4} \n\t" \
    "movs  r7, #0 \n\t" \
    "adds  r1, r1, r5 \n\t" \
    "adcs  r2, r2, r7 \n\t" \
    "adcs  r3, r3, r7 \n\t" \
    "adcs  r4, r4, r7 \n\t" \
    "stmia r6!, {r1, r2, r3, r4} \n\t" \
    "add   sp, #48 \n\t" \
    "pop   {r4-r7} \n\t"
#endif

#else /* uECC_PLATFORM != uECC_arm_thumb */

/* ============================================================================
 * Thumb-2 / ARM without UMAAL (Cortex-M3 / ARMv7-M) Multiplication & Squaring
 *
 * Row-wise multiplication in passes of up to 4 multiplier words. Pass 1 holds
 * b[0..3] in r2-r5 and, for each a[i], adds a[i] * b[0..3] into a 4-word rolling
 * accumulator: the even products (a[i] * b0, a[i] * b2) in one adcs chain and the
 * odd products (a[i] * b1, a[i] * b3) in a second, shifted chain. umull does not
 * modify the flags, so it can be interleaved inside live carry chains (4 umull +
 * 9 adds per 4x1 row). Pass 2 repeats this for b[4..nw-1], adding in the pass-1
 * words from registers or memory.
 * Squaring computes the off-diagonal products a[i] * a[j] (i < j) the same way,
 * then doubles them and adds the diagonal squares a[i]^2.
 * ============================================================================ */

/* Pass 1, row 0: result[0] = lo(a[0] * b0); r8, r9, r10, r7 = columns 1..4. */
#define M3_PASS1_ROW0 \
    "ldr   r6, [r1], #4 \n\t" \
    "umull r7, r8, r6, r2 \n\t" \
    "str   r7, [r0], #4 \n\t" \
    "umull r9, r10, r6, r4 \n\t" \
    "umull r11, r12, r6, r3 \n\t" \
    "adds  r8, r8, r11 \n\t" \
    "adcs  r9, r9, r12 \n\t" \
    "umull r11, r12, r5, r6 \n\t" \
    "adcs  r10, r10, r11 \n\t" \
    "adc   r7, r12, #0 \n\t"

/* Pass 1 row: acc0..acc3 += a[i] * b[0..3] (a[i] loaded into r6). acc0 is
   complete and stored, then reused as the new top word, so callers rotate the
   accumulator arguments from row to row. */
#define M3_ROW_PASS1_R6(acc0, acc1, acc2, acc3) \
    "ldr   r6, [r1], #4 \n\t" \
    "umull r11, r12, r6, r2 \n\t" \
    "adds  " #acc0 ", " #acc0 ", r11 \n\t" \
    "str   " #acc0 ", [r0], #4 \n\t" \
    "adcs  " #acc1 ", " #acc1 ", r12 \n\t" \
    "umull r11, r12, r6, r4 \n\t" \
    "adcs  " #acc2 ", " #acc2 ", r11 \n\t" \
    "adcs  " #acc3 ", " #acc3 ", r12 \n\t" \
    "umull r11, " #acc0 ", r6, r5 \n\t" \
    "adc   " #acc0 ", " #acc0 ", #0 \n\t" \
    "umull r12, r6, r3, r6 \n\t" \
    "adds  " #acc1 ", " #acc1 ", r12 \n\t" \
    "adcs  " #acc2 ", " #acc2 ", r6 \n\t" \
    "adcs  " #acc3 ", " #acc3 ", r11 \n\t" \
    "adc   " #acc0 ", " #acc0 ", #0 \n\t"

/* Last pass-1 row: like M3_ROW_PASS1_R6, but acc0 is not stored and the new
   top word goes in r11 (clobbers b0 in r2). */
#define M3_ROW_PASS1_LAST_R11(acc0, acc1, acc2, acc3) \
    "ldr   r6, [r1], #4 \n\t" \
    "umull r11, r12, r6, r2 \n\t" \
    "adds  " #acc0 ", " #acc0 ", r11 \n\t" \
    "adcs  " #acc1 ", " #acc1 ", r12 \n\t" \
    "umull r11, r12, r6, r4 \n\t" \
    "adcs  " #acc2 ", " #acc2 ", r11 \n\t" \
    "adcs  " #acc3 ", " #acc3 ", r12 \n\t" \
    "umull r2, r11, r6, r5 \n\t" \
    "adc   r11, r11, #0 \n\t" \
    "umull r12, r6, r3, r6 \n\t" \
    "adds  " #acc1 ", " #acc1 ", r12 \n\t" \
    "adcs  " #acc2 ", " #acc2 ", r6 \n\t" \
    "adcs  " #acc3 ", " #acc3 ", r2 \n\t" \
    "adc   r11, r11, #0 \n\t"

/* Like M3_ROW_PASS1_R6, but acc0 stays in its register (not stored) and the
   new top word goes in c_hi. */
#define M3_ROW_PASS1_NOSTORE(acc0, acc1, acc2, acc3, c_hi) \
    "ldr   r6, [r1], #4 \n\t" \
    "umull r11, r12, r6, r2 \n\t" \
    "adds  " #acc0 ", " #acc0 ", r11 \n\t" \
    "adcs  " #acc1 ", " #acc1 ", r12 \n\t" \
    "umull r11, r12, r6, r4 \n\t" \
    "adcs  " #acc2 ", " #acc2 ", r11 \n\t" \
    "adcs  " #acc3 ", " #acc3 ", r12 \n\t" \
    "umull r11, " #c_hi ", r6, r5 \n\t" \
    "adc   " #c_hi ", " #c_hi ", #0 \n\t" \
    "umull r12, r6, r3, r6 \n\t" \
    "adds  " #acc1 ", " #acc1 ", r12 \n\t" \
    "adcs  " #acc2 ", " #acc2 ", r6 \n\t" \
    "adcs  " #acc3 ", " #acc3 ", r11 \n\t" \
    "adc   " #c_hi ", " #c_hi ", #0 \n\t"

/* Pass 2 row for 2 multiplier words (b4, b5 in r2, r3): acc0:acc1 += a[i] * b[4..5]
   + the pass-1 word w_reg. Stores acc0, which then becomes the new top word. */
#define M3_PASS2_2COL_REG(acc0, acc1, w_reg) \
    "ldr   r14, [r1], #4 \n\t" \
    "umull r7, r12, r14, r2 \n\t" \
    "adds  " #acc0 ", " #acc0 ", r7 \n\t" \
    "adcs  " #acc1 ", " #acc1 ", r12 \n\t" \
    "umull r7, r12, r14, r3 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "adds  " #acc0 ", " #acc0 ", " #w_reg " \n\t" \
    "str   " #acc0 ", [r0], #4 \n\t" \
    "adcs  " #acc1 ", " #acc1 ", r7 \n\t" \
    "adc   " #acc0 ", r12, #0 \n\t"

/* Pass 2 row for 3 multiplier words (r2-r4), adding the pass-1 word w_reg.
   Clobbers r5. */
#define M3_PASS2_3COL_REG(acc0, acc1, acc2, w_reg) \
    "ldr   r14, [r1], #4 \n\t" \
    "umull r11, r12, r14, r2 \n\t" \
    "adds  " #acc0 ", " #acc0 ", r11 \n\t" \
    "adcs  " #acc1 ", " #acc1 ", r12 \n\t" \
    "umull r11, r12, r14, r4 \n\t" \
    "adcs  " #acc2 ", " #acc2 ", r11 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "umull r5, r11, r14, r3 \n\t" \
    "adds  " #acc0 ", " #acc0 ", " #w_reg " \n\t" \
    "str   " #acc0 ", [r0], #4 \n\t" \
    "adcs  " #acc1 ", " #acc1 ", r5 \n\t" \
    "adcs  " #acc2 ", " #acc2 ", r11 \n\t" \
    "adc   " #acc0 ", r12, #0 \n\t"

/* Like M3_PASS2_3COL_REG, but the pass-1 word is read from result memory. */
#define M3_PASS2_3COL_MEM(acc0, acc1, acc2) \
    "ldr   r14, [r1], #4 \n\t" \
    "umull r11, r12, r14, r2 \n\t" \
    "adds  " #acc0 ", " #acc0 ", r11 \n\t" \
    "adcs  " #acc1 ", " #acc1 ", r12 \n\t" \
    "umull r11, r12, r14, r4 \n\t" \
    "adcs  " #acc2 ", " #acc2 ", r11 \n\t" \
    "ldr   r5, [r0] \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "adds  " #acc0 ", " #acc0 ", r5 \n\t" \
    "str   " #acc0 ", [r0], #4 \n\t" \
    "umull r5, r11, r14, r3 \n\t" \
    "adcs  " #acc1 ", " #acc1 ", r5 \n\t" \
    "adcs  " #acc2 ", " #acc2 ", r11 \n\t" \
    "adc   " #acc0 ", r12, #0 \n\t"

/* Pass 2 row for 4 multiplier words (b4..b7 in r2-r5); the pass-1 word is read
   from result memory. */
#define M3_ROW_PASS2_MEM(acc0, acc1, acc2, acc3) \
    "ldr   r14, [r1], #4 \n\t" \
    "umull r11, r12, r14, r2 \n\t" \
    "adds  " #acc0 ", " #acc0 ", r11 \n\t" \
    "adcs  " #acc1 ", " #acc1 ", r12 \n\t" \
    "umull r11, r12, r14, r4 \n\t" \
    "adcs  " #acc2 ", " #acc2 ", r11 \n\t" \
    "adcs  " #acc3 ", " #acc3 ", r12 \n\t" \
    "ldr   r6, [r0] \n\t" \
    "umull r11, r12, r14, r5 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "adds  " #acc0 ", " #acc0 ", r6 \n\t" \
    "str   " #acc0 ", [r0], #4 \n\t" \
    "umull r6, " #acc0 ", r3, r14 \n\t" \
    "adcs  " #acc1 ", " #acc1 ", r6 \n\t" \
    "adcs  " #acc2 ", " #acc2 ", " #acc0 " \n\t" \
    "adcs  " #acc3 ", " #acc3 ", r11 \n\t" \
    "adc   " #acc0 ", r12, #0 \n\t"

/* ---- Level 4 Standalone Multiplication Macros ---- */

/* nw = 5: pass 1 with b[0..3], then a[0..4] * b[4] (b[4] in r14) is added
   straight into the register accumulator (no intermediate memory traffic). */
#define FAST_MULT_ASM_5 \
    "ldr   r14, [r2, #16] \n\t" \
    "ldmia r2, {r2, r3, r4, r5} \n\t" \
    M3_PASS1_ROW0 \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
    M3_ROW_PASS1_LAST_R11(r7, r8, r9, r10) \
    /* Reload a[0..4]; add a[i] * b[4] (even i, then odd i) into columns 4..9. */ \
    "sub   r1, r1, #20 \n\t" \
    "ldmia r1, {r1, r3, r4, r5, r6} \n\t" \
    "umull r2, r12, r1, r14 \n\t" \
    "adds  r7, r7, r2 \n\t" \
    "adcs  r8, r8, r12 \n\t" \
    "umull r2, r12, r4, r14 \n\t" \
    "adcs  r9, r9, r2 \n\t" \
    "adcs  r10, r10, r12 \n\t" \
    "umull r2, r12, r6, r14 \n\t" \
    "adcs  r11, r11, r2 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "umull r1, r2, r3, r14 \n\t" \
    "adds  r8, r8, r1 \n\t" \
    "adcs  r9, r9, r2 \n\t" \
    "umull r1, r2, r5, r14 \n\t" \
    "adcs  r10, r10, r1 \n\t" \
    "adcs  r11, r11, r2 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "stmia r0, {r7, r8, r9, r10, r11, r12} \n\t"

/* nw = 6: pass 1 with b[0..3] keeps columns 4..9 in registers, then pass 2 with
   b[4..5] (no intermediate memory traffic). */
#define FAST_MULT_ASM_6 \
    "push  {r2} \n\t" \
    "ldmia r2, {r2, r3, r4, r5} \n\t" \
    M3_PASS1_ROW0 \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
    M3_ROW_PASS1_NOSTORE(r7, r8, r9, r10, r14) \
    M3_ROW_PASS1_LAST_R11(r8, r9, r10, r14) \
    /* Pass 2 with b4, b5 in r2, r3; pass-1 columns 4..9 are in r7-r10, r14 (-> r6), r11. */ \
    "pop   {r2} \n\t" \
    "ldr   r3, [r2, #20] \n\t" \
    "ldr   r2, [r2, #16] \n\t" \
    "sub   r1, r1, #24 \n\t" \
    "mov   r6, r14 \n\t" \
    "ldr   r14, [r1], #4 \n\t" \
    "umull r4, r5, r14, r2 \n\t" \
    "adds  r4, r4, r7 \n\t" \
    "str   r4, [r0], #4 \n\t" \
    "umull r4, r7, r14, r3 \n\t" \
    "adcs  r5, r5, r4 \n\t" \
    "adc   r4, r7, #0 \n\t" \
    M3_PASS2_2COL_REG(r5, r4, r8) \
    M3_PASS2_2COL_REG(r4, r5, r9) \
    M3_PASS2_2COL_REG(r5, r4, r10) \
    M3_PASS2_2COL_REG(r4, r5, r6) \
    M3_PASS2_2COL_REG(r5, r4, r11) \
    "stmia r0, {r4, r5} \n\t"

/* nw = 7: pass 1 with b[0..3] stores columns 0..8 (columns 9, 10 stay in r8, r9),
   then pass 2 with b[4..6]. */
#define FAST_MULT_ASM_7 \
    "push  {r2} \n\t" \
    "ldmia r2, {r2, r3, r4, r5} \n\t" \
    M3_PASS1_ROW0 \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
    M3_ROW_PASS1_R6(r7, r8, r9, r10) \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    /* Store columns 7, 8; rewind to result[4] and a[0] for pass 2 (b4..b6 in r2-r4). */ \
    "str   r10, [r0] \n\t" \
    "str   r7, [r0, #4] \n\t" \
    "sub   r0, r0, #12 \n\t" \
    "sub   r1, r1, #28 \n\t" \
    "pop   {r2} \n\t" \
    "add   r2, r2, #16 \n\t" \
    "ldmia r2, {r2, r3, r4} \n\t" \
    "ldr   r14, [r1], #4 \n\t" \
    "ldr   r5, [r0] \n\t" \
    "umull r11, r6, r14, r2 \n\t" \
    "umull r7, r10, r14, r4 \n\t" \
    "adds  r11, r11, r5 \n\t" \
    "str   r11, [r0], #4 \n\t" \
    "umull r11, r12, r14, r3 \n\t" \
    "adcs  r6, r6, r11 \n\t" \
    "adcs  r7, r7, r12 \n\t" \
    "adc   r10, r10, #0 \n\t" \
    M3_PASS2_3COL_MEM(r6, r7, r10) \
    M3_PASS2_3COL_MEM(r7, r10, r6) \
    M3_PASS2_3COL_MEM(r10, r6, r7) \
    M3_PASS2_3COL_MEM(r6, r7, r10) \
    M3_PASS2_3COL_REG(r7, r10, r6, r8) \
    M3_PASS2_3COL_REG(r10, r6, r7, r9) \
    "stmia r0, {r6, r7, r10} \n\t"

/* nw = 8: pass 1 with b[0..3], then pass 2 with b[4..7] adding into result[4..]. */
#define FAST_MULT_ASM_8 \
    "add   r14, r2, #16 \n\t" \
    "ldmia r2, {r2, r3, r4, r5} \n\t" \
    M3_PASS1_ROW0 \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
    M3_ROW_PASS1_R6(r7, r8, r9, r10) \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
    /* Store columns 8..11; rewind to result[4] and a[0] for pass 2 (b4..b7 in r2-r5). */ \
    "stmia r0, {r7, r8, r9, r10} \n\t" \
    "sub   r0, r0, #16 \n\t" \
    "sub   r1, r1, #32 \n\t" \
    "ldmia r14, {r2, r3, r4, r5} \n\t" \
    "ldr   r14, [r1], #4 \n\t" \
    "ldr   r6, [r0] \n\t" \
    "umull r7, r8, r14, r2 \n\t" \
    "umull r9, r10, r14, r4 \n\t" \
    "adds  r7, r7, r6 \n\t" \
    "str   r7, [r0], #4 \n\t" \
    "umull r11, r12, r14, r3 \n\t" \
    "adcs  r8, r8, r11 \n\t" \
    "adcs  r9, r9, r12 \n\t" \
    "umull r11, r12, r14, r5 \n\t" \
    "adcs  r10, r10, r11 \n\t" \
    "adc   r7, r12, #0 \n\t" \
    M3_ROW_PASS2_MEM(r8, r9, r10, r7) \
    M3_ROW_PASS2_MEM(r9, r10, r7, r8) \
    M3_ROW_PASS2_MEM(r10, r7, r8, r9) \
    M3_ROW_PASS2_MEM(r7, r8, r9, r10) \
    M3_ROW_PASS2_MEM(r8, r9, r10, r7) \
    M3_ROW_PASS2_MEM(r9, r10, r7, r8) \
    M3_ROW_PASS2_MEM(r10, r7, r8, r9) \
    "stmia r0, {r7, r8, r9, r10} \n\t"

/* ---- Level 3 Shared-Pass Multiplication Macro ---- */
/* Level 3: a single macro for all sizes (num_words in r3, moved to r14). Shares
   pass 1 as far as possible, then branches to a size-specific tail. Every path
   ends at local label 1, which the caller defines. */
#define FAST_MULT_ASM_LEVEL3 \
    "push  {r2} \n\t" \
    "mov   r14, r3 \n\t" \
    "ldmia r2, {r2, r3, r4, r5} \n\t" \
    M3_PASS1_ROW0 \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
    M3_ROW_PASS1_R6(r7, r8, r9, r10) \
    "cmp   r14, #5 \n\t" \
    "beq   50f \n\t" \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    "cmp   r14, #6 \n\t" \
    "beq   60f \n\t" \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    "cmp   r14, #7 \n\t" \
    "beq   70f \n\t" \
    /* nw == 8 */ \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
    "stmia r0, {r7, r8, r9, r10} \n\t" \
    "sub   r0, r0, #16 \n\t" \
    "sub   r1, r1, #32 \n\t" \
    "pop   {r2} \n\t" \
    "add   r2, r2, #16 \n\t" \
    "ldmia r2, {r2, r3, r4, r5} \n\t" \
    "ldr   r14, [r1], #4 \n\t" \
    "ldr   r6, [r0] \n\t" \
    "umull r7, r8, r14, r2 \n\t" \
    "umull r9, r10, r14, r4 \n\t" \
    "adds  r7, r7, r6 \n\t" \
    "str   r7, [r0], #4 \n\t" \
    "umull r11, r12, r14, r3 \n\t" \
    "adcs  r8, r8, r11 \n\t" \
    "adcs  r9, r9, r12 \n\t" \
    "umull r11, r12, r14, r5 \n\t" \
    "adcs  r10, r10, r11 \n\t" \
    "adc   r7, r12, #0 \n\t" \
    M3_ROW_PASS2_MEM(r8, r9, r10, r7) \
    M3_ROW_PASS2_MEM(r9, r10, r7, r8) \
    M3_ROW_PASS2_MEM(r10, r7, r8, r9) \
    M3_ROW_PASS2_MEM(r7, r8, r9, r10) \
    M3_ROW_PASS2_MEM(r8, r9, r10, r7) \
    M3_ROW_PASS2_MEM(r9, r10, r7, r8) \
    M3_ROW_PASS2_MEM(r10, r7, r8, r9) \
    "stmia r0, {r7, r8, r9, r10} \n\t" \
    "b     1f \n\t" \
    /* nw = 5: column 4 was stored by the shared rows; reload it and add a[0..4] * b[4]. */ \
    "50: \n\t" \
    "pop   {r2} \n\t" \
    "ldr   r14, [r2, #16] \n\t" \
    "ldr   r11, [r0, #-4]! \n\t" \
    "sub   r1, r1, #20 \n\t" \
    "ldmia r1, {r1, r3, r4, r5, r6} \n\t" \
    "umull r2, r12, r1, r14 \n\t" \
    "adds  r11, r11, r2 \n\t" \
    "str   r11, [r0], #4 \n\t" \
    "adcs  r8, r8, r12 \n\t" \
    "umull r2, r12, r4, r14 \n\t" \
    "adcs  r9, r9, r2 \n\t" \
    "adcs  r10, r10, r12 \n\t" \
    "umull r2, r11, r6, r14 \n\t" \
    "adcs  r7, r7, r2 \n\t" \
    "adc   r11, r11, #0 \n\t" \
    "umull r1, r2, r3, r14 \n\t" \
    "adds  r8, r8, r1 \n\t" \
    "adcs  r9, r9, r2 \n\t" \
    "umull r1, r2, r5, r14 \n\t" \
    "adcs  r10, r10, r1 \n\t" \
    "adcs  r7, r7, r2 \n\t" \
    "adc   r11, r11, #0 \n\t" \
    "stmia r0!, {r8, r9, r10} \n\t" \
    "stmia r0, {r7, r11} \n\t" \
    "b     1f \n\t" \
    /* nw = 6: reload stored columns 4, 5 and run pass 2 with b4, b5. */ \
    "60: \n\t" \
    "pop   {r2} \n\t" \
    "ldr   r3, [r2, #20] \n\t" \
    "ldr   r2, [r2, #16] \n\t" \
    "sub   r1, r1, #24 \n\t" \
    "ldmdb r0!, {r6, r11} \n\t" \
    "ldr   r14, [r1], #4 \n\t" \
    "umull r4, r5, r14, r2 \n\t" \
    "adds  r4, r4, r6 \n\t" \
    "str   r4, [r0], #4 \n\t" \
    "umull r4, r6, r14, r3 \n\t" \
    "adcs  r5, r5, r4 \n\t" \
    "adc   r4, r6, #0 \n\t" \
    "mov   r6, r7 \n\t" \
    M3_PASS2_2COL_REG(r5, r4, r11) \
    M3_PASS2_2COL_REG(r4, r5, r9) \
    M3_PASS2_2COL_REG(r5, r4, r10) \
    M3_PASS2_2COL_REG(r4, r5, r6) \
    M3_PASS2_2COL_REG(r5, r4, r8) \
    "stmia r0, {r4, r5} \n\t" \
    "b     1f \n\t" \
    /* nw = 7: same as FAST_MULT_ASM_7. */ \
    "70: \n\t" \
    "str   r10, [r0] \n\t" \
    "str   r7, [r0, #4] \n\t" \
    "sub   r0, r0, #12 \n\t" \
    "sub   r1, r1, #28 \n\t" \
    "pop   {r2} \n\t" \
    "add   r2, r2, #16 \n\t" \
    "ldmia r2, {r2, r3, r4} \n\t" \
    "ldr   r14, [r1], #4 \n\t" \
    "ldr   r5, [r0] \n\t" \
    "umull r11, r6, r14, r2 \n\t" \
    "umull r7, r10, r14, r4 \n\t" \
    "adds  r11, r11, r5 \n\t" \
    "str   r11, [r0], #4 \n\t" \
    "umull r11, r12, r14, r3 \n\t" \
    "adcs  r6, r6, r11 \n\t" \
    "adcs  r7, r7, r12 \n\t" \
    "adc   r10, r10, #0 \n\t" \
    M3_PASS2_3COL_MEM(r6, r7, r10) \
    M3_PASS2_3COL_MEM(r7, r10, r6) \
    M3_PASS2_3COL_MEM(r10, r6, r7) \
    M3_PASS2_3COL_MEM(r6, r7, r10) \
    M3_PASS2_3COL_REG(r7, r10, r6, r8) \
    M3_PASS2_3COL_REG(r10, r6, r7, r9) \
    "stmia r0, {r6, r7, r10} \n\t"

/* ---- Level 4 Standalone & Level 3 Shared-Prefix Squaring Macros ---- */

/* Squaring step 1: load a[0..3] into r2-r5 and compute their off-diagonal
   products. Stores columns 1..3 to result[1..3] (result[0] is written later)
   and leaves columns 4..7 in r8, r9, r10, r7, ready for M3_ROW_PASS1_R6 rows
   with b[0..3] = a[0..3]. mov (not movs) keeps the carry flag live. */
#define M3_SQR_STEP1 \
    "ldmia r1!, {r2, r3, r4, r5} \n\t" \
    "umull r6, r7, r2, r3 \n\t" \
    "umull r8, r9, r2, r5 \n\t" \
    "umull r10, r11, r4, r5 \n\t" \
    "umull r12, r14, r2, r4 \n\t" \
    "adds  r7, r7, r12 \n\t" \
    "adcs  r8, r8, r14 \n\t" \
    "umull r12, r14, r3, r5 \n\t" \
    "adcs  r9, r9, r12 \n\t" \
    "adcs  r10, r10, r14 \n\t" \
    "adc   r11, r11, #0 \n\t" \
    "umull r12, r14, r3, r4 \n\t" \
    "adds  r8, r8, r12 \n\t" \
    "add   r0, r0, #4 \n\t" \
    "stmia r0!, {r6, r7, r8} \n\t" \
    "mov   r7, #0 \n\t" \
    "adcs  r8, r9, r14 \n\t" \
    "adcs  r9, r10, #0 \n\t" \
    "adcs  r10, r11, #0 \n\t" \
    "adc   r7, r7, #0 \n\t"

/* nw = 5: off-diagonal products (step 1, then row a[4] * a[0..3]), then double
   them and add the squares. */
#define FAST_SQUARE_ASM_5 \
    M3_SQR_STEP1 \
    /* Row a[4] * a[0..3] gives columns 4..8 in r7..r11; a[4] is saved on the stack. */ \
    "ldr   r6, [r1] \n\t" \
    "push  {r6} \n\t" \
    "mov   r11, r7 \n\t" \
    "umull r1, r12, r6, r2 \n\t" \
    "adds  r7, r8, r1 \n\t" \
    "adcs  r8, r9, r12 \n\t" \
    "umull r1, r12, r6, r4 \n\t" \
    "adcs  r9, r10, r1 \n\t" \
    "adcs  r10, r11, r12 \n\t" \
    "umull r1, r11, r6, r5 \n\t" \
    "adc   r11, r11, #0 \n\t" \
    "umull r12, r14, r3, r6 \n\t" \
    "adds  r8, r8, r12 \n\t" \
    "adcs  r9, r9, r14 \n\t" \
    "adcs  r10, r10, r1 \n\t" \
    "adc   r11, r11, #0 \n\t" \
    /* result[0] = lo(a0^2); reload columns 1..3 and double columns 1..8 (top carry in r1). */ \
    "umull r6, r2, r2, r2 \n\t" \
    "str   r6, [r0, #-16] \n\t" \
    "ldmdb r0!, {r6, r12, r14} \n\t" \
    "movs  r1, #0 \n\t" \
    "adds  r6, r6, r6 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "adcs  r14, r14, r14 \n\t" \
    "adcs  r7, r7, r7 \n\t" \
    "adcs  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adc   r1, r1, #0 \n\t" \
    /* Add the diagonal squares a[0..4]^2. */ \
    "adds  r6, r6, r2 \n\t" \
    "umull r2, r3, r3, r3 \n\t" \
    "adcs  r12, r12, r2 \n\t" \
    "adcs  r14, r14, r3 \n\t" \
    "stmia r0!, {r6, r12, r14} \n\t" \
    "umull r2, r4, r4, r4 \n\t" \
    "adcs  r7, r7, r2 \n\t" \
    "adcs  r8, r8, r4 \n\t" \
    "umull r2, r5, r5, r5 \n\t" \
    "adcs  r9, r9, r2 \n\t" \
    "adcs  r10, r10, r5 \n\t" \
    "pop   {r3} \n\t" \
    "umull r2, r12, r3, r3 \n\t" \
    "adcs  r11, r11, r2 \n\t" \
    "adc   r12, r12, r1 \n\t" \
    "stmia r0, {r7, r8, r9, r10, r11, r12} \n\t"

/* nw = 6: off-diagonal products (step 1, rows a[4], a[5], then a[4] * a[5]), then
   double and add the squares in two halves. */
#define FAST_SQUARE_ASM_6 \
    M3_SQR_STEP1 \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    /* Add a[4] * a[5] at column 9; store columns 6..10. */ \
    "ldmdb r1!, {r11, r12} \n\t" \
    "mov   r6, r10 \n\t" \
    "umull r10, r14, r11, r12 \n\t" \
    "adds  r9, r9, r10 \n\t" \
    "adc   r10, r14, #0 \n\t" \
    "stmia r0, {r6, r7, r8, r9, r10} \n\t" \
    /* result[0..5]: double columns 1..5 and add a[0..2]^2; carry (0..2) in r2. */ \
    "sub   r0, r0, #24 \n\t" \
    "add   r14, r0, #4 \n\t" \
    "ldmia r14, {r6, r7, r8, r9, r10} \n\t" \
    "umull r1, r14, r2, r2 \n\t" \
    "str   r1, [r0], #4 \n\t" \
    "adds  r6, r6, r6 \n\t" \
    "adcs  r7, r7, r7 \n\t" \
    "adcs  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "mov   r2, #0 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  r6, r6, r14 \n\t" \
    "umull r3, r14, r3, r3 \n\t" \
    "adcs  r7, r7, r3 \n\t" \
    "adcs  r8, r8, r14 \n\t" \
    "umull r4, r14, r4, r4 \n\t" \
    "adcs  r9, r9, r4 \n\t" \
    "adcs  r10, r10, r14 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "stmia r0!, {r6, r7, r8, r9, r10} \n\t" \
    /* result[6..11]: double columns 6..10 and add a[3..5]^2 plus the carry. */ \
    /* r2 <= 2 and lo(a^2) is never 2 or 3 mod 4, so add r1, r1, r2 cannot overflow. */ \
    "ldmia r0, {r6, r7, r8, r9, r10} \n\t" \
    "umull r1, r14, r5, r5 \n\t" \
    "add   r1, r1, r2 \n\t" \
    "adds  r6, r6, r6 \n\t" \
    "adcs  r7, r7, r7 \n\t" \
    "adcs  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "mov   r2, #0 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  r6, r6, r1 \n\t" \
    "adcs  r7, r7, r14 \n\t" \
    "umull r3, r14, r11, r11 \n\t" \
    "adcs  r8, r8, r3 \n\t" \
    "adcs  r9, r9, r14 \n\t" \
    "umull r4, r11, r12, r12 \n\t" \
    "adcs  r10, r10, r4 \n\t" \
    "adc   r11, r11, r2 \n\t" \
    "stmia r0, {r6, r7, r8, r9, r10, r11} \n\t"

/* nw = 7: off-diagonal products (step 1, rows a[4..6], then the products among
   a[4..6]), then double and add the squares in two halves. */
#define FAST_SQUARE_ASM_7 \
    M3_SQR_STEP1 \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
    /* Add a[4] * a[5], a[4] * a[6], a[5] * a[6]; store columns 7..12. */ \
    "ldmdb r1, {r2, r3, r4} \n\t" \
    "umull r11, r12, r2, r3 \n\t" \
    "adds  r9, r9, r11 \n\t" \
    "adcs  r10, r10, r12 \n\t" \
    "umull r11, r12, r3, r4 \n\t" \
    "adcs  r11, r11, #0 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "umull r5, r14, r2, r4 \n\t" \
    "adds  r10, r10, r5 \n\t" \
    "adcs  r11, r11, r14 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "stmia r0, {r7, r8, r9, r10, r11, r12} \n\t" \
    /* result[0..7]: double columns 1..7 and add a[0..3]^2; carry in r2. */ \
    "sub   r0, r0, #28 \n\t" \
    "sub   r1, r1, #28 \n\t" \
    "ldmia r1!, {r2, r3, r4, r5} \n\t" \
    "add   r14, r0, #4 \n\t" \
    "ldmia r14, {r6, r7, r8, r9, r10, r11, r12} \n\t" \
    "umull r2, r14, r2, r2 \n\t" \
    "str   r2, [r0], #4 \n\t" \
    "adds  r6, r6, r6 \n\t" \
    "adcs  r7, r7, r7 \n\t" \
    "adcs  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "mov   r2, #0 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  r6, r6, r14 \n\t" \
    "umull r3, r14, r3, r3 \n\t" \
    "adcs  r7, r7, r3 \n\t" \
    "adcs  r8, r8, r14 \n\t" \
    "umull r4, r14, r4, r4 \n\t" \
    "adcs  r9, r9, r4 \n\t" \
    "adcs  r10, r10, r14 \n\t" \
    "umull r5, r14, r5, r5 \n\t" \
    "adcs  r11, r11, r5 \n\t" \
    "adcs  r12, r12, r14 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "stmia r0!, {r6, r7, r8, r9, r10, r11, r12} \n\t" \
    /* result[8..13]: double columns 8..12 and add a[4..6]^2 plus the carry. */ \
    "ldmia r1, {r1, r3, r4} \n\t" \
    "ldmia r0, {r6, r7, r8, r9, r10} \n\t" \
    "umull r1, r14, r1, r1 \n\t" \
    "add   r1, r1, r2 \n\t" \
    "adds  r6, r6, r6 \n\t" \
    "adcs  r7, r7, r7 \n\t" \
    "adcs  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "mov   r2, #0 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  r6, r6, r1 \n\t" \
    "adcs  r7, r7, r14 \n\t" \
    "umull r3, r14, r3, r3 \n\t" \
    "adcs  r8, r8, r3 \n\t" \
    "adcs  r9, r9, r14 \n\t" \
    "umull r4, r11, r4, r4 \n\t" \
    "adcs  r10, r10, r4 \n\t" \
    "adc   r11, r11, r2 \n\t" \
    "stmia r0, {r6, r7, r8, r9, r10, r11} \n\t"

/* nw = 8: off-diagonal products (step 1, rows a[4..7], then the products among
   a[4..7]), then double and add the squares in two halves. */
#define FAST_SQUARE_ASM_8 \
    M3_SQR_STEP1 \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
    M3_ROW_PASS1_R6(r7, r8, r9, r10) \
    /* Store column 8; add the products among a[4..7] and store columns 9..14. */ \
    "str   r8, [r0], #4 \n\t" \
    "ldmdb r1, {r2, r3, r4, r5} \n\t" \
    "umull r6, r8, r2, r3 \n\t" \
    "adds  r6, r6, r9 \n\t" \
    "adcs  r8, r8, r10 \n\t" \
    "umull r9, r10, r2, r5 \n\t" \
    "adcs  r9, r9, r7 \n\t" \
    "umull r11, r12, r4, r5 \n\t" \
    "adcs  r10, r10, #0 \n\t" \
    "adcs  r11, r11, #0 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "umull r7, r14, r2, r4 \n\t" \
    "adds  r8, r8, r7 \n\t" \
    "adcs  r9, r9, r14 \n\t" \
    "umull r7, r14, r3, r5 \n\t" \
    "adcs  r10, r10, r7 \n\t" \
    "adcs  r11, r11, r14 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "umull r7, r14, r3, r4 \n\t" \
    "adds  r9, r9, r7 \n\t" \
    "adcs  r10, r10, r14 \n\t" \
    "adcs  r11, r11, #0 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "stmia r0, {r6, r8, r9, r10, r11, r12} \n\t" \
    /* result[0..7]: double columns 1..7 and add a[0..3]^2; carry in r2. */ \
    "sub   r0, r0, #36 \n\t" \
    "sub   r1, r1, #32 \n\t" \
    "ldmia r1!, {r2, r3, r4, r5} \n\t" \
    "add   r14, r0, #4 \n\t" \
    "ldmia r14, {r6, r7, r8, r9, r10, r11, r12} \n\t" \
    "umull r2, r14, r2, r2 \n\t" \
    "str   r2, [r0], #4 \n\t" \
    "adds  r6, r6, r6 \n\t" \
    "adcs  r7, r7, r7 \n\t" \
    "adcs  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "mov   r2, #0 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  r6, r6, r14 \n\t" \
    "umull r3, r14, r3, r3 \n\t" \
    "adcs  r7, r7, r3 \n\t" \
    "adcs  r8, r8, r14 \n\t" \
    "umull r4, r14, r4, r4 \n\t" \
    "adcs  r9, r9, r4 \n\t" \
    "adcs  r10, r10, r14 \n\t" \
    "umull r5, r14, r5, r5 \n\t" \
    "adcs  r11, r11, r5 \n\t" \
    "adcs  r12, r12, r14 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "stmia r0!, {r6, r7, r8, r9, r10, r11, r12} \n\t" \
    /* result[8..15]: double columns 8..14 and add a[4..7]^2 plus the carry. */ \
    "ldmia r1, {r1, r3, r4, r5} \n\t" \
    "ldmia r0, {r6, r7, r8, r9, r10, r11, r12} \n\t" \
    "umull r1, r14, r1, r1 \n\t" \
    "add   r1, r1, r2 \n\t" \
    "adds  r6, r6, r6 \n\t" \
    "adcs  r7, r7, r7 \n\t" \
    "adcs  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "mov   r2, #0 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  r6, r6, r1 \n\t" \
    "adcs  r7, r7, r14 \n\t" \
    "umull r3, r14, r3, r3 \n\t" \
    "adcs  r8, r8, r3 \n\t" \
    "adcs  r9, r9, r14 \n\t" \
    "umull r4, r14, r4, r4 \n\t" \
    "adcs  r10, r10, r4 \n\t" \
    "adcs  r11, r11, r14 \n\t" \
    "umull r5, r14, r5, r5 \n\t" \
    "adcs  r12, r12, r5 \n\t" \
    "adc   r14, r14, r2 \n\t" \
    "stmia r0, {r6, r7, r8, r9, r10, r11, r12, r14} \n\t"

/* Level 3: a single macro for all sizes (num_words in r2, kept on the stack).
   Shares step 1, the pass-1 rows and (for nw = 7, 8) the first doubling half.
   Every path ends at local label 1, which the caller defines. */
#define FAST_SQUARE_ASM_LEVEL3 \
    "push  {r2} \n\t" \
    M3_SQR_STEP1 \
    "ldr   r14, [sp] \n\t" \
    "cmp   r14, #5 \n\t" \
    "beq   50f \n\t" \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    "ldr   r14, [sp] \n\t" \
    "cmp   r14, #6 \n\t" \
    "beq   60f \n\t" \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
    "ldr   r14, [sp] \n\t" \
    "cmp   r14, #7 \n\t" \
    "beq   70f \n\t" \
    /* nw == 8 */ \
    M3_ROW_PASS1_R6(r7, r8, r9, r10) \
    "str   r8, [r0], #4 \n\t" \
    "ldmdb r1, {r2, r3, r4, r5} \n\t" \
    "umull r6, r8, r2, r3 \n\t" \
    "adds  r6, r6, r9 \n\t" \
    "adcs  r8, r8, r10 \n\t" \
    "umull r9, r10, r2, r5 \n\t" \
    "adcs  r9, r9, r7 \n\t" \
    "umull r11, r12, r4, r5 \n\t" \
    "adcs  r10, r10, #0 \n\t" \
    "adcs  r11, r11, #0 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "umull r7, r14, r2, r4 \n\t" \
    "adds  r8, r8, r7 \n\t" \
    "adcs  r9, r9, r14 \n\t" \
    "umull r7, r14, r3, r5 \n\t" \
    "adcs  r10, r10, r7 \n\t" \
    "adcs  r11, r11, r14 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "umull r7, r14, r3, r4 \n\t" \
    "adds  r9, r9, r7 \n\t" \
    "adcs  r10, r10, r14 \n\t" \
    "adcs  r11, r11, #0 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "stmia r0, {r6, r8, r9, r10, r11, r12} \n\t" \
    "sub   r0, r0, #36 \n\t" \
    "sub   r1, r1, #32 \n\t" \
    "b     78f \n\t" \
    /* nw = 7: products among a[4..6], then join the nw = 8 path. */ \
    "70: \n\t" \
    "ldmdb r1, {r2, r3, r4} \n\t" \
    "umull r11, r12, r2, r3 \n\t" \
    "adds  r9, r9, r11 \n\t" \
    "adcs  r10, r10, r12 \n\t" \
    "umull r11, r12, r3, r4 \n\t" \
    "adcs  r11, r11, #0 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "umull r5, r14, r2, r4 \n\t" \
    "adds  r10, r10, r5 \n\t" \
    "adcs  r11, r11, r14 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "stmia r0, {r7, r8, r9, r10, r11, r12} \n\t" \
    "sub   r0, r0, #28 \n\t" \
    "sub   r1, r1, #28 \n\t" \
    /* nw = 7, 8: result[0..7] = 2 * columns 1..7 + a[0..3]^2; carry in r2. */ \
    "78: \n\t" \
    "ldmia r1!, {r2, r3, r4, r5} \n\t" \
    "add   r14, r0, #4 \n\t" \
    "ldmia r14, {r6, r7, r8, r9, r10, r11, r12} \n\t" \
    "umull r2, r14, r2, r2 \n\t" \
    "str   r2, [r0], #4 \n\t" \
    "adds  r6, r6, r6 \n\t" \
    "adcs  r7, r7, r7 \n\t" \
    "adcs  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "mov   r2, #0 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  r6, r6, r14 \n\t" \
    "umull r3, r14, r3, r3 \n\t" \
    "adcs  r7, r7, r3 \n\t" \
    "adcs  r8, r8, r14 \n\t" \
    "umull r4, r14, r4, r4 \n\t" \
    "adcs  r9, r9, r4 \n\t" \
    "adcs  r10, r10, r14 \n\t" \
    "umull r5, r14, r5, r5 \n\t" \
    "adcs  r11, r11, r5 \n\t" \
    "adcs  r12, r12, r14 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "stmia r0!, {r6, r7, r8, r9, r10, r11, r12} \n\t" \
    "pop   {r14} \n\t" \
    "cmp   r14, #7 \n\t" \
    "beq   79f \n\t" \
    /* nw = 8: result[8..15]. */ \
    "ldmia r1, {r1, r3, r4, r5} \n\t" \
    "ldmia r0, {r6, r7, r8, r9, r10, r11, r12} \n\t" \
    "umull r1, r14, r1, r1 \n\t" \
    "add   r1, r1, r2 \n\t" \
    "adds  r6, r6, r6 \n\t" \
    "adcs  r7, r7, r7 \n\t" \
    "adcs  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "mov   r2, #0 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  r6, r6, r1 \n\t" \
    "adcs  r7, r7, r14 \n\t" \
    "umull r3, r14, r3, r3 \n\t" \
    "adcs  r8, r8, r3 \n\t" \
    "adcs  r9, r9, r14 \n\t" \
    "umull r4, r14, r4, r4 \n\t" \
    "adcs  r10, r10, r4 \n\t" \
    "adcs  r11, r11, r14 \n\t" \
    "umull r5, r14, r5, r5 \n\t" \
    "adcs  r12, r12, r5 \n\t" \
    "adc   r14, r14, r2 \n\t" \
    "stmia r0, {r6, r7, r8, r9, r10, r11, r12, r14} \n\t" \
    "b     1f \n\t" \
    "79: \n\t" \
    /* nw = 7: result[8..13]. */ \
    "ldmia r1, {r1, r3, r4} \n\t" \
    "ldmia r0, {r6, r7, r8, r9, r10} \n\t" \
    "umull r1, r14, r1, r1 \n\t" \
    "add   r1, r1, r2 \n\t" \
    "adds  r6, r6, r6 \n\t" \
    "adcs  r7, r7, r7 \n\t" \
    "adcs  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "mov   r2, #0 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  r6, r6, r1 \n\t" \
    "adcs  r7, r7, r14 \n\t" \
    "umull r3, r14, r3, r3 \n\t" \
    "adcs  r8, r8, r3 \n\t" \
    "adcs  r9, r9, r14 \n\t" \
    "umull r4, r11, r4, r4 \n\t" \
    "adcs  r10, r10, r4 \n\t" \
    "adc   r11, r11, r2 \n\t" \
    "stmia r0, {r6, r7, r8, r9, r10, r11} \n\t" \
    "b     1f \n\t" \
    "60: \n\t" \
    /* nw = 6: drop the saved num_words; same as FAST_SQUARE_ASM_6. */ \
    "add   sp, sp, #4 \n\t" \
    "ldmdb r1!, {r11, r12} \n\t" \
    "mov   r6, r10 \n\t" \
    "umull r10, r14, r11, r12 \n\t" \
    "adds  r9, r9, r10 \n\t" \
    "adc   r10, r14, #0 \n\t" \
    "stmia r0, {r6, r7, r8, r9, r10} \n\t" \
    "sub   r0, r0, #24 \n\t" \
    "add   r14, r0, #4 \n\t" \
    "ldmia r14, {r6, r7, r8, r9, r10} \n\t" \
    "umull r1, r14, r2, r2 \n\t" \
    "str   r1, [r0], #4 \n\t" \
    "adds  r6, r6, r6 \n\t" \
    "adcs  r7, r7, r7 \n\t" \
    "adcs  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "mov   r2, #0 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  r6, r6, r14 \n\t" \
    "umull r3, r14, r3, r3 \n\t" \
    "adcs  r7, r7, r3 \n\t" \
    "adcs  r8, r8, r14 \n\t" \
    "umull r4, r14, r4, r4 \n\t" \
    "adcs  r9, r9, r4 \n\t" \
    "adcs  r10, r10, r14 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "stmia r0!, {r6, r7, r8, r9, r10} \n\t" \
    "ldmia r0, {r6, r7, r8, r9, r10} \n\t" \
    "umull r1, r14, r5, r5 \n\t" \
    "add   r1, r1, r2 \n\t" \
    "adds  r6, r6, r6 \n\t" \
    "adcs  r7, r7, r7 \n\t" \
    "adcs  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "mov   r2, #0 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  r6, r6, r1 \n\t" \
    "adcs  r7, r7, r14 \n\t" \
    "umull r3, r14, r11, r11 \n\t" \
    "adcs  r8, r8, r3 \n\t" \
    "adcs  r9, r9, r14 \n\t" \
    "umull r4, r11, r12, r12 \n\t" \
    "adcs  r10, r10, r4 \n\t" \
    "adc   r11, r11, r2 \n\t" \
    "stmia r0, {r6, r7, r8, r9, r10, r11} \n\t" \
    "b     1f \n\t" \
    "50: \n\t" \
    /* nw = 5: same as FAST_SQUARE_ASM_5, reusing the num_words slot to save a[4]. */ \
    "ldr   r6, [r1] \n\t" \
    "str   r6, [sp] \n\t" \
    "mov   r11, r7 \n\t" \
    "umull r1, r12, r6, r2 \n\t" \
    "adds  r7, r8, r1 \n\t" \
    "adcs  r8, r9, r12 \n\t" \
    "umull r1, r12, r6, r4 \n\t" \
    "adcs  r9, r10, r1 \n\t" \
    "adcs  r10, r11, r12 \n\t" \
    "umull r1, r11, r6, r5 \n\t" \
    "adc   r11, r11, #0 \n\t" \
    "umull r12, r14, r3, r6 \n\t" \
    "adds  r8, r8, r12 \n\t" \
    "adcs  r9, r9, r14 \n\t" \
    "adcs  r10, r10, r1 \n\t" \
    "adc   r11, r11, #0 \n\t" \
    "umull r6, r2, r2, r2 \n\t" \
    "str   r6, [r0, #-16] \n\t" \
    "ldmdb r0!, {r6, r12, r14} \n\t" \
    "movs  r1, #0 \n\t" \
    "adds  r6, r6, r6 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "adcs  r14, r14, r14 \n\t" \
    "adcs  r7, r7, r7 \n\t" \
    "adcs  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adc   r1, r1, #0 \n\t" \
    "adds  r6, r6, r2 \n\t" \
    "umull r2, r3, r3, r3 \n\t" \
    "adcs  r12, r12, r2 \n\t" \
    "adcs  r14, r14, r3 \n\t" \
    "stmia r0!, {r6, r12, r14} \n\t" \
    "umull r2, r4, r4, r4 \n\t" \
    "adcs  r7, r7, r2 \n\t" \
    "adcs  r8, r8, r4 \n\t" \
    "umull r2, r5, r5, r5 \n\t" \
    "adcs  r9, r9, r2 \n\t" \
    "adcs  r10, r10, r5 \n\t" \
    "pop   {r3} \n\t" \
    "umull r2, r12, r3, r3 \n\t" \
    "adcs  r11, r11, r2 \n\t" \
    "adc   r12, r12, r1 \n\t" \
    "stmia r0, {r7, r8, r9, r10, r11, r12} \n\t"

#endif /* uECC_PLATFORM == uECC_arm_thumb */

#endif /* _UECC_ASM_ARM_MULT_SQUARE_H_ */
