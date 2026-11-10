/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#ifndef _UECC_ASM_ARM_MULT_SQUARE_H_
#define _UECC_ASM_ARM_MULT_SQUARE_H_

#if (uECC_PLATFORM == uECC_arm_thumb)

/* ============================================================================
 * Thumb-1 (ARMv6-M / Cortex-M0/M0+) Level 4 Unrolled & Karatsuba Multiplication
 *
 * By splitting each 32-bit word into even (low 16-bit) and odd (high 16-bit)
 * halfwords:
 *   A = A_E + (A_O << 16),  B = B_E + (B_O << 16)
 * where A_E[i], B_E[i], A_O[i], B_O[i] < 2^16, all 16x16 -> 32-bit products
 * accumulate on a 32-bit word grid (stride 2 halfwords) using native 1-cycle
 * 32-bit muls + adds/adcs without per-product 16-bit shifts, followed by a
 * single linear merge pass: result = E + (O << 16).
 *
 * - For 160-bit (5-word) multiplication: direct unrolled Even/Odd split.
 * - For 192-bit (6-word), 224-bit (7-word), and 256-bit (8-word) multiplication:
 *   1-level subtractive Karatsuba calling shared unrolled 3x3-word (mul3) and
 *   4x4-word (mul4) Even/Odd leaf subroutines.
 * ============================================================================ */

#if (uECC_SUPPORTS_secp160r1 || uECC_SUPPORTS_secp192r1 || uECC_SUPPORTS_secp224r1)
#define FAST_MULT_THUMB1_MUL3_BODY \
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
#define FAST_MULT_THUMB1_MUL4_BODY \
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
#define FAST_MULT_THUMB1_ASM_5 \
    "push  {r4-r7} \n\t" \
    "sub   sp, #160 \n\t" \
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
    "movs  r6, #0 \n\t" \
    "mov   r0, lr \n\t" \
    "adds  r5, r5, r0 \n\t" \
    "adcs  r6, r6, r7 \n\t" \
    "str   r5, [sp, #36] \n\t" \
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
#define FAST_MULT_THUMB1_ASM_6 \
    "push  {r4-r7} \n\t" \
    "sub   sp, #48 \n\t" \
    "str   r0, [sp, #36] \n\t" \
    "str   r1, [sp, #40] \n\t" \
    "str   r2, [sp, #44] \n\t" \
    "mov   r9, r2 \n\t" \
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
    "mov   r5, r9 \n\t" \
    "adcs  r5, r5, r0 \n\t" \
    "mov   r0, r6 \n\t" \
    "mov   r6, r0 \n\t" \
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
#define FAST_MULT_THUMB1_ASM_7 \
    "push  {r4-r7} \n\t" \
    "sub   sp, #48 \n\t" \
    "str   r0, [sp, #36] \n\t" \
    "str   r1, [sp, #40] \n\t" \
    "str   r2, [sp, #44] \n\t" \
    "mov   r9, r2 \n\t" \
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
    "mov   r2, r9 \n\t" \
    "ldmia r2!, {r4, r5, r6, r7} \n\t" \
    "ldm   r2, {r0, r1, r2} \n\t" \
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
    "add   r1, sp, #16 \n\t" \
    "stmia r1!, {r4, r5, r6, r7} \n\t" \
    "mov   r1, r10 \n\t" \
    "eors  r0, r1 \n\t" \
    "str   r0, [sp, #32] \n\t" \
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
#define FAST_MULT_THUMB1_ASM_8 \
    "push  {r4-r7} \n\t" \
    "sub   sp, #48 \n\t" \
    "str   r0, [sp, #36] \n\t" \
    "str   r1, [sp, #40] \n\t" \
    "str   r2, [sp, #44] \n\t" \
    "mov   r9, r2 \n\t" \
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
 * Uses Even/Odd Split Row Multiplication:
 * In each 4-column pass (holding multiplier words b0, b1, b2, b3 in r2..r5):
 *   1. Compute non-overlapping even products a_i * b0 and a_i * b2 via umull,
 *      adding into the 4-word rolling accumulator with a single adcs chain.
 *   2. Compute non-overlapping odd products a_i * b3 and a_i * b1 via umull,
 *      adding into the shifted accumulator positions with a second adcs chain.
 * Because umull does not modify the CPSR carry flag, umull instructions can be
 * interleaved directly inside live adcs carry chains, reducing each 4x1 row to
 * 14 instructions (3.5 instructions per 32x32 multiply-accumulate).
 * ============================================================================ */

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

/* nw = 5: 4+1 Even/Odd Split, zero intermediate memory traffic */
#define FAST_MULT_ASM_5 \
    "ldr   r14, [r2, #16] \n\t" \
    "ldmia r2, {r2, r3, r4, r5} \n\t" \
    M3_PASS1_ROW0 \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
    M3_ROW_PASS1_LAST_R11(r7, r8, r9, r10) \
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

/* nw = 6: 4+2 Even/Odd Split, zero intermediate memory traffic */
#define FAST_MULT_ASM_6 \
    "push  {r2} \n\t" \
    "ldmia r2, {r2, r3, r4, r5} \n\t" \
    M3_PASS1_ROW0 \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
    M3_ROW_PASS1_NOSTORE(r7, r8, r9, r10, r14) \
    M3_ROW_PASS1_LAST_R11(r8, r9, r10, r14) \
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

/* nw = 7: 4+3 Even/Odd Split */
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

/* nw = 8: 4+4 Even/Odd Split */
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

#define FAST_SQUARE_ASM_5 \
    M3_SQR_STEP1 \
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

#define FAST_SQUARE_ASM_6 \
    M3_SQR_STEP1 \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
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
    "stmia r0, {r6, r7, r8, r9, r10, r11} \n\t"

#define FAST_SQUARE_ASM_7 \
    M3_SQR_STEP1 \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
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

#define FAST_SQUARE_ASM_8 \
    M3_SQR_STEP1 \
    M3_ROW_PASS1_R6(r8, r9, r10, r7) \
    M3_ROW_PASS1_R6(r9, r10, r7, r8) \
    M3_ROW_PASS1_R6(r10, r7, r8, r9) \
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
