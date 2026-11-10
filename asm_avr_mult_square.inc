/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */

/* Constant-time AVR multiprecision multiplication and squaring macros for
 * uECC_OPTIMIZATION_LEVEL >= 3.
 *
 * Register conventions on entry (matching asm_avr.inc):
 *   - Multiplication: X (r27:r26) = left, Y (r29:r28) = right, Z (r31:r30) = result.
 *   - Squaring:       X (r27:r26) = left, Z (r31:r30) = result.
 *
 * Design overview:
 *   1. FAST_MULT_ASM_20 (160-bit):
 *      Fully unrolled 2-level branch-free subtractive Karatsuba based on
 *      Michael Hutter and Peter Schwabe, "Multiprecision multiplication on AVR
 *      revisited" (2015, public domain), enhanced with:
 *        - Direct 2's-complement leaf accumulation at the 80-bit (k = 5 byte)
 *          leaves: computes unsigned residues D_a = (A_l - A_h) mod 2^40 (mask
 *          c_a in {0, -1}) and D_b = (B_h - B_l) mod 2^40 (mask c_b in {0, -1}),
 *          then forms M' = D_a * D_b - (D_a & c_b)*2^40 - (D_b & c_a)*2^40
 *          directly without conditional negation of M. Tracking borrows b1, b2
 *          from the two masked subtractions via c = (-c_a)*(-c_b) - b1 - b2
 *          keeps the D_a == 0 / D_b == 0 boundary cases exact and allows
 *          (D_a & c_b) to be subtracted before multiplying D_a * D_b.
 *        - ISR-safe atomic stack-frame allocation (fixing the original code's
 *          writes below SP).
 *        - Single-pass Level-1 carry propagation across Z+30..Z+39 (fixing a
 *          carry-drop bug in the original code when Z+30..Z+31 == 0xFFFF).
 *
 *   2. FAST_MULT_ASM_20_EXT_21 / FAST_SQUARE_ASM_20_EXT_21 (168-bit / 21-byte):
 *      Exact border-row/column extension from 20 bytes to 21 bytes for
 *      secp160r1's 161-bit curve order n, avoiding out-of-bounds reads/writes.
 *
 *   3. FAST_MULT_ASM_{24,28,32} + FAST_MULT_LEAF_{24,28,32} (192/224/256-bit):
 *      1-level branch-free subtractive Karatsuba calling an unrolled half-width
 *      (12/14/16-byte) leaf multiplier subroutine three times via rcall:
 *        A * B = L + 2^8k * (L + H - |A_l - A_h| * |B_l - B_h|) + 2^16k * H.
 *      Stack frames for |A_l - A_h|, |B_l - B_h|, and M are allocated above SP
 *      with atomic __SP_H__/__SP_L__ updates.
 *
 *   4. FAST_SQUARE_ASM_{20,24} (160/192-bit squaring):
 *      Upgraded MOVW-aligned sliding-window schoolbook squarer: computes each
 *      off-diagonal cross-product a_i * a_j (i < j) once while shifting the
 *      10-byte register window by 2 bytes at a time via single-cycle MOVW
 *      instructions, doubles the cross-product stream, and adds the diagonal
 *      squares a_i^2.
 *
 *   5. FAST_SQUARE_ASM_{28,32} + FAST_SQUARE_LEAF_{28,32} (224/256-bit squaring):
 *      1-level branch-free Karatsuba squarer using:
 *        A^2 = A_l^2 + 2^8k * (A_l^2 + A_h^2 - |A_l - A_h|^2) + 2^16k * A_h^2,
 *      with three rcalls to a shared half-width (14/16-byte) MOVW-aligned leaf
 *      squarer (including a register-input entry point for |A_l - A_h|^2).
 */

#ifndef _UECC_ASM_AVR_MULT_SQUARE_H_
#define _UECC_ASM_AVR_MULT_SQUARE_H_

/* 160-bit (20x20 -> 40-byte) fully unrolled hybrid 2-level Karatsuba multiplier. */
#define FAST_MULT_ASM_20 \
    /* level 1: compute L */ \
    /* init zero registers */ \
    "clr r18 \n\t" \
    "clr r19 \n\t" \
    "movw r20, r18 \n\t" \
    /* Compute L */ \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ld r6, x+ \n\t" \
    "ldd r7, y+0 \n\t" \
    "ldd r8, y+1 \n\t" \
    "ldd r9, y+2 \n\t" \
    "ldd r10, y+3 \n\t" \
    "ldd r11, y+4 \n\t" \
    "mul r2, r9 \n\t" \
    "movw r14, r0 \n\t" \
    "mul r2, r7 \n\t" \
    "movw r12, r0 \n\t" \
    "mul r2, r8 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "adc r15, r21 \n\t" \
    "mul r2, r11 \n\t" \
    "movw r16, r0 \n\t" \
    "mul r2, r10 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r21 \n\t" \
    "mul r3, r9 \n\t" \
    "movw r24, r0 \n\t" \
    "mul r3, r7 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "adc r15, r24 \n\t" \
    "adc r25, r21 \n\t" \
    "mul r3, r8 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r25, r21 \n\t" \
    "mul r3, r11 \n\t" \
    "add r16, r25 \n\t" \
    "adc r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "mul r3, r10 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r18, r21 \n\t" \
    "mul r4, r9 \n\t" \
    "movw r24, r0 \n\t" \
    "mul r4, r7 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r16, r24 \n\t" \
    "adc r25, r21 \n\t" \
    "mul r4, r8 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r25, r21 \n\t" \
    "mul r4, r11 \n\t" \
    "add r17, r25 \n\t" \
    "adc r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "mul r4, r10 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r19, r21 \n\t" \
    "mul r5, r9 \n\t" \
    "movw r24, r0 \n\t" \
    "mul r5, r7 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r24 \n\t" \
    "adc r25, r21 \n\t" \
    "mul r5, r8 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r25, r21 \n\t" \
    "mul r5, r11 \n\t" \
    "add r18, r25 \n\t" \
    "adc r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "mul r5, r10 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r21 \n\t" \
    "mul r6, r9 \n\t" \
    "movw r24, r0 \n\t" \
    "mul r6, r7 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r18, r24 \n\t" \
    "adc r25, r21 \n\t" \
    "mul r6, r8 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r25, r21 \n\t" \
    "mul r6, r10 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r21 \n\t" \
    "mul r6, r11 \n\t" \
    "add r19, r25 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "std z+0, r12 \n\t" \
    "std z+1, r13 \n\t" \
    "std z+2, r14 \n\t" \
    "std z+3, r15 \n\t" \
    "std z+4, r16 \n\t" \
    /* load b5..b9 into R12..R16 and a5..a9 into R23..R27 */ \
    "ldd r12, y+5 \n\t" \
    "ldd r13, y+6 \n\t" \
    "ldd r14, y+7 \n\t" \
    "ldd r15, y+8 \n\t" \
    "ldd r16, y+9 \n\t" \
    "ld r23, x+ \n\t" \
    "ld r24, x+ \n\t" \
    "ld r25, x+ \n\t" \
    "ld r0, x+ \n\t" \
    "ld r1, x+ \n\t" \
    "push r26 \n\t" \
    "push r27 \n\t" \
    "push r28 \n\t" \
    "push r29 \n\t" \
    "movw r26, r0 \n\t" \
    /* Compute D_a = A_l - A_h */ \
    "sub r2, r23 \n\t" \
    "sbc r3, r24 \n\t" \
    "sbc r4, r25 \n\t" \
    "sbc r5, r26 \n\t" \
    "sbc r6, r27 \n\t" \
    "sbc r0, r0 \n\t" \
    "bst r0, 0 \n\t" \
    "clr r22 \n\t" \
    /* Compute H + (l5,l6,l7,l8,l9) */ \
    "mul r23, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r23, r12 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r19, r28 \n\t" \
    "adc r29, r22 \n\t" \
    "mul r23, r13 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r29, r22 \n\t" \
    "mul r23, r15 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r22 \n\t" \
    "mul r23, r16 \n\t" \
    "add r20, r29 \n\t" \
    "adc r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "clr r23 \n\t" \
    "mul r24, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r24, r12 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r28 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r24, r13 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r24, r15 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r23 \n\t" \
    "mul r24, r16 \n\t" \
    "add r21, r29 \n\t" \
    "adc r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "clr r24 \n\t" \
    "mul r25, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r25, r12 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r28 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r25, r13 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r25, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r24 \n\t" \
    "mul r25, r16 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "clr r25 \n\t" \
    "mul r26, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r26, r12 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r28 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r26, r13 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r26, r15 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r25 \n\t" \
    "mul r26, r16 \n\t" \
    "add r23, r29 \n\t" \
    "adc r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "clr r26 \n\t" \
    "mul r27, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r27, r12 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r28 \n\t" \
    "adc r29, r26 \n\t" \
    "mul r27, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r26 \n\t" \
    "mul r27, r15 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r26, r26 \n\t" \
    "mul r27, r16 \n\t" \
    "add r24, r29 \n\t" \
    "adc r25, r0 \n\t" \
    "adc r26, r1 \n\t" \
    /* Compute D_b = B_h - B_l in R12..R16 */ \
    "sub r12, r7 \n\t" \
    "sbc r13, r8 \n\t" \
    "sbc r14, r9 \n\t" \
    "sbc r15, r10 \n\t" \
    "sbc r16, r11 \n\t" \
    "sbc r28, r28 \n\t" /* c_b in R28 */ \
    /* Load l0..l4 and form U_0..U_4 (in R7..R11) and U_5..U_9 (in R17..R21) */ \
    "ldd r7, z+0 \n\t" \
    "ldd r8, z+1 \n\t" \
    "ldd r9, z+2 \n\t" \
    "ldd r10, z+3 \n\t" \
    "ldd r11, z+4 \n\t" \
    "clr r27 \n\t" \
    "bld r27, 0 \n\t" /* R27 = -c_a (0 or 1) */ \
    "and r27, r28 \n\t" /* R27 = c_a * c_b (0 or 1) */ \
    "clr r29 \n\t" /* R29 = 0 */ \
    "add r7, r17 \n\t" \
    "adc r8, r18 \n\t" \
    "adc r9, r19 \n\t" \
    "adc r10, r20 \n\t" \
    "adc r11, r21 \n\t" \
    "adc r17, r22 \n\t" \
    "adc r18, r23 \n\t" \
    "adc r19, r24 \n\t" \
    "adc r20, r25 \n\t" \
    "adc r21, r26 \n\t" \
    "adc r27, r29 \n\t" /* R27 = c_a * c_b + c1 */ \
    /* Subtract (D_a & c_b) from U_5..U_9 (R17..R21) */ \
    "movw r0, r2 \n\t" \
    "and r0, r28 \n\t" \
    "sub r17, r0 \n\t" \
    "and r1, r28 \n\t" \
    "sbc r18, r1 \n\t" \
    "movw r0, r4 \n\t" \
    "and r0, r28 \n\t" \
    "sbc r19, r0 \n\t" \
    "and r1, r28 \n\t" \
    "sbc r20, r1 \n\t" \
    "mov r0, r6 \n\t" \
    "and r0, r28 \n\t" \
    "sbc r21, r0 \n\t" \
    "sbc r27, r29 \n\t" /* R27 = c_a * c_b + c1 - b1 */ \
    /* Compute W = U_0..U_4 + D_a * D_b */ \
    "mul r2, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r2, r12 \n\t" \
    "add r7, r0 \n\t" \
    "std z+5, r7 \n\t" \
    "mov r7, r27 \n\t" /* save carry accumulator in R7! */ \
    "clr r27 \n\t" \
    "adc r8, r1 \n\t" \
    "adc r9, r28 \n\t" \
    "adc r29, r27 \n\t" \
    "mul r2, r13 \n\t" \
    "add r8, r0 \n\t" \
    "adc r9, r1 \n\t" \
    "adc r29, r27 \n\t" \
    "mul r2, r15 \n\t" \
    "add r10, r0 \n\t" \
    "adc r11, r1 \n\t" \
    "adc r27, r27 \n\t" \
    "mul r2, r16 \n\t" \
    "add r10, r29 \n\t" \
    "adc r11, r0 \n\t" \
    "clr r2 \n\t" \
    "adc r27, r1 \n\t" \
    "mul r3, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r3, r12 \n\t" \
    "add r8, r0 \n\t" \
    "adc r9, r1 \n\t" \
    "adc r10, r28 \n\t" \
    "adc r29, r2 \n\t" \
    "mul r3, r13 \n\t" \
    "add r9, r0 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r29, r2 \n\t" \
    "mul r3, r15 \n\t" \
    "add r11, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r2, r2 \n\t" \
    "mul r3, r16 \n\t" \
    "add r11, r29 \n\t" \
    "adc r27, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "clr r3 \n\t" \
    "mul r4, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r4, r12 \n\t" \
    "add r9, r0 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r11, r28 \n\t" \
    "adc r29, r3 \n\t" \
    "mul r4, r13 \n\t" \
    "add r10, r0 \n\t" \
    "adc r11, r1 \n\t" \
    "adc r29, r3 \n\t" \
    "mul r4, r15 \n\t" \
    "add r27, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r3, r3 \n\t" \
    "mul r4, r16 \n\t" \
    "add r27, r29 \n\t" \
    "adc r2, r0 \n\t" \
    "adc r3, r1 \n\t" \
    "clr r4 \n\t" \
    "mul r5, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r5, r12 \n\t" \
    "add r10, r0 \n\t" \
    "adc r11, r1 \n\t" \
    "adc r27, r28 \n\t" \
    "adc r29, r4 \n\t" \
    "mul r5, r13 \n\t" \
    "add r11, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r29, r4 \n\t" \
    "mul r5, r15 \n\t" \
    "add r2, r0 \n\t" \
    "adc r3, r1 \n\t" \
    "adc r4, r4 \n\t" \
    "mul r5, r16 \n\t" \
    "add r2, r29 \n\t" \
    "adc r3, r0 \n\t" \
    "adc r4, r1 \n\t" \
    "clr r5 \n\t" \
    "mul r6, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r6, r12 \n\t" \
    "add r11, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r2, r28 \n\t" \
    "adc r29, r5 \n\t" \
    "mul r6, r13 \n\t" \
    "add r27, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r29, r5 \n\t" \
    "mul r6, r15 \n\t" \
    "add r3, r0 \n\t" \
    "adc r4, r1 \n\t" \
    "adc r5, r5 \n\t" \
    "mul r6, r16 \n\t" \
    "add r3, r29 \n\t" \
    "adc r4, r0 \n\t" \
    "adc r5, r1 \n\t" \
    /* Subtract (D_b & c_a) from U_5..U_9 (R17..R21) */ \
    "clr r1 \n\t" \
    "clr r28 \n\t" \
    "bld r28, 0 \n\t" \
    "neg r28 \n\t" /* R28 = c_a (0x00 or 0xff) */ \
    "and r12, r28 \n\t" \
    "sub r17, r12 \n\t" \
    "and r13, r28 \n\t" \
    "sbc r18, r13 \n\t" \
    "and r14, r28 \n\t" \
    "sbc r19, r14 \n\t" \
    "and r15, r28 \n\t" \
    "sbc r20, r15 \n\t" \
    "and r16, r28 \n\t" \
    "sbc r21, r16 \n\t" \
    "sbc r7, r1 \n\t" /* R7 = c_a * c_b + c1 - b1 - b2 */ \
    "mov r0, r7 \n\t" \
    "asr r0 \n\t" \
    /* Add W_5..W_9 (R27, R2, R3, R4, R5) to U_5..U_9 (R17..R21) and propagate carry */ \
    "add r17, r27 \n\t" \
    "adc r18, r2 \n\t" \
    "adc r19, r3 \n\t" \
    "adc r20, r4 \n\t" \
    "adc r21, r5 \n\t" \
    "adc r22, r7 \n\t" \
    "adc r23, r0 \n\t" \
    "adc r24, r0 \n\t" \
    "adc r25, r0 \n\t" \
    "adc r26, r0 \n\t" \
    "std z+6, r8 \n\t" \
    "std z+7, r9 \n\t" \
    "std z+8, r10 \n\t" \
    "std z+9, r11 \n\t" \
    /* push h9 on stack */ \
    "push r26 \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "ldd r1, y+2 \n\t" \
    "ldd r0, y+3 \n\t" \
    "ldd r27, y+4 \n\t" \
    "ldd r26, y+5 \n\t" \
    "movw r28, r0 \n\t" \
    /* push h5,h6,h7 on stack */ \
    "push r19 \n\t" \
    "push r18 \n\t" \
    "push r17 \n\t" \
    /* level 1: compute H */ \
    /* init zero registers */ \
    "clr r14 \n\t" \
    "clr r15 \n\t" \
    "movw r16, r14 \n\t" \
    "movw r18, r14 \n\t" \
    /* level 2: compute L */ \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ld r6, x+ \n\t" \
    "ldd r7, y+10 \n\t" \
    "ldd r8, y+11 \n\t" \
    "ldd r9, y+12 \n\t" \
    "ldd r10, y+13 \n\t" \
    "ldd r11, y+14 \n\t" \
    "mul r2, r7 \n\t" \
    "movw r12, r0 \n\t" \
    "mul r2, r8 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "mul r3, r7 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "adc r15, r19 \n\t" \
    "mul r2, r9 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r16, r19 \n\t" \
    "mul r3, r8 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r16, r19 \n\t" \
    "mul r4, r7 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r16, r19 \n\t" \
    "mul r2, r10 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r19 \n\t" \
    "mul r3, r9 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r19 \n\t" \
    "mul r4, r8 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r19 \n\t" \
    "mul r5, r7 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r19 \n\t" \
    "mul r2, r11 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r18, r19 \n\t" \
    "mul r3, r10 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r18, r19 \n\t" \
    "mul r4, r9 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r18, r19 \n\t" \
    "mul r5, r8 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r18, r19 \n\t" \
    "mul r6, r7 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r18, r19 \n\t" \
    /* now add h0+l10 and h0+l14 */ \
    "pop r0 \n\t" \
    "add r0, r12 \n\t" \
    "std z+20, r0 \n\t" \
    "pop r0 \n\t" \
    "adc r0, r13 \n\t" \
    "std z+21, r0 \n\t" \
    "pop r0 \n\t" \
    "adc r0, r14 \n\t" \
    "std z+22, r0 \n\t" \
    "adc r20, r15 \n\t" \
    "adc r21, r16 \n\t" \
    "std z+23, r20 \n\t" \
    "std z+24, r21 \n\t" \
    "adc r12, r22 \n\t" \
    "adc r13, r23 \n\t" \
    "adc r14, r24 \n\t" \
    "adc r15, r25 \n\t" \
    "pop r0 \n\t" \
    "adc r16, r0 \n\t" \
    "std z+15, r12 \n\t" \
    "std z+16, r13 \n\t" \
    "std z+17, r14 \n\t" \
    "std z+18, r15 \n\t" \
    "std z+19, r16 \n\t" \
    /* push carry on stack */ \
    "sbc r0, r0 \n\t" \
    "push r0 \n\t" \
    /* continue */ \
    "clr r20 \n\t" \
    "clr r21 \n\t" \
    "clr r22 \n\t" \
    "mul r3, r11 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r19, r22 \n\t" \
    "mul r4, r10 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r19, r22 \n\t" \
    "mul r5, r9 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r19, r22 \n\t" \
    "mul r6, r8 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r19, r22 \n\t" \
    "mul r4, r11 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r22 \n\t" \
    "mul r5, r10 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r22 \n\t" \
    "mul r6, r9 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r22 \n\t" \
    "mul r5, r11 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r22 \n\t" \
    "mul r6, r10 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r22 \n\t" \
    "mul r6, r11 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    /* load a5..a9 and b5..b9 */ \
    "ldd r12, y+15 \n\t" \
    "ldd r13, y+16 \n\t" \
    "ldd r14, y+17 \n\t" \
    "ldd r15, y+18 \n\t" \
    "ldd r16, y+19 \n\t" \
    "ld r23, x+ \n\t" \
    "ld r24, x+ \n\t" \
    "ld r25, x+ \n\t" \
    "ld r0, x+ \n\t" \
    "ld r1, x+ \n\t" \
    "movw r26, r0 \n\t" \
    /* Compute D_a = A_l - A_h */ \
    "sub r2, r23 \n\t" \
    "sbc r3, r24 \n\t" \
    "sbc r4, r25 \n\t" \
    "sbc r5, r26 \n\t" \
    "sbc r6, r27 \n\t" \
    "sbc r0, r0 \n\t" \
    "bst r0, 0 \n\t" \
    "clr r22 \n\t" \
    /* Compute H + (l5,l6,l7,l8,l9) */ \
    "mul r23, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r23, r12 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r19, r28 \n\t" \
    "adc r29, r22 \n\t" \
    "mul r23, r13 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r29, r22 \n\t" \
    "mul r23, r15 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r22 \n\t" \
    "mul r23, r16 \n\t" \
    "add r20, r29 \n\t" \
    "adc r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "clr r23 \n\t" \
    "mul r24, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r24, r12 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r28 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r24, r13 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r24, r15 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r23 \n\t" \
    "mul r24, r16 \n\t" \
    "add r21, r29 \n\t" \
    "adc r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "clr r24 \n\t" \
    "mul r25, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r25, r12 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r28 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r25, r13 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r25, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r24 \n\t" \
    "mul r25, r16 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "clr r25 \n\t" \
    "mul r26, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r26, r12 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r28 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r26, r13 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r26, r15 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r25 \n\t" \
    "mul r26, r16 \n\t" \
    "add r23, r29 \n\t" \
    "adc r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "clr r26 \n\t" \
    "mul r27, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r27, r12 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r28 \n\t" \
    "adc r29, r26 \n\t" \
    "mul r27, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r26 \n\t" \
    "mul r27, r15 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r26, r26 \n\t" \
    "mul r27, r16 \n\t" \
    "add r24, r29 \n\t" \
    "adc r25, r0 \n\t" \
    "adc r26, r1 \n\t" \
    /* Compute D_b = B_h - B_l in R12..R16 */ \
    "sub r12, r7 \n\t" \
    "sbc r13, r8 \n\t" \
    "sbc r14, r9 \n\t" \
    "sbc r15, r10 \n\t" \
    "sbc r16, r11 \n\t" \
    "sbc r28, r28 \n\t" /* c_b in R28 */ \
    /* Load l0..l4 and form U_0..U_4 (in R7..R11) and U_5..U_9 (in R17..R21) */ \
    "ldd r7, z+15 \n\t" \
    "ldd r8, z+16 \n\t" \
    "ldd r9, z+17 \n\t" \
    "ldd r10, z+18 \n\t" \
    "ldd r11, z+19 \n\t" \
    "clr r27 \n\t" \
    "bld r27, 0 \n\t" /* R27 = -c_a (0 or 1) */ \
    "and r27, r28 \n\t" /* R27 = c_a * c_b (0 or 1) */ \
    "clr r29 \n\t" /* R29 = 0 */ \
    "add r7, r17 \n\t" \
    "adc r8, r18 \n\t" \
    "adc r9, r19 \n\t" \
    "adc r10, r20 \n\t" \
    "adc r11, r21 \n\t" \
    "adc r17, r22 \n\t" \
    "adc r18, r23 \n\t" \
    "adc r19, r24 \n\t" \
    "adc r20, r25 \n\t" \
    "adc r21, r26 \n\t" \
    "adc r27, r29 \n\t" /* R27 = c_a * c_b + c1 */ \
    "pop r0 \n\t" \
    "neg r0 \n\t" \
    "lsr r0 \n\t" \
    "adc r17, r0 \n\t" \
    "adc r18, r29 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r29 \n\t" \
    "adc r21, r29 \n\t" \
    "adc r27, r29 \n\t" \
    /* Subtract (D_a & c_b) from U_5..U_9 (R17..R21) */ \
    "movw r0, r2 \n\t" \
    "and r0, r28 \n\t" \
    "sub r17, r0 \n\t" \
    "and r1, r28 \n\t" \
    "sbc r18, r1 \n\t" \
    "movw r0, r4 \n\t" \
    "and r0, r28 \n\t" \
    "sbc r19, r0 \n\t" \
    "and r1, r28 \n\t" \
    "sbc r20, r1 \n\t" \
    "mov r0, r6 \n\t" \
    "and r0, r28 \n\t" \
    "sbc r21, r0 \n\t" \
    "sbc r27, r29 \n\t" /* R27 = c_a * c_b + c1 - b1 */ \
    /* Compute W = U_0..U_4 + D_a * D_b */ \
    "mul r2, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r2, r12 \n\t" \
    "add r7, r0 \n\t" \
    "std z+25, r7 \n\t" \
    "mov r7, r27 \n\t" /* save carry accumulator in R7! */ \
    "clr r27 \n\t" \
    "adc r8, r1 \n\t" \
    "adc r9, r28 \n\t" \
    "adc r29, r27 \n\t" \
    "mul r2, r13 \n\t" \
    "add r8, r0 \n\t" \
    "adc r9, r1 \n\t" \
    "adc r29, r27 \n\t" \
    "mul r2, r15 \n\t" \
    "add r10, r0 \n\t" \
    "adc r11, r1 \n\t" \
    "adc r27, r27 \n\t" \
    "mul r2, r16 \n\t" \
    "add r10, r29 \n\t" \
    "adc r11, r0 \n\t" \
    "clr r2 \n\t" \
    "adc r27, r1 \n\t" \
    "mul r3, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r3, r12 \n\t" \
    "add r8, r0 \n\t" \
    "adc r9, r1 \n\t" \
    "adc r10, r28 \n\t" \
    "adc r29, r2 \n\t" \
    "mul r3, r13 \n\t" \
    "add r9, r0 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r29, r2 \n\t" \
    "mul r3, r15 \n\t" \
    "add r11, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r2, r2 \n\t" \
    "mul r3, r16 \n\t" \
    "add r11, r29 \n\t" \
    "adc r27, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "clr r3 \n\t" \
    "mul r4, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r4, r12 \n\t" \
    "add r9, r0 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r11, r28 \n\t" \
    "adc r29, r3 \n\t" \
    "mul r4, r13 \n\t" \
    "add r10, r0 \n\t" \
    "adc r11, r1 \n\t" \
    "adc r29, r3 \n\t" \
    "mul r4, r15 \n\t" \
    "add r27, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r3, r3 \n\t" \
    "mul r4, r16 \n\t" \
    "add r27, r29 \n\t" \
    "adc r2, r0 \n\t" \
    "adc r3, r1 \n\t" \
    "clr r4 \n\t" \
    "mul r5, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r5, r12 \n\t" \
    "add r10, r0 \n\t" \
    "adc r11, r1 \n\t" \
    "adc r27, r28 \n\t" \
    "adc r29, r4 \n\t" \
    "mul r5, r13 \n\t" \
    "add r11, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r29, r4 \n\t" \
    "mul r5, r15 \n\t" \
    "add r2, r0 \n\t" \
    "adc r3, r1 \n\t" \
    "adc r4, r4 \n\t" \
    "mul r5, r16 \n\t" \
    "add r2, r29 \n\t" \
    "adc r3, r0 \n\t" \
    "adc r4, r1 \n\t" \
    "clr r5 \n\t" \
    "mul r6, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r6, r12 \n\t" \
    "add r11, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r2, r28 \n\t" \
    "adc r29, r5 \n\t" \
    "mul r6, r13 \n\t" \
    "add r27, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r29, r5 \n\t" \
    "mul r6, r15 \n\t" \
    "add r3, r0 \n\t" \
    "adc r4, r1 \n\t" \
    "adc r5, r5 \n\t" \
    "mul r6, r16 \n\t" \
    "add r3, r29 \n\t" \
    "adc r4, r0 \n\t" \
    "adc r5, r1 \n\t" \
    /* Subtract (D_b & c_a) from U_5..U_9 (R17..R21) */ \
    "clr r1 \n\t" \
    "clr r28 \n\t" \
    "bld r28, 0 \n\t" \
    "neg r28 \n\t" /* R28 = c_a (0x00 or 0xff) */ \
    "and r12, r28 \n\t" \
    "sub r17, r12 \n\t" \
    "and r13, r28 \n\t" \
    "sbc r18, r13 \n\t" \
    "and r14, r28 \n\t" \
    "sbc r19, r14 \n\t" \
    "and r15, r28 \n\t" \
    "sbc r20, r15 \n\t" \
    "and r16, r28 \n\t" \
    "sbc r21, r16 \n\t" \
    "sbc r7, r1 \n\t" /* R7 = c_a * c_b + c1 - b1 - b2 */ \
    "mov r0, r7 \n\t" \
    "asr r0 \n\t" \
    /* Add W_5..W_9 (R27, R2, R3, R4, R5) to U_5..U_9 (R17..R21) and propagate carry */ \
    "add r17, r27 \n\t" \
    "adc r18, r2 \n\t" \
    "adc r19, r3 \n\t" \
    "adc r20, r4 \n\t" \
    "adc r21, r5 \n\t" \
    "adc r22, r7 \n\t" \
    "adc r23, r0 \n\t" \
    "adc r24, r0 \n\t" \
    "adc r25, r0 \n\t" \
    "adc r26, r0 \n\t" \
    "std z+26, r8 \n\t" \
    "std z+27, r9 \n\t" \
    "std z+28, r10 \n\t" \
    "std z+29, r11 \n\t" \
    "std z+30, r17 \n\t" \
    "std z+31, r18 \n\t" \
    "std z+32, r19 \n\t" \
    "std z+33, r20 \n\t" \
    "std z+34, r21 \n\t" \
    "std z+35, r22 \n\t" \
    "std z+36, r23 \n\t" \
    "std z+37, r24 \n\t" \
    "std z+38, r25 \n\t" \
    "std z+39, r26 \n\t" \
    "pop r29 \n\t" \
    "pop r28 \n\t" \
    "pop r27 \n\t" \
    "pop r26 \n\t" \
    /* Level 1: prepare ISR-safe stack frame for |A_l - A_h| and |B_l - B_h| */ \
    /* store address of result on stack */ \
    "push r30 \n\t" \
    "push r31 \n\t" \
    /* use Z as stack-pointer base address */ \
    /* Bug fix (ISR stack safety): Hutter & Schwabe's original karatsuba160 */ \
    /* wrote 15 temporary bytes at SP-16..SP-2 without updating SP, which   */ \
    /* any hardware interrupt on AVR would clobber. Allocate 15 bytes above */ \
    /* SP (Z+1..Z+15) and update __SP_H__/__SP_L__ atomically.              */ \
    "in r30, __SP_L__ \n\t" \
    "in r31, __SP_H__ \n\t" \
    "sbiw r30, 15 \n\t" \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r31 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r30 \n\t" \
    /* level 1: subtract a0-a7 */ \
    "sbiw r26, 10 \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ld r6, x+ \n\t" \
    "ld r22, x+ \n\t" \
    "ld r23, x+ \n\t" \
    "ld r24, x+ \n\t" \
    "ld r25, x+ \n\t" \
    "ld r16, x+ \n\t" \
    "ld r0, x+ \n\t" \
    "sub r2, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r3, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r4, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r5, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r6, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r22, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r23, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r24, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r25, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r16, r0 \n\t" \
    /* 0xff if carry and 0x00 if no carry */ \
    "sbc r0, r0 \n\t" \
    /* level 1: absolute values */ \
    "eor r2, r0 \n\t" \
    "eor r3, r0 \n\t" \
    "eor r4, r0 \n\t" \
    "eor r5, r0 \n\t" \
    "eor r6, r0 \n\t" \
    "eor r22, r0 \n\t" \
    "eor r23, r0 \n\t" \
    "eor r24, r0 \n\t" \
    "eor r25, r0 \n\t" \
    "eor r16, r0 \n\t" \
    "sub r2, r0 \n\t" \
    "sbc r3, r0 \n\t" \
    "sbc r4, r0 \n\t" \
    "sbc r5, r0 \n\t" \
    "sbc r6, r0 \n\t" \
    "sbc r22, r0 \n\t" \
    "sbc r23, r0 \n\t" \
    "sbc r24, r0 \n\t" \
    "sbc r25, r0 \n\t" \
    "sbc r16, r0 \n\t" \
    "std z+1, r12 \n\t" \
    "std z+2, r13 \n\t" \
    "std z+3, r14 \n\t" \
    "std z+4, r15 \n\t" \
    "std z+5, r16 \n\t" \
    /* level 1: subtract b0-b7 */ \
    "ldd r7, y+0 \n\t" \
    "ldd r8, y+1 \n\t" \
    "ldd r9, y+2 \n\t" \
    "ldd r10, y+3 \n\t" \
    "ldd r11, y+4 \n\t" \
    "ldd r12, y+5 \n\t" \
    "ldd r13, y+6 \n\t" \
    "ldd r14, y+7 \n\t" \
    "ldd r15, y+8 \n\t" \
    "ldd r16, y+9 \n\t" \
    "ldd r1, y+10 \n\t" \
    "sub r7, r1 \n\t" \
    "ldd r1, y+11 \n\t" \
    "sbc r8, r1 \n\t" \
    "ldd r1, y+12 \n\t" \
    "sbc r9, r1 \n\t" \
    "ldd r1, y+13 \n\t" \
    "sbc r10, r1 \n\t" \
    "ldd r1, y+14 \n\t" \
    "sbc r11, r1 \n\t" \
    "ldd r1, y+15 \n\t" \
    "sbc r12, r1 \n\t" \
    "ldd r1, y+16 \n\t" \
    "sbc r13, r1 \n\t" \
    "ldd r1, y+17 \n\t" \
    "sbc r14, r1 \n\t" \
    "ldd r1, y+18 \n\t" \
    "sbc r15, r1 \n\t" \
    "ldd r1, y+19 \n\t" \
    "sbc r16, r1 \n\t" \
    /* 0xff if carry and 0x00 if no carry */ \
    "sbc r1, r1 \n\t" \
    /* level 1: absolute values */ \
    "eor r7, r1 \n\t" \
    "eor r8, r1 \n\t" \
    "eor r9, r1 \n\t" \
    "eor r10, r1 \n\t" \
    "eor r11, r1 \n\t" \
    "eor r12, r1 \n\t" \
    "eor r13, r1 \n\t" \
    "eor r14, r1 \n\t" \
    "eor r15, r1 \n\t" \
    "eor r16, r1 \n\t" \
    "sub r7, r1 \n\t" \
    "sbc r8, r1 \n\t" \
    "sbc r9, r1 \n\t" \
    "sbc r10, r1 \n\t" \
    "sbc r11, r1 \n\t" \
    "sbc r12, r1 \n\t" \
    "sbc r13, r1 \n\t" \
    "sbc r14, r1 \n\t" \
    "sbc r15, r1 \n\t" \
    "sbc r16, r1 \n\t" \
    "std z+6, r12 \n\t" \
    "std z+7, r13 \n\t" \
    "std z+8, r14 \n\t" \
    "std z+9, r15 \n\t" \
    "std z+10, r16 \n\t" \
    "eor r0, r1 \n\t" \
    "push r0 \n\t" \
    /* level 1: compute M */ \
    "clr r18 \n\t" \
    "clr r19 \n\t" \
    "mul r2, r9 \n\t" \
    "movw r14, r0 \n\t" \
    "mul r2, r7 \n\t" \
    "movw r12, r0 \n\t" \
    "mul r2, r8 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "adc r15, r19 \n\t" \
    "mul r2, r11 \n\t" \
    "movw r16, r0 \n\t" \
    "mul r2, r10 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r19 \n\t" \
    "mul r3, r9 \n\t" \
    "movw r20, r0 \n\t" \
    "mul r3, r7 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "adc r15, r20 \n\t" \
    "adc r21, r19 \n\t" \
    "mul r3, r8 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r21, r19 \n\t" \
    "mul r3, r11 \n\t" \
    "add r16, r21 \n\t" \
    "adc r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "mul r3, r10 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r18, r19 \n\t" \
    "mul r4, r9 \n\t" \
    "movw r20, r0 \n\t" \
    "mul r4, r7 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r16, r20 \n\t" \
    "adc r21, r19 \n\t" \
    "mul r4, r8 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r21, r19 \n\t" \
    "mul r4, r10 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r19, r19 \n\t" \
    "mul r4, r11 \n\t" \
    "add r17, r21 \n\t" \
    "adc r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "std z+11, r12 \n\t" \
    "std z+12, r13 \n\t" \
    "std z+13, r14 \n\t" \
    "movw r12, r22 \n\t" \
    "mov r14, r24 \n\t" \
    "clr r20 \n\t" \
    "clr r21 \n\t" \
    "mul r5, r9 \n\t" \
    "movw r22, r0 \n\t" \
    "mul r5, r7 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r22 \n\t" \
    "adc r23, r21 \n\t" \
    "mul r5, r8 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r23, r21 \n\t" \
    "mul r5, r11 \n\t" \
    "add r18, r23 \n\t" \
    "adc r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "mul r5, r10 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r21 \n\t" \
    "mul r6, r9 \n\t" \
    "movw r22, r0 \n\t" \
    "mul r6, r7 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r18, r22 \n\t" \
    "adc r23, r21 \n\t" \
    "mul r6, r8 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r23, r21 \n\t" \
    "mul r6, r10 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r21 \n\t" \
    "mul r6, r11 \n\t" \
    "add r19, r23 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "std z+14, r15 \n\t" \
    "std z+15, r16 \n\t" \
    "mov r15, r25 \n\t" \
    /* load missing operands */ \
    "ldd r16, z+5 \n\t" \
    "ldd r23, z+6 \n\t" \
    "ldd r24, z+7 \n\t" \
    "ldd r25, z+8 \n\t" \
    "ldd r26, z+9 \n\t" \
    "ldd r27, z+10 \n\t" \
    /* subtract a0-a5 */ \
    "sub r2, r12 \n\t" \
    "sbc r3, r13 \n\t" \
    "sbc r4, r14 \n\t" \
    "sbc r5, r15 \n\t" \
    "sbc r6, r16 \n\t" \
    /* 0xff if carry and 0x00 if no carry */ \
    "sbc r0, r0 \n\t" \
    /* subtract b0-b5 */ \
    "sub r7, r23 \n\t" \
    "sbc r8, r24 \n\t" \
    "sbc r9, r25 \n\t" \
    "sbc r10, r26 \n\t" \
    "sbc r11, r27 \n\t" \
    /* 0xff if carry and 0x00 if no carry */ \
    "sbc r1, r1 \n\t" \
    /* absolute values */ \
    "eor r2, r0 \n\t" \
    "eor r3, r0 \n\t" \
    "eor r4, r0 \n\t" \
    "eor r5, r0 \n\t" \
    "eor r6, r0 \n\t" \
    "eor r7, r1 \n\t" \
    "eor r8, r1 \n\t" \
    "eor r9, r1 \n\t" \
    "eor r10, r1 \n\t" \
    "eor r11, r1 \n\t" \
    "sub r2, r0 \n\t" \
    "sbc r3, r0 \n\t" \
    "sbc r4, r0 \n\t" \
    "sbc r5, r0 \n\t" \
    "sbc r6, r0 \n\t" \
    "sub r7, r1 \n\t" \
    "sbc r8, r1 \n\t" \
    "sbc r9, r1 \n\t" \
    "sbc r10, r1 \n\t" \
    "sbc r11, r1 \n\t" \
    "push r30 \n\t" \
    "clr r30 \n\t" \
    "eor r0, r1 \n\t" \
    "bst r0, 0 \n\t" \
    /* Compute H + (l5,l6,l7,l8,l9) */ \
    "mul r23, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r23, r12 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r19, r28 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r23, r13 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r23, r16 \n\t" \
    "add r20, r29 \n\t" \
    "adc r21, r0 \n\t" \
    "clr r22 \n\t" \
    "adc r22, r1 \n\t" \
    "mul r23, r15 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r30 \n\t" \
    "mul r24, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r24, r12 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r28 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r24, r13 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r24, r16 \n\t" \
    "add r21, r29 \n\t" \
    "adc r22, r0 \n\t" \
    "clr r23 \n\t" \
    "adc r23, r1 \n\t" \
    "mul r24, r15 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r30 \n\t" \
    "mul r25, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r25, r12 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r28 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r25, r13 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r25, r16 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r0 \n\t" \
    "clr r24 \n\t" \
    "adc r24, r1 \n\t" \
    "mul r25, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r30 \n\t" \
    "mul r26, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r26, r12 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r28 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r26, r13 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r26, r16 \n\t" \
    "add r23, r29 \n\t" \
    "adc r24, r0 \n\t" \
    "clr r25 \n\t" \
    "adc r25, r1 \n\t" \
    "mul r26, r15 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r30 \n\t" \
    "mul r27, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r27, r12 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r28 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r27, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r27, r16 \n\t" \
    "add r24, r29 \n\t" \
    "adc r25, r0 \n\t" \
    "clr r26 \n\t" \
    "adc r26, r1 \n\t" \
    "mul r27, r15 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r26, r30 \n\t" \
    /* Compute M */ \
    "mul r2, r9 \n\t" \
    "movw r14, r0 \n\t" \
    "mul r2, r7 \n\t" \
    "movw r12, r0 \n\t" \
    "mul r2, r8 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "adc r15, r30 \n\t" \
    "mul r2, r11 \n\t" \
    "mov r16, r0 \n\t" \
    "clr r27 \n\t" \
    "mov r27, r1 \n\t" \
    "mul r2, r10 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r27, r30 \n\t" \
    "mul r3, r9 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r3, r7 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "adc r15, r28 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r3, r8 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r3, r11 \n\t" \
    "add r16, r29 \n\t" \
    "adc r27, r0 \n\t" \
    "clr r2 \n\t" \
    "adc r2, r1 \n\t" \
    "mul r3, r10 \n\t" \
    "add r16, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r2, r30 \n\t" \
    "mul r4, r9 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r4, r7 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r16, r28 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r4, r8 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r4, r11 \n\t" \
    "add r27, r29 \n\t" \
    "adc r2, r0 \n\t" \
    "clr r3 \n\t" \
    "adc r3, r1 \n\t" \
    "mul r4, r10 \n\t" \
    "add r27, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r3, r30 \n\t" \
    "mul r5, r9 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r5, r7 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r27, r28 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r5, r8 \n\t" \
    "add r16, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r5, r11 \n\t" \
    "add r2, r29 \n\t" \
    "adc r3, r0 \n\t" \
    "clr r4 \n\t" \
    "adc r4, r1 \n\t" \
    "mul r5, r10 \n\t" \
    "add r2, r0 \n\t" \
    "adc r3, r1 \n\t" \
    "adc r4, r30 \n\t" \
    "mul r6, r9 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r6, r7 \n\t" \
    "add r16, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r2, r28 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r6, r8 \n\t" \
    "add r27, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r29, r30 \n\t" \
    "mul r6, r11 \n\t" \
    "add r3, r29 \n\t" \
    "adc r4, r0 \n\t" \
    "clr r5 \n\t" \
    "adc r5, r1 \n\t" \
    "mul r6, r10 \n\t" \
    "add r3, r0 \n\t" \
    "adc r4, r1 \n\t" \
    "adc r5, r30 \n\t" \
    /* add l5+h0 to h5 etc. */ \
    "pop r30 \n\t" \
    "ldd r7, z+11 \n\t" \
    "ldd r8, z+12 \n\t" \
    "ldd r9, z+13 \n\t" \
    "ldd r10, z+14 \n\t" \
    "ldd r11, z+15 \n\t" \
    "add r7, r17 \n\t" \
    "adc r8, r18 \n\t" \
    "adc r9, r19 \n\t" \
    "adc r10, r20 \n\t" \
    "adc r11, r21 \n\t" \
    "adc r17, r22 \n\t" \
    "adc r18, r23 \n\t" \
    "adc r19, r24 \n\t" \
    "adc r20, r25 \n\t" \
    "adc r21, r26 \n\t" \
    /* store carry in R28 */ \
    /* process sign bit */ \
    "clr r28 \n\t" \
    "clr r29 \n\t" \
    "bld r29, 0 \n\t" \
    "dec r29 \n\t" \
    /* merge carry and borrow */ \
    "adc r28, r29 \n\t" \
    "mov r0, r28 \n\t" \
    "asr r0 \n\t" \
    /* invert all bits or do nothing */ \
    "eor r12, r29 \n\t" \
    "eor r13, r29 \n\t" \
    "eor r14, r29 \n\t" \
    "eor r15, r29 \n\t" \
    "eor r16, r29 \n\t" \
    "eor r27, r29 \n\t" \
    "eor r2, r29 \n\t" \
    "eor r3, r29 \n\t" \
    "eor r4, r29 \n\t" \
    "eor r5, r29 \n\t" \
    "add r29, r29 \n\t" /* sets carry flag if R29 = 0xff */ \
    /* add in M */ \
    "adc r7, r12 \n\t" \
    "adc r8, r13 \n\t" \
    "adc r9, r14 \n\t" \
    "adc r10, r15 \n\t" \
    "adc r11, r16 \n\t" \
    "adc r17, r27 \n\t" \
    "adc r18, r2 \n\t" \
    "adc r19, r3 \n\t" \
    "adc r20, r4 \n\t" \
    "adc r21, r5 \n\t" \
    /* propagate carry/borrow */ \
    "adc r22, r28 \n\t" \
    "adc r23, r0 \n\t" \
    "adc r24, r0 \n\t" \
    "adc r25, r0 \n\t" \
    "adc r26, r0 \n\t" \
    "ldd r2, z+11 \n\t" \
    "ldd r3, z+12 \n\t" \
    "ldd r4, z+13 \n\t" \
    "ldd r5, z+14 \n\t" \
    "ldd r6, z+15 \n\t" \
    /* M is stored in: 2,3,4,5,6,7,8,9,10,11,17,18,19,20,21,22,23,24,25,26 */ \
    /* Level 1: combine L, H, and M into result[0..39] */ \
    /* Restore SP atomically to release the 15-byte Level-1 frame before popping Z. */ \
    "pop r0 \n\t" \
    "adiw r30, 15 \n\t" \
    "in r1, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r31 \n\t" \
    "out __SREG__, r1 \n\t" \
    "out __SP_L__, r30 \n\t" \
    "pop r31 \n\t" \
    "pop r30 \n\t" \
    "push r24 \n\t" \
    "push r25 \n\t" \
    "ldd r24, z+0 \n\t" \
    "ldd r25, z+1 \n\t" \
    "ldd r12, z+2 \n\t" \
    "ldd r13, z+3 \n\t" \
    "ldd r14, z+4 \n\t" \
    "ldd r15, z+5 \n\t" \
    "ldd r16, z+6 \n\t" \
    "ldd r27, z+7 \n\t" \
    "ldd r28, z+8 \n\t" \
    "ldd r29, z+9 \n\t" \
    /* process sign bit */ \
    "com r0 \n\t" \
    /* merge carry and borrow */ \
    "mov r1, r0 \n\t" \
    "asr r1 \n\t" \
    /* invert all bits or do nothing */ \
    "eor r2, r0 \n\t" \
    "eor r3, r0 \n\t" \
    "eor r4, r0 \n\t" \
    "eor r5, r0 \n\t" \
    "eor r6, r0 \n\t" \
    "eor r7, r0 \n\t" \
    "eor r8, r0 \n\t" \
    "eor r9, r0 \n\t" \
    "eor r10, r0 \n\t" \
    "eor r11, r0 \n\t" \
    "add r0, r0 \n\t" /* sets carry flag if R0 = 0xff */ \
    /* add in M */ \
    "adc r24, r2 \n\t" \
    "adc r25, r3 \n\t" \
    "adc r12, r4 \n\t" \
    "adc r13, r5 \n\t" \
    "adc r14, r6 \n\t" \
    "adc r15, r7 \n\t" \
    "adc r16, r8 \n\t" \
    "adc r27, r9 \n\t" \
    "adc r28, r10 \n\t" \
    "adc r29, r11 \n\t" \
    /* store carry in T register */ \
    "sbc r2, r2 \n\t" \
    "bst r2, 0 \n\t" \
    "ldd r2, z+20 \n\t" \
    "ldd r3, z+21 \n\t" \
    "add r24, r2 \n\t" \
    "adc r25, r3 \n\t" \
    "std z+10, r24 \n\t" \
    "std z+11, r25 \n\t" \
    "ldd r24, z+22 \n\t" \
    "ldd r25, z+23 \n\t" \
    "adc r12, r24 \n\t" \
    "adc r13, r25 \n\t" \
    "std z+12, r12 \n\t" \
    "std z+13, r13 \n\t" \
    "movw r12, r24 \n\t" \
    "ldd r24, z+24 \n\t" \
    "ldd r25, z+25 \n\t" \
    "adc r14, r24 \n\t" \
    "adc r15, r25 \n\t" \
    "std z+14, r14 \n\t" \
    "std z+15, r15 \n\t" \
    "movw r14, r24 \n\t" \
    "ldd r24, z+26 \n\t" \
    "ldd r25, z+27 \n\t" \
    "adc r16, r24 \n\t" \
    "adc r27, r25 \n\t" \
    "std z+16, r16 \n\t" \
    "std z+17, r27 \n\t" \
    "mov r16, r24 \n\t" \
    "mov r27, r25 \n\t" \
    "ldd r24, z+28 \n\t" \
    "ldd r25, z+29 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r25 \n\t" \
    "std z+18, r28 \n\t" \
    "std z+19, r29 \n\t" \
    "mov r28, r24 \n\t" \
    "mov r29, r25 \n\t" \
    /* store carry in R24 */ \
    "sbc r24, r24 \n\t" \
    "pop r10 \n\t" \
    "pop r9 \n\t" \
    /* merge carry and borrow */ \
    "asr r0 \n\t" \
    /* invert all bits or do nothing */ \
    "eor r17, r0 \n\t" \
    "eor r18, r0 \n\t" \
    "eor r19, r0 \n\t" \
    "eor r20, r0 \n\t" \
    "eor r21, r0 \n\t" \
    "eor r22, r0 \n\t" \
    "eor r23, r0 \n\t" \
    "eor r9, r0 \n\t" \
    "eor r10, r0 \n\t" \
    "eor r26, r0 \n\t" \
    "bld r25, 0 \n\t" /* sets carry flag if R25 = 0xff */ \
    "lsr r25 \n\t" \
    /* add in M */ \
    "adc r2, r17 \n\t" \
    "adc r3, r18 \n\t" \
    "adc r12, r19 \n\t" \
    "adc r13, r20 \n\t" \
    "adc r14, r21 \n\t" \
    "adc r15, r22 \n\t" \
    "adc r16, r23 \n\t" \
    "adc r27, r9 \n\t" \
    "adc r28, r10 \n\t" \
    "adc r29, r26 \n\t" \
    /* store carry in R1:R0 */ \
    "bst r24,0 \n\t" \
    "clr r25 \n\t" \
    "adc r0, r25 \n\t" \
    "adc r1, r25 \n\t" \
    "ldd r24, z+30 \n\t" \
    "ldd r25, z+31 \n\t" \
    "ldd r4, z+32 \n\t" \
    "ldd r5, z+33 \n\t" \
    "ldd r6, z+34 \n\t" \
    "ldd r7, z+35 \n\t" \
    "ldd r8, z+36 \n\t" \
    "ldd r9, z+37 \n\t" \
    "ldd r10, z+38 \n\t" \
    "ldd r11, z+39 \n\t" \
    "bld r19, 0 \n\t" \
    "lsr r19 \n\t" \
    "adc r2, r24 \n\t" \
    "adc r3, r25 \n\t" \
    "adc r12, r4 \n\t" \
    "adc r13, r5 \n\t" \
    "adc r14, r6 \n\t" \
    "adc r15, r7 \n\t" \
    "adc r16, r8 \n\t" \
    "adc r27, r9 \n\t" \
    "adc r28, r10 \n\t" \
    "adc r29, r11 \n\t" \
    /* Bug fix (carry drop in Hutter & Schwabe's published karatsuba160):    */ \
    /* The original code added the carry out of Z+29 only to Z+30..Z+31      */ \
    /* (ADC R24, R19; ADC R25, R19), losing the carry into Z+32..Z+39 when   */ \
    /* Z+30..Z+31 == 0xFFFF. Merge this carry into the signed 16-bit carry   */ \
    /* accumulator R1:R0 in {-1, 0, +1, +2} so the final 10-byte chain       */ \
    /* propagates across all of Z+30..Z+39 in a single pass.                 */ \
    "clr r19 \n\t" \
    "adc r0, r19 \n\t" \
    "adc r1, r19 \n\t" \
    "std z+20, r2 \n\t" \
    "std z+21, r3 \n\t" \
    "std z+22, r12 \n\t" \
    "std z+23, r13 \n\t" \
    "std z+24, r14 \n\t" \
    "std z+25, r15 \n\t" \
    "std z+26, r16 \n\t" \
    "std z+27, r27 \n\t" \
    "std z+28, r28 \n\t" \
    "std z+29, r29 \n\t" \
    /* propagate carry to end */ \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r4, r1 \n\t" \
    "adc r5, r1 \n\t" \
    "adc r6, r1 \n\t" \
    "adc r7, r1 \n\t" \
    "adc r8, r1 \n\t" \
    "adc r9, r1 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r11, r1 \n\t" \
    "std z+30, r24 \n\t" \
    "std z+31, r25 \n\t" \
    "std z+32, r4 \n\t" \
    "std z+33, r5 \n\t" \
    "std z+34, r6 \n\t" \
    "std z+35, r7 \n\t" \
    "std z+36, r8 \n\t" \
    "std z+37, r9 \n\t" \
    "std z+38, r10 \n\t" \
    "std z+39, r11 \n\t"

/* Extends a 20x20 product in result[0..39] to a 21x21 product in result[0..41]
   for ECDSA operations modulo secp160r1's 21-byte curve order n. */
#define FAST_MULT_ASM_20_EXT_21 \
    /* Restore original result (Z), right (Y), and left (X) pointers. */ \
    "pop r31 \n\t" \
    "pop r30 \n\t" \
    "pop r29 \n\t" \
    "pop r28 \n\t" \
    "pop r27 \n\t" \
    "pop r26 \n\t" \
    "clr r24 \n\t" \
    "adiw r30, 40 \n\t" \
    "st z+, r24 \n\t" \
    "st z, r24 \n\t" \
    "sbiw r30, 21 \n\t" \
    "adiw r26, 20 \n\t" \
    "ld r18, x \n\t" \
    "sbiw r26, 20 \n\t" \
    "adiw r28, 20 \n\t" \
    "ld r19, y \n\t" \
    "sbiw r28, 20 \n\t" \
    /* Pass 1: result[20..40] += left[20] * right[0..19] */ \
    "ldi r20, 20 \n\t" \
    "clr r21 \n\t" \
    "81: \n\t" \
    "ld r22, y+ \n\t" \
    "mul r18, r22 \n\t" \
    "ld r23, z \n\t" \
    "add r0, r23 \n\t" \
    "adc r1, r24 \n\t" \
    "add r0, r21 \n\t" \
    "adc r1, r24 \n\t" \
    "st z+, r0 \n\t" \
    "mov r21, r1 \n\t" \
    "dec r20 \n\t" \
    "brne 81b \n\t" \
    "st z, r21 \n\t" \
    /* Pass 2: result[20..41] += right[20] * left[0..20] */ \
    "sbiw r30, 20 \n\t" \
    "ldi r20, 20 \n\t" \
    "clr r21 \n\t" \
    "82: \n\t" \
    "ld r22, x+ \n\t" \
    "mul r19, r22 \n\t" \
    "ld r23, z \n\t" \
    "add r0, r23 \n\t" \
    "adc r1, r24 \n\t" \
    "add r0, r21 \n\t" \
    "adc r1, r24 \n\t" \
    "st z+, r0 \n\t" \
    "mov r21, r1 \n\t" \
    "dec r20 \n\t" \
    "brne 82b \n\t" \
    "mul r18, r19 \n\t" \
    "ld r23, z \n\t" \
    "add r0, r23 \n\t" \
    "adc r1, r24 \n\t" \
    "add r0, r21 \n\t" \
    "adc r1, r24 \n\t" \
    "st z+, r0 \n\t" \
    "st z, r1 \n\t"

/* 192-bit (24x24 -> 48-byte) 1-level branch-free Karatsuba multiplier
   (calls 12-byte leaf subroutine at label 91f three times). */
#define FAST_MULT_ASM_24 \
    "push r26 \n\t" \
    "push r27 \n\t" \
    "push r28 \n\t" \
    "push r29 \n\t" \
    "push r30 \n\t" \
    "push r31 \n\t" \
    "rcall 91f \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "ldd r31, y+1 \n\t" \
    "ldd r30, y+2 \n\t" \
    "ldd r1, y+3 \n\t" \
    "ldd r0, y+4 \n\t" \
    "ldd r27, y+5 \n\t" \
    "ldd r26, y+6 \n\t" \
    "movw r28, r0 \n\t" \
    "adiw r26, 12 \n\t" \
    "adiw r28, 12 \n\t" \
    "adiw r30, 24 \n\t" \
    "rcall 91f \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "ldd r27, y+5 \n\t" \
    "ldd r26, y+6 \n\t" \
    "ldd r31, y+3 \n\t" \
    "ldd r30, y+4 \n\t" \
    "sbiw r28, 48 \n\t" \
    /* Atomic ISR-safe SP update. */ \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r29 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r28 \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ld r6, x+ \n\t" \
    "ld r7, x+ \n\t" \
    "ld r8, x+ \n\t" \
    "ld r9, x+ \n\t" \
    "ld r10, x+ \n\t" \
    "ld r11, x+ \n\t" \
    "ld r12, x+ \n\t" \
    "ld r13, x+ \n\t" \
    "ld r0, x+ \n\t" \
    "sub r2, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r3, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r4, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r5, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r6, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r7, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r8, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r9, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r10, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r11, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r12, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r13, r0 \n\t" \
    "sbc r18, r18 \n\t" \
    "eor r2, r18 \n\t" \
    "eor r3, r18 \n\t" \
    "eor r4, r18 \n\t" \
    "eor r5, r18 \n\t" \
    "eor r6, r18 \n\t" \
    "eor r7, r18 \n\t" \
    "eor r8, r18 \n\t" \
    "eor r9, r18 \n\t" \
    "eor r10, r18 \n\t" \
    "eor r11, r18 \n\t" \
    "eor r12, r18 \n\t" \
    "eor r13, r18 \n\t" \
    "sub r2, r18 \n\t" \
    "sbc r3, r18 \n\t" \
    "sbc r4, r18 \n\t" \
    "sbc r5, r18 \n\t" \
    "sbc r6, r18 \n\t" \
    "sbc r7, r18 \n\t" \
    "sbc r8, r18 \n\t" \
    "sbc r9, r18 \n\t" \
    "sbc r10, r18 \n\t" \
    "sbc r11, r18 \n\t" \
    "sbc r12, r18 \n\t" \
    "sbc r13, r18 \n\t" \
    "std y+1, r2 \n\t" \
    "std y+2, r3 \n\t" \
    "std y+3, r4 \n\t" \
    "std y+4, r5 \n\t" \
    "std y+5, r6 \n\t" \
    "std y+6, r7 \n\t" \
    "std y+7, r8 \n\t" \
    "std y+8, r9 \n\t" \
    "std y+9, r10 \n\t" \
    "std y+10, r11 \n\t" \
    "std y+11, r12 \n\t" \
    "std y+12, r13 \n\t" \
    "ldd r2, z+0 \n\t" \
    "ldd r3, z+1 \n\t" \
    "ldd r4, z+2 \n\t" \
    "ldd r5, z+3 \n\t" \
    "ldd r6, z+4 \n\t" \
    "ldd r7, z+5 \n\t" \
    "ldd r8, z+6 \n\t" \
    "ldd r9, z+7 \n\t" \
    "ldd r10, z+8 \n\t" \
    "ldd r11, z+9 \n\t" \
    "ldd r12, z+10 \n\t" \
    "ldd r13, z+11 \n\t" \
    "ldd r0, z+12 \n\t" \
    "sub r2, r0 \n\t" \
    "ldd r0, z+13 \n\t" \
    "sbc r3, r0 \n\t" \
    "ldd r0, z+14 \n\t" \
    "sbc r4, r0 \n\t" \
    "ldd r0, z+15 \n\t" \
    "sbc r5, r0 \n\t" \
    "ldd r0, z+16 \n\t" \
    "sbc r6, r0 \n\t" \
    "ldd r0, z+17 \n\t" \
    "sbc r7, r0 \n\t" \
    "ldd r0, z+18 \n\t" \
    "sbc r8, r0 \n\t" \
    "ldd r0, z+19 \n\t" \
    "sbc r9, r0 \n\t" \
    "ldd r0, z+20 \n\t" \
    "sbc r10, r0 \n\t" \
    "ldd r0, z+21 \n\t" \
    "sbc r11, r0 \n\t" \
    "ldd r0, z+22 \n\t" \
    "sbc r12, r0 \n\t" \
    "ldd r0, z+23 \n\t" \
    "sbc r13, r0 \n\t" \
    "sbc r19, r19 \n\t" \
    "eor r2, r19 \n\t" \
    "eor r3, r19 \n\t" \
    "eor r4, r19 \n\t" \
    "eor r5, r19 \n\t" \
    "eor r6, r19 \n\t" \
    "eor r7, r19 \n\t" \
    "eor r8, r19 \n\t" \
    "eor r9, r19 \n\t" \
    "eor r10, r19 \n\t" \
    "eor r11, r19 \n\t" \
    "eor r12, r19 \n\t" \
    "eor r13, r19 \n\t" \
    "sub r2, r19 \n\t" \
    "sbc r3, r19 \n\t" \
    "sbc r4, r19 \n\t" \
    "sbc r5, r19 \n\t" \
    "sbc r6, r19 \n\t" \
    "sbc r7, r19 \n\t" \
    "sbc r8, r19 \n\t" \
    "sbc r9, r19 \n\t" \
    "sbc r10, r19 \n\t" \
    "sbc r11, r19 \n\t" \
    "sbc r12, r19 \n\t" \
    "sbc r13, r19 \n\t" \
    "std y+13, r2 \n\t" \
    "std y+14, r3 \n\t" \
    "std y+15, r4 \n\t" \
    "std y+16, r5 \n\t" \
    "std y+17, r6 \n\t" \
    "std y+18, r7 \n\t" \
    "std y+19, r8 \n\t" \
    "std y+20, r9 \n\t" \
    "std y+21, r10 \n\t" \
    "std y+22, r11 \n\t" \
    "std y+23, r12 \n\t" \
    "std y+24, r13 \n\t" \
    "eor r18, r19 \n\t" \
    "com r18 \n\t" \
    "push r18 \n\t" \
    "movw r26, r28 \n\t" \
    "adiw r26, 1 \n\t" \
    "movw r30, r28 \n\t" \
    "adiw r30, 25 \n\t" \
    "adiw r28, 13 \n\t" \
    "rcall 91f \n\t" \
    "pop r18 \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "adiw r28, 24 \n\t" \
    /* Atomic ISR-safe SP update. */ \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r29 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r28 \n\t" \
    "movw r26, r28 \n\t" \
    "adiw r26, 1 \n\t" \
    "ldd r31, y+25 \n\t" \
    "ldd r30, y+26 \n\t" \
    "ldd r2, z+0 \n\t" \
    "ldd r3, z+1 \n\t" \
    "ldd r4, z+2 \n\t" \
    "ldd r5, z+3 \n\t" \
    "ldd r6, z+4 \n\t" \
    "ldd r7, z+5 \n\t" \
    "ldd r8, z+6 \n\t" \
    "ldd r9, z+7 \n\t" \
    "ldd r10, z+8 \n\t" \
    "ldd r11, z+9 \n\t" \
    "ldd r12, z+10 \n\t" \
    "ldd r13, z+11 \n\t" \
    "ldd r20, z+24 \n\t" \
    "add r2, r20 \n\t" \
    "ldd r20, z+25 \n\t" \
    "adc r3, r20 \n\t" \
    "ldd r20, z+26 \n\t" \
    "adc r4, r20 \n\t" \
    "ldd r20, z+27 \n\t" \
    "adc r5, r20 \n\t" \
    "ldd r20, z+28 \n\t" \
    "adc r6, r20 \n\t" \
    "ldd r20, z+29 \n\t" \
    "adc r7, r20 \n\t" \
    "ldd r20, z+30 \n\t" \
    "adc r8, r20 \n\t" \
    "ldd r20, z+31 \n\t" \
    "adc r9, r20 \n\t" \
    "ldd r20, z+32 \n\t" \
    "adc r10, r20 \n\t" \
    "ldd r20, z+33 \n\t" \
    "adc r11, r20 \n\t" \
    "ldd r20, z+34 \n\t" \
    "adc r12, r20 \n\t" \
    "ldd r20, z+35 \n\t" \
    "adc r13, r20 \n\t" \
    "clr r19 \n\t" \
    "adc r19, r19 \n\t" \
    "mov r0, r18 \n\t" \
    "add r0, r0 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r2, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r3, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r4, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r5, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r6, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r7, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r8, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r9, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r10, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r11, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r12, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r13, r20 \n\t" \
    "clr r21 \n\t" \
    "adc r21, r21 \n\t" \
    "ldd r20, z+12 \n\t" \
    "add r2, r20 \n\t" \
    "std z+12, r2 \n\t" \
    "mov r2, r20 \n\t" \
    "ldd r20, z+13 \n\t" \
    "adc r3, r20 \n\t" \
    "std z+13, r3 \n\t" \
    "mov r3, r20 \n\t" \
    "ldd r20, z+14 \n\t" \
    "adc r4, r20 \n\t" \
    "std z+14, r4 \n\t" \
    "mov r4, r20 \n\t" \
    "ldd r20, z+15 \n\t" \
    "adc r5, r20 \n\t" \
    "std z+15, r5 \n\t" \
    "mov r5, r20 \n\t" \
    "ldd r20, z+16 \n\t" \
    "adc r6, r20 \n\t" \
    "std z+16, r6 \n\t" \
    "mov r6, r20 \n\t" \
    "ldd r20, z+17 \n\t" \
    "adc r7, r20 \n\t" \
    "std z+17, r7 \n\t" \
    "mov r7, r20 \n\t" \
    "ldd r20, z+18 \n\t" \
    "adc r8, r20 \n\t" \
    "std z+18, r8 \n\t" \
    "mov r8, r20 \n\t" \
    "ldd r20, z+19 \n\t" \
    "adc r9, r20 \n\t" \
    "std z+19, r9 \n\t" \
    "mov r9, r20 \n\t" \
    "ldd r20, z+20 \n\t" \
    "adc r10, r20 \n\t" \
    "std z+20, r10 \n\t" \
    "mov r10, r20 \n\t" \
    "ldd r20, z+21 \n\t" \
    "adc r11, r20 \n\t" \
    "std z+21, r11 \n\t" \
    "mov r11, r20 \n\t" \
    "ldd r20, z+22 \n\t" \
    "adc r12, r20 \n\t" \
    "std z+22, r12 \n\t" \
    "mov r12, r20 \n\t" \
    "ldd r20, z+23 \n\t" \
    "adc r13, r20 \n\t" \
    "std z+23, r13 \n\t" \
    "mov r13, r20 \n\t" \
    "clr r23 \n\t" \
    "adc r23, r23 \n\t" \
    "lsr r19 \n\t" \
    "ldd r20, z+36 \n\t" \
    "adc r2, r20 \n\t" \
    "ldd r20, z+37 \n\t" \
    "adc r3, r20 \n\t" \
    "ldd r20, z+38 \n\t" \
    "adc r4, r20 \n\t" \
    "ldd r20, z+39 \n\t" \
    "adc r5, r20 \n\t" \
    "ldd r20, z+40 \n\t" \
    "adc r6, r20 \n\t" \
    "ldd r20, z+41 \n\t" \
    "adc r7, r20 \n\t" \
    "ldd r20, z+42 \n\t" \
    "adc r8, r20 \n\t" \
    "ldd r20, z+43 \n\t" \
    "adc r9, r20 \n\t" \
    "ldd r20, z+44 \n\t" \
    "adc r10, r20 \n\t" \
    "ldd r20, z+45 \n\t" \
    "adc r11, r20 \n\t" \
    "ldd r20, z+46 \n\t" \
    "adc r12, r20 \n\t" \
    "ldd r20, z+47 \n\t" \
    "adc r13, r20 \n\t" \
    "adc r19, r19 \n\t" \
    "lsr r21 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r2, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r3, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r4, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r5, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r6, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r7, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r8, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r9, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r10, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r11, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r12, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r13, r20 \n\t" \
    "adc r19, r18 \n\t" \
    "lsr r23 \n\t" \
    "ldd r20, z+24 \n\t" \
    "adc r2, r20 \n\t" \
    "std z+24, r2 \n\t" \
    "ldd r20, z+25 \n\t" \
    "adc r3, r20 \n\t" \
    "std z+25, r3 \n\t" \
    "ldd r20, z+26 \n\t" \
    "adc r4, r20 \n\t" \
    "std z+26, r4 \n\t" \
    "ldd r20, z+27 \n\t" \
    "adc r5, r20 \n\t" \
    "std z+27, r5 \n\t" \
    "ldd r20, z+28 \n\t" \
    "adc r6, r20 \n\t" \
    "std z+28, r6 \n\t" \
    "ldd r20, z+29 \n\t" \
    "adc r7, r20 \n\t" \
    "std z+29, r7 \n\t" \
    "ldd r20, z+30 \n\t" \
    "adc r8, r20 \n\t" \
    "std z+30, r8 \n\t" \
    "ldd r20, z+31 \n\t" \
    "adc r9, r20 \n\t" \
    "std z+31, r9 \n\t" \
    "ldd r20, z+32 \n\t" \
    "adc r10, r20 \n\t" \
    "std z+32, r10 \n\t" \
    "ldd r20, z+33 \n\t" \
    "adc r11, r20 \n\t" \
    "std z+33, r11 \n\t" \
    "ldd r20, z+34 \n\t" \
    "adc r12, r20 \n\t" \
    "std z+34, r12 \n\t" \
    "ldd r20, z+35 \n\t" \
    "adc r13, r20 \n\t" \
    "std z+35, r13 \n\t" \
    "adc r19, r23 \n\t" \
    "mov r20, r19 \n\t" \
    "lsl r20 \n\t" \
    "sbc r20, r20 \n\t" \
    "ldd r0, z+36 \n\t" \
    "add r0, r19 \n\t" \
    "std z+36, r0 \n\t" \
    "ldd r0, z+37 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+37, r0 \n\t" \
    "ldd r0, z+38 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+38, r0 \n\t" \
    "ldd r0, z+39 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+39, r0 \n\t" \
    "ldd r0, z+40 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+40, r0 \n\t" \
    "ldd r0, z+41 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+41, r0 \n\t" \
    "ldd r0, z+42 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+42, r0 \n\t" \
    "ldd r0, z+43 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+43, r0 \n\t" \
    "ldd r0, z+44 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+44, r0 \n\t" \
    "ldd r0, z+45 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+45, r0 \n\t" \
    "ldd r0, z+46 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+46, r0 \n\t" \
    "ldd r0, z+47 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+47, r0 \n\t" \
    "adiw r28, 30 \n\t" \
    /* Atomic ISR-safe SP update. */ \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r29 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r28 \n\t"

/* Shared 96-bit (12x12 -> 24-byte) unrolled leaf multiplier for FAST_MULT_ASM_24. */
#define FAST_MULT_LEAF_24 \
    "91: \n\t" \
    "clr r20 \n\t" \
    "clr r21 \n\t" \
    "movw r22, r20 \n\t" \
    "movw r24, r20 \n\t" \
    "ld r2, x+ \n\t" \
    "ldd r8, y+0 \n\t" \
    "ldd r9, y+1 \n\t" \
    "ldd r10, y+2 \n\t" \
    "ldd r11, y+3 \n\t" \
    "ldd r12, y+4 \n\t" \
    "ldd r13, y+5 \n\t" \
    "mul r2, r10 \n\t" \
    "movw r16, r0 \n\t" \
    "mul r2, r8 \n\t" \
    "movw r14, r0 \n\t" \
    "mul r2, r9 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r25 \n\t" \
    "mul r2, r12 \n\t" \
    "movw r18, r0 \n\t" \
    "mul r2, r11 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r19, r25 \n\t" \
    "mul r2, r13 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "ld r3, x+ \n\t" \
    "mul r3, r10 \n\t" \
    "movw r6, r0 \n\t" \
    "mul r3, r8 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r6 \n\t" \
    "adc r7, r25 \n\t" \
    "mul r3, r9 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r7, r25 \n\t" \
    "mul r3, r12 \n\t" \
    "add r18, r7 \n\t" \
    "adc r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r25 \n\t" \
    "mul r3, r11 \n\t" \
    "movw r6, r0 \n\t" \
    "mul r3, r13 \n\t" \
    "add r18, r6 \n\t" \
    "adc r19, r7 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "ld r4, x+ \n\t" \
    "mul r4, r10 \n\t" \
    "movw r6, r0 \n\t" \
    "mul r4, r8 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r18, r6 \n\t" \
    "adc r7, r25 \n\t" \
    "mul r4, r9 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r7, r25 \n\t" \
    "mul r4, r12 \n\t" \
    "add r19, r7 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r4, r11 \n\t" \
    "movw r6, r0 \n\t" \
    "mul r4, r13 \n\t" \
    "add r19, r6 \n\t" \
    "adc r20, r7 \n\t" \
    "adc r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "std z+0, r14 \n\t" \
    "std z+1, r15 \n\t" \
    "std z+2, r16 \n\t" \
    "ld r5, x+ \n\t" \
    "mul r5, r10 \n\t" \
    "movw r14, r0 \n\t" \
    "mul r5, r8 \n\t" \
    "add r17, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r19, r14 \n\t" \
    "adc r15, r25 \n\t" \
    "mul r5, r9 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r15, r25 \n\t" \
    "mul r5, r12 \n\t" \
    "add r20, r15 \n\t" \
    "adc r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r5, r11 \n\t" \
    "movw r14, r0 \n\t" \
    "mul r5, r13 \n\t" \
    "add r20, r14 \n\t" \
    "adc r21, r15 \n\t" \
    "adc r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "ld r6, x+ \n\t" \
    "mul r6, r10 \n\t" \
    "movw r14, r0 \n\t" \
    "mul r6, r8 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r14 \n\t" \
    "adc r15, r25 \n\t" \
    "mul r6, r9 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r15, r25 \n\t" \
    "mul r6, r12 \n\t" \
    "add r21, r15 \n\t" \
    "adc r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r11 \n\t" \
    "movw r14, r0 \n\t" \
    "mul r6, r13 \n\t" \
    "add r21, r14 \n\t" \
    "adc r22, r15 \n\t" \
    "adc r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "ld r7, x+ \n\t" \
    "mul r7, r10 \n\t" \
    "movw r14, r0 \n\t" \
    "mul r7, r8 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r14 \n\t" \
    "adc r15, r25 \n\t" \
    "mul r7, r9 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r15, r25 \n\t" \
    "mul r7, r12 \n\t" \
    "add r22, r15 \n\t" \
    "adc r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r25 \n\t" \
    "mul r7, r11 \n\t" \
    "movw r14, r0 \n\t" \
    "mul r7, r13 \n\t" \
    "add r22, r14 \n\t" \
    "adc r23, r15 \n\t" \
    "adc r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "std z+3, r17 \n\t" \
    "std z+4, r18 \n\t" \
    "std z+5, r19 \n\t" \
    "ld r14, x+ \n\t" \
    "ld r15, x+ \n\t" \
    "ld r16, x+ \n\t" \
    "ld r17, x+ \n\t" \
    "ld r18, x+ \n\t" \
    "ld r19, x+ \n\t" \
    "ldd r8, y+6 \n\t" \
    "ldd r9, y+7 \n\t" \
    "ldd r10, y+8 \n\t" \
    "ldd r11, y+9 \n\t" \
    "ldd r12, y+10 \n\t" \
    "ldd r13, y+11 \n\t" \
    "push r30 \n\t" \
    "push r31 \n\t" \
    "clr r30 \n\t" \
    "clr r31 \n\t" \
    "movw r2, r30 \n\t" \
    "movw r4, r30 \n\t" \
    "movw r6, r30 \n\t" \
    "mul r8, r14 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r31 \n\t" \
    "adc r6, r31 \n\t" \
    "mul r8, r15 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r6, r31 \n\t" \
    "mul r9, r14 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r6 \n\t" \
    "adc r7, r31 \n\t" \
    "clr r6 \n\t" \
    "mul r8, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r7, r31 \n\t" \
    "mul r9, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r7, r31 \n\t" \
    "mul r10, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r7 \n\t" \
    "adc r6, r31 \n\t" \
    "clr r7 \n\t" \
    "mul r8, r17 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r6, r31 \n\t" \
    "mul r9, r16 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r6, r31 \n\t" \
    "mul r10, r15 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r6, r31 \n\t" \
    "mul r11, r14 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r6 \n\t" \
    "adc r7, r31 \n\t" \
    "clr r6 \n\t" \
    "mul r8, r18 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r7, r31 \n\t" \
    "mul r9, r17 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r7, r31 \n\t" \
    "mul r10, r16 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r7, r31 \n\t" \
    "mul r11, r15 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r7, r31 \n\t" \
    "mul r12, r14 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r7, r31 \n\t" \
    "clr r6 \n\t" \
    "mul r8, r19 \n\t" \
    "add r25, r0 \n\t" \
    "adc r7, r1 \n\t" \
    "adc r6, r31 \n\t" \
    "mul r9, r18 \n\t" \
    "add r25, r0 \n\t" \
    "adc r7, r1 \n\t" \
    "adc r6, r31 \n\t" \
    "mul r10, r17 \n\t" \
    "add r25, r0 \n\t" \
    "adc r7, r1 \n\t" \
    "adc r6, r31 \n\t" \
    "mul r11, r16 \n\t" \
    "add r25, r0 \n\t" \
    "adc r7, r1 \n\t" \
    "adc r6, r31 \n\t" \
    "mul r12, r15 \n\t" \
    "add r25, r0 \n\t" \
    "adc r7, r1 \n\t" \
    "adc r6, r31 \n\t" \
    "mul r13, r14 \n\t" \
    "add r25, r0 \n\t" \
    "adc r7, r1 \n\t" \
    "adc r6, r31 \n\t" \
    "mul r15, r13 \n\t" \
    "add r7, r0 \n\t" \
    "adc r6, r1 \n\t" \
    "adc r2, r31 \n\t" \
    "mul r16, r12 \n\t" \
    "add r7, r0 \n\t" \
    "adc r6, r1 \n\t" \
    "adc r2, r31 \n\t" \
    "mul r17, r11 \n\t" \
    "add r7, r0 \n\t" \
    "adc r6, r1 \n\t" \
    "adc r2, r31 \n\t" \
    "mul r18, r10 \n\t" \
    "add r7, r0 \n\t" \
    "adc r6, r1 \n\t" \
    "adc r2, r31 \n\t" \
    "mul r19, r9 \n\t" \
    "add r7, r0 \n\t" \
    "adc r6, r1 \n\t" \
    "adc r2, r31 \n\t" \
    "mul r16, r13 \n\t" \
    "add r6, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r3, r31 \n\t" \
    "mul r17, r12 \n\t" \
    "add r6, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r3, r31 \n\t" \
    "mul r18, r11 \n\t" \
    "add r6, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r3, r31 \n\t" \
    "mul r19, r10 \n\t" \
    "add r6, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r3, r31 \n\t" \
    "mul r17, r13 \n\t" \
    "add r2, r0 \n\t" \
    "adc r3, r1 \n\t" \
    "adc r4, r31 \n\t" \
    "mul r18, r12 \n\t" \
    "add r2, r0 \n\t" \
    "adc r3, r1 \n\t" \
    "adc r4, r31 \n\t" \
    "mul r19, r11 \n\t" \
    "add r2, r0 \n\t" \
    "adc r3, r1 \n\t" \
    "adc r4, r31 \n\t" \
    "mul r18, r13 \n\t" \
    "add r3, r0 \n\t" \
    "adc r4, r1 \n\t" \
    "adc r5, r31 \n\t" \
    "mul r19, r12 \n\t" \
    "add r3, r0 \n\t" \
    "adc r4, r1 \n\t" \
    "adc r5, r31 \n\t" \
    "mul r19, r13 \n\t" \
    "add r4, r0 \n\t" \
    "adc r5, r1 \n\t" \
    "pop r31 \n\t" \
    "pop r30 \n\t" \
    "push r6 \n\t" \
    "push r7 \n\t" \
    "ldd r1, y+0 \n\t" \
    "sub r8, r1 \n\t" \
    "ldd r1, y+1 \n\t" \
    "sbc r9, r1 \n\t" \
    "ldd r1, y+2 \n\t" \
    "sbc r10, r1 \n\t" \
    "ldd r1, y+3 \n\t" \
    "sbc r11, r1 \n\t" \
    "ldd r1, y+4 \n\t" \
    "sbc r12, r1 \n\t" \
    "ldd r1, y+5 \n\t" \
    "sbc r13, r1 \n\t" \
    "sbc r1, r1 \n\t" \
    "sbiw r26, 12 \n\t" \
    "ld r28, x+ \n\t" \
    "ld r29, x+ \n\t" \
    "sub r28, r14 \n\t" \
    "sbc r29, r15 \n\t" \
    "movw r14, r28 \n\t" \
    "ld r28, x+ \n\t" \
    "ld r29, x+ \n\t" \
    "sbc r28, r16 \n\t" \
    "sbc r29, r17 \n\t" \
    "movw r16, r28 \n\t" \
    "ld r28, x+ \n\t" \
    "ld r29, x+ \n\t" \
    "sbc r28, r18 \n\t" \
    "sbc r29, r19 \n\t" \
    "movw r18, r28 \n\t" \
    "sbc r0, r0 \n\t" \
    "bst r0, 0 \n\t" \
    "push r1 \n\t" \
    "ldd r0, z+0 \n\t" \
    "ldd r1, z+1 \n\t" \
    "ldd r26, z+2 \n\t" \
    "ldd r27, z+3 \n\t" \
    "ldd r28, z+4 \n\t" \
    "ldd r29, z+5 \n\t" \
    "add r0, r20 \n\t" \
    "adc r1, r21 \n\t" \
    "adc r26, r22 \n\t" \
    "adc r27, r23 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r25 \n\t" \
    "adc r20, r7 \n\t" \
    "adc r21, r6 \n\t" \
    "adc r22, r2 \n\t" \
    "adc r23, r3 \n\t" \
    "adc r24, r4 \n\t" \
    "adc r25, r5 \n\t" \
    "movw r6, r0 \n\t" \
    "clr r1 \n\t" \
    "adc r1, r1 \n\t" \
    "push r1 \n\t" \
    "mul r14, r8 \n\t" \
    "add r6, r0 \n\t" \
    "std z+6, r6 \n\t" \
    "clr r6 \n\t" \
    "adc r7, r1 \n\t" \
    "adc r6, r6 \n\t" \
    "mul r14, r9 \n\t" \
    "add r7, r0 \n\t" \
    "adc r6, r1 \n\t" \
    "add r26, r6 \n\t" \
    "clr r6 \n\t" \
    "adc r6, r6 \n\t" \
    "mul r15, r8 \n\t" \
    "add r7, r0 \n\t" \
    "std z+7, r7 \n\t" \
    "clr r7 \n\t" \
    "adc r26, r1 \n\t" \
    "adc r27, r6 \n\t" \
    "mov r6, r7 \n\t" \
    "adc r6, r7 \n\t" \
    "mul r14, r10 \n\t" \
    "add r26, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r6, r7 \n\t" \
    "mul r15, r9 \n\t" \
    "add r26, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r6, r7 \n\t" \
    "mul r16, r8 \n\t" \
    "add r26, r0 \n\t" \
    "std z+8, r26 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r6 \n\t" \
    "mov r6, r7 \n\t" \
    "adc r6, r7 \n\t" \
    "mul r14, r11 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r6, r7 \n\t" \
    "mul r15, r10 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r6, r7 \n\t" \
    "mul r16, r9 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r6, r7 \n\t" \
    "mul r17, r8 \n\t" \
    "add r27, r0 \n\t" \
    "std z+9, r27 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r6 \n\t" \
    "mov r6, r7 \n\t" \
    "adc r6, r7 \n\t" \
    "clr r26 \n\t" \
    "mul r14, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r6, r7 \n\t" \
    "mul r15, r11 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r6, r7 \n\t" \
    "mul r16, r10 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r6, r7 \n\t" \
    "mul r17, r9 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r6, r7 \n\t" \
    "mul r18, r8 \n\t" \
    "add r28, r0 \n\t" \
    "std z+10, r28 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r6, r7 \n\t" \
    "mul r14, r13 \n\t" \
    "add r29, r0 \n\t" \
    "adc r6, r1 \n\t" \
    "adc r26, r7 \n\t" \
    "mul r15, r12 \n\t" \
    "add r29, r0 \n\t" \
    "adc r6, r1 \n\t" \
    "adc r26, r7 \n\t" \
    "mul r16, r11 \n\t" \
    "add r29, r0 \n\t" \
    "adc r6, r1 \n\t" \
    "adc r26, r7 \n\t" \
    "mul r17, r10 \n\t" \
    "add r29, r0 \n\t" \
    "adc r6, r1 \n\t" \
    "adc r26, r7 \n\t" \
    "mul r18, r9 \n\t" \
    "add r29, r0 \n\t" \
    "adc r6, r1 \n\t" \
    "adc r26, r7 \n\t" \
    "mul r19, r8 \n\t" \
    "add r29, r0 \n\t" \
    "std z+11, r29 \n\t" \
    "adc r6, r1 \n\t" \
    "adc r26, r7 \n\t" \
    "clr r27 \n\t" \
    "mul r15, r13 \n\t" \
    "add r6, r0 \n\t" \
    "adc r26, r1 \n\t" \
    "adc r27, r7 \n\t" \
    "mul r16, r12 \n\t" \
    "add r6, r0 \n\t" \
    "adc r26, r1 \n\t" \
    "adc r27, r7 \n\t" \
    "mul r17, r11 \n\t" \
    "add r6, r0 \n\t" \
    "adc r26, r1 \n\t" \
    "adc r27, r7 \n\t" \
    "mul r18, r10 \n\t" \
    "add r6, r0 \n\t" \
    "adc r26, r1 \n\t" \
    "adc r27, r7 \n\t" \
    "mul r19, r9 \n\t" \
    "add r6, r0 \n\t" \
    "adc r26, r1 \n\t" \
    "adc r27, r7 \n\t" \
    "clr r28 \n\t" \
    "mul r16, r13 \n\t" \
    "add r26, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r7 \n\t" \
    "mul r17, r12 \n\t" \
    "add r26, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r7 \n\t" \
    "mul r18, r11 \n\t" \
    "add r26, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r7 \n\t" \
    "mul r19, r10 \n\t" \
    "add r26, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r7 \n\t" \
    "clr r29 \n\t" \
    "mul r17, r13 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r7 \n\t" \
    "mul r18, r12 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r7 \n\t" \
    "mul r19, r11 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r7 \n\t" \
    "mul r18, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r7, r7 \n\t" \
    "mul r19, r13 \n\t" \
    "add r29, r0 \n\t" \
    "adc r7, r1 \n\t" \
    "mul r19, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "clr r0 \n\t" \
    "adc r7, r0 \n\t" \
    "pop r1 \n\t" \
    "pop r0 \n\t" \
    "and r14, r0 \n\t" \
    "sub r20, r14 \n\t" \
    "clr r14 \n\t" \
    "and r15, r0 \n\t" \
    "sbc r21, r15 \n\t" \
    "and r16, r0 \n\t" \
    "sbc r22, r16 \n\t" \
    "and r17, r0 \n\t" \
    "sbc r23, r17 \n\t" \
    "and r18, r0 \n\t" \
    "sbc r24, r18 \n\t" \
    "and r19, r0 \n\t" \
    "sbc r25, r19 \n\t" \
    "sbc r1, r14 \n\t" \
    "mov r15, r14 \n\t" \
    "bld r15, 0 \n\t" \
    "neg r15 \n\t" \
    "and r0, r15 \n\t" \
    "sub r1, r0 \n\t" \
    "and r8, r15 \n\t" \
    "sub r20, r8 \n\t" \
    "and r9, r15 \n\t" \
    "sbc r21, r9 \n\t" \
    "and r10, r15 \n\t" \
    "sbc r22, r10 \n\t" \
    "and r11, r15 \n\t" \
    "sbc r23, r11 \n\t" \
    "and r12, r15 \n\t" \
    "sbc r24, r12 \n\t" \
    "and r13, r15 \n\t" \
    "sbc r25, r13 \n\t" \
    "sbc r1, r14 \n\t" \
    "pop r0 \n\t" \
    "pop r8 \n\t" \
    "mov r14, r1 \n\t" \
    "asr r14 \n\t" \
    "add r20, r6 \n\t" \
    "adc r21, r26 \n\t" \
    "adc r22, r27 \n\t" \
    "adc r23, r28 \n\t" \
    "adc r24, r29 \n\t" \
    "adc r25, r7 \n\t" \
    "adc r0, r1 \n\t" \
    "adc r8, r14 \n\t" \
    "adc r2, r14 \n\t" \
    "adc r3, r14 \n\t" \
    "adc r4, r14 \n\t" \
    "adc r5, r14 \n\t" \
    "std z+12, r20 \n\t" \
    "std z+13, r21 \n\t" \
    "std z+14, r22 \n\t" \
    "std z+15, r23 \n\t" \
    "std z+16, r24 \n\t" \
    "std z+17, r25 \n\t" \
    "std z+18, r0 \n\t" \
    "std z+19, r8 \n\t" \
    "std z+20, r2 \n\t" \
    "std z+21, r3 \n\t" \
    "std z+22, r4 \n\t" \
    "std z+23, r5 \n\t" \
    "ret \n\t"

/* 224-bit (28x28 -> 56-byte) 1-level branch-free Karatsuba multiplier
   (calls 14-byte leaf subroutine at label 92f three times). */
#define FAST_MULT_ASM_28 \
    "push r26 \n\t" \
    "push r27 \n\t" \
    "push r28 \n\t" \
    "push r29 \n\t" \
    "push r30 \n\t" \
    "push r31 \n\t" \
    "rcall 92f \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "ldd r31, y+1 \n\t" \
    "ldd r30, y+2 \n\t" \
    "ldd r1, y+3 \n\t" \
    "ldd r0, y+4 \n\t" \
    "ldd r27, y+5 \n\t" \
    "ldd r26, y+6 \n\t" \
    "movw r28, r0 \n\t" \
    "adiw r26, 14 \n\t" \
    "adiw r28, 14 \n\t" \
    "adiw r30, 28 \n\t" \
    "rcall 92f \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "ldd r27, y+5 \n\t" \
    "ldd r26, y+6 \n\t" \
    "ldd r31, y+3 \n\t" \
    "ldd r30, y+4 \n\t" \
    "sbiw r28, 56 \n\t" \
    /* Atomic ISR-safe SP update. */ \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r29 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r28 \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ld r6, x+ \n\t" \
    "ld r7, x+ \n\t" \
    "ld r8, x+ \n\t" \
    "ld r9, x+ \n\t" \
    "ld r10, x+ \n\t" \
    "ld r11, x+ \n\t" \
    "ld r12, x+ \n\t" \
    "ld r13, x+ \n\t" \
    "ld r14, x+ \n\t" \
    "ld r15, x+ \n\t" \
    "ld r0, x+ \n\t" \
    "sub r2, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r3, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r4, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r5, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r6, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r7, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r8, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r9, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r10, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r11, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r12, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r13, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r14, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r15, r0 \n\t" \
    "sbc r18, r18 \n\t" \
    "eor r2, r18 \n\t" \
    "eor r3, r18 \n\t" \
    "eor r4, r18 \n\t" \
    "eor r5, r18 \n\t" \
    "eor r6, r18 \n\t" \
    "eor r7, r18 \n\t" \
    "eor r8, r18 \n\t" \
    "eor r9, r18 \n\t" \
    "eor r10, r18 \n\t" \
    "eor r11, r18 \n\t" \
    "eor r12, r18 \n\t" \
    "eor r13, r18 \n\t" \
    "eor r14, r18 \n\t" \
    "eor r15, r18 \n\t" \
    "sub r2, r18 \n\t" \
    "sbc r3, r18 \n\t" \
    "sbc r4, r18 \n\t" \
    "sbc r5, r18 \n\t" \
    "sbc r6, r18 \n\t" \
    "sbc r7, r18 \n\t" \
    "sbc r8, r18 \n\t" \
    "sbc r9, r18 \n\t" \
    "sbc r10, r18 \n\t" \
    "sbc r11, r18 \n\t" \
    "sbc r12, r18 \n\t" \
    "sbc r13, r18 \n\t" \
    "sbc r14, r18 \n\t" \
    "sbc r15, r18 \n\t" \
    "std y+1, r2 \n\t" \
    "std y+2, r3 \n\t" \
    "std y+3, r4 \n\t" \
    "std y+4, r5 \n\t" \
    "std y+5, r6 \n\t" \
    "std y+6, r7 \n\t" \
    "std y+7, r8 \n\t" \
    "std y+8, r9 \n\t" \
    "std y+9, r10 \n\t" \
    "std y+10, r11 \n\t" \
    "std y+11, r12 \n\t" \
    "std y+12, r13 \n\t" \
    "std y+13, r14 \n\t" \
    "std y+14, r15 \n\t" \
    "ldd r2, z+0 \n\t" \
    "ldd r3, z+1 \n\t" \
    "ldd r4, z+2 \n\t" \
    "ldd r5, z+3 \n\t" \
    "ldd r6, z+4 \n\t" \
    "ldd r7, z+5 \n\t" \
    "ldd r8, z+6 \n\t" \
    "ldd r9, z+7 \n\t" \
    "ldd r10, z+8 \n\t" \
    "ldd r11, z+9 \n\t" \
    "ldd r12, z+10 \n\t" \
    "ldd r13, z+11 \n\t" \
    "ldd r14, z+12 \n\t" \
    "ldd r15, z+13 \n\t" \
    "ldd r0, z+14 \n\t" \
    "sub r2, r0 \n\t" \
    "ldd r0, z+15 \n\t" \
    "sbc r3, r0 \n\t" \
    "ldd r0, z+16 \n\t" \
    "sbc r4, r0 \n\t" \
    "ldd r0, z+17 \n\t" \
    "sbc r5, r0 \n\t" \
    "ldd r0, z+18 \n\t" \
    "sbc r6, r0 \n\t" \
    "ldd r0, z+19 \n\t" \
    "sbc r7, r0 \n\t" \
    "ldd r0, z+20 \n\t" \
    "sbc r8, r0 \n\t" \
    "ldd r0, z+21 \n\t" \
    "sbc r9, r0 \n\t" \
    "ldd r0, z+22 \n\t" \
    "sbc r10, r0 \n\t" \
    "ldd r0, z+23 \n\t" \
    "sbc r11, r0 \n\t" \
    "ldd r0, z+24 \n\t" \
    "sbc r12, r0 \n\t" \
    "ldd r0, z+25 \n\t" \
    "sbc r13, r0 \n\t" \
    "ldd r0, z+26 \n\t" \
    "sbc r14, r0 \n\t" \
    "ldd r0, z+27 \n\t" \
    "sbc r15, r0 \n\t" \
    "sbc r19, r19 \n\t" \
    "eor r2, r19 \n\t" \
    "eor r3, r19 \n\t" \
    "eor r4, r19 \n\t" \
    "eor r5, r19 \n\t" \
    "eor r6, r19 \n\t" \
    "eor r7, r19 \n\t" \
    "eor r8, r19 \n\t" \
    "eor r9, r19 \n\t" \
    "eor r10, r19 \n\t" \
    "eor r11, r19 \n\t" \
    "eor r12, r19 \n\t" \
    "eor r13, r19 \n\t" \
    "eor r14, r19 \n\t" \
    "eor r15, r19 \n\t" \
    "sub r2, r19 \n\t" \
    "sbc r3, r19 \n\t" \
    "sbc r4, r19 \n\t" \
    "sbc r5, r19 \n\t" \
    "sbc r6, r19 \n\t" \
    "sbc r7, r19 \n\t" \
    "sbc r8, r19 \n\t" \
    "sbc r9, r19 \n\t" \
    "sbc r10, r19 \n\t" \
    "sbc r11, r19 \n\t" \
    "sbc r12, r19 \n\t" \
    "sbc r13, r19 \n\t" \
    "sbc r14, r19 \n\t" \
    "sbc r15, r19 \n\t" \
    "std y+15, r2 \n\t" \
    "std y+16, r3 \n\t" \
    "std y+17, r4 \n\t" \
    "std y+18, r5 \n\t" \
    "std y+19, r6 \n\t" \
    "std y+20, r7 \n\t" \
    "std y+21, r8 \n\t" \
    "std y+22, r9 \n\t" \
    "std y+23, r10 \n\t" \
    "std y+24, r11 \n\t" \
    "std y+25, r12 \n\t" \
    "std y+26, r13 \n\t" \
    "std y+27, r14 \n\t" \
    "std y+28, r15 \n\t" \
    "eor r18, r19 \n\t" \
    "com r18 \n\t" \
    "push r18 \n\t" \
    "movw r26, r28 \n\t" \
    "adiw r26, 1 \n\t" \
    "movw r30, r28 \n\t" \
    "adiw r30, 29 \n\t" \
    "adiw r28, 15 \n\t" \
    "rcall 92f \n\t" \
    "pop r18 \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "adiw r28, 28 \n\t" \
    /* Atomic ISR-safe SP update. */ \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r29 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r28 \n\t" \
    "movw r26, r28 \n\t" \
    "adiw r26, 1 \n\t" \
    "ldd r31, y+29 \n\t" \
    "ldd r30, y+30 \n\t" \
    "ldd r2, z+0 \n\t" \
    "ldd r3, z+1 \n\t" \
    "ldd r4, z+2 \n\t" \
    "ldd r5, z+3 \n\t" \
    "ldd r6, z+4 \n\t" \
    "ldd r7, z+5 \n\t" \
    "ldd r8, z+6 \n\t" \
    "ldd r9, z+7 \n\t" \
    "ldd r10, z+8 \n\t" \
    "ldd r11, z+9 \n\t" \
    "ldd r12, z+10 \n\t" \
    "ldd r13, z+11 \n\t" \
    "ldd r14, z+12 \n\t" \
    "ldd r15, z+13 \n\t" \
    "ldd r20, z+28 \n\t" \
    "add r2, r20 \n\t" \
    "ldd r20, z+29 \n\t" \
    "adc r3, r20 \n\t" \
    "ldd r20, z+30 \n\t" \
    "adc r4, r20 \n\t" \
    "ldd r20, z+31 \n\t" \
    "adc r5, r20 \n\t" \
    "ldd r20, z+32 \n\t" \
    "adc r6, r20 \n\t" \
    "ldd r20, z+33 \n\t" \
    "adc r7, r20 \n\t" \
    "ldd r20, z+34 \n\t" \
    "adc r8, r20 \n\t" \
    "ldd r20, z+35 \n\t" \
    "adc r9, r20 \n\t" \
    "ldd r20, z+36 \n\t" \
    "adc r10, r20 \n\t" \
    "ldd r20, z+37 \n\t" \
    "adc r11, r20 \n\t" \
    "ldd r20, z+38 \n\t" \
    "adc r12, r20 \n\t" \
    "ldd r20, z+39 \n\t" \
    "adc r13, r20 \n\t" \
    "ldd r20, z+40 \n\t" \
    "adc r14, r20 \n\t" \
    "ldd r20, z+41 \n\t" \
    "adc r15, r20 \n\t" \
    "clr r19 \n\t" \
    "adc r19, r19 \n\t" \
    "mov r0, r18 \n\t" \
    "add r0, r0 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r2, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r3, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r4, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r5, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r6, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r7, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r8, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r9, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r10, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r11, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r12, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r13, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r14, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r15, r20 \n\t" \
    "clr r21 \n\t" \
    "adc r21, r21 \n\t" \
    "ldd r20, z+14 \n\t" \
    "add r2, r20 \n\t" \
    "std z+14, r2 \n\t" \
    "mov r2, r20 \n\t" \
    "ldd r20, z+15 \n\t" \
    "adc r3, r20 \n\t" \
    "std z+15, r3 \n\t" \
    "mov r3, r20 \n\t" \
    "ldd r20, z+16 \n\t" \
    "adc r4, r20 \n\t" \
    "std z+16, r4 \n\t" \
    "mov r4, r20 \n\t" \
    "ldd r20, z+17 \n\t" \
    "adc r5, r20 \n\t" \
    "std z+17, r5 \n\t" \
    "mov r5, r20 \n\t" \
    "ldd r20, z+18 \n\t" \
    "adc r6, r20 \n\t" \
    "std z+18, r6 \n\t" \
    "mov r6, r20 \n\t" \
    "ldd r20, z+19 \n\t" \
    "adc r7, r20 \n\t" \
    "std z+19, r7 \n\t" \
    "mov r7, r20 \n\t" \
    "ldd r20, z+20 \n\t" \
    "adc r8, r20 \n\t" \
    "std z+20, r8 \n\t" \
    "mov r8, r20 \n\t" \
    "ldd r20, z+21 \n\t" \
    "adc r9, r20 \n\t" \
    "std z+21, r9 \n\t" \
    "mov r9, r20 \n\t" \
    "ldd r20, z+22 \n\t" \
    "adc r10, r20 \n\t" \
    "std z+22, r10 \n\t" \
    "mov r10, r20 \n\t" \
    "ldd r20, z+23 \n\t" \
    "adc r11, r20 \n\t" \
    "std z+23, r11 \n\t" \
    "mov r11, r20 \n\t" \
    "ldd r20, z+24 \n\t" \
    "adc r12, r20 \n\t" \
    "std z+24, r12 \n\t" \
    "mov r12, r20 \n\t" \
    "ldd r20, z+25 \n\t" \
    "adc r13, r20 \n\t" \
    "std z+25, r13 \n\t" \
    "mov r13, r20 \n\t" \
    "ldd r20, z+26 \n\t" \
    "adc r14, r20 \n\t" \
    "std z+26, r14 \n\t" \
    "mov r14, r20 \n\t" \
    "ldd r20, z+27 \n\t" \
    "adc r15, r20 \n\t" \
    "std z+27, r15 \n\t" \
    "mov r15, r20 \n\t" \
    "clr r23 \n\t" \
    "adc r23, r23 \n\t" \
    "lsr r19 \n\t" \
    "ldd r20, z+42 \n\t" \
    "adc r2, r20 \n\t" \
    "ldd r20, z+43 \n\t" \
    "adc r3, r20 \n\t" \
    "ldd r20, z+44 \n\t" \
    "adc r4, r20 \n\t" \
    "ldd r20, z+45 \n\t" \
    "adc r5, r20 \n\t" \
    "ldd r20, z+46 \n\t" \
    "adc r6, r20 \n\t" \
    "ldd r20, z+47 \n\t" \
    "adc r7, r20 \n\t" \
    "ldd r20, z+48 \n\t" \
    "adc r8, r20 \n\t" \
    "ldd r20, z+49 \n\t" \
    "adc r9, r20 \n\t" \
    "ldd r20, z+50 \n\t" \
    "adc r10, r20 \n\t" \
    "ldd r20, z+51 \n\t" \
    "adc r11, r20 \n\t" \
    "ldd r20, z+52 \n\t" \
    "adc r12, r20 \n\t" \
    "ldd r20, z+53 \n\t" \
    "adc r13, r20 \n\t" \
    "ldd r20, z+54 \n\t" \
    "adc r14, r20 \n\t" \
    "ldd r20, z+55 \n\t" \
    "adc r15, r20 \n\t" \
    "adc r19, r19 \n\t" \
    "lsr r21 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r2, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r3, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r4, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r5, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r6, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r7, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r8, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r9, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r10, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r11, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r12, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r13, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r14, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r15, r20 \n\t" \
    "adc r19, r18 \n\t" \
    "lsr r23 \n\t" \
    "ldd r20, z+28 \n\t" \
    "adc r2, r20 \n\t" \
    "std z+28, r2 \n\t" \
    "ldd r20, z+29 \n\t" \
    "adc r3, r20 \n\t" \
    "std z+29, r3 \n\t" \
    "ldd r20, z+30 \n\t" \
    "adc r4, r20 \n\t" \
    "std z+30, r4 \n\t" \
    "ldd r20, z+31 \n\t" \
    "adc r5, r20 \n\t" \
    "std z+31, r5 \n\t" \
    "ldd r20, z+32 \n\t" \
    "adc r6, r20 \n\t" \
    "std z+32, r6 \n\t" \
    "ldd r20, z+33 \n\t" \
    "adc r7, r20 \n\t" \
    "std z+33, r7 \n\t" \
    "ldd r20, z+34 \n\t" \
    "adc r8, r20 \n\t" \
    "std z+34, r8 \n\t" \
    "ldd r20, z+35 \n\t" \
    "adc r9, r20 \n\t" \
    "std z+35, r9 \n\t" \
    "ldd r20, z+36 \n\t" \
    "adc r10, r20 \n\t" \
    "std z+36, r10 \n\t" \
    "ldd r20, z+37 \n\t" \
    "adc r11, r20 \n\t" \
    "std z+37, r11 \n\t" \
    "ldd r20, z+38 \n\t" \
    "adc r12, r20 \n\t" \
    "std z+38, r12 \n\t" \
    "ldd r20, z+39 \n\t" \
    "adc r13, r20 \n\t" \
    "std z+39, r13 \n\t" \
    "ldd r20, z+40 \n\t" \
    "adc r14, r20 \n\t" \
    "std z+40, r14 \n\t" \
    "ldd r20, z+41 \n\t" \
    "adc r15, r20 \n\t" \
    "std z+41, r15 \n\t" \
    "adc r19, r23 \n\t" \
    "mov r20, r19 \n\t" \
    "lsl r20 \n\t" \
    "sbc r20, r20 \n\t" \
    "ldd r0, z+42 \n\t" \
    "add r0, r19 \n\t" \
    "std z+42, r0 \n\t" \
    "ldd r0, z+43 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+43, r0 \n\t" \
    "ldd r0, z+44 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+44, r0 \n\t" \
    "ldd r0, z+45 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+45, r0 \n\t" \
    "ldd r0, z+46 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+46, r0 \n\t" \
    "ldd r0, z+47 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+47, r0 \n\t" \
    "ldd r0, z+48 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+48, r0 \n\t" \
    "ldd r0, z+49 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+49, r0 \n\t" \
    "ldd r0, z+50 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+50, r0 \n\t" \
    "ldd r0, z+51 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+51, r0 \n\t" \
    "ldd r0, z+52 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+52, r0 \n\t" \
    "ldd r0, z+53 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+53, r0 \n\t" \
    "ldd r0, z+54 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+54, r0 \n\t" \
    "ldd r0, z+55 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+55, r0 \n\t" \
    "adiw r28, 34 \n\t" \
    /* Atomic ISR-safe SP update. */ \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r29 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r28 \n\t"

/* Shared 112-bit (14x14 -> 28-byte) unrolled leaf multiplier for FAST_MULT_ASM_28. */
#define FAST_MULT_LEAF_28 \
    "92: \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ld r6, x+ \n\t" \
    "ld r7, x+ \n\t" \
    "ld r8, x+ \n\t" \
    "ldd r9, y+0 \n\t" \
    "ldd r10, y+1 \n\t" \
    "ldd r11, y+2 \n\t" \
    "ldd r12, y+3 \n\t" \
    "ldd r13, y+4 \n\t" \
    "ldd r14, y+5 \n\t" \
    "ldd r15, y+6 \n\t" \
    "clr r25 \n\t" \
    "clr r24 \n\t" \
    "mul r2, r9 \n\t" \
    "std z+0, r0 \n\t" \
    "mov r23, r1 \n\t" \
    "clr r22 \n\t" \
    "mul r2, r10 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "mul r3, r9 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "std z+1, r23 \n\t" \
    "clr r23 \n\t" \
    "mul r2, r11 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r3, r10 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r4, r9 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "std z+2, r24 \n\t" \
    "clr r24 \n\t" \
    "mul r2, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r3, r11 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r10 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r9 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "std z+3, r22 \n\t" \
    "clr r22 \n\t" \
    "mul r2, r13 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r3, r12 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r4, r11 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r5, r10 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r6, r9 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "std z+4, r23 \n\t" \
    "clr r23 \n\t" \
    "mul r2, r14 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r3, r13 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r4, r12 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r5, r11 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r6, r10 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r7, r9 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "std z+5, r24 \n\t" \
    "clr r24 \n\t" \
    "mul r2, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r3, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r11 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r7, r10 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r8, r9 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "std z+6, r22 \n\t" \
    "clr r22 \n\t" \
    "mul r3, r15 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r4, r14 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r5, r13 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r6, r12 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r7, r11 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r8, r10 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mov r16, r23 \n\t" \
    "clr r23 \n\t" \
    "mul r4, r15 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r5, r14 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r6, r13 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r7, r12 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r8, r11 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mov r17, r24 \n\t" \
    "clr r24 \n\t" \
    "mul r5, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r7, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r8, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mov r18, r22 \n\t" \
    "clr r22 \n\t" \
    "mul r6, r15 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r7, r14 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r8, r13 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mov r19, r23 \n\t" \
    "clr r23 \n\t" \
    "mul r7, r15 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r8, r14 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mov r20, r24 \n\t" \
    "mul r8, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "mov r21, r22 \n\t" \
    "mov r22, r23 \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ld r6, x+ \n\t" \
    "ld r7, x+ \n\t" \
    "ld r8, x+ \n\t" \
    "ldd r9, y+7 \n\t" \
    "ldd r10, y+8 \n\t" \
    "ldd r11, y+9 \n\t" \
    "ldd r12, y+10 \n\t" \
    "ldd r13, y+11 \n\t" \
    "ldd r14, y+12 \n\t" \
    "ldd r15, y+13 \n\t" \
    "push r26 \n\t" \
    "clr r26 \n\t" \
    "mov r23, r16 \n\t" \
    "clr r24 \n\t" \
    "mul r2, r9 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "mov r16, r23 \n\t" \
    "clr r25 \n\t" \
    "add r24, r17 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r2, r10 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "clr r23 \n\t" \
    "mul r3, r9 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "mov r17, r24 \n\t" \
    "clr r24 \n\t" \
    "add r25, r18 \n\t" \
    "adc r23, r26 \n\t" \
    "mul r2, r11 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r3, r10 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r4, r9 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mov r18, r25 \n\t" \
    "clr r25 \n\t" \
    "add r23, r19 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r2, r12 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r3, r11 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r4, r10 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r5, r9 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mov r19, r23 \n\t" \
    "clr r23 \n\t" \
    "add r24, r20 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r2, r13 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "mul r3, r12 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "mul r4, r11 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "mul r5, r10 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "mul r6, r9 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "mov r20, r24 \n\t" \
    "clr r24 \n\t" \
    "add r25, r21 \n\t" \
    "adc r23, r26 \n\t" \
    "mul r2, r14 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r3, r13 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r4, r12 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r5, r11 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r6, r10 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r7, r9 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mov r21, r25 \n\t" \
    "clr r25 \n\t" \
    "add r23, r22 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r2, r15 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r3, r14 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r4, r13 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r5, r12 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r6, r11 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r7, r10 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r8, r9 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mov r22, r23 \n\t" \
    "clr r23 \n\t" \
    "mul r3, r15 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "mul r4, r14 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "mul r5, r13 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "mul r6, r12 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "mul r7, r11 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "mul r8, r10 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "std z+21, r24 \n\t" \
    "clr r24 \n\t" \
    "mul r4, r15 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r5, r14 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r6, r13 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r7, r12 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r8, r11 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "std z+22, r25 \n\t" \
    "clr r25 \n\t" \
    "mul r5, r15 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r6, r14 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r7, r13 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "mul r8, r12 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r26 \n\t" \
    "std z+23, r23 \n\t" \
    "clr r23 \n\t" \
    "mul r6, r15 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "mul r7, r14 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "mul r8, r13 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r23, r26 \n\t" \
    "std z+24, r24 \n\t" \
    "clr r24 \n\t" \
    "mul r7, r15 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "mul r8, r14 \n\t" \
    "add r25, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r26 \n\t" \
    "std z+25, r25 \n\t" \
    "mul r8, r15 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "std z+26, r23 \n\t" \
    "std z+27, r24 \n\t" \
    "pop r26 \n\t" \
    "sbiw r26, 14 \n\t" \
    "ld r0, x+ \n\t" \
    "sub r2, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r3, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r4, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r5, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r6, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r7, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r8, r0 \n\t" \
    "sbc r24, r24 \n\t" \
    "bst r24, 0 \n\t" \
    "ldd r0, y+0 \n\t" \
    "sub r0, r9 \n\t" \
    "mov r9, r0 \n\t" \
    "ldd r0, y+1 \n\t" \
    "sbc r0, r10 \n\t" \
    "mov r10, r0 \n\t" \
    "ldd r0, y+2 \n\t" \
    "sbc r0, r11 \n\t" \
    "mov r11, r0 \n\t" \
    "ldd r0, y+3 \n\t" \
    "sbc r0, r12 \n\t" \
    "mov r12, r0 \n\t" \
    "ldd r0, y+4 \n\t" \
    "sbc r0, r13 \n\t" \
    "mov r13, r0 \n\t" \
    "ldd r0, y+5 \n\t" \
    "sbc r0, r14 \n\t" \
    "mov r14, r0 \n\t" \
    "ldd r0, y+6 \n\t" \
    "sbc r0, r15 \n\t" \
    "mov r15, r0 \n\t" \
    "sbc r25, r25 \n\t" \
    "and r24, r25 \n\t" \
    "andi r24, 1 \n\t" \
    "mov r23, r24 \n\t" \
    "mov r1, r25 \n\t" \
    "ldd r0, z+0 \n\t" \
    "add r0, r16 \n\t" \
    "std z+7, r0 \n\t" \
    "ldd r24, z+1 \n\t" \
    "adc r24, r17 \n\t" \
    "ldd r25, z+2 \n\t" \
    "adc r25, r18 \n\t" \
    "ldd r26, z+3 \n\t" \
    "adc r26, r19 \n\t" \
    "ldd r27, z+4 \n\t" \
    "adc r27, r20 \n\t" \
    "ldd r28, z+5 \n\t" \
    "adc r28, r21 \n\t" \
    "ldd r29, z+6 \n\t" \
    "adc r29, r22 \n\t" \
    "ldd r0, z+21 \n\t" \
    "adc r16, r0 \n\t" \
    "ldd r0, z+22 \n\t" \
    "adc r17, r0 \n\t" \
    "ldd r0, z+23 \n\t" \
    "adc r18, r0 \n\t" \
    "ldd r0, z+24 \n\t" \
    "adc r19, r0 \n\t" \
    "ldd r0, z+25 \n\t" \
    "adc r20, r0 \n\t" \
    "ldd r0, z+26 \n\t" \
    "adc r21, r0 \n\t" \
    "ldd r0, z+27 \n\t" \
    "adc r22, r0 \n\t" \
    "clr r0 \n\t" \
    "adc r23, r0 \n\t" \
    "mov r0, r2 \n\t" \
    "and r0, r1 \n\t" \
    "sub r16, r0 \n\t" \
    "mov r0, r3 \n\t" \
    "and r0, r1 \n\t" \
    "sbc r17, r0 \n\t" \
    "mov r0, r4 \n\t" \
    "and r0, r1 \n\t" \
    "sbc r18, r0 \n\t" \
    "mov r0, r5 \n\t" \
    "and r0, r1 \n\t" \
    "sbc r19, r0 \n\t" \
    "mov r0, r6 \n\t" \
    "and r0, r1 \n\t" \
    "sbc r20, r0 \n\t" \
    "mov r0, r7 \n\t" \
    "and r0, r1 \n\t" \
    "sbc r21, r0 \n\t" \
    "mov r0, r8 \n\t" \
    "and r0, r1 \n\t" \
    "sbc r22, r0 \n\t" \
    "clr r0 \n\t" \
    "sbc r23, r0 \n\t" \
    "push r23 \n\t" \
    "push r29 \n\t" \
    "push r28 \n\t" \
    "push r27 \n\t" \
    "clr r23 \n\t" \
    "ldd r27, z+7 \n\t" \
    "clr r28 \n\t" \
    "mul r2, r9 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "std z+7, r27 \n\t" \
    "clr r29 \n\t" \
    "add r28, r24 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r2, r10 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "clr r27 \n\t" \
    "mul r3, r9 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "std z+8, r28 \n\t" \
    "clr r28 \n\t" \
    "mov r0, r25 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r23 \n\t" \
    "mul r2, r11 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r3, r10 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r4, r9 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "std z+9, r29 \n\t" \
    "clr r29 \n\t" \
    "mov r0, r26 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r2, r12 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r3, r11 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r4, r10 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r5, r9 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "std z+10, r27 \n\t" \
    "clr r27 \n\t" \
    "pop r0 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r2, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "mul r3, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "mul r4, r11 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "mul r5, r10 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "mul r6, r9 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "std z+11, r28 \n\t" \
    "clr r28 \n\t" \
    "pop r0 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r23 \n\t" \
    "mul r2, r14 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r3, r13 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r4, r12 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r5, r11 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r6, r10 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r7, r9 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "std z+12, r29 \n\t" \
    "clr r29 \n\t" \
    "pop r0 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r2, r15 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r3, r14 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r4, r13 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r5, r12 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r6, r11 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r7, r10 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r8, r9 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "std z+13, r27 \n\t" \
    "clr r27 \n\t" \
    "mul r3, r15 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "mul r4, r14 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "mul r5, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "mul r6, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "mul r7, r11 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "mul r8, r10 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "mov r24, r28 \n\t" \
    "clr r28 \n\t" \
    "mul r4, r15 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r5, r14 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r6, r13 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r7, r12 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r8, r11 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mov r25, r29 \n\t" \
    "clr r29 \n\t" \
    "mul r5, r15 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r6, r14 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r7, r13 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mul r8, r12 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r23 \n\t" \
    "mov r26, r27 \n\t" \
    "clr r27 \n\t" \
    "mul r6, r15 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "mul r7, r14 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "mul r8, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r27, r23 \n\t" \
    "mov r2, r28 \n\t" \
    "clr r28 \n\t" \
    "mul r7, r15 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mul r8, r14 \n\t" \
    "add r29, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r28, r23 \n\t" \
    "mov r3, r29 \n\t" \
    "mul r8, r15 \n\t" \
    "add r27, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "mov r4, r27 \n\t" \
    "mov r5, r28 \n\t" \
    "pop r6 \n\t" \
    "clr r7 \n\t" \
    "bld r7, 0 \n\t" \
    "neg r7 \n\t" \
    "and r9, r7 \n\t" \
    "sub r16, r9 \n\t" \
    "and r10, r7 \n\t" \
    "sbc r17, r10 \n\t" \
    "and r11, r7 \n\t" \
    "sbc r18, r11 \n\t" \
    "and r12, r7 \n\t" \
    "sbc r19, r12 \n\t" \
    "and r13, r7 \n\t" \
    "sbc r20, r13 \n\t" \
    "and r14, r7 \n\t" \
    "sbc r21, r14 \n\t" \
    "and r15, r7 \n\t" \
    "sbc r22, r15 \n\t" \
    "sbc r6, r23 \n\t" \
    "add r16, r24 \n\t" \
    "std z+14, r16 \n\t" \
    "adc r17, r25 \n\t" \
    "std z+15, r17 \n\t" \
    "adc r18, r26 \n\t" \
    "std z+16, r18 \n\t" \
    "adc r19, r2 \n\t" \
    "std z+17, r19 \n\t" \
    "adc r20, r3 \n\t" \
    "std z+18, r20 \n\t" \
    "adc r21, r4 \n\t" \
    "std z+19, r21 \n\t" \
    "adc r22, r5 \n\t" \
    "std z+20, r22 \n\t" \
    "adc r6, r23 \n\t" \
    "mov r7, r6 \n\t" \
    "lsl r7 \n\t" \
    "sbc r7, r7 \n\t" \
    "ldd r0, z+21 \n\t" \
    "add r0, r6 \n\t" \
    "std z+21, r0 \n\t" \
    "ldd r0, z+22 \n\t" \
    "adc r0, r7 \n\t" \
    "std z+22, r0 \n\t" \
    "ldd r0, z+23 \n\t" \
    "adc r0, r7 \n\t" \
    "std z+23, r0 \n\t" \
    "ldd r0, z+24 \n\t" \
    "adc r0, r7 \n\t" \
    "std z+24, r0 \n\t" \
    "ldd r0, z+25 \n\t" \
    "adc r0, r7 \n\t" \
    "std z+25, r0 \n\t" \
    "ldd r0, z+26 \n\t" \
    "adc r0, r7 \n\t" \
    "std z+26, r0 \n\t" \
    "ldd r0, z+27 \n\t" \
    "adc r0, r7 \n\t" \
    "std z+27, r0 \n\t" \
    "ret \n\t"

/* 256-bit (32x32 -> 64-byte) 1-level branch-free Karatsuba multiplier
   (calls 16-byte leaf subroutine at label 93f three times). */
#define FAST_MULT_ASM_32 \
    "push r26 \n\t" \
    "push r27 \n\t" \
    "push r28 \n\t" \
    "push r29 \n\t" \
    "push r30 \n\t" \
    "push r31 \n\t" \
    "rcall 93f \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "ldd r31, y+1 \n\t" \
    "ldd r30, y+2 \n\t" \
    "ldd r1, y+3 \n\t" \
    "ldd r0, y+4 \n\t" \
    "ldd r27, y+5 \n\t" \
    "ldd r26, y+6 \n\t" \
    "movw r28, r0 \n\t" \
    "adiw r26, 16 \n\t" \
    "adiw r28, 16 \n\t" \
    "adiw r30, 32 \n\t" \
    "rcall 93f \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "ldd r27, y+5 \n\t" \
    "ldd r26, y+6 \n\t" \
    "ldd r31, y+3 \n\t" \
    "ldd r30, y+4 \n\t" \
    "sbiw r28, 32 \n\t" \
    "sbiw r28, 32 \n\t" \
    /* Atomic ISR-safe SP update. */ \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r29 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r28 \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ld r6, x+ \n\t" \
    "ld r7, x+ \n\t" \
    "ld r8, x+ \n\t" \
    "ld r9, x+ \n\t" \
    "ld r10, x+ \n\t" \
    "ld r11, x+ \n\t" \
    "ld r12, x+ \n\t" \
    "ld r13, x+ \n\t" \
    "ld r14, x+ \n\t" \
    "ld r15, x+ \n\t" \
    "ld r16, x+ \n\t" \
    "ld r17, x+ \n\t" \
    "ld r0, x+ \n\t" \
    "sub r2, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r3, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r4, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r5, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r6, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r7, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r8, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r9, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r10, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r11, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r12, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r13, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r14, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r15, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r16, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r17, r0 \n\t" \
    "sbc r18, r18 \n\t" \
    "eor r2, r18 \n\t" \
    "eor r3, r18 \n\t" \
    "eor r4, r18 \n\t" \
    "eor r5, r18 \n\t" \
    "eor r6, r18 \n\t" \
    "eor r7, r18 \n\t" \
    "eor r8, r18 \n\t" \
    "eor r9, r18 \n\t" \
    "eor r10, r18 \n\t" \
    "eor r11, r18 \n\t" \
    "eor r12, r18 \n\t" \
    "eor r13, r18 \n\t" \
    "eor r14, r18 \n\t" \
    "eor r15, r18 \n\t" \
    "eor r16, r18 \n\t" \
    "eor r17, r18 \n\t" \
    "sub r2, r18 \n\t" \
    "sbc r3, r18 \n\t" \
    "sbc r4, r18 \n\t" \
    "sbc r5, r18 \n\t" \
    "sbc r6, r18 \n\t" \
    "sbc r7, r18 \n\t" \
    "sbc r8, r18 \n\t" \
    "sbc r9, r18 \n\t" \
    "sbc r10, r18 \n\t" \
    "sbc r11, r18 \n\t" \
    "sbc r12, r18 \n\t" \
    "sbc r13, r18 \n\t" \
    "sbc r14, r18 \n\t" \
    "sbc r15, r18 \n\t" \
    "sbc r16, r18 \n\t" \
    "sbc r17, r18 \n\t" \
    "std y+1, r2 \n\t" \
    "std y+2, r3 \n\t" \
    "std y+3, r4 \n\t" \
    "std y+4, r5 \n\t" \
    "std y+5, r6 \n\t" \
    "std y+6, r7 \n\t" \
    "std y+7, r8 \n\t" \
    "std y+8, r9 \n\t" \
    "std y+9, r10 \n\t" \
    "std y+10, r11 \n\t" \
    "std y+11, r12 \n\t" \
    "std y+12, r13 \n\t" \
    "std y+13, r14 \n\t" \
    "std y+14, r15 \n\t" \
    "std y+15, r16 \n\t" \
    "std y+16, r17 \n\t" \
    "ldd r2, z+0 \n\t" \
    "ldd r3, z+1 \n\t" \
    "ldd r4, z+2 \n\t" \
    "ldd r5, z+3 \n\t" \
    "ldd r6, z+4 \n\t" \
    "ldd r7, z+5 \n\t" \
    "ldd r8, z+6 \n\t" \
    "ldd r9, z+7 \n\t" \
    "ldd r10, z+8 \n\t" \
    "ldd r11, z+9 \n\t" \
    "ldd r12, z+10 \n\t" \
    "ldd r13, z+11 \n\t" \
    "ldd r14, z+12 \n\t" \
    "ldd r15, z+13 \n\t" \
    "ldd r16, z+14 \n\t" \
    "ldd r17, z+15 \n\t" \
    "ldd r0, z+16 \n\t" \
    "sub r2, r0 \n\t" \
    "ldd r0, z+17 \n\t" \
    "sbc r3, r0 \n\t" \
    "ldd r0, z+18 \n\t" \
    "sbc r4, r0 \n\t" \
    "ldd r0, z+19 \n\t" \
    "sbc r5, r0 \n\t" \
    "ldd r0, z+20 \n\t" \
    "sbc r6, r0 \n\t" \
    "ldd r0, z+21 \n\t" \
    "sbc r7, r0 \n\t" \
    "ldd r0, z+22 \n\t" \
    "sbc r8, r0 \n\t" \
    "ldd r0, z+23 \n\t" \
    "sbc r9, r0 \n\t" \
    "ldd r0, z+24 \n\t" \
    "sbc r10, r0 \n\t" \
    "ldd r0, z+25 \n\t" \
    "sbc r11, r0 \n\t" \
    "ldd r0, z+26 \n\t" \
    "sbc r12, r0 \n\t" \
    "ldd r0, z+27 \n\t" \
    "sbc r13, r0 \n\t" \
    "ldd r0, z+28 \n\t" \
    "sbc r14, r0 \n\t" \
    "ldd r0, z+29 \n\t" \
    "sbc r15, r0 \n\t" \
    "ldd r0, z+30 \n\t" \
    "sbc r16, r0 \n\t" \
    "ldd r0, z+31 \n\t" \
    "sbc r17, r0 \n\t" \
    "sbc r19, r19 \n\t" \
    "eor r2, r19 \n\t" \
    "eor r3, r19 \n\t" \
    "eor r4, r19 \n\t" \
    "eor r5, r19 \n\t" \
    "eor r6, r19 \n\t" \
    "eor r7, r19 \n\t" \
    "eor r8, r19 \n\t" \
    "eor r9, r19 \n\t" \
    "eor r10, r19 \n\t" \
    "eor r11, r19 \n\t" \
    "eor r12, r19 \n\t" \
    "eor r13, r19 \n\t" \
    "eor r14, r19 \n\t" \
    "eor r15, r19 \n\t" \
    "eor r16, r19 \n\t" \
    "eor r17, r19 \n\t" \
    "sub r2, r19 \n\t" \
    "sbc r3, r19 \n\t" \
    "sbc r4, r19 \n\t" \
    "sbc r5, r19 \n\t" \
    "sbc r6, r19 \n\t" \
    "sbc r7, r19 \n\t" \
    "sbc r8, r19 \n\t" \
    "sbc r9, r19 \n\t" \
    "sbc r10, r19 \n\t" \
    "sbc r11, r19 \n\t" \
    "sbc r12, r19 \n\t" \
    "sbc r13, r19 \n\t" \
    "sbc r14, r19 \n\t" \
    "sbc r15, r19 \n\t" \
    "sbc r16, r19 \n\t" \
    "sbc r17, r19 \n\t" \
    "std y+17, r2 \n\t" \
    "std y+18, r3 \n\t" \
    "std y+19, r4 \n\t" \
    "std y+20, r5 \n\t" \
    "std y+21, r6 \n\t" \
    "std y+22, r7 \n\t" \
    "std y+23, r8 \n\t" \
    "std y+24, r9 \n\t" \
    "std y+25, r10 \n\t" \
    "std y+26, r11 \n\t" \
    "std y+27, r12 \n\t" \
    "std y+28, r13 \n\t" \
    "std y+29, r14 \n\t" \
    "std y+30, r15 \n\t" \
    "std y+31, r16 \n\t" \
    "std y+32, r17 \n\t" \
    "eor r18, r19 \n\t" \
    "com r18 \n\t" \
    "push r18 \n\t" \
    "movw r26, r28 \n\t" \
    "adiw r26, 1 \n\t" \
    "movw r30, r28 \n\t" \
    "adiw r30, 33 \n\t" \
    "adiw r28, 17 \n\t" \
    "rcall 93f \n\t" \
    "pop r18 \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "adiw r28, 32 \n\t" \
    /* Atomic ISR-safe SP update. */ \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r29 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r28 \n\t" \
    "movw r26, r28 \n\t" \
    "adiw r26, 1 \n\t" \
    "ldd r31, y+33 \n\t" \
    "ldd r30, y+34 \n\t" \
    "ldd r2, z+0 \n\t" \
    "ldd r3, z+1 \n\t" \
    "ldd r4, z+2 \n\t" \
    "ldd r5, z+3 \n\t" \
    "ldd r6, z+4 \n\t" \
    "ldd r7, z+5 \n\t" \
    "ldd r8, z+6 \n\t" \
    "ldd r9, z+7 \n\t" \
    "ldd r10, z+8 \n\t" \
    "ldd r11, z+9 \n\t" \
    "ldd r12, z+10 \n\t" \
    "ldd r13, z+11 \n\t" \
    "ldd r14, z+12 \n\t" \
    "ldd r15, z+13 \n\t" \
    "ldd r16, z+14 \n\t" \
    "ldd r17, z+15 \n\t" \
    "ldd r20, z+32 \n\t" \
    "add r2, r20 \n\t" \
    "ldd r20, z+33 \n\t" \
    "adc r3, r20 \n\t" \
    "ldd r20, z+34 \n\t" \
    "adc r4, r20 \n\t" \
    "ldd r20, z+35 \n\t" \
    "adc r5, r20 \n\t" \
    "ldd r20, z+36 \n\t" \
    "adc r6, r20 \n\t" \
    "ldd r20, z+37 \n\t" \
    "adc r7, r20 \n\t" \
    "ldd r20, z+38 \n\t" \
    "adc r8, r20 \n\t" \
    "ldd r20, z+39 \n\t" \
    "adc r9, r20 \n\t" \
    "ldd r20, z+40 \n\t" \
    "adc r10, r20 \n\t" \
    "ldd r20, z+41 \n\t" \
    "adc r11, r20 \n\t" \
    "ldd r20, z+42 \n\t" \
    "adc r12, r20 \n\t" \
    "ldd r20, z+43 \n\t" \
    "adc r13, r20 \n\t" \
    "ldd r20, z+44 \n\t" \
    "adc r14, r20 \n\t" \
    "ldd r20, z+45 \n\t" \
    "adc r15, r20 \n\t" \
    "ldd r20, z+46 \n\t" \
    "adc r16, r20 \n\t" \
    "ldd r20, z+47 \n\t" \
    "adc r17, r20 \n\t" \
    "clr r19 \n\t" \
    "adc r19, r19 \n\t" \
    "mov r0, r18 \n\t" \
    "add r0, r0 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r2, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r3, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r4, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r5, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r6, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r7, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r8, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r9, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r10, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r11, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r12, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r13, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r14, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r15, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r16, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r17, r20 \n\t" \
    "clr r21 \n\t" \
    "adc r21, r21 \n\t" \
    "ldd r20, z+16 \n\t" \
    "add r2, r20 \n\t" \
    "std z+16, r2 \n\t" \
    "mov r2, r20 \n\t" \
    "ldd r20, z+17 \n\t" \
    "adc r3, r20 \n\t" \
    "std z+17, r3 \n\t" \
    "mov r3, r20 \n\t" \
    "ldd r20, z+18 \n\t" \
    "adc r4, r20 \n\t" \
    "std z+18, r4 \n\t" \
    "mov r4, r20 \n\t" \
    "ldd r20, z+19 \n\t" \
    "adc r5, r20 \n\t" \
    "std z+19, r5 \n\t" \
    "mov r5, r20 \n\t" \
    "ldd r20, z+20 \n\t" \
    "adc r6, r20 \n\t" \
    "std z+20, r6 \n\t" \
    "mov r6, r20 \n\t" \
    "ldd r20, z+21 \n\t" \
    "adc r7, r20 \n\t" \
    "std z+21, r7 \n\t" \
    "mov r7, r20 \n\t" \
    "ldd r20, z+22 \n\t" \
    "adc r8, r20 \n\t" \
    "std z+22, r8 \n\t" \
    "mov r8, r20 \n\t" \
    "ldd r20, z+23 \n\t" \
    "adc r9, r20 \n\t" \
    "std z+23, r9 \n\t" \
    "mov r9, r20 \n\t" \
    "ldd r20, z+24 \n\t" \
    "adc r10, r20 \n\t" \
    "std z+24, r10 \n\t" \
    "mov r10, r20 \n\t" \
    "ldd r20, z+25 \n\t" \
    "adc r11, r20 \n\t" \
    "std z+25, r11 \n\t" \
    "mov r11, r20 \n\t" \
    "ldd r20, z+26 \n\t" \
    "adc r12, r20 \n\t" \
    "std z+26, r12 \n\t" \
    "mov r12, r20 \n\t" \
    "ldd r20, z+27 \n\t" \
    "adc r13, r20 \n\t" \
    "std z+27, r13 \n\t" \
    "mov r13, r20 \n\t" \
    "ldd r20, z+28 \n\t" \
    "adc r14, r20 \n\t" \
    "std z+28, r14 \n\t" \
    "mov r14, r20 \n\t" \
    "ldd r20, z+29 \n\t" \
    "adc r15, r20 \n\t" \
    "std z+29, r15 \n\t" \
    "mov r15, r20 \n\t" \
    "ldd r20, z+30 \n\t" \
    "adc r16, r20 \n\t" \
    "std z+30, r16 \n\t" \
    "mov r16, r20 \n\t" \
    "ldd r20, z+31 \n\t" \
    "adc r17, r20 \n\t" \
    "std z+31, r17 \n\t" \
    "mov r17, r20 \n\t" \
    "clr r23 \n\t" \
    "adc r23, r23 \n\t" \
    "lsr r19 \n\t" \
    "ldd r20, z+48 \n\t" \
    "adc r2, r20 \n\t" \
    "ldd r20, z+49 \n\t" \
    "adc r3, r20 \n\t" \
    "ldd r20, z+50 \n\t" \
    "adc r4, r20 \n\t" \
    "ldd r20, z+51 \n\t" \
    "adc r5, r20 \n\t" \
    "ldd r20, z+52 \n\t" \
    "adc r6, r20 \n\t" \
    "ldd r20, z+53 \n\t" \
    "adc r7, r20 \n\t" \
    "ldd r20, z+54 \n\t" \
    "adc r8, r20 \n\t" \
    "ldd r20, z+55 \n\t" \
    "adc r9, r20 \n\t" \
    "ldd r20, z+56 \n\t" \
    "adc r10, r20 \n\t" \
    "ldd r20, z+57 \n\t" \
    "adc r11, r20 \n\t" \
    "ldd r20, z+58 \n\t" \
    "adc r12, r20 \n\t" \
    "ldd r20, z+59 \n\t" \
    "adc r13, r20 \n\t" \
    "ldd r20, z+60 \n\t" \
    "adc r14, r20 \n\t" \
    "ldd r20, z+61 \n\t" \
    "adc r15, r20 \n\t" \
    "ldd r20, z+62 \n\t" \
    "adc r16, r20 \n\t" \
    "ldd r20, z+63 \n\t" \
    "adc r17, r20 \n\t" \
    "adc r19, r19 \n\t" \
    "lsr r21 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r2, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r3, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r4, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r5, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r6, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r7, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r8, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r9, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r10, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r11, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r12, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r13, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r14, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r15, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r16, r20 \n\t" \
    "ld r20, x+ \n\t" \
    "eor r20, r18 \n\t" \
    "adc r17, r20 \n\t" \
    "adc r19, r18 \n\t" \
    "lsr r23 \n\t" \
    "ldd r20, z+32 \n\t" \
    "adc r2, r20 \n\t" \
    "std z+32, r2 \n\t" \
    "ldd r20, z+33 \n\t" \
    "adc r3, r20 \n\t" \
    "std z+33, r3 \n\t" \
    "ldd r20, z+34 \n\t" \
    "adc r4, r20 \n\t" \
    "std z+34, r4 \n\t" \
    "ldd r20, z+35 \n\t" \
    "adc r5, r20 \n\t" \
    "std z+35, r5 \n\t" \
    "ldd r20, z+36 \n\t" \
    "adc r6, r20 \n\t" \
    "std z+36, r6 \n\t" \
    "ldd r20, z+37 \n\t" \
    "adc r7, r20 \n\t" \
    "std z+37, r7 \n\t" \
    "ldd r20, z+38 \n\t" \
    "adc r8, r20 \n\t" \
    "std z+38, r8 \n\t" \
    "ldd r20, z+39 \n\t" \
    "adc r9, r20 \n\t" \
    "std z+39, r9 \n\t" \
    "ldd r20, z+40 \n\t" \
    "adc r10, r20 \n\t" \
    "std z+40, r10 \n\t" \
    "ldd r20, z+41 \n\t" \
    "adc r11, r20 \n\t" \
    "std z+41, r11 \n\t" \
    "ldd r20, z+42 \n\t" \
    "adc r12, r20 \n\t" \
    "std z+42, r12 \n\t" \
    "ldd r20, z+43 \n\t" \
    "adc r13, r20 \n\t" \
    "std z+43, r13 \n\t" \
    "ldd r20, z+44 \n\t" \
    "adc r14, r20 \n\t" \
    "std z+44, r14 \n\t" \
    "ldd r20, z+45 \n\t" \
    "adc r15, r20 \n\t" \
    "std z+45, r15 \n\t" \
    "ldd r20, z+46 \n\t" \
    "adc r16, r20 \n\t" \
    "std z+46, r16 \n\t" \
    "ldd r20, z+47 \n\t" \
    "adc r17, r20 \n\t" \
    "std z+47, r17 \n\t" \
    "adc r19, r23 \n\t" \
    "mov r20, r19 \n\t" \
    "lsl r20 \n\t" \
    "sbc r20, r20 \n\t" \
    "ldd r0, z+48 \n\t" \
    "add r0, r19 \n\t" \
    "std z+48, r0 \n\t" \
    "ldd r0, z+49 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+49, r0 \n\t" \
    "ldd r0, z+50 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+50, r0 \n\t" \
    "ldd r0, z+51 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+51, r0 \n\t" \
    "ldd r0, z+52 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+52, r0 \n\t" \
    "ldd r0, z+53 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+53, r0 \n\t" \
    "ldd r0, z+54 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+54, r0 \n\t" \
    "ldd r0, z+55 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+55, r0 \n\t" \
    "ldd r0, z+56 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+56, r0 \n\t" \
    "ldd r0, z+57 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+57, r0 \n\t" \
    "ldd r0, z+58 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+58, r0 \n\t" \
    "ldd r0, z+59 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+59, r0 \n\t" \
    "ldd r0, z+60 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+60, r0 \n\t" \
    "ldd r0, z+61 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+61, r0 \n\t" \
    "ldd r0, z+62 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+62, r0 \n\t" \
    "ldd r0, z+63 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+63, r0 \n\t" \
    "adiw r28, 38 \n\t" \
    /* Atomic ISR-safe SP update. */ \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r29 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r28 \n\t"

/* Shared 128-bit (16x16 -> 32-byte) unrolled leaf multiplier for FAST_MULT_ASM_32. */
#define FAST_MULT_LEAF_32 \
    "93: \n\t" \
    "clr r20 \n\t" \
    "clr r21 \n\t" \
    "movw r16, r20 \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ldd r22, y+0 \n\t" \
    "ldd r23, y+1 \n\t" \
    "ldd r24, y+2 \n\t" \
    "ldd r25, y+3 \n\t" \
    "mul r2, r24 \n\t" \
    "movw r12, r0 \n\t" \
    "mul r2, r22 \n\t" \
    "movw r10, r0 \n\t" \
    "mul r2, r23 \n\t" \
    "add r11, r0 \n\t" \
    "adc r12, r1 \n\t" \
    "adc r13, r21 \n\t" \
    "mul r3, r25 \n\t" \
    "movw r14, r0 \n\t" \
    "mul r2, r25 \n\t" \
    "movw r18, r0 \n\t" \
    "mul r3, r22 \n\t" \
    "add r11, r0 \n\t" \
    "adc r12, r1 \n\t" \
    "adc r13, r18 \n\t" \
    "adc r19, r21 \n\t" \
    "mul r3, r23 \n\t" \
    "add r12, r0 \n\t" \
    "adc r13, r1 \n\t" \
    "adc r19, r21 \n\t" \
    "mul r4, r25 \n\t" \
    "add r14, r19 \n\t" \
    "adc r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "mul r4, r24 \n\t" \
    "movw r18, r0 \n\t" \
    "mul r4, r22 \n\t" \
    "add r12, r0 \n\t" \
    "adc r13, r1 \n\t" \
    "adc r14, r18 \n\t" \
    "adc r19, r21 \n\t" \
    "mul r3, r24 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "adc r19, r21 \n\t" \
    "mul r5, r25 \n\t" \
    "add r15, r19 \n\t" \
    "adc r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "mul r5, r23 \n\t" \
    "movw r18, r0 \n\t" \
    "mul r4, r23 \n\t" \
    "add r13, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r19, r21 \n\t" \
    "mul r5, r22 \n\t" \
    "add r13, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r19, r21 \n\t" \
    "mul r5, r24 \n\t" \
    "add r14, r18 \n\t" \
    "adc r0, r19 \n\t" \
    "adc r1, r21 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r21 \n\t" \
    "std z+0, r10 \n\t" \
    "std z+1, r11 \n\t" \
    "std z+2, r12 \n\t" \
    "std z+3, r13 \n\t" \
    "movw r10, r20 \n\t" \
    "ld r18, x+ \n\t" \
    "ld r19, x+ \n\t" \
    "ld r20, x+ \n\t" \
    "ldd r6, y+4 \n\t" \
    "ldd r7, y+5 \n\t" \
    "ldd r8, y+6 \n\t" \
    "ldd r9, y+7 \n\t" \
    "mul r18, r6 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r16, r21 \n\t" \
    "adc r11, r21 \n\t" \
    "mul r18, r7 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r11, r21 \n\t" \
    "mul r19, r6 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r11 \n\t" \
    "adc r10, r21 \n\t" \
    "mul r18, r8 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r10, r21 \n\t" \
    "mul r19, r7 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r10, r21 \n\t" \
    "mul r20, r6 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r10, r21 \n\t" \
    "clr r11 \n\t" \
    "mul r18, r9 \n\t" \
    "add r17, r0 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r11, r21 \n\t" \
    "mul r19, r8 \n\t" \
    "add r17, r0 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r11, r21 \n\t" \
    "mul r20, r7 \n\t" \
    "add r17, r0 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r11, r21 \n\t" \
    "sub r2, r18 \n\t" \
    "sbc r3, r19 \n\t" \
    "sbc r4, r20 \n\t" \
    "ld r18, x+ \n\t" \
    "sbc r5, r18 \n\t" \
    "sbc r0, r0 \n\t" \
    "bst r0, 0 \n\t" \
    "mul r18, r6 \n\t" \
    "add r17, r0 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r11, r21 \n\t" \
    "mul r19, r9 \n\t" \
    "clr r19 \n\t" \
    "add r10, r0 \n\t" \
    "adc r11, r1 \n\t" \
    "adc r19, r21 \n\t" \
    "mul r20, r8 \n\t" \
    "add r10, r0 \n\t" \
    "adc r11, r1 \n\t" \
    "adc r19, r21 \n\t" \
    "mul r18, r7 \n\t" \
    "add r10, r0 \n\t" \
    "adc r11, r1 \n\t" \
    "adc r19, r21 \n\t" \
    "mul r20, r9 \n\t" \
    "clr r20 \n\t" \
    "add r11, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r21 \n\t" \
    "mul r18, r8 \n\t" \
    "add r11, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r21 \n\t" \
    "mul r18, r9 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "sub r6, r22 \n\t" \
    "sbc r7, r23 \n\t" \
    "sbc r8, r24 \n\t" \
    "sbc r9, r25 \n\t" \
    "sbc r1, r1 \n\t" \
    "ldd r22, z+0 \n\t" \
    "ldd r23, z+1 \n\t" \
    "add r22, r14 \n\t" \
    "adc r23, r15 \n\t" \
    "adc r12, r16 \n\t" \
    "adc r13, r17 \n\t" \
    "adc r14, r10 \n\t" \
    "adc r15, r11 \n\t" \
    "adc r16, r19 \n\t" \
    "adc r17, r20 \n\t" \
    "clr r18 \n\t" \
    "bld r18, 0 \n\t" \
    "and r18, r1 \n\t" \
    "adc r18, r21 \n\t" \
    "movw r24, r2 \n\t" \
    "and r24, r1 \n\t" \
    "sub r14, r24 \n\t" \
    "and r25, r1 \n\t" \
    "sbc r15, r25 \n\t" \
    "movw r24, r4 \n\t" \
    "and r24, r1 \n\t" \
    "sbc r16, r24 \n\t" \
    "and r25, r1 \n\t" \
    "sbc r17, r25 \n\t" \
    "sbc r18, r21 \n\t" \
    "clr r24 \n\t" \
    "clr r25 \n\t" \
    "mul r2, r6 \n\t" \
    "add r22, r0 \n\t" \
    "std z+4, r22 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r21 \n\t" \
    "mul r2, r7 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "mul r3, r6 \n\t" \
    "add r23, r0 \n\t" \
    "std z+5, r23 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r21 \n\t" \
    "clr r22 \n\t" \
    "mul r2, r8 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r22, r21 \n\t" \
    "mul r3, r7 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r22, r21 \n\t" \
    "mul r4, r6 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r22, r21 \n\t" \
    "mul r2, r9 \n\t" \
    "clr r2 \n\t" \
    "add r25, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r2, r21 \n\t" \
    "mul r3, r8 \n\t" \
    "add r25, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r2, r21 \n\t" \
    "mul r4, r7 \n\t" \
    "add r25, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r2, r21 \n\t" \
    "mul r5, r6 \n\t" \
    "add r25, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r2, r21 \n\t" \
    "mul r3, r9 \n\t" \
    "clr r3 \n\t" \
    "add r22, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r3, r21 \n\t" \
    "mul r4, r8 \n\t" \
    "add r22, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r3, r21 \n\t" \
    "mul r5, r7 \n\t" \
    "add r22, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r3, r21 \n\t" \
    "mul r4, r9 \n\t" \
    "clr r4 \n\t" \
    "add r2, r0 \n\t" \
    "adc r3, r1 \n\t" \
    "adc r4, r21 \n\t" \
    "mul r5, r8 \n\t" \
    "add r2, r0 \n\t" \
    "adc r3, r1 \n\t" \
    "adc r4, r21 \n\t" \
    "mul r5, r9 \n\t" \
    "add r3, r0 \n\t" \
    "adc r4, r1 \n\t" \
    "clr r0 \n\t" \
    "bld r0, 0 \n\t" \
    "neg r0 \n\t" \
    "and r6, r0 \n\t" \
    "sub r14, r6 \n\t" \
    "and r7, r0 \n\t" \
    "sbc r15, r7 \n\t" \
    "and r8, r0 \n\t" \
    "sbc r16, r8 \n\t" \
    "and r9, r0 \n\t" \
    "sbc r17, r9 \n\t" \
    "sbc r18, r21 \n\t" \
    "mov r0, r18 \n\t" \
    "asr r0 \n\t" \
    "add r12, r24 \n\t" \
    "std z+6, r12 \n\t" \
    "adc r13, r25 \n\t" \
    "std z+7, r13 \n\t" \
    "adc r14, r22 \n\t" \
    "adc r15, r2 \n\t" \
    "adc r16, r3 \n\t" \
    "adc r17, r4 \n\t" \
    "adc r10, r18 \n\t" \
    "adc r11, r0 \n\t" \
    "adc r19, r0 \n\t" \
    "adc r20, r0 \n\t" \
    "movw r6, r14 \n\t" \
    "movw r8, r16 \n\t" \
    "mov r18, r10 \n\t" \
    "mov r21, r11 \n\t" \
    "clr r12 \n\t" \
    "clr r13 \n\t" \
    "movw r14, r12 \n\t" \
    "movw r16, r12 \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ldd r22, y+8 \n\t" \
    "ldd r23, y+9 \n\t" \
    "ldd r24, y+10 \n\t" \
    "ldd r25, y+11 \n\t" \
    "mul r2, r22 \n\t" \
    "movw r10, r0 \n\t" \
    "mul r2, r23 \n\t" \
    "add r11, r0 \n\t" \
    "adc r12, r1 \n\t" \
    "mul r3, r22 \n\t" \
    "add r11, r0 \n\t" \
    "adc r12, r1 \n\t" \
    "adc r13, r17 \n\t" \
    "mul r2, r24 \n\t" \
    "add r12, r0 \n\t" \
    "adc r13, r1 \n\t" \
    "adc r14, r17 \n\t" \
    "mul r3, r23 \n\t" \
    "add r12, r0 \n\t" \
    "adc r13, r1 \n\t" \
    "adc r14, r17 \n\t" \
    "mul r4, r22 \n\t" \
    "add r12, r0 \n\t" \
    "adc r13, r1 \n\t" \
    "adc r14, r17 \n\t" \
    "mul r2, r25 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "adc r15, r17 \n\t" \
    "mul r3, r24 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "adc r15, r17 \n\t" \
    "mul r4, r23 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "adc r15, r17 \n\t" \
    "mul r5, r22 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "adc r15, r17 \n\t" \
    "add r6, r10 \n\t" \
    "std z+16, r6 \n\t" \
    "adc r7, r11 \n\t" \
    "std z+17, r7 \n\t" \
    "adc r8, r12 \n\t" \
    "std z+18, r8 \n\t" \
    "adc r9, r13 \n\t" \
    "std z+19, r9 \n\t" \
    "adc r10, r18 \n\t" \
    "std z+20, r10 \n\t" \
    "adc r11, r21 \n\t" \
    "std z+21, r11 \n\t" \
    "adc r12, r19 \n\t" \
    "adc r13, r20 \n\t" \
    "sbc r0, r0 \n\t" \
    "push r0 \n\t" \
    "clr r20 \n\t" \
    "clr r21 \n\t" \
    "mul r3, r25 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r16, r21 \n\t" \
    "mul r4, r24 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r16, r21 \n\t" \
    "mul r5, r23 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r16, r21 \n\t" \
    "mul r4, r25 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r21 \n\t" \
    "mul r5, r24 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r21 \n\t" \
    "mul r5, r25 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "movw r10, r20 \n\t" \
    "ld r18, x+ \n\t" \
    "ld r19, x+ \n\t" \
    "ld r20, x+ \n\t" \
    "ldd r6, y+12 \n\t" \
    "ldd r7, y+13 \n\t" \
    "ldd r8, y+14 \n\t" \
    "ldd r9, y+15 \n\t" \
    "mul r18, r6 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r16, r21 \n\t" \
    "adc r11, r21 \n\t" \
    "mul r18, r7 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r11, r21 \n\t" \
    "mul r19, r6 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r11 \n\t" \
    "adc r10, r21 \n\t" \
    "mul r18, r8 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r10, r21 \n\t" \
    "mul r19, r7 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r10, r21 \n\t" \
    "mul r20, r6 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r10, r21 \n\t" \
    "clr r11 \n\t" \
    "mul r18, r9 \n\t" \
    "add r17, r0 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r11, r21 \n\t" \
    "mul r19, r8 \n\t" \
    "add r17, r0 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r11, r21 \n\t" \
    "mul r20, r7 \n\t" \
    "add r17, r0 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r11, r21 \n\t" \
    "sub r2, r18 \n\t" \
    "sbc r3, r19 \n\t" \
    "sbc r4, r20 \n\t" \
    "ld r18, x+ \n\t" \
    "sbc r5, r18 \n\t" \
    "sbc r0, r0 \n\t" \
    "bst r0, 0 \n\t" \
    "mul r18, r6 \n\t" \
    "add r17, r0 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r11, r21 \n\t" \
    "mul r19, r9 \n\t" \
    "clr r19 \n\t" \
    "add r10, r0 \n\t" \
    "adc r11, r1 \n\t" \
    "adc r19, r21 \n\t" \
    "mul r20, r8 \n\t" \
    "add r10, r0 \n\t" \
    "adc r11, r1 \n\t" \
    "adc r19, r21 \n\t" \
    "mul r18, r7 \n\t" \
    "add r10, r0 \n\t" \
    "adc r11, r1 \n\t" \
    "adc r19, r21 \n\t" \
    "mul r20, r9 \n\t" \
    "clr r20 \n\t" \
    "add r11, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r21 \n\t" \
    "mul r18, r8 \n\t" \
    "add r11, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r21 \n\t" \
    "mul r18, r9 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "sub r6, r22 \n\t" \
    "sbc r7, r23 \n\t" \
    "sbc r8, r24 \n\t" \
    "sbc r9, r25 \n\t" \
    "sbc r1, r1 \n\t" \
    "ldd r22, z+20 \n\t" \
    "ldd r23, z+21 \n\t" \
    "add r22, r14 \n\t" \
    "adc r23, r15 \n\t" \
    "adc r12, r16 \n\t" \
    "adc r13, r17 \n\t" \
    "adc r14, r10 \n\t" \
    "adc r15, r11 \n\t" \
    "adc r16, r19 \n\t" \
    "adc r17, r20 \n\t" \
    "clr r18 \n\t" \
    "bld r18, 0 \n\t" \
    "and r18, r1 \n\t" \
    "adc r18, r21 \n\t" \
    "pop r0 \n\t" \
    "neg r0 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r21 \n\t" \
    "adc r16, r21 \n\t" \
    "adc r17, r21 \n\t" \
    "adc r18, r21 \n\t" \
    "movw r24, r2 \n\t" \
    "and r24, r1 \n\t" \
    "sub r14, r24 \n\t" \
    "and r25, r1 \n\t" \
    "sbc r15, r25 \n\t" \
    "movw r24, r4 \n\t" \
    "and r24, r1 \n\t" \
    "sbc r16, r24 \n\t" \
    "and r25, r1 \n\t" \
    "sbc r17, r25 \n\t" \
    "sbc r18, r21 \n\t" \
    "clr r24 \n\t" \
    "clr r25 \n\t" \
    "mul r2, r6 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r12, r21 \n\t" \
    "adc r24, r21 \n\t" \
    "mul r2, r7 \n\t" \
    "add r23, r0 \n\t" \
    "adc r12, r1 \n\t" \
    "adc r24, r21 \n\t" \
    "mul r3, r6 \n\t" \
    "add r23, r0 \n\t" \
    "adc r12, r1 \n\t" \
    "adc r24, r21 \n\t" \
    "mul r2, r8 \n\t" \
    "add r12, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r21 \n\t" \
    "mul r3, r7 \n\t" \
    "add r12, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r21 \n\t" \
    "mul r4, r6 \n\t" \
    "add r12, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r25, r21 \n\t" \
    "mul r2, r9 \n\t" \
    "clr r2 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r2, r21 \n\t" \
    "mul r3, r8 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r2, r21 \n\t" \
    "mul r4, r7 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r2, r21 \n\t" \
    "mul r5, r6 \n\t" \
    "add r24, r0 \n\t" \
    "adc r25, r1 \n\t" \
    "adc r2, r21 \n\t" \
    "mul r3, r9 \n\t" \
    "clr r3 \n\t" \
    "add r25, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r3, r21 \n\t" \
    "mul r4, r8 \n\t" \
    "add r25, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r3, r21 \n\t" \
    "mul r5, r7 \n\t" \
    "add r25, r0 \n\t" \
    "adc r2, r1 \n\t" \
    "adc r3, r21 \n\t" \
    "mul r4, r9 \n\t" \
    "clr r4 \n\t" \
    "add r2, r0 \n\t" \
    "adc r3, r1 \n\t" \
    "adc r4, r21 \n\t" \
    "mul r5, r8 \n\t" \
    "add r2, r0 \n\t" \
    "adc r3, r1 \n\t" \
    "adc r4, r21 \n\t" \
    "mul r5, r9 \n\t" \
    "add r3, r0 \n\t" \
    "adc r4, r1 \n\t" \
    "clr r0 \n\t" \
    "bld r0, 0 \n\t" \
    "neg r0 \n\t" \
    "and r6, r0 \n\t" \
    "sub r14, r6 \n\t" \
    "and r7, r0 \n\t" \
    "sbc r15, r7 \n\t" \
    "and r8, r0 \n\t" \
    "sbc r16, r8 \n\t" \
    "and r9, r0 \n\t" \
    "sbc r17, r9 \n\t" \
    "sbc r18, r21 \n\t" \
    "mov r0, r18 \n\t" \
    "asr r0 \n\t" \
    "add r13, r24 \n\t" \
    "adc r14, r25 \n\t" \
    "adc r15, r2 \n\t" \
    "adc r16, r3 \n\t" \
    "adc r17, r4 \n\t" \
    "adc r10, r18 \n\t" \
    "adc r11, r0 \n\t" \
    "adc r19, r0 \n\t" \
    "adc r20, r0 \n\t" \
    "std z+20, r22 \n\t" \
    "std z+21, r23 \n\t" \
    "std z+22, r12 \n\t" \
    "std z+23, r13 \n\t" \
    "std z+24, r14 \n\t" \
    "std z+25, r15 \n\t" \
    "std z+26, r16 \n\t" \
    "std z+27, r17 \n\t" \
    "std z+28, r10 \n\t" \
    "std z+29, r11 \n\t" \
    "std z+30, r19 \n\t" \
    "std z+31, r20 \n\t" \
    "sbiw r26, 16 \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ld r18, x+ \n\t" \
    "ld r19, x+ \n\t" \
    "ld r20, x+ \n\t" \
    "ld r21, x+ \n\t" \
    "ld r10, x+ \n\t" \
    "ld r11, x+ \n\t" \
    "ld r12, x+ \n\t" \
    "ld r13, x+ \n\t" \
    "ld r14, x+ \n\t" \
    "ld r15, x+ \n\t" \
    "ld r16, x+ \n\t" \
    "ld r17, x \n\t" \
    "sub r2, r10 \n\t" \
    "sbc r3, r11 \n\t" \
    "sbc r4, r12 \n\t" \
    "sbc r5, r13 \n\t" \
    "sbc r18, r14 \n\t" \
    "sbc r19, r15 \n\t" \
    "sbc r20, r16 \n\t" \
    "sbc r21, r17 \n\t" \
    "sbc r0, r0 \n\t" \
    "ldd r6, y+0 \n\t" \
    "ldd r7, y+1 \n\t" \
    "ldd r8, y+2 \n\t" \
    "ldd r9, y+3 \n\t" \
    "ldd r22, y+4 \n\t" \
    "ldd r23, y+5 \n\t" \
    "ldd r24, y+6 \n\t" \
    "ldd r25, y+7 \n\t" \
    "ldd r10, y+8 \n\t" \
    "ldd r11, y+9 \n\t" \
    "ldd r12, y+10 \n\t" \
    "ldd r13, y+11 \n\t" \
    "ldd r14, y+12 \n\t" \
    "ldd r15, y+13 \n\t" \
    "ldd r16, y+14 \n\t" \
    "ldd r17, y+15 \n\t" \
    "sub r6, r10 \n\t" \
    "sbc r7, r11 \n\t" \
    "sbc r8, r12 \n\t" \
    "sbc r9, r13 \n\t" \
    "sbc r22, r14 \n\t" \
    "sbc r23, r15 \n\t" \
    "sbc r24, r16 \n\t" \
    "sbc r25, r17 \n\t" \
    "sbc r1, r1 \n\t" \
    "eor r2, r0 \n\t" \
    "eor r3, r0 \n\t" \
    "eor r4, r0 \n\t" \
    "eor r5, r0 \n\t" \
    "eor r18, r0 \n\t" \
    "eor r19, r0 \n\t" \
    "eor r20, r0 \n\t" \
    "eor r21, r0 \n\t" \
    "eor r6, r1 \n\t" \
    "eor r7, r1 \n\t" \
    "eor r8, r1 \n\t" \
    "eor r9, r1 \n\t" \
    "eor r22, r1 \n\t" \
    "eor r23, r1 \n\t" \
    "eor r24, r1 \n\t" \
    "eor r25, r1 \n\t" \
    "sub r2, r0 \n\t" \
    "sbc r3, r0 \n\t" \
    "sbc r4, r0 \n\t" \
    "sbc r5, r0 \n\t" \
    "sbc r18, r0 \n\t" \
    "sbc r19, r0 \n\t" \
    "sbc r20, r0 \n\t" \
    "sbc r21, r0 \n\t" \
    "sub r6, r1 \n\t" \
    "sbc r7, r1 \n\t" \
    "sbc r8, r1 \n\t" \
    "sbc r9, r1 \n\t" \
    "sbc r22, r1 \n\t" \
    "sbc r23, r1 \n\t" \
    "sbc r24, r1 \n\t" \
    "sbc r25, r1 \n\t" \
    "clr r26 \n\t" \
    "clr r27 \n\t" \
    "eor r0, r1 \n\t" \
    "push r0 \n\t" \
    "movw r16, r26 \n\t" \
    "mul r2, r8 \n\t" \
    "movw r12, r0 \n\t" \
    "mul r2, r6 \n\t" \
    "movw r10, r0 \n\t" \
    "mul r2, r7 \n\t" \
    "add r11, r0 \n\t" \
    "adc r12, r1 \n\t" \
    "adc r13, r26 \n\t" \
    "mul r3, r9 \n\t" \
    "movw r14, r0 \n\t" \
    "mul r2, r9 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r3, r6 \n\t" \
    "add r11, r0 \n\t" \
    "adc r12, r1 \n\t" \
    "adc r13, r28 \n\t" \
    "adc r29, r26 \n\t" \
    "mul r3, r7 \n\t" \
    "add r12, r0 \n\t" \
    "adc r13, r1 \n\t" \
    "adc r29, r26 \n\t" \
    "mul r4, r9 \n\t" \
    "add r14, r29 \n\t" \
    "adc r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "mul r4, r8 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r4, r6 \n\t" \
    "add r12, r0 \n\t" \
    "adc r13, r1 \n\t" \
    "adc r14, r28 \n\t" \
    "adc r29, r26 \n\t" \
    "mul r3, r8 \n\t" \
    "add r13, r0 \n\t" \
    "adc r14, r1 \n\t" \
    "adc r29, r26 \n\t" \
    "mul r5, r9 \n\t" \
    "add r15, r29 \n\t" \
    "adc r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "mul r5, r7 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r4, r7 \n\t" \
    "add r13, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r26 \n\t" \
    "mul r5, r6 \n\t" \
    "add r13, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r26 \n\t" \
    "mul r5, r8 \n\t" \
    "add r14, r28 \n\t" \
    "adc r0, r29 \n\t" \
    "adc r1, r26 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r26 \n\t" \
    "sub r2, r18 \n\t" \
    "sbc r3, r19 \n\t" \
    "sbc r4, r20 \n\t" \
    "sbc r5, r21 \n\t" \
    "sbc r0, r0 \n\t" \
    "sub r6, r22 \n\t" \
    "sbc r7, r23 \n\t" \
    "sbc r8, r24 \n\t" \
    "sbc r9, r25 \n\t" \
    "sbc r1, r1 \n\t" \
    "eor r2, r0 \n\t" \
    "eor r3, r0 \n\t" \
    "eor r4, r0 \n\t" \
    "eor r5, r0 \n\t" \
    "eor r6, r1 \n\t" \
    "eor r7, r1 \n\t" \
    "eor r8, r1 \n\t" \
    "eor r9, r1 \n\t" \
    "sub r2, r0 \n\t" \
    "sbc r3, r0 \n\t" \
    "sbc r4, r0 \n\t" \
    "sbc r5, r0 \n\t" \
    "sub r6, r1 \n\t" \
    "sbc r7, r1 \n\t" \
    "sbc r8, r1 \n\t" \
    "sbc r9, r1 \n\t" \
    "eor r0, r1 \n\t" \
    "bst r0, 0 \n\t" \
    "mul r18, r24 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r18, r22 \n\t" \
    "add r14, r0 \n\t" \
    "adc r15, r1 \n\t" \
    "adc r16, r28 \n\t" \
    "adc r29, r26 \n\t" \
    "mul r18, r23 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r29, r26 \n\t" \
    "mul r19, r25 \n\t" \
    "add r17, r29 \n\t" \
    "adc r26, r0 \n\t" \
    "adc r27, r1 \n\t" \
    "mul r18, r25 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r19, r22 \n\t" \
    "add r15, r0 \n\t" \
    "adc r16, r1 \n\t" \
    "adc r17, r28 \n\t" \
    "clr r18 \n\t" \
    "adc r29, r18 \n\t" \
    "mul r19, r23 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r29, r18 \n\t" \
    "mul r20, r25 \n\t" \
    "add r26, r29 \n\t" \
    "adc r27, r0 \n\t" \
    "clr r18 \n\t" \
    "adc r18, r1 \n\t" \
    "mul r20, r24 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r20, r22 \n\t" \
    "add r16, r0 \n\t" \
    "adc r17, r1 \n\t" \
    "adc r26, r28 \n\t" \
    "clr r0 \n\t" \
    "adc r29, r0 \n\t" \
    "mul r19, r24 \n\t" \
    "add r17, r0 \n\t" \
    "adc r26, r1 \n\t" \
    "clr r19 \n\t" \
    "adc r29, r19 \n\t" \
    "mul r21, r25 \n\t" \
    "add r27, r29 \n\t" \
    "adc r18, r0 \n\t" \
    "clr r25 \n\t" \
    "adc r25, r1 \n\t" \
    "mul r21, r23 \n\t" \
    "movw r28,r0 \n\t" \
    "mul r20, r23 \n\t" \
    "add r17, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r19 \n\t" \
    "mul r21, r22 \n\t" \
    "add r17, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r19 \n\t" \
    "mul r21, r24 \n\t" \
    "add r26, r28 \n\t" \
    "adc r0, r29 \n\t" \
    "adc r1, r19 \n\t" \
    "add r27, r0 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r25, r19 \n\t" \
    "mul r2, r8 \n\t" \
    "movw r22, r0 \n\t" \
    "mul r2, r6 \n\t" \
    "movw r20, r0 \n\t" \
    "mul r2, r7 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r19 \n\t" \
    "mul r3, r9 \n\t" \
    "mov r24, r0 \n\t" \
    "mov r0, r2 \n\t" \
    "mov r2, r1 \n\t" \
    "mul r0, r9 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r3, r6 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r28 \n\t" \
    "adc r29, r19 \n\t" \
    "mul r3, r7 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r19 \n\t" \
    "mul r4, r9 \n\t" \
    "add r24, r29 \n\t" \
    "adc r2, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "mul r4, r8 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r4, r6 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r28 \n\t" \
    "clr r0 \n\t" \
    "adc r29, r0 \n\t" \
    "mul r3, r8 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "clr r3 \n\t" \
    "adc r29, r3 \n\t" \
    "mul r5, r9 \n\t" \
    "add r2, r29 \n\t" \
    "adc r19, r0 \n\t" \
    "clr r9 \n\t" \
    "adc r9, r1 \n\t" \
    "mul r5, r7 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r4, r7 \n\t" \
    "add r23, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r3 \n\t" \
    "mul r5, r6 \n\t" \
    "add r23, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r3 \n\t" \
    "mul r5, r8 \n\t" \
    "add r24, r28 \n\t" \
    "adc r0, r29 \n\t" \
    "adc r1, r3 \n\t" \
    "add r2, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r9, r3 \n\t" \
    "movw r4, r10 \n\t" \
    "movw r6, r12 \n\t" \
    "add r10, r14 \n\t" \
    "adc r11, r15 \n\t" \
    "adc r12, r16 \n\t" \
    "adc r13, r17 \n\t" \
    "adc r14, r26 \n\t" \
    "adc r15, r27 \n\t" \
    "adc r16, r18 \n\t" \
    "adc r17, r25 \n\t" \
    "clr r0 \n\t" \
    "bld r0, 0 \n\t" \
    "dec r0 \n\t" \
    "adc r3, r0 \n\t" \
    "mov r1, r3 \n\t" \
    "asr r1 \n\t" \
    "eor r20, r0 \n\t" \
    "eor r21, r0 \n\t" \
    "eor r22, r0 \n\t" \
    "eor r23, r0 \n\t" \
    "eor r24, r0 \n\t" \
    "eor r2, r0 \n\t" \
    "eor r19, r0 \n\t" \
    "eor r9, r0 \n\t" \
    "add r0, r0 \n\t" \
    "adc r10, r20 \n\t" \
    "adc r11, r21 \n\t" \
    "adc r12, r22 \n\t" \
    "adc r13, r23 \n\t" \
    "adc r14, r24 \n\t" \
    "adc r15, r2 \n\t" \
    "adc r16, r19 \n\t" \
    "adc r17, r9 \n\t" \
    "adc r26, r3 \n\t" \
    "adc r27, r1 \n\t" \
    "adc r18, r1 \n\t" \
    "adc r25, r1 \n\t" \
    "mov r19, r25 \n\t" \
    "ldd r20, z+0 \n\t" \
    "ldd r21, z+1 \n\t" \
    "ldd r22, z+2 \n\t" \
    "ldd r23, z+3 \n\t" \
    "ldd r24, z+4 \n\t" \
    "ldd r25, z+5 \n\t" \
    "ldd r8, z+6 \n\t" \
    "ldd r9, z+7 \n\t" \
    "pop r0 \n\t" \
    "com r0 \n\t" \
    "mov r1, r0 \n\t" \
    "asr r1 \n\t" \
    "eor r4, r0 \n\t" \
    "eor r5, r0 \n\t" \
    "eor r6, r0 \n\t" \
    "eor r7, r0 \n\t" \
    "eor r10, r0 \n\t" \
    "eor r11, r0 \n\t" \
    "eor r12, r0 \n\t" \
    "eor r13, r0 \n\t" \
    "eor r14, r0 \n\t" \
    "eor r15, r0 \n\t" \
    "eor r16, r0 \n\t" \
    "eor r17, r0 \n\t" \
    "eor r26, r0 \n\t" \
    "eor r27, r0 \n\t" \
    "eor r18, r0 \n\t" \
    "eor r19, r0 \n\t" \
    "mov r28, r0 \n\t" \
    "add r28, r28 \n\t" \
    "adc r20, r4 \n\t" \
    "adc r21, r5 \n\t" \
    "adc r22, r6 \n\t" \
    "adc r23, r7 \n\t" \
    "adc r24, r10 \n\t" \
    "adc r25, r11 \n\t" \
    "adc r8, r12 \n\t" \
    "adc r9, r13 \n\t" \
    "clr r28 \n\t" \
    "adc r28, r28 \n\t" \
    "ldd r2, z+16 \n\t" \
    "ldd r3, z+17 \n\t" \
    "add r20, r2 \n\t" \
    "adc r21, r3 \n\t" \
    "std z+8, r20 \n\t" \
    "std z+9, r21 \n\t" \
    "movw r20, r2 \n\t" \
    "ldd r2, z+18 \n\t" \
    "ldd r3, z+19 \n\t" \
    "adc r22, r2 \n\t" \
    "adc r23, r3 \n\t" \
    "std z+10, r22 \n\t" \
    "std z+11, r23 \n\t" \
    "movw r22, r2 \n\t" \
    "ldd r2, z+20 \n\t" \
    "ldd r3, z+21 \n\t" \
    "adc r24, r2 \n\t" \
    "adc r25, r3 \n\t" \
    "std z+12, r24 \n\t" \
    "std z+13, r25 \n\t" \
    "movw r24, r2 \n\t" \
    "ldd r2, z+22 \n\t" \
    "ldd r3, z+23 \n\t" \
    "adc r8, r2 \n\t" \
    "adc r9, r3 \n\t" \
    "std z+14, r8 \n\t" \
    "std z+15, r9 \n\t" \
    "movw r8, r2 \n\t" \
    "clr r2 \n\t" \
    "adc r2, r2 \n\t" \
    "lsr r28 \n\t" \
    "adc r20, r14 \n\t" \
    "adc r21, r15 \n\t" \
    "adc r22, r16 \n\t" \
    "adc r23, r17 \n\t" \
    "adc r24, r26 \n\t" \
    "adc r25, r27 \n\t" \
    "adc r8, r18 \n\t" \
    "adc r9, r19 \n\t" \
    "adc r0, r28 \n\t" \
    "adc r1, r28 \n\t" \
    "ldd r4, z+24 \n\t" \
    "ldd r5, z+25 \n\t" \
    "ldd r6, z+26 \n\t" \
    "ldd r7, z+27 \n\t" \
    "ldd r10, z+28 \n\t" \
    "ldd r11, z+29 \n\t" \
    "ldd r12, z+30 \n\t" \
    "ldd r13, z+31 \n\t" \
    "lsr r2 \n\t" \
    "adc r20, r4 \n\t" \
    "adc r21, r5 \n\t" \
    "adc r22, r6 \n\t" \
    "adc r23, r7 \n\t" \
    "adc r24, r10 \n\t" \
    "adc r25, r11 \n\t" \
    "adc r8, r12 \n\t" \
    "adc r9, r13 \n\t" \
    "adc r0, r2 \n\t" \
    "adc r1, r2 \n\t" \
    "std z+16, r20 \n\t" \
    "std z+17, r21 \n\t" \
    "std z+18, r22 \n\t" \
    "std z+19, r23 \n\t" \
    "std z+20, r24 \n\t" \
    "std z+21, r25 \n\t" \
    "std z+22, r8 \n\t" \
    "std z+23, r9 \n\t" \
    "add r4, r0 \n\t" \
    "adc r5, r1 \n\t" \
    "adc r6, r1 \n\t" \
    "adc r7, r1 \n\t" \
    "adc r10, r1 \n\t" \
    "adc r11, r1 \n\t" \
    "adc r12, r1 \n\t" \
    "adc r13, r1 \n\t" \
    "std z+24, r4 \n\t" \
    "std z+25, r5 \n\t" \
    "std z+26, r6 \n\t" \
    "std z+27, r7 \n\t" \
    "std z+28, r10 \n\t" \
    "std z+29, r11 \n\t" \
    "std z+30, r12 \n\t" \
    "std z+31, r13 \n\t" \
    "ret \n\t"

#if uECC_SQUARE_FUNC

/* 160-bit (20 -> 40-byte) MOVW-aligned sliding-window squarer. */
#define FAST_SQUARE_ASM_20 \
    "clr r25 \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ld r6, x+ \n\t" \
    "ld r7, x+ \n\t" \
    "ld r8, x+ \n\t" \
    "ld r9, x+ \n\t" \
    "ld r10, x+ \n\t" \
    "ld r11, x+ \n\t" \
    "ld r12, x+ \n\t" \
    "ld r13, x+ \n\t" \
    "ld r14, x+ \n\t" \
    "ld r15, x+ \n\t" \
    "ld r16, x+ \n\t" \
    "ld r17, x+ \n\t" \
    "ld r18, x+ \n\t" \
    "ld r19, x+ \n\t" \
    "ld r20, x+ \n\t" \
    "ld r21, x+ \n\t" \
    "mul r2, r2 \n\t" \
    "st z+, r0 \n\t" \
    "mov r29, r1 \n\t" \
    "clr r26 \n\t" \
    "mul r2, r3 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r4 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r3, r3 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r5 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r4 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r6 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r5 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r4, r4 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r7 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r6 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r5 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r8 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r7 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r6 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r5, r5 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r9 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r8 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r7 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r6 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r10 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r9 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r8 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r5, r7 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r6, r6 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r11 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r10 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r9 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r8 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r7 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r12 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r11 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r10 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r5, r9 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r6, r8 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r7, r7 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r13 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r11 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r10 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r9 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r7, r8 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r5, r11 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r6, r10 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r7, r9 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r8, r8 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r15 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r11 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r7, r10 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r8, r9 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r16 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r15 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r14 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r5, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r6, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r7, r11 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r8, r10 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r9, r9 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r17 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r7, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r8, r11 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r9, r10 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r18 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r17 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r16 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r5, r15 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r6, r14 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r7, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r8, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r9, r11 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r10, r10 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r19 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r7, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r8, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r9, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r10, r11 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r20 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r18 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r5, r17 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r6, r16 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r7, r15 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r8, r14 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r9, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r10, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r11, r11 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r21 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r7, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r8, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r9, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r10, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r11, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r3, r21 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r4, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r5, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r6, r18 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r7, r17 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r8, r16 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r9, r15 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r10, r14 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r11, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r12, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r4, r21 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r5, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r7, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r8, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r9, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r10, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r11, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r12, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r5, r21 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r6, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r7, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r8, r18 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r9, r17 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r10, r16 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r11, r15 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r12, r14 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r13, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r6, r21 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r7, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r8, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r9, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r10, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r11, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r12, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r13, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r7, r21 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r8, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r9, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r10, r18 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r11, r17 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r12, r16 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r13, r15 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r14, r14 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r8, r21 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r9, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r10, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r11, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r12, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r13, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r14, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r9, r21 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r10, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r11, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r12, r18 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r13, r17 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r14, r16 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r15, r15 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r10, r21 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r11, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r12, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r13, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r14, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r15, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r11, r21 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r12, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r13, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r14, r18 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r15, r17 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r16, r16 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r12, r21 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r13, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r14, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r15, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r16, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r13, r21 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r14, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r15, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r16, r18 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r17, r17 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r14, r21 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r15, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r16, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r17, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r15, r21 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r16, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r17, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r18, r18 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r16, r21 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r17, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r18, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r17, r21 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r18, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r19, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r18, r21 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r19, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r19, r21 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r20, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r20, r21 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r21, r21 \n\t" \
    "add r0, r23 \n\t" \
    "adc r1, r24 \n\t" \
    "st z+, r0 \n\t" \
    "st z+, r1 \n\t"

/* Extends a 20-byte square in result[0..39] to a 21-byte square in result[0..41]. */
#define FAST_SQUARE_ASM_20_EXT_21 \
    /* Restore original result (Z) and left (X) pointers. */ \
    "pop r31 \n\t" \
    "pop r30 \n\t" \
    "pop r27 \n\t" \
    "pop r26 \n\t" \
    "clr r24 \n\t" \
    "adiw r30, 40 \n\t" \
    "st z+, r24 \n\t" \
    "st z, r24 \n\t" \
    "sbiw r30, 21 \n\t" \
    "adiw r26, 20 \n\t" \
    "ld r18, x \n\t" \
    "sbiw r26, 20 \n\t" \
    /* Accumulate 2 * left[20] * left[0..19] + left[20]^2 * 2^320 into result[20..41] */ \
    /* using a 16-bit carry (R25:R21) so the doubled 17-bit term never overflows.     */ \
    "ldi r20, 20 \n\t" \
    "clr r21 \n\t" \
    "clr r25 \n\t" \
    "83: \n\t" \
    "ld r22, x+ \n\t" \
    "mul r18, r22 \n\t" \
    "lsl r0 \n\t" \
    "rol r1 \n\t" \
    "clr r19 \n\t" \
    "rol r19 \n\t" \
    "ld r23, z \n\t" \
    "add r0, r23 \n\t" \
    "adc r1, r24 \n\t" \
    "adc r19, r24 \n\t" \
    "add r0, r21 \n\t" \
    "adc r1, r25 \n\t" \
    "adc r19, r24 \n\t" \
    "st z+, r0 \n\t" \
    "mov r21, r1 \n\t" \
    "mov r25, r19 \n\t" \
    "dec r20 \n\t" \
    "brne 83b \n\t" \
    "mul r18, r18 \n\t" \
    "add r0, r21 \n\t" \
    "adc r1, r25 \n\t" \
    "st z+, r0 \n\t" \
    "st z, r1 \n\t"

/* 192-bit (24 -> 48-byte) MOVW-aligned sliding-window squarer. */
#define FAST_SQUARE_ASM_24 \
    "clr r25 \n\t" \
    "movw r28, r26 \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "adiw r28, 20 \n\t" \
    "ld r12, y+ \n\t" \
    "ld r13, y+ \n\t" \
    "adiw r30, 20 \n\t" \
    "clr r23 \n\t" \
    "mul r2, r12 \n\t" \
    "st z+, r0 \n\t" \
    "mov r22, r1 \n\t" \
    "clr r24 \n\t" \
    "mul r2, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "ld r12, y+ \n\t" \
    "clr r22 \n\t" \
    "mul r2, r12 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r3, r13 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "st z+, r23 \n\t" \
    "ld r13, y+ \n\t" \
    "clr r23 \n\t" \
    "mul r2, r13 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "mul r3, r12 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r25 \n\t" \
    "st z+, r24 \n\t" \
    "ld r2, x+ \n\t" \
    "clr r24 \n\t" \
    "mul r3, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r2, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "ld r3, x+ \n\t" \
    "clr r22 \n\t" \
    "mul r2, r13 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "st z+, r23 \n\t" \
    "mul r3, r13 \n\t" \
    "add r24, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "st z+, r24 \n\t" \
    "st z+, r22 \n\t" \
    "sbiw r26, 4 \n\t" \
    "sbiw r30, 28 \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ld r6, x+ \n\t" \
    "ld r7, x+ \n\t" \
    "ld r8, x+ \n\t" \
    "ld r9, x+ \n\t" \
    "ld r10, x+ \n\t" \
    "ld r11, x+ \n\t" \
    "ld r12, x+ \n\t" \
    "ld r13, x+ \n\t" \
    "ld r14, x+ \n\t" \
    "ld r15, x+ \n\t" \
    "ld r16, x+ \n\t" \
    "ld r17, x+ \n\t" \
    "ld r18, x+ \n\t" \
    "ld r19, x+ \n\t" \
    "ld r20, x+ \n\t" \
    "ld r21, x+ \n\t" \
    "push r26 \n\t" \
    "mul r2, r2 \n\t" \
    "st z+, r0 \n\t" \
    "mov r29, r1 \n\t" \
    "clr r26 \n\t" \
    "mul r2, r3 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r4 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r3, r3 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r5 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r4 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r6 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r5 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r4, r4 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r7 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r6 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r5 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r8 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r7 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r6 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r5, r5 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r9 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r8 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r7 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r6 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r10 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r9 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r8 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r5, r7 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r6, r6 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r11 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r10 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r9 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r8 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r7 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r12 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r11 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r10 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r5, r9 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r6, r8 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r7, r7 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r13 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r11 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r10 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r9 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r7, r8 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r5, r11 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r6, r10 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r7, r9 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r8, r8 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r15 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r11 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r7, r10 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r8, r9 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r16 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r15 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r14 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r5, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r6, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r7, r11 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r8, r10 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r9, r9 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r17 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r7, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r8, r11 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r9, r10 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r18 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r17 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r16 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r5, r15 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r6, r14 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r7, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r8, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r9, r11 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r10, r10 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r19 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r7, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r8, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r9, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r10, r11 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r20 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r4, r18 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r5, r17 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r6, r16 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r7, r15 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r8, r14 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r9, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r10, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "mul r11, r11 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r2, r21 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r3, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r4, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r5, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r6, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r7, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r8, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r9, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r10, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r11, r12 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "pop r26 \n\t" \
    "ld r2, x+ \n\t" \
    "clr r22 \n\t" \
    "mul r3, r21 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r4, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r5, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r6, r18 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r7, r17 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r8, r16 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r9, r15 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r10, r14 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "mul r11, r13 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "ld r0, z \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r25 \n\t" \
    "adc r22, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r22 \n\t" \
    "mul r12, r12 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r22, r25 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r22, r25 \n\t" \
    "st z+, r28 \n\t" \
    "clr r28 \n\t" \
    "mul r3, r2 \n\t" \
    "mov r23, r0 \n\t" \
    "mov r24, r1 \n\t" \
    "mul r4, r21 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r5, r20 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r6, r19 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r7, r18 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r8, r17 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r9, r16 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r10, r15 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r11, r14 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r12, r13 \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "ld r0, z \n\t" \
    "add r23, r0 \n\t" \
    "adc r24, r25 \n\t" \
    "adc r28, r25 \n\t" \
    "lsl r23 \n\t" \
    "rol r24 \n\t" \
    "rol r28 \n\t" \
    "add r23, r29 \n\t" \
    "adc r24, r22 \n\t" \
    "adc r28, r25 \n\t" \
    "st z+, r23 \n\t" \
    "ld r3, x+ \n\t" \
    "clr r29 \n\t" \
    "mul r4, r2 \n\t" \
    "movw r22, r0 \n\t" \
    "mul r5, r21 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r6, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r7, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r8, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r9, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r10, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r11, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r12, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "ld r0, z \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r25 \n\t" \
    "adc r29, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r29 \n\t" \
    "mul r13, r13 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "add r22, r24 \n\t" \
    "adc r23, r28 \n\t" \
    "adc r29, r25 \n\t" \
    "st z+, r22 \n\t" \
    "clr r28 \n\t" \
    "mul r4, r3 \n\t" \
    "mov r22, r0 \n\t" \
    "mov r24, r1 \n\t" \
    "mul r5, r2 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r6, r21 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r7, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r8, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r9, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r10, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r11, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r12, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r13, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "ld r0, z \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r25 \n\t" \
    "adc r28, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r24 \n\t" \
    "rol r28 \n\t" \
    "add r22, r23 \n\t" \
    "adc r24, r29 \n\t" \
    "adc r28, r25 \n\t" \
    "st z+, r22 \n\t" \
    "ld r4, x+ \n\t" \
    "clr r29 \n\t" \
    "mul r5, r3 \n\t" \
    "movw r22, r0 \n\t" \
    "mul r6, r2 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r7, r21 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r8, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r9, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r10, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r11, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r12, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r13, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "ld r0, z \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r25 \n\t" \
    "adc r29, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r29 \n\t" \
    "mul r14, r14 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "add r22, r24 \n\t" \
    "adc r23, r28 \n\t" \
    "adc r29, r25 \n\t" \
    "st z+, r22 \n\t" \
    "clr r28 \n\t" \
    "mul r5, r4 \n\t" \
    "mov r22, r0 \n\t" \
    "mov r24, r1 \n\t" \
    "mul r6, r3 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r7, r2 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r8, r21 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r9, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r10, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r11, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r12, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r13, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r14, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "ld r0, z \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r25 \n\t" \
    "adc r28, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r24 \n\t" \
    "rol r28 \n\t" \
    "add r22, r23 \n\t" \
    "adc r24, r29 \n\t" \
    "adc r28, r25 \n\t" \
    "st z+, r22 \n\t" \
    "ld r5, x+ \n\t" \
    "clr r29 \n\t" \
    "mul r6, r4 \n\t" \
    "movw r22, r0 \n\t" \
    "mul r7, r3 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r8, r2 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r9, r21 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r10, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r11, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r12, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r13, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "mul r14, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "ld r0, z \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r25 \n\t" \
    "adc r29, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r29 \n\t" \
    "mul r15, r15 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r29, r25 \n\t" \
    "add r22, r24 \n\t" \
    "adc r23, r28 \n\t" \
    "adc r29, r25 \n\t" \
    "st z+, r22 \n\t" \
    "clr r28 \n\t" \
    "mul r6, r5 \n\t" \
    "mov r22, r0 \n\t" \
    "mov r24, r1 \n\t" \
    "mul r7, r4 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r8, r3 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r9, r2 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r10, r21 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r11, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r12, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r13, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r14, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "mul r15, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r1 \n\t" \
    "adc r28, r25 \n\t" \
    "ld r0, z \n\t" \
    "add r22, r0 \n\t" \
    "adc r24, r25 \n\t" \
    "adc r28, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r24 \n\t" \
    "rol r28 \n\t" \
    "add r22, r23 \n\t" \
    "adc r24, r29 \n\t" \
    "adc r28, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mov r26, r28 \n\t" \
    "mov r29, r24 \n\t" \
    "mul r7, r5 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r8, r4 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r9, r3 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r10, r2 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r11, r21 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r12, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r13, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r14, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r15, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "mul r16, r16 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r8, r5 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r9, r4 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r10, r3 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r11, r2 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r12, r21 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r13, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r14, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r15, r18 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r16, r17 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r9, r5 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r10, r4 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r11, r3 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r12, r2 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r13, r21 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r14, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r15, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r16, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "mul r17, r17 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r10, r5 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r11, r4 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r12, r3 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r13, r2 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r14, r21 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r15, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r16, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r17, r18 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r11, r5 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r12, r4 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r13, r3 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r14, r2 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r15, r21 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r16, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r17, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "mul r18, r18 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r12, r5 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r13, r4 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r14, r3 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r15, r2 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r16, r21 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r17, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r18, r19 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r13, r5 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r14, r4 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r15, r3 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r16, r2 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r17, r21 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r18, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "mul r19, r19 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r14, r5 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r15, r4 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r16, r3 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r17, r2 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r18, r21 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r19, r20 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r15, r5 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r16, r4 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r17, r3 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r18, r2 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r19, r21 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "mul r20, r20 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r16, r5 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r17, r4 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r18, r3 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r19, r2 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r20, r21 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r17, r5 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r18, r4 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r19, r3 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r20, r2 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "mul r21, r21 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r18, r5 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r19, r4 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r20, r3 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r21, r2 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r19, r5 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r20, r4 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "mul r21, r3 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "mul r2, r2 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r20, r5 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r21, r4 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "mul r2, r3 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r21, r5 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "mul r2, r4 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "mul r3, r3 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r2, r5 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "mul r3, r4 \n\t" \
    "add r28, r0 \n\t" \
    "adc r29, r1 \n\t" \
    "adc r26, r25 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r3, r5 \n\t" \
    "movw r22, r0 \n\t" \
    "clr r24 \n\t" \
    "lsl r22 \n\t" \
    "rol r23 \n\t" \
    "rol r24 \n\t" \
    "mul r4, r4 \n\t" \
    "add r22, r0 \n\t" \
    "adc r23, r1 \n\t" \
    "adc r24, r25 \n\t" \
    "add r22, r29 \n\t" \
    "adc r23, r26 \n\t" \
    "adc r24, r25 \n\t" \
    "st z+, r22 \n\t" \
    "mul r4, r5 \n\t" \
    "movw r28, r0 \n\t" \
    "clr r26 \n\t" \
    "lsl r28 \n\t" \
    "rol r29 \n\t" \
    "rol r26 \n\t" \
    "add r28, r23 \n\t" \
    "adc r29, r24 \n\t" \
    "adc r26, r25 \n\t" \
    "st z+, r28 \n\t" \
    "mul r5, r5 \n\t" \
    "add r0, r29 \n\t" \
    "adc r1, r26 \n\t" \
    "st z+, r0 \n\t" \
    "st z+, r1 \n\t"

/* 224-bit (28 -> 56-byte) 1-level branch-free Karatsuba squarer
   (calls 14-byte leaf squarer at labels 94f / 95f). */
#define FAST_SQUARE_ASM_28 \
    "rcall 94f \n\t" \
    "adiw r30, 28 \n\t" \
    "rcall 94f \n\t" \
    "sbiw r30, 28 \n\t" \
    "sbiw r26, 28 \n\t" \
    "ld r0, x+ \n\t" \
    "sub r2, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r3, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r4, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r5, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r6, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r7, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r8, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r9, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r10, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r11, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r12, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r13, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r14, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r15, r0 \n\t" \
    "sbc r18, r18 \n\t" \
    "eor r2, r18 \n\t" \
    "eor r3, r18 \n\t" \
    "eor r4, r18 \n\t" \
    "eor r5, r18 \n\t" \
    "eor r6, r18 \n\t" \
    "eor r7, r18 \n\t" \
    "eor r8, r18 \n\t" \
    "eor r9, r18 \n\t" \
    "eor r10, r18 \n\t" \
    "eor r11, r18 \n\t" \
    "eor r12, r18 \n\t" \
    "eor r13, r18 \n\t" \
    "eor r14, r18 \n\t" \
    "eor r15, r18 \n\t" \
    "sub r2, r18 \n\t" \
    "sbc r3, r18 \n\t" \
    "sbc r4, r18 \n\t" \
    "sbc r5, r18 \n\t" \
    "sbc r6, r18 \n\t" \
    "sbc r7, r18 \n\t" \
    "sbc r8, r18 \n\t" \
    "sbc r9, r18 \n\t" \
    "sbc r10, r18 \n\t" \
    "sbc r11, r18 \n\t" \
    "sbc r12, r18 \n\t" \
    "sbc r13, r18 \n\t" \
    "sbc r14, r18 \n\t" \
    "sbc r15, r18 \n\t" \
    "push r30 \n\t" \
    "push r31 \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "sbiw r28, 28 \n\t" \
    /* Atomic ISR-safe SP update. */ \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r29 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r28 \n\t" \
    "movw r30, r28 \n\t" \
    "adiw r30, 1 \n\t" \
    "rcall 95f \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "movw r26, r28 \n\t" \
    "adiw r26, 1 \n\t" \
    "ldd r31, y+29 \n\t" \
    "ldd r30, y+30 \n\t" \
    "ldd r2, z+14 \n\t" \
    "ldd r3, z+15 \n\t" \
    "ldd r4, z+16 \n\t" \
    "ldd r5, z+17 \n\t" \
    "ldd r6, z+18 \n\t" \
    "ldd r7, z+19 \n\t" \
    "ldd r8, z+20 \n\t" \
    "ldd r9, z+21 \n\t" \
    "ldd r10, z+22 \n\t" \
    "ldd r11, z+23 \n\t" \
    "ldd r12, z+24 \n\t" \
    "ldd r13, z+25 \n\t" \
    "ldd r14, z+26 \n\t" \
    "ldd r15, z+27 \n\t" \
    "ldd r2, z+14 \n\t" \
    "ldd r3, z+15 \n\t" \
    "ldd r4, z+16 \n\t" \
    "ldd r5, z+17 \n\t" \
    "ldd r6, z+18 \n\t" \
    "ldd r7, z+19 \n\t" \
    "ldd r8, z+20 \n\t" \
    "ldd r9, z+21 \n\t" \
    "ldd r10, z+22 \n\t" \
    "ldd r11, z+23 \n\t" \
    "ldd r12, z+24 \n\t" \
    "ldd r13, z+25 \n\t" \
    "ldd r14, z+26 \n\t" \
    "ldd r15, z+27 \n\t" \
    "ldd r20, z+28 \n\t" \
    "add r2, r20 \n\t" \
    "ldd r20, z+29 \n\t" \
    "adc r3, r20 \n\t" \
    "ldd r20, z+30 \n\t" \
    "adc r4, r20 \n\t" \
    "ldd r20, z+31 \n\t" \
    "adc r5, r20 \n\t" \
    "ldd r20, z+32 \n\t" \
    "adc r6, r20 \n\t" \
    "ldd r20, z+33 \n\t" \
    "adc r7, r20 \n\t" \
    "ldd r20, z+34 \n\t" \
    "adc r8, r20 \n\t" \
    "ldd r20, z+35 \n\t" \
    "adc r9, r20 \n\t" \
    "ldd r20, z+36 \n\t" \
    "adc r10, r20 \n\t" \
    "ldd r20, z+37 \n\t" \
    "adc r11, r20 \n\t" \
    "ldd r20, z+38 \n\t" \
    "adc r12, r20 \n\t" \
    "ldd r20, z+39 \n\t" \
    "adc r13, r20 \n\t" \
    "ldd r20, z+40 \n\t" \
    "adc r14, r20 \n\t" \
    "ldd r20, z+41 \n\t" \
    "adc r15, r20 \n\t" \
    "clr r18 \n\t" \
    "adc r18, r18 \n\t" \
    "clr r19 \n\t" \
    "clr r25 \n\t" \
    "movw r20, r2 \n\t" \
    "movw r22, r4 \n\t" \
    "movw r26, r6 \n\t" \
    "lsr r19 \n\t" \
    "ldd r0, z+0 \n\t" \
    "adc r20, r0 \n\t" \
    "ldd r0, z+1 \n\t" \
    "adc r21, r0 \n\t" \
    "ldd r0, z+2 \n\t" \
    "adc r22, r0 \n\t" \
    "ldd r0, z+3 \n\t" \
    "adc r23, r0 \n\t" \
    "ldd r0, z+4 \n\t" \
    "adc r26, r0 \n\t" \
    "ldd r0, z+5 \n\t" \
    "adc r27, r0 \n\t" \
    "adc r19, r19 \n\t" \
    "lsr r25 \n\t" \
    "ldd r0, y+1 \n\t" \
    "sbc r20, r0 \n\t" \
    "std z+14, r20 \n\t" \
    "ldd r0, y+2 \n\t" \
    "sbc r21, r0 \n\t" \
    "std z+15, r21 \n\t" \
    "ldd r0, y+3 \n\t" \
    "sbc r22, r0 \n\t" \
    "std z+16, r22 \n\t" \
    "ldd r0, y+4 \n\t" \
    "sbc r23, r0 \n\t" \
    "std z+17, r23 \n\t" \
    "ldd r0, y+5 \n\t" \
    "sbc r26, r0 \n\t" \
    "std z+18, r26 \n\t" \
    "ldd r0, y+6 \n\t" \
    "sbc r27, r0 \n\t" \
    "std z+19, r27 \n\t" \
    "adc r25, r25 \n\t" \
    "movw r20, r8 \n\t" \
    "movw r22, r10 \n\t" \
    "movw r26, r12 \n\t" \
    "lsr r19 \n\t" \
    "ldd r0, z+6 \n\t" \
    "adc r20, r0 \n\t" \
    "ldd r0, z+7 \n\t" \
    "adc r21, r0 \n\t" \
    "ldd r0, z+8 \n\t" \
    "adc r22, r0 \n\t" \
    "ldd r0, z+9 \n\t" \
    "adc r23, r0 \n\t" \
    "ldd r0, z+10 \n\t" \
    "adc r26, r0 \n\t" \
    "ldd r0, z+11 \n\t" \
    "adc r27, r0 \n\t" \
    "adc r19, r19 \n\t" \
    "lsr r25 \n\t" \
    "ldd r0, y+7 \n\t" \
    "sbc r20, r0 \n\t" \
    "std z+20, r20 \n\t" \
    "ldd r0, y+8 \n\t" \
    "sbc r21, r0 \n\t" \
    "std z+21, r21 \n\t" \
    "ldd r0, y+9 \n\t" \
    "sbc r22, r0 \n\t" \
    "std z+22, r22 \n\t" \
    "ldd r0, y+10 \n\t" \
    "sbc r23, r0 \n\t" \
    "std z+23, r23 \n\t" \
    "ldd r0, y+11 \n\t" \
    "sbc r26, r0 \n\t" \
    "std z+24, r26 \n\t" \
    "ldd r0, y+12 \n\t" \
    "sbc r27, r0 \n\t" \
    "std z+25, r27 \n\t" \
    "adc r25, r25 \n\t" \
    "movw r20, r14 \n\t" \
    "lsr r19 \n\t" \
    "ldd r0, z+12 \n\t" \
    "adc r20, r0 \n\t" \
    "ldd r0, z+13 \n\t" \
    "adc r21, r0 \n\t" \
    "adc r19, r19 \n\t" \
    "lsr r25 \n\t" \
    "ldd r0, y+13 \n\t" \
    "sbc r20, r0 \n\t" \
    "std z+26, r20 \n\t" \
    "ldd r0, y+14 \n\t" \
    "sbc r21, r0 \n\t" \
    "std z+27, r21 \n\t" \
    "adc r25, r25 \n\t" \
    "add r19, r18 \n\t" \
    "sub r19, r25 \n\t" \
    "clr r25 \n\t" \
    "ldd r0, z+42 \n\t" \
    "add r2, r0 \n\t" \
    "ldd r0, z+43 \n\t" \
    "adc r3, r0 \n\t" \
    "ldd r0, z+44 \n\t" \
    "adc r4, r0 \n\t" \
    "ldd r0, z+45 \n\t" \
    "adc r5, r0 \n\t" \
    "ldd r0, z+46 \n\t" \
    "adc r6, r0 \n\t" \
    "ldd r0, z+47 \n\t" \
    "adc r7, r0 \n\t" \
    "ldd r0, z+48 \n\t" \
    "adc r8, r0 \n\t" \
    "ldd r0, z+49 \n\t" \
    "adc r9, r0 \n\t" \
    "ldd r0, z+50 \n\t" \
    "adc r10, r0 \n\t" \
    "ldd r0, z+51 \n\t" \
    "adc r11, r0 \n\t" \
    "ldd r0, z+52 \n\t" \
    "adc r12, r0 \n\t" \
    "ldd r0, z+53 \n\t" \
    "adc r13, r0 \n\t" \
    "ldd r0, z+54 \n\t" \
    "adc r14, r0 \n\t" \
    "ldd r0, z+55 \n\t" \
    "adc r15, r0 \n\t" \
    "adc r18, r25 \n\t" \
    "ldd r0, y+15 \n\t" \
    "sub r2, r0 \n\t" \
    "ldd r0, y+16 \n\t" \
    "sbc r3, r0 \n\t" \
    "ldd r0, y+17 \n\t" \
    "sbc r4, r0 \n\t" \
    "ldd r0, y+18 \n\t" \
    "sbc r5, r0 \n\t" \
    "ldd r0, y+19 \n\t" \
    "sbc r6, r0 \n\t" \
    "ldd r0, y+20 \n\t" \
    "sbc r7, r0 \n\t" \
    "ldd r0, y+21 \n\t" \
    "sbc r8, r0 \n\t" \
    "ldd r0, y+22 \n\t" \
    "sbc r9, r0 \n\t" \
    "ldd r0, y+23 \n\t" \
    "sbc r10, r0 \n\t" \
    "ldd r0, y+24 \n\t" \
    "sbc r11, r0 \n\t" \
    "ldd r0, y+25 \n\t" \
    "sbc r12, r0 \n\t" \
    "ldd r0, y+26 \n\t" \
    "sbc r13, r0 \n\t" \
    "ldd r0, y+27 \n\t" \
    "sbc r14, r0 \n\t" \
    "ldd r0, y+28 \n\t" \
    "sbc r15, r0 \n\t" \
    "sbci r18, 0 \n\t" \
    "mov r20, r19 \n\t" \
    "lsl r20 \n\t" \
    "sbc r20, r20 \n\t" \
    "add r2, r19 \n\t" \
    "std z+28, r2 \n\t" \
    "adc r3, r20 \n\t" \
    "std z+29, r3 \n\t" \
    "adc r4, r20 \n\t" \
    "std z+30, r4 \n\t" \
    "adc r5, r20 \n\t" \
    "std z+31, r5 \n\t" \
    "adc r6, r20 \n\t" \
    "std z+32, r6 \n\t" \
    "adc r7, r20 \n\t" \
    "std z+33, r7 \n\t" \
    "adc r8, r20 \n\t" \
    "std z+34, r8 \n\t" \
    "adc r9, r20 \n\t" \
    "std z+35, r9 \n\t" \
    "adc r10, r20 \n\t" \
    "std z+36, r10 \n\t" \
    "adc r11, r20 \n\t" \
    "std z+37, r11 \n\t" \
    "adc r12, r20 \n\t" \
    "std z+38, r12 \n\t" \
    "adc r13, r20 \n\t" \
    "std z+39, r13 \n\t" \
    "adc r14, r20 \n\t" \
    "std z+40, r14 \n\t" \
    "adc r15, r20 \n\t" \
    "std z+41, r15 \n\t" \
    "adc r18, r20 \n\t" \
    "mov r20, r18 \n\t" \
    "lsl r20 \n\t" \
    "sbc r20, r20 \n\t" \
    "ldd r0, z+42 \n\t" \
    "add r0, r18 \n\t" \
    "std z+42, r0 \n\t" \
    "ldd r0, z+43 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+43, r0 \n\t" \
    "ldd r0, z+44 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+44, r0 \n\t" \
    "ldd r0, z+45 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+45, r0 \n\t" \
    "ldd r0, z+46 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+46, r0 \n\t" \
    "ldd r0, z+47 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+47, r0 \n\t" \
    "ldd r0, z+48 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+48, r0 \n\t" \
    "ldd r0, z+49 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+49, r0 \n\t" \
    "ldd r0, z+50 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+50, r0 \n\t" \
    "ldd r0, z+51 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+51, r0 \n\t" \
    "ldd r0, z+52 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+52, r0 \n\t" \
    "ldd r0, z+53 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+53, r0 \n\t" \
    "ldd r0, z+54 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+54, r0 \n\t" \
    "ldd r0, z+55 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+55, r0 \n\t" \
    "adiw r28, 30 \n\t" \
    /* Atomic ISR-safe SP update. */ \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r29 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r28 \n\t"

/* Shared 112-bit (14 -> 28-byte) MOVW-aligned leaf squarer for FAST_SQUARE_ASM_28
   (94: loads from X+; 95: squares |A_l - A_h| already in registers). */
#define FAST_SQUARE_LEAF_28 \
    "94: \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ld r6, x+ \n\t" \
    "ld r7, x+ \n\t" \
    "ld r8, x+ \n\t" \
    "ld r9, x+ \n\t" \
    "ld r10, x+ \n\t" \
    "ld r11, x+ \n\t" \
    "ld r12, x+ \n\t" \
    "ld r13, x+ \n\t" \
    "ld r14, x+ \n\t" \
    "ld r15, x+ \n\t" \
    "95: \n\t" \
    "clr r24 \n\t" \
    "clr r18 \n\t" \
    "clr r19 \n\t" \
    "clr r20 \n\t" \
    "clr r21 \n\t" \
    "mul r2, r3 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "clr r22 \n\t" \
    "mul r2, r4 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "clr r23 \n\t" \
    "mul r2, r5 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r3, r4 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "lsl r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "rol r22 \n\t" \
    "rol r23 \n\t" \
    "mul r2, r2 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r3, r3 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "adc r23, r24 \n\t" \
    "std z+0, r18 \n\t" \
    "std z+1, r19 \n\t" \
    "std z+2, r20 \n\t" \
    "std z+3, r21 \n\t" \
    "mul r2, r6 \n\t" \
    "movw r18, r0 \n\t" \
    "clr r20 \n\t" \
    "mul r3, r5 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "clr r21 \n\t" \
    "mul r2, r7 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r3, r6 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r4, r5 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "clr r28 \n\t" \
    "mul r2, r8 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r3, r7 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r4, r6 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "clr r29 \n\t" \
    "mul r2, r9 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r3, r8 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r4, r7 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r5, r6 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "lsl r18 \n\t" \
    "rol r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "rol r28 \n\t" \
    "rol r29 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r24 \n\t" \
    "adc r21, r24 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r4, r4 \n\t" \
    "movw r22, r0 \n\t" \
    "mul r5, r5 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r24 \n\t" \
    "std z+4, r18 \n\t" \
    "std z+5, r19 \n\t" \
    "std z+6, r20 \n\t" \
    "std z+7, r21 \n\t" \
    "mul r2, r10 \n\t" \
    "movw r18, r0 \n\t" \
    "clr r20 \n\t" \
    "mul r3, r9 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r4, r8 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r5, r7 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "clr r21 \n\t" \
    "mul r2, r11 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r3, r10 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r4, r9 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r5, r8 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r6, r7 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "clr r22 \n\t" \
    "mul r2, r12 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r3, r11 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r4, r10 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r5, r9 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r6, r8 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "clr r23 \n\t" \
    "mul r2, r13 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r3, r12 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r4, r11 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r5, r10 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r6, r9 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r7, r8 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "lsl r18 \n\t" \
    "rol r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "rol r22 \n\t" \
    "rol r23 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r24 \n\t" \
    "adc r21, r24 \n\t" \
    "adc r22, r24 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r6, r6 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r7, r7 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "adc r23, r24 \n\t" \
    "std z+8, r18 \n\t" \
    "std z+9, r19 \n\t" \
    "std z+10, r20 \n\t" \
    "std z+11, r21 \n\t" \
    "mul r2, r14 \n\t" \
    "movw r18, r0 \n\t" \
    "clr r20 \n\t" \
    "mul r3, r13 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r4, r12 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r5, r11 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r6, r10 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r7, r9 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "clr r21 \n\t" \
    "mul r2, r15 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r3, r14 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r4, r13 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r5, r12 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r6, r11 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r7, r10 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r8, r9 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "clr r28 \n\t" \
    "mul r3, r15 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r4, r14 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r5, r13 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r6, r12 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r7, r11 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r8, r10 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "clr r29 \n\t" \
    "mul r4, r15 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r5, r14 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r6, r13 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r7, r12 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r8, r11 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r9, r10 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "lsl r18 \n\t" \
    "rol r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "rol r28 \n\t" \
    "rol r29 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r24 \n\t" \
    "adc r21, r24 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r8, r8 \n\t" \
    "movw r22, r0 \n\t" \
    "mul r9, r9 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r24 \n\t" \
    "std z+12, r18 \n\t" \
    "std z+13, r19 \n\t" \
    "std z+14, r20 \n\t" \
    "std z+15, r21 \n\t" \
    "mul r5, r15 \n\t" \
    "movw r18, r0 \n\t" \
    "clr r20 \n\t" \
    "mul r6, r14 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r7, r13 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r8, r12 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r9, r11 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "clr r21 \n\t" \
    "mul r6, r15 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r7, r14 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r8, r13 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r9, r12 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r10, r11 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "clr r22 \n\t" \
    "mul r7, r15 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r8, r14 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r9, r13 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r10, r12 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "clr r23 \n\t" \
    "mul r8, r15 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r9, r14 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r10, r13 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r11, r12 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "lsl r18 \n\t" \
    "rol r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "rol r22 \n\t" \
    "rol r23 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r24 \n\t" \
    "adc r21, r24 \n\t" \
    "adc r22, r24 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r10, r10 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r11, r11 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "adc r23, r24 \n\t" \
    "std z+16, r18 \n\t" \
    "std z+17, r19 \n\t" \
    "std z+18, r20 \n\t" \
    "std z+19, r21 \n\t" \
    "mul r9, r15 \n\t" \
    "movw r18, r0 \n\t" \
    "clr r20 \n\t" \
    "mul r10, r14 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r11, r13 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "clr r21 \n\t" \
    "mul r10, r15 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r11, r14 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r12, r13 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "clr r28 \n\t" \
    "mul r11, r15 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r12, r14 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "clr r29 \n\t" \
    "mul r12, r15 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r13, r14 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "lsl r18 \n\t" \
    "rol r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "rol r28 \n\t" \
    "rol r29 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r24 \n\t" \
    "adc r21, r24 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r12, r12 \n\t" \
    "movw r22, r0 \n\t" \
    "mul r13, r13 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r24 \n\t" \
    "std z+20, r18 \n\t" \
    "std z+21, r19 \n\t" \
    "std z+22, r20 \n\t" \
    "std z+23, r21 \n\t" \
    "mul r13, r15 \n\t" \
    "movw r18, r0 \n\t" \
    "clr r20 \n\t" \
    "clr r21 \n\t" \
    "mul r14, r15 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "lsl r18 \n\t" \
    "rol r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r24 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r14, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r15, r15 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "std z+24, r18 \n\t" \
    "std z+25, r19 \n\t" \
    "std z+26, r20 \n\t" \
    "std z+27, r21 \n\t" \
    "ret \n\t"

/* 256-bit (32 -> 64-byte) 1-level branch-free Karatsuba squarer
   (calls 16-byte leaf squarer at labels 96f / 97f). */
#define FAST_SQUARE_ASM_32 \
    "rcall 96f \n\t" \
    "adiw r30, 32 \n\t" \
    "rcall 96f \n\t" \
    "sbiw r30, 32 \n\t" \
    "sbiw r26, 32 \n\t" \
    "ld r0, x+ \n\t" \
    "sub r2, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r3, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r4, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r5, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r6, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r7, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r8, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r9, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r10, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r11, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r12, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r13, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r14, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r15, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r16, r0 \n\t" \
    "ld r0, x+ \n\t" \
    "sbc r17, r0 \n\t" \
    "sbc r18, r18 \n\t" \
    "eor r2, r18 \n\t" \
    "eor r3, r18 \n\t" \
    "eor r4, r18 \n\t" \
    "eor r5, r18 \n\t" \
    "eor r6, r18 \n\t" \
    "eor r7, r18 \n\t" \
    "eor r8, r18 \n\t" \
    "eor r9, r18 \n\t" \
    "eor r10, r18 \n\t" \
    "eor r11, r18 \n\t" \
    "eor r12, r18 \n\t" \
    "eor r13, r18 \n\t" \
    "eor r14, r18 \n\t" \
    "eor r15, r18 \n\t" \
    "eor r16, r18 \n\t" \
    "eor r17, r18 \n\t" \
    "sub r2, r18 \n\t" \
    "sbc r3, r18 \n\t" \
    "sbc r4, r18 \n\t" \
    "sbc r5, r18 \n\t" \
    "sbc r6, r18 \n\t" \
    "sbc r7, r18 \n\t" \
    "sbc r8, r18 \n\t" \
    "sbc r9, r18 \n\t" \
    "sbc r10, r18 \n\t" \
    "sbc r11, r18 \n\t" \
    "sbc r12, r18 \n\t" \
    "sbc r13, r18 \n\t" \
    "sbc r14, r18 \n\t" \
    "sbc r15, r18 \n\t" \
    "sbc r16, r18 \n\t" \
    "sbc r17, r18 \n\t" \
    "push r30 \n\t" \
    "push r31 \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "sbiw r28, 32 \n\t" \
    /* Atomic ISR-safe SP update. */ \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r29 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r28 \n\t" \
    "movw r30, r28 \n\t" \
    "adiw r30, 1 \n\t" \
    "rcall 97f \n\t" \
    "in r28, __SP_L__ \n\t" \
    "in r29, __SP_H__ \n\t" \
    "movw r26, r28 \n\t" \
    "adiw r26, 1 \n\t" \
    "ldd r31, y+33 \n\t" \
    "ldd r30, y+34 \n\t" \
    "ldd r2, z+16 \n\t" \
    "ldd r3, z+17 \n\t" \
    "ldd r4, z+18 \n\t" \
    "ldd r5, z+19 \n\t" \
    "ldd r6, z+20 \n\t" \
    "ldd r7, z+21 \n\t" \
    "ldd r8, z+22 \n\t" \
    "ldd r9, z+23 \n\t" \
    "ldd r10, z+24 \n\t" \
    "ldd r11, z+25 \n\t" \
    "ldd r12, z+26 \n\t" \
    "ldd r13, z+27 \n\t" \
    "ldd r14, z+28 \n\t" \
    "ldd r15, z+29 \n\t" \
    "ldd r16, z+30 \n\t" \
    "ldd r17, z+31 \n\t" \
    "ldd r2, z+16 \n\t" \
    "ldd r3, z+17 \n\t" \
    "ldd r4, z+18 \n\t" \
    "ldd r5, z+19 \n\t" \
    "ldd r6, z+20 \n\t" \
    "ldd r7, z+21 \n\t" \
    "ldd r8, z+22 \n\t" \
    "ldd r9, z+23 \n\t" \
    "ldd r10, z+24 \n\t" \
    "ldd r11, z+25 \n\t" \
    "ldd r12, z+26 \n\t" \
    "ldd r13, z+27 \n\t" \
    "ldd r14, z+28 \n\t" \
    "ldd r15, z+29 \n\t" \
    "ldd r16, z+30 \n\t" \
    "ldd r17, z+31 \n\t" \
    "ldd r20, z+32 \n\t" \
    "add r2, r20 \n\t" \
    "ldd r20, z+33 \n\t" \
    "adc r3, r20 \n\t" \
    "ldd r20, z+34 \n\t" \
    "adc r4, r20 \n\t" \
    "ldd r20, z+35 \n\t" \
    "adc r5, r20 \n\t" \
    "ldd r20, z+36 \n\t" \
    "adc r6, r20 \n\t" \
    "ldd r20, z+37 \n\t" \
    "adc r7, r20 \n\t" \
    "ldd r20, z+38 \n\t" \
    "adc r8, r20 \n\t" \
    "ldd r20, z+39 \n\t" \
    "adc r9, r20 \n\t" \
    "ldd r20, z+40 \n\t" \
    "adc r10, r20 \n\t" \
    "ldd r20, z+41 \n\t" \
    "adc r11, r20 \n\t" \
    "ldd r20, z+42 \n\t" \
    "adc r12, r20 \n\t" \
    "ldd r20, z+43 \n\t" \
    "adc r13, r20 \n\t" \
    "ldd r20, z+44 \n\t" \
    "adc r14, r20 \n\t" \
    "ldd r20, z+45 \n\t" \
    "adc r15, r20 \n\t" \
    "ldd r20, z+46 \n\t" \
    "adc r16, r20 \n\t" \
    "ldd r20, z+47 \n\t" \
    "adc r17, r20 \n\t" \
    "clr r18 \n\t" \
    "adc r18, r18 \n\t" \
    "clr r19 \n\t" \
    "clr r25 \n\t" \
    "movw r20, r2 \n\t" \
    "movw r22, r4 \n\t" \
    "movw r26, r6 \n\t" \
    "lsr r19 \n\t" \
    "ldd r0, z+0 \n\t" \
    "adc r20, r0 \n\t" \
    "ldd r0, z+1 \n\t" \
    "adc r21, r0 \n\t" \
    "ldd r0, z+2 \n\t" \
    "adc r22, r0 \n\t" \
    "ldd r0, z+3 \n\t" \
    "adc r23, r0 \n\t" \
    "ldd r0, z+4 \n\t" \
    "adc r26, r0 \n\t" \
    "ldd r0, z+5 \n\t" \
    "adc r27, r0 \n\t" \
    "adc r19, r19 \n\t" \
    "lsr r25 \n\t" \
    "ldd r0, y+1 \n\t" \
    "sbc r20, r0 \n\t" \
    "std z+16, r20 \n\t" \
    "ldd r0, y+2 \n\t" \
    "sbc r21, r0 \n\t" \
    "std z+17, r21 \n\t" \
    "ldd r0, y+3 \n\t" \
    "sbc r22, r0 \n\t" \
    "std z+18, r22 \n\t" \
    "ldd r0, y+4 \n\t" \
    "sbc r23, r0 \n\t" \
    "std z+19, r23 \n\t" \
    "ldd r0, y+5 \n\t" \
    "sbc r26, r0 \n\t" \
    "std z+20, r26 \n\t" \
    "ldd r0, y+6 \n\t" \
    "sbc r27, r0 \n\t" \
    "std z+21, r27 \n\t" \
    "adc r25, r25 \n\t" \
    "movw r20, r8 \n\t" \
    "movw r22, r10 \n\t" \
    "movw r26, r12 \n\t" \
    "lsr r19 \n\t" \
    "ldd r0, z+6 \n\t" \
    "adc r20, r0 \n\t" \
    "ldd r0, z+7 \n\t" \
    "adc r21, r0 \n\t" \
    "ldd r0, z+8 \n\t" \
    "adc r22, r0 \n\t" \
    "ldd r0, z+9 \n\t" \
    "adc r23, r0 \n\t" \
    "ldd r0, z+10 \n\t" \
    "adc r26, r0 \n\t" \
    "ldd r0, z+11 \n\t" \
    "adc r27, r0 \n\t" \
    "adc r19, r19 \n\t" \
    "lsr r25 \n\t" \
    "ldd r0, y+7 \n\t" \
    "sbc r20, r0 \n\t" \
    "std z+22, r20 \n\t" \
    "ldd r0, y+8 \n\t" \
    "sbc r21, r0 \n\t" \
    "std z+23, r21 \n\t" \
    "ldd r0, y+9 \n\t" \
    "sbc r22, r0 \n\t" \
    "std z+24, r22 \n\t" \
    "ldd r0, y+10 \n\t" \
    "sbc r23, r0 \n\t" \
    "std z+25, r23 \n\t" \
    "ldd r0, y+11 \n\t" \
    "sbc r26, r0 \n\t" \
    "std z+26, r26 \n\t" \
    "ldd r0, y+12 \n\t" \
    "sbc r27, r0 \n\t" \
    "std z+27, r27 \n\t" \
    "adc r25, r25 \n\t" \
    "movw r20, r14 \n\t" \
    "movw r22, r16 \n\t" \
    "lsr r19 \n\t" \
    "ldd r0, z+12 \n\t" \
    "adc r20, r0 \n\t" \
    "ldd r0, z+13 \n\t" \
    "adc r21, r0 \n\t" \
    "ldd r0, z+14 \n\t" \
    "adc r22, r0 \n\t" \
    "ldd r0, z+15 \n\t" \
    "adc r23, r0 \n\t" \
    "adc r19, r19 \n\t" \
    "lsr r25 \n\t" \
    "ldd r0, y+13 \n\t" \
    "sbc r20, r0 \n\t" \
    "std z+28, r20 \n\t" \
    "ldd r0, y+14 \n\t" \
    "sbc r21, r0 \n\t" \
    "std z+29, r21 \n\t" \
    "ldd r0, y+15 \n\t" \
    "sbc r22, r0 \n\t" \
    "std z+30, r22 \n\t" \
    "ldd r0, y+16 \n\t" \
    "sbc r23, r0 \n\t" \
    "std z+31, r23 \n\t" \
    "adc r25, r25 \n\t" \
    "add r19, r18 \n\t" \
    "sub r19, r25 \n\t" \
    "clr r25 \n\t" \
    "ldd r0, z+48 \n\t" \
    "add r2, r0 \n\t" \
    "ldd r0, z+49 \n\t" \
    "adc r3, r0 \n\t" \
    "ldd r0, z+50 \n\t" \
    "adc r4, r0 \n\t" \
    "ldd r0, z+51 \n\t" \
    "adc r5, r0 \n\t" \
    "ldd r0, z+52 \n\t" \
    "adc r6, r0 \n\t" \
    "ldd r0, z+53 \n\t" \
    "adc r7, r0 \n\t" \
    "ldd r0, z+54 \n\t" \
    "adc r8, r0 \n\t" \
    "ldd r0, z+55 \n\t" \
    "adc r9, r0 \n\t" \
    "ldd r0, z+56 \n\t" \
    "adc r10, r0 \n\t" \
    "ldd r0, z+57 \n\t" \
    "adc r11, r0 \n\t" \
    "ldd r0, z+58 \n\t" \
    "adc r12, r0 \n\t" \
    "ldd r0, z+59 \n\t" \
    "adc r13, r0 \n\t" \
    "ldd r0, z+60 \n\t" \
    "adc r14, r0 \n\t" \
    "ldd r0, z+61 \n\t" \
    "adc r15, r0 \n\t" \
    "ldd r0, z+62 \n\t" \
    "adc r16, r0 \n\t" \
    "ldd r0, z+63 \n\t" \
    "adc r17, r0 \n\t" \
    "adc r18, r25 \n\t" \
    "ldd r0, y+17 \n\t" \
    "sub r2, r0 \n\t" \
    "ldd r0, y+18 \n\t" \
    "sbc r3, r0 \n\t" \
    "ldd r0, y+19 \n\t" \
    "sbc r4, r0 \n\t" \
    "ldd r0, y+20 \n\t" \
    "sbc r5, r0 \n\t" \
    "ldd r0, y+21 \n\t" \
    "sbc r6, r0 \n\t" \
    "ldd r0, y+22 \n\t" \
    "sbc r7, r0 \n\t" \
    "ldd r0, y+23 \n\t" \
    "sbc r8, r0 \n\t" \
    "ldd r0, y+24 \n\t" \
    "sbc r9, r0 \n\t" \
    "ldd r0, y+25 \n\t" \
    "sbc r10, r0 \n\t" \
    "ldd r0, y+26 \n\t" \
    "sbc r11, r0 \n\t" \
    "ldd r0, y+27 \n\t" \
    "sbc r12, r0 \n\t" \
    "ldd r0, y+28 \n\t" \
    "sbc r13, r0 \n\t" \
    "ldd r0, y+29 \n\t" \
    "sbc r14, r0 \n\t" \
    "ldd r0, y+30 \n\t" \
    "sbc r15, r0 \n\t" \
    "ldd r0, y+31 \n\t" \
    "sbc r16, r0 \n\t" \
    "ldd r0, y+32 \n\t" \
    "sbc r17, r0 \n\t" \
    "sbci r18, 0 \n\t" \
    "mov r20, r19 \n\t" \
    "lsl r20 \n\t" \
    "sbc r20, r20 \n\t" \
    "add r2, r19 \n\t" \
    "std z+32, r2 \n\t" \
    "adc r3, r20 \n\t" \
    "std z+33, r3 \n\t" \
    "adc r4, r20 \n\t" \
    "std z+34, r4 \n\t" \
    "adc r5, r20 \n\t" \
    "std z+35, r5 \n\t" \
    "adc r6, r20 \n\t" \
    "std z+36, r6 \n\t" \
    "adc r7, r20 \n\t" \
    "std z+37, r7 \n\t" \
    "adc r8, r20 \n\t" \
    "std z+38, r8 \n\t" \
    "adc r9, r20 \n\t" \
    "std z+39, r9 \n\t" \
    "adc r10, r20 \n\t" \
    "std z+40, r10 \n\t" \
    "adc r11, r20 \n\t" \
    "std z+41, r11 \n\t" \
    "adc r12, r20 \n\t" \
    "std z+42, r12 \n\t" \
    "adc r13, r20 \n\t" \
    "std z+43, r13 \n\t" \
    "adc r14, r20 \n\t" \
    "std z+44, r14 \n\t" \
    "adc r15, r20 \n\t" \
    "std z+45, r15 \n\t" \
    "adc r16, r20 \n\t" \
    "std z+46, r16 \n\t" \
    "adc r17, r20 \n\t" \
    "std z+47, r17 \n\t" \
    "adc r18, r20 \n\t" \
    "mov r20, r18 \n\t" \
    "lsl r20 \n\t" \
    "sbc r20, r20 \n\t" \
    "ldd r0, z+48 \n\t" \
    "add r0, r18 \n\t" \
    "std z+48, r0 \n\t" \
    "ldd r0, z+49 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+49, r0 \n\t" \
    "ldd r0, z+50 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+50, r0 \n\t" \
    "ldd r0, z+51 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+51, r0 \n\t" \
    "ldd r0, z+52 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+52, r0 \n\t" \
    "ldd r0, z+53 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+53, r0 \n\t" \
    "ldd r0, z+54 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+54, r0 \n\t" \
    "ldd r0, z+55 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+55, r0 \n\t" \
    "ldd r0, z+56 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+56, r0 \n\t" \
    "ldd r0, z+57 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+57, r0 \n\t" \
    "ldd r0, z+58 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+58, r0 \n\t" \
    "ldd r0, z+59 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+59, r0 \n\t" \
    "ldd r0, z+60 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+60, r0 \n\t" \
    "ldd r0, z+61 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+61, r0 \n\t" \
    "ldd r0, z+62 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+62, r0 \n\t" \
    "ldd r0, z+63 \n\t" \
    "adc r0, r20 \n\t" \
    "std z+63, r0 \n\t" \
    "adiw r28, 34 \n\t" \
    /* Atomic ISR-safe SP update. */ \
    "in r0, __SREG__ \n\t" \
    "cli \n\t" \
    "out __SP_H__, r29 \n\t" \
    "out __SREG__, r0 \n\t" \
    "out __SP_L__, r28 \n\t"

/* Shared 128-bit (16 -> 32-byte) MOVW-aligned leaf squarer for FAST_SQUARE_ASM_32
   (96: loads from X+; 97: squares |A_l - A_h| already in registers). */
#define FAST_SQUARE_LEAF_32 \
    "96: \n\t" \
    "ld r2, x+ \n\t" \
    "ld r3, x+ \n\t" \
    "ld r4, x+ \n\t" \
    "ld r5, x+ \n\t" \
    "ld r6, x+ \n\t" \
    "ld r7, x+ \n\t" \
    "ld r8, x+ \n\t" \
    "ld r9, x+ \n\t" \
    "ld r10, x+ \n\t" \
    "ld r11, x+ \n\t" \
    "ld r12, x+ \n\t" \
    "ld r13, x+ \n\t" \
    "ld r14, x+ \n\t" \
    "ld r15, x+ \n\t" \
    "ld r16, x+ \n\t" \
    "ld r17, x+ \n\t" \
    "97: \n\t" \
    "clr r24 \n\t" \
    "clr r18 \n\t" \
    "clr r19 \n\t" \
    "clr r20 \n\t" \
    "clr r21 \n\t" \
    "mul r2, r3 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "clr r22 \n\t" \
    "mul r2, r4 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "clr r23 \n\t" \
    "mul r2, r5 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r3, r4 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "lsl r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "rol r22 \n\t" \
    "rol r23 \n\t" \
    "mul r2, r2 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r3, r3 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "adc r23, r24 \n\t" \
    "std z+0, r18 \n\t" \
    "std z+1, r19 \n\t" \
    "std z+2, r20 \n\t" \
    "std z+3, r21 \n\t" \
    "mul r2, r6 \n\t" \
    "movw r18, r0 \n\t" \
    "clr r20 \n\t" \
    "mul r3, r5 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "clr r21 \n\t" \
    "mul r2, r7 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r3, r6 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r4, r5 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "clr r28 \n\t" \
    "mul r2, r8 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r3, r7 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r4, r6 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "clr r29 \n\t" \
    "mul r2, r9 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r3, r8 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r4, r7 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r5, r6 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "lsl r18 \n\t" \
    "rol r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "rol r28 \n\t" \
    "rol r29 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r24 \n\t" \
    "adc r21, r24 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r4, r4 \n\t" \
    "movw r22, r0 \n\t" \
    "mul r5, r5 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r24 \n\t" \
    "std z+4, r18 \n\t" \
    "std z+5, r19 \n\t" \
    "std z+6, r20 \n\t" \
    "std z+7, r21 \n\t" \
    "mul r2, r10 \n\t" \
    "movw r18, r0 \n\t" \
    "clr r20 \n\t" \
    "mul r3, r9 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r4, r8 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r5, r7 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "clr r21 \n\t" \
    "mul r2, r11 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r3, r10 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r4, r9 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r5, r8 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r6, r7 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "clr r22 \n\t" \
    "mul r2, r12 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r3, r11 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r4, r10 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r5, r9 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r6, r8 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "clr r23 \n\t" \
    "mul r2, r13 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r3, r12 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r4, r11 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r5, r10 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r6, r9 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r7, r8 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "lsl r18 \n\t" \
    "rol r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "rol r22 \n\t" \
    "rol r23 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r24 \n\t" \
    "adc r21, r24 \n\t" \
    "adc r22, r24 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r6, r6 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r7, r7 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "adc r23, r24 \n\t" \
    "std z+8, r18 \n\t" \
    "std z+9, r19 \n\t" \
    "std z+10, r20 \n\t" \
    "std z+11, r21 \n\t" \
    "mul r2, r14 \n\t" \
    "movw r18, r0 \n\t" \
    "clr r20 \n\t" \
    "mul r3, r13 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r4, r12 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r5, r11 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r6, r10 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r7, r9 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "clr r21 \n\t" \
    "mul r2, r15 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r3, r14 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r4, r13 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r5, r12 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r6, r11 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r7, r10 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r8, r9 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "clr r28 \n\t" \
    "mul r2, r16 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r3, r15 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r4, r14 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r5, r13 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r6, r12 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r7, r11 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r8, r10 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "clr r29 \n\t" \
    "mul r2, r17 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r3, r16 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r4, r15 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r5, r14 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r6, r13 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r7, r12 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r8, r11 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r9, r10 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "lsl r18 \n\t" \
    "rol r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "rol r28 \n\t" \
    "rol r29 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r24 \n\t" \
    "adc r21, r24 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r8, r8 \n\t" \
    "movw r22, r0 \n\t" \
    "mul r9, r9 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r24 \n\t" \
    "std z+12, r18 \n\t" \
    "std z+13, r19 \n\t" \
    "std z+14, r20 \n\t" \
    "std z+15, r21 \n\t" \
    "mul r3, r17 \n\t" \
    "movw r18, r0 \n\t" \
    "clr r20 \n\t" \
    "mul r4, r16 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r5, r15 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r6, r14 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r7, r13 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r8, r12 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r9, r11 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "clr r21 \n\t" \
    "mul r4, r17 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r5, r16 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r6, r15 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r7, r14 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r8, r13 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r9, r12 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r10, r11 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "clr r22 \n\t" \
    "mul r5, r17 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r6, r16 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r7, r15 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r8, r14 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r9, r13 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r10, r12 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "clr r23 \n\t" \
    "mul r6, r17 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r7, r16 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r8, r15 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r9, r14 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r10, r13 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r11, r12 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "lsl r18 \n\t" \
    "rol r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "rol r22 \n\t" \
    "rol r23 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r24 \n\t" \
    "adc r21, r24 \n\t" \
    "adc r22, r24 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r10, r10 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r11, r11 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "adc r23, r24 \n\t" \
    "std z+16, r18 \n\t" \
    "std z+17, r19 \n\t" \
    "std z+18, r20 \n\t" \
    "std z+19, r21 \n\t" \
    "mul r7, r17 \n\t" \
    "movw r18, r0 \n\t" \
    "clr r20 \n\t" \
    "mul r8, r16 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r9, r15 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r10, r14 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r11, r13 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "clr r21 \n\t" \
    "mul r8, r17 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r9, r16 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r10, r15 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r11, r14 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r12, r13 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "clr r28 \n\t" \
    "mul r9, r17 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r10, r16 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r11, r15 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "mul r12, r14 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "clr r29 \n\t" \
    "mul r10, r17 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r11, r16 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r12, r15 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r13, r14 \n\t" \
    "add r21, r0 \n\t" \
    "adc r28, r1 \n\t" \
    "adc r29, r24 \n\t" \
    "lsl r18 \n\t" \
    "rol r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "rol r28 \n\t" \
    "rol r29 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r24 \n\t" \
    "adc r21, r24 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r24 \n\t" \
    "mul r12, r12 \n\t" \
    "movw r22, r0 \n\t" \
    "mul r13, r13 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r28, r24 \n\t" \
    "adc r29, r24 \n\t" \
    "std z+20, r18 \n\t" \
    "std z+21, r19 \n\t" \
    "std z+22, r20 \n\t" \
    "std z+23, r21 \n\t" \
    "mul r11, r17 \n\t" \
    "movw r18, r0 \n\t" \
    "clr r20 \n\t" \
    "mul r12, r16 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "mul r13, r15 \n\t" \
    "add r18, r0 \n\t" \
    "adc r19, r1 \n\t" \
    "adc r20, r24 \n\t" \
    "clr r21 \n\t" \
    "mul r12, r17 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r13, r16 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r14, r15 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "clr r22 \n\t" \
    "mul r13, r17 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "mul r14, r16 \n\t" \
    "add r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "clr r23 \n\t" \
    "mul r14, r17 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r15, r16 \n\t" \
    "add r21, r0 \n\t" \
    "adc r22, r1 \n\t" \
    "adc r23, r24 \n\t" \
    "lsl r18 \n\t" \
    "rol r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "rol r22 \n\t" \
    "rol r23 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r24 \n\t" \
    "adc r21, r24 \n\t" \
    "adc r22, r24 \n\t" \
    "adc r23, r24 \n\t" \
    "mul r14, r14 \n\t" \
    "movw r28, r0 \n\t" \
    "mul r15, r15 \n\t" \
    "add r18, r28 \n\t" \
    "adc r19, r29 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "adc r22, r24 \n\t" \
    "adc r23, r24 \n\t" \
    "std z+24, r18 \n\t" \
    "std z+25, r19 \n\t" \
    "std z+26, r20 \n\t" \
    "std z+27, r21 \n\t" \
    "mul r15, r17 \n\t" \
    "movw r18, r0 \n\t" \
    "clr r20 \n\t" \
    "clr r21 \n\t" \
    "mul r16, r17 \n\t" \
    "add r19, r0 \n\t" \
    "adc r20, r1 \n\t" \
    "adc r21, r24 \n\t" \
    "lsl r18 \n\t" \
    "rol r19 \n\t" \
    "rol r20 \n\t" \
    "rol r21 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r24 \n\t" \
    "adc r21, r24 \n\t" \
    "mul r16, r16 \n\t" \
    "movw r22, r0 \n\t" \
    "mul r17, r17 \n\t" \
    "add r18, r22 \n\t" \
    "adc r19, r23 \n\t" \
    "adc r20, r0 \n\t" \
    "adc r21, r1 \n\t" \
    "std z+28, r18 \n\t" \
    "std z+29, r19 \n\t" \
    "std z+30, r20 \n\t" \
    "std z+31, r21 \n\t" \
    "ret \n\t"

#endif /* uECC_SQUARE_FUNC */

#endif /* _UECC_ASM_AVR_MULT_SQUARE_H_ */
