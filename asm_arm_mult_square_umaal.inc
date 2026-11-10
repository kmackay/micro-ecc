/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#ifndef _UECC_ASM_ARM_MULT_SQUARE_UMAAL_H_
#define _UECC_ASM_ARM_MULT_SQUARE_UMAAL_H_

/* ============================================================================
 * Thumb-2 / ARM with UMAAL (Cortex-M4 / M7 / M33 / ARMv7E-M)
 *
 * Uses Multi-Pass Wide-Register UMAAL Row Accumulation for multiplication:
 *   umaal RdLo, RdHi, Rn, Rm  computes  (RdHi:RdLo) = Rn * Rm + RdLo + RdHi
 * in 1 cycle without reading or modifying CPSR flags.
 * - nw = 5: 1-pass 5-column UMAAL row loop (zero intermediate memory spills).
 * - nw = 6: 2-pass (4+2) UMAAL row split keeping all 6 partial words (result[4..9])
 *           and a5 in registers across passes (zero intermediate memory spills).
 * - nw = 7: 2-pass (5+2) UMAAL row split (only 1 intermediate word spilled).
 * - nw = 8: 2-pass (5+3) UMAAL row split using dual-addend UMAAL in Pass 2 to
 *           fold stored Pass-1 partial words with zero extra add instructions.
 *
 * For squaring:
 * - Zero-spill UMAAL upper-triangle accumulation with interleaved early
 *   finalization of low result pairs to free operand registers, followed by an
 *   unbroken diagonal-square accumulation chain (umaal + umull/adcs).
 * ============================================================================ */

/* ---- Level 4 Standalone Multiplication Macros ---- */

/* nw = 5: 1-pass 5-column UMAAL row loop (zero intermediate memory spills) */
#define FAST_MULT_ASM_5 \
    "ldmia r2, {r3, r4, r5, r6, r7} \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umull r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "movs  r10, #0 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "movs  r12, #0 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "movs  lr, #0 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "stmia r0!, {r9, r10, r11, r12, lr} \n\t"

/* nw = 6: 2-pass (4+2) UMAAL row split (zero intermediate memory spills) */
#define FAST_MULT_ASM_6 \
    "ldmia r2!, {r4, r5, r6, r7} \n\t" \
    "ldr   r3, [r1], #4 \n\t" \
    "umull r8, r9, r4, r3 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "movs  r10, #0 \n\t" \
    "umaal r9, r10, r5, r3 \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r10, r11, r6, r3 \n\t" \
    "movs  r12, #0 \n\t" \
    "umaal r11, r12, r7, r3 \n\t" \
    "ldr   r3, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r4, r3 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r5, r3 \n\t" \
    "umaal r10, r11, r6, r3 \n\t" \
    "umaal r11, r12, r7, r3 \n\t" \
    "ldr   r3, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r4, r3 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r5, r3 \n\t" \
    "umaal r10, r11, r6, r3 \n\t" \
    "umaal r11, r12, r7, r3 \n\t" \
    "ldr   r3, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r4, r3 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r5, r3 \n\t" \
    "umaal r10, r11, r6, r3 \n\t" \
    "umaal r11, r12, r7, r3 \n\t" \
    "ldr   r3, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r4, r3 \n\t" \
    "umaal r9, r10, r5, r3 \n\t" \
    "umaal r10, r11, r6, r3 \n\t" \
    "umaal r11, r12, r7, r3 \n\t" \
    "ldr   lr, [r1], #-20 \n\t" \
    "movs  r3, #0 \n\t" \
    "umaal r9, r3, r4, lr \n\t" \
    "umaal r10, r3, r5, lr \n\t" \
    "umaal r11, r3, r6, lr \n\t" \
    "umaal r12, r3, r7, lr \n\t" \
    "ldmia r2, {r4, r5} \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r6, #0 \n\t" \
    "umaal r8, r6, r4, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r6, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r7, #0 \n\t" \
    "umaal r9, r7, r4, r2 \n\t" \
    "str   r9, [r0], #4 \n\t" \
    "umaal r6, r7, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r10, r6, r4, r2 \n\t" \
    "str   r10, [r0], #4 \n\t" \
    "umaal r7, r6, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r11, r7, r4, r2 \n\t" \
    "str   r11, [r0], #4 \n\t" \
    "umaal r6, r7, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r12, r6, r4, r2 \n\t" \
    "str   r12, [r0], #4 \n\t" \
    "umaal r7, r6, r5, r2 \n\t" \
    "umaal r3, r7, r4, lr \n\t" \
    "umaal r6, r7, r5, lr \n\t" \
    "stmia r0!, {r3, r6, r7} \n\t"

/* nw = 7: 2-pass (5+2) UMAAL row split */
#define FAST_MULT_ASM_7 \
    "push  {r2} \n\t" \
    "ldmia r2, {r3, r4, r5, r6, r7} \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umull r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "movs  r10, #0 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "movs  r12, #0 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "movs  lr, #0 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0] \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #-24 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r9, r8, r3, r2 \n\t" \
    "umaal r10, r8, r4, r2 \n\t" \
    "umaal r11, r8, r5, r2 \n\t" \
    "umaal r12, r8, r6, r2 \n\t" \
    "umaal lr, r8, r7, r2 \n\t" \
    "pop   {r3} \n\t" \
    "add   r3, r3, #20 \n\t" \
    "ldmia r3, {r4, r5} \n\t" \
    "ldr   r3, [r0] \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r6, #0 \n\t" \
    "umaal r3, r6, r4, r2 \n\t" \
    "str   r3, [r0], #4 \n\t" \
    "umaal r9, r6, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r7, #0 \n\t" \
    "umaal r9, r7, r4, r2 \n\t" \
    "str   r9, [r0], #4 \n\t" \
    "umaal r6, r7, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r10, r6, r4, r2 \n\t" \
    "str   r10, [r0], #4 \n\t" \
    "umaal r7, r6, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r11, r7, r4, r2 \n\t" \
    "str   r11, [r0], #4 \n\t" \
    "umaal r6, r7, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r12, r6, r4, r2 \n\t" \
    "str   r12, [r0], #4 \n\t" \
    "umaal r7, r6, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal lr, r7, r4, r2 \n\t" \
    "str   lr, [r0], #4 \n\t" \
    "umaal r6, r7, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r8, r6, r4, r2 \n\t" \
    "umaal r6, r7, r5, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "stmia r0!, {r6, r7} \n\t"

/* nw = 8: 2-pass (5+3) UMAAL row split with dual-addend UMAAL */
#define FAST_MULT_ASM_8 \
    "push  {r2} \n\t" \
    "ldmia r2, {r3, r4, r5, r6, r7} \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umull r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "movs  r10, #0 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "movs  r12, #0 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "movs  lr, #0 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0, #0] \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0, #4] \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #-28 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0, #8] \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "str   r9, [r0, #12] \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "pop   {r2} \n\t" \
    "add   r2, r2, #20 \n\t" \
    "ldmia r2, {r5, r6, r7} \n\t" \
    "ldmia r0, {r3, r4, r8} \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r9, #0 \n\t" \
    "umaal r3, r9, r5, r2 \n\t" \
    "str   r3, [r0], #4 \n\t" \
    "umaal r4, r9, r6, r2 \n\t" \
    "umaal r8, r9, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r3, #0 \n\t" \
    "umaal r4, r3, r5, r2 \n\t" \
    "str   r4, [r0], #4 \n\t" \
    "umaal r8, r3, r6, r2 \n\t" \
    "umaal r9, r3, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r4, #0 \n\t" \
    "umaal r8, r4, r5, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r4, r6, r2 \n\t" \
    "umaal r3, r4, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "ldr   r8, [r0] \n\t" \
    "umaal r9, r8, r5, r2 \n\t" \
    "str   r9, [r0], #4 \n\t" \
    "umaal r3, r8, r6, r2 \n\t" \
    "umaal r4, r8, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r3, r10, r5, r2 \n\t" \
    "str   r3, [r0], #4 \n\t" \
    "umaal r4, r10, r6, r2 \n\t" \
    "umaal r8, r10, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r4, r11, r5, r2 \n\t" \
    "str   r4, [r0], #4 \n\t" \
    "umaal r8, r11, r6, r2 \n\t" \
    "umaal r10, r11, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r8, r12, r5, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r10, r12, r6, r2 \n\t" \
    "umaal r11, r12, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r10, lr, r5, r2 \n\t" \
    "umaal r11, lr, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "stmia r0!, {r10, r11, r12, lr} \n\t"

/* ---- Level 3 Shared-Pass Multiplication Macro ---- */
#define FAST_MULT_ASM_LEVEL3 \
    "cmp   r3, #6 \n\t" \
    "beq   60f \n\t" \
    "push  {r2, r3} \n\t" \
    "ldmia r2, {r3, r4, r5, r6, r7} \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umull r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "movs  r10, #0 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "movs  r12, #0 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "movs  lr, #0 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r8, [sp, #4] \n\t" \
    "cmp   r8, #5 \n\t" \
    "bne   70f \n\t" \
    "stmia r0!, {r9, r10, r11, r12, lr} \n\t" \
    "add   sp, sp, #8 \n\t" \
    "b     1f \n\t" \
    "70: \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0, #0] \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r8, [sp, #4] \n\t" \
    "cmp   r8, #7 \n\t" \
    "beq   75f \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0, #4] \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "ldr   r2, [r1], #-28 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0, #8] \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "str   r9, [r0, #12] \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "pop   {r2, r3} \n\t" \
    "add   r2, r2, #20 \n\t" \
    "ldmia r2, {r5, r6, r7} \n\t" \
    "ldmia r0, {r3, r4, r8} \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r9, #0 \n\t" \
    "umaal r3, r9, r5, r2 \n\t" \
    "str   r3, [r0], #4 \n\t" \
    "umaal r4, r9, r6, r2 \n\t" \
    "umaal r8, r9, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r3, #0 \n\t" \
    "umaal r4, r3, r5, r2 \n\t" \
    "str   r4, [r0], #4 \n\t" \
    "umaal r8, r3, r6, r2 \n\t" \
    "umaal r9, r3, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r4, #0 \n\t" \
    "umaal r8, r4, r5, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r9, r4, r6, r2 \n\t" \
    "umaal r3, r4, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "ldr   r8, [r0] \n\t" \
    "umaal r9, r8, r5, r2 \n\t" \
    "str   r9, [r0], #4 \n\t" \
    "umaal r3, r8, r6, r2 \n\t" \
    "umaal r4, r8, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r3, r10, r5, r2 \n\t" \
    "str   r3, [r0], #4 \n\t" \
    "umaal r4, r10, r6, r2 \n\t" \
    "umaal r8, r10, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r4, r11, r5, r2 \n\t" \
    "str   r4, [r0], #4 \n\t" \
    "umaal r8, r11, r6, r2 \n\t" \
    "umaal r10, r11, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r8, r12, r5, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "umaal r10, r12, r6, r2 \n\t" \
    "umaal r11, r12, r7, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r10, lr, r5, r2 \n\t" \
    "umaal r11, lr, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    "stmia r0!, {r10, r11, r12, lr} \n\t" \
    "b     1f \n\t" \
    "75: \n\t" \
    "ldr   r2, [r1], #-24 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r9, r8, r3, r2 \n\t" \
    "umaal r10, r8, r4, r2 \n\t" \
    "umaal r11, r8, r5, r2 \n\t" \
    "umaal r12, r8, r6, r2 \n\t" \
    "umaal lr, r8, r7, r2 \n\t" \
    "pop   {r3, r4} \n\t" \
    "add   r3, r3, #20 \n\t" \
    "ldmia r3, {r4, r5} \n\t" \
    "ldr   r3, [r0] \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r6, #0 \n\t" \
    "umaal r3, r6, r4, r2 \n\t" \
    "str   r3, [r0], #4 \n\t" \
    "umaal r9, r6, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r7, #0 \n\t" \
    "umaal r9, r7, r4, r2 \n\t" \
    "str   r9, [r0], #4 \n\t" \
    "umaal r6, r7, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r10, r6, r4, r2 \n\t" \
    "str   r10, [r0], #4 \n\t" \
    "umaal r7, r6, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r11, r7, r4, r2 \n\t" \
    "str   r11, [r0], #4 \n\t" \
    "umaal r6, r7, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r12, r6, r4, r2 \n\t" \
    "str   r12, [r0], #4 \n\t" \
    "umaal r7, r6, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal lr, r7, r4, r2 \n\t" \
    "str   lr, [r0], #4 \n\t" \
    "umaal r6, r7, r5, r2 \n\t" \
    "ldr   r2, [r1], #4 \n\t" \
    "umaal r8, r6, r4, r2 \n\t" \
    "umaal r6, r7, r5, r2 \n\t" \
    "str   r8, [r0], #4 \n\t" \
    "stmia r0!, {r6, r7} \n\t" \
    "b     1f \n\t" \
    "60: \n\t" \
    FAST_MULT_ASM_6

/* ---- Level 4 Standalone Squaring Macros ---- */

#define FAST_SQUARE_ASM_5 \
    "ldmia r1, {r2, r3, r4, r5, r6} \n\t" \
    "umull r7, r8, r2, r3 \n\t" \
    "movs  r9, #0 \n\t" \
    "umaal r8, r9, r2, r4 \n\t" \
    "movs  r10, #0 \n\t" \
    "umaal r9, r10, r3, r4 \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r9, r11, r2, r5 \n\t" \
    "umaal r10, r11, r3, r5 \n\t" \
    "movs  r12, #0 \n\t" \
    "umaal r11, r12, r4, r5 \n\t" \
    "movs  lr, #0 \n\t" \
    "umaal r10, lr, r2, r6 \n\t" \
    "umull r2, r1, r2, r2 \n\t" \
    "str   r2, [r0], #4 \n\t" \
    "movs  r2, #0 \n\t" \
    "adds  r7, r7, r7 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  r7, r7, r1 \n\t" \
    "str   r7, [r0], #4 \n\t" \
    "adc   r1, r2, #0 \n\t" \
    "umaal r11, lr, r3, r6 \n\t" \
    "umaal r12, lr, r4, r6 \n\t" \
    "movs  r2, #0 \n\t" \
    "umaal lr, r2, r5, r6 \n\t" \
    "adds  r8, r8, r8 \n\t" \
    "adcs  r9, r9, r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "adcs  lr, lr, lr \n\t" \
    "adcs  r2, r2, r2 \n\t" \
    "movs  r7, #0 \n\t" \
    "adc   r7, r7, #0 \n\t" \
    "umaal r8, r1, r3, r3 \n\t" \
    "adds  r9, r9, r1 \n\t" \
    "umull r1, r3, r4, r4 \n\t" \
    "adcs  r10, r10, r1 \n\t" \
    "adcs  r11, r11, r3 \n\t" \
    "umull r1, r3, r5, r5 \n\t" \
    "adcs  r12, r12, r1 \n\t" \
    "adcs  lr, lr, r3 \n\t" \
    "umull r1, r3, r6, r6 \n\t" \
    "adcs  r2, r2, r1 \n\t" \
    "adc   r3, r3, r7 \n\t" \
    "stmia r0!, {r8, r9, r10, r11, r12, lr} \n\t" \
    "stmia r0!, {r2, r3} \n\t"

#define FAST_SQUARE_ASM_6 \
    "ldmia r1, {r2, r3, r4, r5, r6, r7} \n\t" \
    "umull r9, r10, r2, r3 \n\t" \
    "umull r11, r12, r2, r2 \n\t" \
    "movs  r1, #0 \n\t" \
    "adds  r9, r9, r9 \n\t" \
    "adc   r1, r1, #0 \n\t" \
    "adds  r12, r12, r9 \n\t" \
    "adc   r1, r1, #0 \n\t" \
    "stmia r0!, {r11, r12} \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r10, r11, r2, r4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r11, r8, r3, r4 \n\t" \
    "movs  r9, #0 \n\t" \
    "umaal r11, r9, r2, r5 \n\t" \
    "adds  r10, r10, r10 \n\t" \
    "umaal r10, r1, r3, r3 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "movs  r12, #0 \n\t" \
    "adc   r12, r12, #0 \n\t" \
    "adds  r11, r11, r1 \n\t" \
    "adc   r1, r12, #0 \n\t" \
    "stmia r0!, {r10, r11} \n\t" \
    "umaal r8, r9, r3, r5 \n\t" \
    "movs  r10, #0 \n\t" \
    "umaal r9, r10, r4, r5 \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r8,  r11, r2, r6 \n\t" \
    "umaal r9,  r11, r3, r6 \n\t" \
    "umaal r10, r11, r4, r6 \n\t" \
    "movs  r12, #0 \n\t" \
    "umaal r11, r12, r5, r6 \n\t" \
    "movs  lr, #0 \n\t" \
    "umaal r9,  lr, r2, r7 \n\t" \
    "umaal r10, lr, r3, r7 \n\t" \
    "umaal r11, lr, r4, r7 \n\t" \
    "umaal r12, lr, r5, r7 \n\t" \
    "movs  r2, #0 \n\t" \
    "umaal lr,  r2, r6, r7 \n\t" \
    "adds  r8,  r8,  r8 \n\t" \
    "adcs  r9,  r9,  r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "adcs  lr,  lr,  lr \n\t" \
    "adcs  r2,  r2,  r2 \n\t" \
    "movs  r3, #0 \n\t" \
    "adc   r3,  r3, #0 \n\t" \
    "umaal r8,  r1, r4, r4 \n\t" \
    "adds  r9,  r9, r1 \n\t" \
    "umull r1,  r4, r5, r5 \n\t" \
    "adcs  r10, r10, r1 \n\t" \
    "adcs  r11, r11, r4 \n\t" \
    "umull r1,  r4, r6, r6 \n\t" \
    "adcs  r12, r12, r1 \n\t" \
    "adcs  lr,  lr,  r4 \n\t" \
    "umull r1,  r4, r7, r7 \n\t" \
    "adcs  r2,  r2,  r1 \n\t" \
    "adc   r3,  r3,  r4 \n\t" \
    "stmia r0!, {r8, r9, r10, r11, r12, lr} \n\t" \
    "stmia r0!, {r2, r3} \n\t"

#define FAST_SQUARE_ASM_7 \
    "ldmia r1, {r2, r3, r4, r5, r6, r7, r8} \n\t" \
    "umull r9, r10, r2, r3 \n\t" \
    "umull r11, r12, r2, r2 \n\t" \
    "movs  r1, #0 \n\t" \
    "adds  r9, r9, r9 \n\t" \
    "adc   r1, r1, #0 \n\t" \
    "adds  r12, r12, r9 \n\t" \
    "adc   r1, r1, #0 \n\t" \
    "stmia r0!, {r11, r12} \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r10, r11, r2, r4 \n\t" \
    "movs  r12, #0 \n\t" \
    "umaal r11, r12, r3, r4 \n\t" \
    "movs  lr, #0 \n\t" \
    "umaal r11, lr, r2, r5 \n\t" \
    "adds  r10, r10, r10 \n\t" \
    "umaal r10, r1, r3, r3 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "movs  r9, #0 \n\t" \
    "adc   r9, r9, #0 \n\t" \
    "adds  r11, r11, r1 \n\t" \
    "adc   r1, r9, #0 \n\t" \
    "stmia r0!, {r10, r11} \n\t" \
    "umaal r12, lr, r3, r5 \n\t" \
    "movs  r9, #0 \n\t" \
    "umaal lr, r9, r4, r5 \n\t" \
    "movs  r10, #0 \n\t" \
    "umaal r12, r10, r2, r6 \n\t" \
    "umaal lr,  r10, r2, r7 \n\t" \
    "umaal r9,  r10, r2, r8 \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal lr,  r11, r3, r6 \n\t" \
    "umaal r9,  r11, r3, r7 \n\t" \
    "umaal r10, r11, r3, r8 \n\t" \
    "adds  r12, r12, r12 \n\t" \
    "umaal r12, r1, r4, r4 \n\t" \
    "adcs  lr, lr, lr \n\t" \
    "movs  r2, #0 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  lr, lr, r1 \n\t" \
    "adc   r1, r2, #0 \n\t" \
    "stmia r0!, {r12, lr} \n\t" \
    "movs  r12, #0 \n\t" \
    "umaal r9,  r12, r4, r6 \n\t" \
    "umaal r10, r12, r4, r7 \n\t" \
    "umaal r11, r12, r4, r8 \n\t" \
    "movs  lr, #0 \n\t" \
    "umaal r10, lr, r5, r6 \n\t" \
    "umaal r11, lr, r5, r7 \n\t" \
    "umaal r12, lr, r5, r8 \n\t" \
    "movs  r2, #0 \n\t" \
    "umaal r12, r2, r6, r7 \n\t" \
    "umaal lr,  r2, r6, r8 \n\t" \
    "movs  r3, #0 \n\t" \
    "umaal r2,  r3, r7, r8 \n\t" \
    "adds  r9,  r9,  r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "adcs  lr,  lr,  lr \n\t" \
    "adcs  r2,  r2,  r2 \n\t" \
    "adcs  r3,  r3,  r3 \n\t" \
    "movs  r4, #0 \n\t" \
    "adc   r4,  r4, #0 \n\t" \
    "umaal r9,  r1, r5, r5 \n\t" \
    "adds  r10, r10, r1 \n\t" \
    "umull r1,  r5, r6, r6 \n\t" \
    "adcs  r11, r11, r1 \n\t" \
    "adcs  r12, r12, r5 \n\t" \
    "umull r1,  r5, r7, r7 \n\t" \
    "adcs  lr,  lr,  r1 \n\t" \
    "adcs  r2,  r2,  r5 \n\t" \
    "umull r1,  r5, r8, r8 \n\t" \
    "adcs  r3,  r3,  r1 \n\t" \
    "adc   r4,  r4,  r5 \n\t" \
    "stmia r0!, {r9, r10, r11, r12, lr} \n\t" \
    "stmia r0!, {r2, r3, r4} \n\t"

#define FAST_SQUARE_ASM_8 \
    "ldmia r1!, {r2, r3, r4, r5, r6, r7} \n\t" \
    "umull r9, r10, r2, r3 \n\t" \
    "umull r11, r12, r2, r2 \n\t" \
    "movs  r8, #0 \n\t" \
    "adds  r9, r9, r9 \n\t" \
    "adc   r8, r8, #0 \n\t" \
    "adds  r12, r12, r9 \n\t" \
    "adc   r8, r8, #0 \n\t" \
    "stmia r0!, {r11, r12} \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r10, r11, r2, r4 \n\t" \
    "movs  r9, #0 \n\t" \
    "umaal r11, r9, r3, r4 \n\t" \
    "movs  r12, #0 \n\t" \
    "umaal r11, r12, r2, r5 \n\t" \
    "adds  r10, r10, r10 \n\t" \
    "umaal r10, r8, r3, r3 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "movs  lr, #0 \n\t" \
    "adc   lr, lr, #0 \n\t" \
    "adds  r11, r11, r8 \n\t" \
    "adc   r8, lr, #0 \n\t" \
    "stmia r0!, {r10, r11} \n\t" \
    "umaal r9, r12, r3, r5 \n\t" \
    "movs  r10, #0 \n\t" \
    "umaal r10, r12, r4, r5 \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r9,  r11, r2, r6 \n\t" \
    "umaal r10, r11, r3, r6 \n\t" \
    "umaal r11, r12, r4, r6 \n\t" \
    "movs  lr, #0 \n\t" \
    "umaal r12, lr, r5, r6 \n\t" \
    "adds  r9,  r9,  r9 \n\t" \
    "umaal r9,  r8,  r4, r4 \n\t" \
    "str   r9,  [r0], #4 \n\t" \
    "mov   r9,  #0 \n\t" \
    "umaal r10, r9,  r2, r7 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "push  {r1} \n\t" \
    "movs  r1,  #0 \n\t" \
    "adc   r1,  r1,  #0 \n\t" \
    "adds  r10, r10, r8 \n\t" \
    "adc   r8,  r1,  #0 \n\t" \
    "str   r10, [r0], #4 \n\t" \
    "pop   {r1} \n\t" \
    "ldmia r1,  {r1, r10} \n\t" \
    "umaal r11, r9,  r2, r1 \n\t" \
    "umaal r12, r9,  r2, r10 \n\t" \
    "movs  r2,  #0 \n\t" \
    "umaal r11, r2,  r3, r7 \n\t" \
    "umaal r12, r2,  r3, r1 \n\t" \
    "umaal lr,  r2,  r3, r10 \n\t" \
    "movs  r3,  #0 \n\t" \
    "umaal r12, r3,  r4, r7 \n\t" \
    "umaal lr,  r9,  r4, r1 \n\t" \
    "umaal r2,  r9,  r4, r10 \n\t" \
    "adds  r11, r11, r11 \n\t" \
    "umaal r11, r8,  r5, r5 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "movs  r4,  #0 \n\t" \
    "adc   r4,  r4,  #0 \n\t" \
    "adds  r12, r12, r8 \n\t" \
    "adc   r8,  r4,  #0 \n\t" \
    "stmia r0!, {r11, r12} \n\t" \
    "umaal lr,  r3,  r5, r7 \n\t" \
    "umaal r2,  r3,  r5, r1 \n\t" \
    "umaal r9,  r3,  r5, r10 \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r2,  r11, r6, r7 \n\t" \
    "umaal r9,  r11, r6, r1 \n\t" \
    "umaal r3,  r11, r6, r10 \n\t" \
    "movs  r12, #0 \n\t" \
    "umaal r3,  r12, r7, r1 \n\t" \
    "umaal r11, r12, r7, r10 \n\t" \
    "movs  r4,  #0 \n\t" \
    "umaal r12, r4,  r1, r10 \n\t" \
    "adds  lr,  lr,  lr \n\t" \
    "adcs  r2,  r2,  r2 \n\t" \
    "adcs  r9,  r9,  r9 \n\t" \
    "adcs  r3,  r3,  r3 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "adcs  r4,  r4,  r4 \n\t" \
    "movs  r5,  #0 \n\t" \
    "adc   r5,  r5,  #0 \n\t" \
    "umaal lr,  r8,  r6, r6 \n\t" \
    "adds  r2,  r2,  r8 \n\t" \
    "umull r6,  r8,  r7, r7 \n\t" \
    "adcs  r9,  r9,  r6 \n\t" \
    "adcs  r3,  r3,  r8 \n\t" \
    "umull r6,  r8,  r1, r1 \n\t" \
    "adcs  r11, r11, r6 \n\t" \
    "adcs  r12, r12, r8 \n\t" \
    "umull r6,  r8,  r10, r10 \n\t" \
    "adcs  r4,  r4,  r6 \n\t" \
    "adc   r5,  r5,  r8 \n\t" \
    "str   lr,  [r0], #4 \n\t" \
    "stmia r0!, {r2, r9} \n\t" \
    "stmia r0!, {r3, r11, r12} \n\t" \
    "stmia r0!, {r4, r5} \n\t"

/* ---- Level 3 Shared-Phase Squaring Macro ---- */
#define FAST_SQUARE_ASM_LEVEL3 \
    "mov   r7, r2 \n\t" \
    "ldmia r1!, {r2, r3, r4, r5, r6} \n\t" \
    "umull r9, r10, r2, r3 \n\t" \
    "umull r11, r12, r2, r2 \n\t" \
    "movs  r8, #0 \n\t" \
    "adds  r9, r9, r9 \n\t" \
    "adc   r8, r8, #0 \n\t" \
    "adds  r12, r12, r9 \n\t" \
    "adc   r8, r8, #0 \n\t" \
    "stmia r0!, {r11, r12} \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r10, r11, r2, r4 \n\t" \
    "movs  r9, #0 \n\t" \
    "umaal r11, r9, r3, r4 \n\t" \
    "movs  r12, #0 \n\t" \
    "umaal r11, r12, r2, r5 \n\t" \
    "adds  r10, r10, r10 \n\t" \
    "umaal r10, r8, r3, r3 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "movs  lr, #0 \n\t" \
    "adc   lr, lr, #0 \n\t" \
    "adds  r11, r11, r8 \n\t" \
    "adc   r8, lr, #0 \n\t" \
    "stmia r0!, {r10, r11} \n\t" \
    "umaal r9, r12, r3, r5 \n\t" \
    "movs  r10, #0 \n\t" \
    "umaal r10, r12, r4, r5 \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r9,  r11, r2, r6 \n\t" \
    "umaal r10, r11, r3, r6 \n\t" \
    "umaal r11, r12, r4, r6 \n\t" \
    "movs  lr, #0 \n\t" \
    "umaal r12, lr, r5, r6 \n\t" \
    "cmp   r7, #5 \n\t" \
    "bne   60f \n\t" \
    "adds  r9,  r9,  r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "adcs  lr,  lr,  lr \n\t" \
    "movs  r2, #0 \n\t" \
    "adc   r2,  r2, #0 \n\t" \
    "umaal r9,  r8, r4, r4 \n\t" \
    "adds  r10, r10, r8 \n\t" \
    "umull r1,  r3, r5, r5 \n\t" \
    "adcs  r11, r11, r1 \n\t" \
    "adcs  r12, r12, r3 \n\t" \
    "umull r1,  r3, r6, r6 \n\t" \
    "adcs  lr,  lr,  r1 \n\t" \
    "adc   r3,  r2,  r3 \n\t" \
    "stmia r0!, {r9, r10, r11, r12, lr} \n\t" \
    "str   r3, [r0] \n\t" \
    "b     1f \n\t" \
    "60: \n\t" \
    "cmp   r7, #6 \n\t" \
    "bne   70f \n\t" \
    "ldr   r7, [r1] \n\t" \
    "movs  r1, #0 \n\t" \
    "umaal r10, r1, r2, r7 \n\t" \
    "umaal r11, r1, r3, r7 \n\t" \
    "umaal r12, r1, r4, r7 \n\t" \
    "umaal lr,  r1, r5, r7 \n\t" \
    "movs  r2, #0 \n\t" \
    "umaal r1,  r2, r6, r7 \n\t" \
    "adds  r9,  r9,  r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "adcs  lr,  lr,  lr \n\t" \
    "adcs  r1,  r1,  r1 \n\t" \
    "adcs  r2,  r2,  r2 \n\t" \
    "movs  r3, #0 \n\t" \
    "adc   r3,  r3, #0 \n\t" \
    "umaal r9,  r8, r4, r4 \n\t" \
    "adds  r10, r10, r8 \n\t" \
    "umull r4,  r8, r5, r5 \n\t" \
    "adcs  r11, r11, r4 \n\t" \
    "adcs  r12, r12, r8 \n\t" \
    "umull r4,  r8, r6, r6 \n\t" \
    "adcs  lr,  lr,  r4 \n\t" \
    "adcs  r1,  r1,  r8 \n\t" \
    "umull r4,  r8, r7, r7 \n\t" \
    "adcs  r2,  r2,  r4 \n\t" \
    "adc   r3,  r3,  r8 \n\t" \
    "stmia r0!, {r9, r10, r11, r12, lr} \n\t" \
    "stmia r0!, {r1, r2, r3} \n\t" \
    "b     1f \n\t" \
    "70: \n\t" \
    "cmp   r7, #7 \n\t" \
    "bne   80f \n\t" \
    "adds  r9,  r9,  r9 \n\t" \
    "umaal r9,  r8, r4, r4 \n\t" \
    "str   r9, [r0], #4 \n\t" \
    "ldmia r1, {r1, r7} \n\t" \
    "mov   r9, #0 \n\t" \
    "umaal r10, r9, r2, r1 \n\t" \
    "umaal r11, r9, r2, r7 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "movs  r2, #0 \n\t" \
    "adc   r2,  r2, #0 \n\t" \
    "adds  r10, r10, r8 \n\t" \
    "adc   r8,  r2, #0 \n\t" \
    "str   r10, [r0], #4 \n\t" \
    "movs  r10, #0 \n\t" \
    "umaal r11, r10, r3, r1 \n\t" \
    "umaal r12, r9,  r4, r1 \n\t" \
    "umaal r12, r10, r3, r7 \n\t" \
    "umaal lr,  r9,  r5, r1 \n\t" \
    "umaal lr,  r10, r4, r7 \n\t" \
    "umaal r9,  r10, r6, r1 \n\t" \
    "movs  r2, #0 \n\t" \
    "umaal r9,  r2,  r5, r7 \n\t" \
    "umaal r10, r2,  r6, r7 \n\t" \
    "movs  r3, #0 \n\t" \
    "umaal r2,  r3,  r1, r7 \n\t" \
    "adds  r11, r11, r11 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "adcs  lr,  lr,  lr \n\t" \
    "adcs  r9,  r9,  r9 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "adcs  r2,  r2,  r2 \n\t" \
    "adcs  r3,  r3,  r3 \n\t" \
    "movs  r4, #0 \n\t" \
    "adc   r4,  r4, #0 \n\t" \
    "umaal r11, r8, r5, r5 \n\t" \
    "adds  r12, r12, r8 \n\t" \
    "umull r5,  r8, r6, r6 \n\t" \
    "adcs  lr,  lr,  r5 \n\t" \
    "adcs  r9,  r9,  r8 \n\t" \
    "umull r5,  r8, r1, r1 \n\t" \
    "adcs  r10, r10, r5 \n\t" \
    "adcs  r2,  r2,  r8 \n\t" \
    "umull r5,  r8, r7, r7 \n\t" \
    "adcs  r3,  r3,  r5 \n\t" \
    "adc   r4,  r4,  r8 \n\t" \
    "stmia r0!, {r11, r12, lr} \n\t" \
    "stmia r0!, {r9, r10} \n\t" \
    "stmia r0!, {r2, r3, r4} \n\t" \
    "b     1f \n\t" \
    "80: \n\t" \
    "ldr   r7, [r1], #4 \n\t" \
    "adds  r9,  r9,  r9 \n\t" \
    "umaal r9,  r8,  r4, r4 \n\t" \
    "str   r9,  [r0], #4 \n\t" \
    "mov   r9,  #0 \n\t" \
    "umaal r10, r9,  r2, r7 \n\t" \
    "adcs  r10, r10, r10 \n\t" \
    "push  {r1} \n\t" \
    "movs  r1,  #0 \n\t" \
    "adc   r1,  r1,  #0 \n\t" \
    "adds  r10, r10, r8 \n\t" \
    "adc   r8,  r1,  #0 \n\t" \
    "str   r10, [r0], #4 \n\t" \
    "pop   {r1} \n\t" \
    "ldmia r1,  {r1, r10} \n\t" \
    "umaal r11, r9,  r2, r1 \n\t" \
    "umaal r12, r9,  r2, r10 \n\t" \
    "movs  r2,  #0 \n\t" \
    "umaal r11, r2,  r3, r7 \n\t" \
    "umaal r12, r2,  r3, r1 \n\t" \
    "umaal lr,  r2,  r3, r10 \n\t" \
    "movs  r3,  #0 \n\t" \
    "umaal r12, r3,  r4, r7 \n\t" \
    "umaal lr,  r9,  r4, r1 \n\t" \
    "umaal r2,  r9,  r4, r10 \n\t" \
    "adds  r11, r11, r11 \n\t" \
    "umaal r11, r8,  r5, r5 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "movs  r4,  #0 \n\t" \
    "adc   r4,  r4,  #0 \n\t" \
    "adds  r12, r12, r8 \n\t" \
    "adc   r8,  r4,  #0 \n\t" \
    "stmia r0!, {r11, r12} \n\t" \
    "umaal lr,  r3,  r5, r7 \n\t" \
    "umaal r2,  r3,  r5, r1 \n\t" \
    "umaal r9,  r3,  r5, r10 \n\t" \
    "movs  r11, #0 \n\t" \
    "umaal r2,  r11, r6, r7 \n\t" \
    "umaal r9,  r11, r6, r1 \n\t" \
    "umaal r3,  r11, r6, r10 \n\t" \
    "movs  r12, #0 \n\t" \
    "umaal r3,  r12, r7, r1 \n\t" \
    "umaal r11, r12, r7, r10 \n\t" \
    "movs  r4,  #0 \n\t" \
    "umaal r12, r4,  r1, r10 \n\t" \
    "adds  lr,  lr,  lr \n\t" \
    "adcs  r2,  r2,  r2 \n\t" \
    "adcs  r9,  r9,  r9 \n\t" \
    "adcs  r3,  r3,  r3 \n\t" \
    "adcs  r11, r11, r11 \n\t" \
    "adcs  r12, r12, r12 \n\t" \
    "adcs  r4,  r4,  r4 \n\t" \
    "movs  r5,  #0 \n\t" \
    "adc   r5,  r5,  #0 \n\t" \
    "umaal lr,  r8,  r6, r6 \n\t" \
    "adds  r2,  r2,  r8 \n\t" \
    "umull r6,  r8,  r7, r7 \n\t" \
    "adcs  r9,  r9,  r6 \n\t" \
    "adcs  r3,  r3,  r8 \n\t" \
    "umull r6,  r8,  r1, r1 \n\t" \
    "adcs  r11, r11, r6 \n\t" \
    "adcs  r12, r12, r8 \n\t" \
    "umull r6,  r8,  r10, r10 \n\t" \
    "adcs  r4,  r4,  r6 \n\t" \
    "adc   r5,  r5,  r8 \n\t" \
    "str   lr,  [r0], #4 \n\t" \
    "stmia r0!, {r2, r9} \n\t" \
    "stmia r0!, {r3, r11, r12} \n\t" \
    "stmia r0!, {r4, r5} \n\t"

#endif /* _UECC_ASM_ARM_MULT_SQUARE_UMAAL_H_ */
