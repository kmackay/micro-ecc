/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */

#ifndef _UECC_ASM_ARM_MULT_SQUARE_UMAAL_H_
#define _UECC_ASM_ARM_MULT_SQUARE_UMAAL_H_

/* ============================================================================
 * Thumb-2 / ARM with UMAAL (Cortex-M4 / M7 / M33 / ARMv7E-M)
 *
 * Multiplication uses row-wise (operand scanning) UMAAL accumulation:
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
 *
 * In each multiplication row the umaal chain leaves the column window in the
 * same registers, so no register renaming is needed between rows.
 * ============================================================================ */

/* ---- Level 4 Standalone Multiplication Macros ---- */

/* nw = 5: one pass with b[0..4] in r3-r7. Each row a[i] stores its lowest column
   and keeps the next 5 in r9-r12, lr (zero intermediate memory spills). */
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

/* nw = 6: pass 1 with b[0..3] (r4-r7) keeps columns 4..9 in r8-r12, r3 and a[5]
   in lr; pass 2 adds a * b[4..5] with a 2-word window (zero intermediate spills). */
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
    /* Row a[4]: column 4 stays in r8. */ \
    "ldr   r3, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r4, r3 \n\t" \
    "umaal r9, r10, r5, r3 \n\t" \
    "umaal r10, r11, r6, r3 \n\t" \
    "umaal r11, r12, r7, r3 \n\t" \
    /* Row a[5] (kept in lr for pass 2); r1 rewinds to a[0]. Column 9 goes in r3. */ \
    "ldr   lr, [r1], #-20 \n\t" \
    "movs  r3, #0 \n\t" \
    "umaal r9, r3, r4, lr \n\t" \
    "umaal r10, r3, r5, lr \n\t" \
    "umaal r11, r3, r6, lr \n\t" \
    "umaal r12, r3, r7, lr \n\t" \
    /* Pass 2: b4, b5 in r4, r5; pass-1 columns 4..9 are folded in as umaal addends. */ \
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
    /* Pass 2 row a[5] (still in lr); store columns 9..11. */ \
    "umaal r3, r7, r4, lr \n\t" \
    "umaal r6, r7, r5, lr \n\t" \
    "stmia r0!, {r3, r6, r7} \n\t"

/* nw = 7: pass 1 with b[0..4] (r3-r7), then pass 2 with b[5..6]. Only column 5
   is stored and reloaded. */
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
    /* Row a[5]: store column 5 without advancing r0 (it is reloaded in pass 2). */ \
    "ldr   r2, [r1], #4 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r8, r9, r3, r2 \n\t" \
    "str   r8, [r0] \n\t" \
    "umaal r9, r10, r4, r2 \n\t" \
    "umaal r10, r11, r5, r2 \n\t" \
    "umaal r11, r12, r6, r2 \n\t" \
    "umaal r12, lr, r7, r2 \n\t" \
    /* Row a[6]; r1 rewinds to a[0]. Columns 6..11 are in r9-r12, lr, r8. */ \
    "ldr   r2, [r1], #-24 \n\t" \
    "movs  r8, #0 \n\t" \
    "umaal r9, r8, r3, r2 \n\t" \
    "umaal r10, r8, r4, r2 \n\t" \
    "umaal r11, r8, r5, r2 \n\t" \
    "umaal r12, r8, r6, r2 \n\t" \
    "umaal lr, r8, r7, r2 \n\t" \
    /* Pass 2: b5, b6 in r4, r5; column 5 is reloaded, columns 6..11 are umaal addends. */ \
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

/* nw = 8: pass 1 with b[0..4] (r3-r7), pass 2 with b[5..7] (r5-r7). Pass-1
   columns 5..8 are stored and reloaded; columns 9..12 stay in r10-r12, lr and
   are folded in as the second umaal addend (dual-addend UMAAL). */
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
    /* Rows a[5..7] store columns 5..8 without advancing r0; columns 9..12 stay in r10-r12, lr. */ \
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
    "umaal r12, lr, r7, r2 \n\t" \
    /* Pass 2: b5..b7 in r5-r7; reload columns 5..7 (column 8 is loaded in row a[3]). */ \
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
/* Level 3: a single macro for all sizes (num_words in r3). nw = 6 uses
   FAST_MULT_ASM_6 (label 60); nw = 5, 7, 8 share the 5-column pass 1 for rows
   a[0..4], then branch. Every path ends at local label 1, which the caller
   defines. */
#define FAST_MULT_ASM_LEVEL3 \
    "cmp   r3, #6 \n\t" \
    "beq   60f \n\t" \
    /* Rows a[0..4] with b[0..4]; right and num_words are saved on the stack. */ \
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
    /* nw = 5: store the top 5 columns. */ \
    "ldr   r8, [sp, #4] \n\t" \
    "cmp   r8, #5 \n\t" \
    "bne   70f \n\t" \
    "stmia r0!, {r9, r10, r11, r12, lr} \n\t" \
    "add   sp, sp, #8 \n\t" \
    "b     1f \n\t" \
    "70: \n\t" \
    /* nw = 7, 8: row a[5]. */ \
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
    /* nw = 8: continue as FAST_MULT_ASM_8 from row a[6]. */ \
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
    /* nw = 7: continue as FAST_MULT_ASM_7 from row a[6]. */ \
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

/* nw = 5: off-diagonal products via umaal, finalizing columns 0, 1 early; then
   double the rest and add the diagonal squares. umaal does not modify the flags,
   so it can be interleaved inside adds/adcs chains. */
#define FAST_SQUARE_ASM_5 \
    "ldmia r1, {r2, r3, r4, r5, r6} \n\t" \
    /* Off-diagonal products; columns 1..8 accumulate in r7..r12, lr, r2. */ \
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
    /* Columns 0, 1 = a0^2 + 2 * a0 * a1; carry (0..2) in r1. */ \
    "umull r2, r1, r2, r2 \n\t" \
    "str   r2, [r0], #4 \n\t" \
    "movs  r2, #0 \n\t" \
    "adds  r7, r7, r7 \n\t" \
    "adc   r2, r2, #0 \n\t" \
    "adds  r7, r7, r1 \n\t" \
    "str   r7, [r0], #4 \n\t" \
    "adc   r1, r2, #0 \n\t" \
    /* Remaining products with a[4]. */ \
    "umaal r11, lr, r3, r6 \n\t" \
    "umaal r12, lr, r4, r6 \n\t" \
    "movs  r2, #0 \n\t" \
    "umaal lr, r2, r5, r6 \n\t" \
    /* Double columns 2..8 (top carry in r7), then add a[1..4]^2; umaal also adds the r1 carry. */ \
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

/* nw = 6: like FAST_SQUARE_ASM_5, finalizing columns 0..3 two at a time to free
   registers. */
#define FAST_SQUARE_ASM_6 \
    "ldmia r1, {r2, r3, r4, r5, r6, r7} \n\t" \
    /* Columns 0, 1 = a0^2 + 2 * a0 * a1; carry in r1. */ \
    "umull r9, r10, r2, r3 \n\t" \
    "umull r11, r12, r2, r2 \n\t" \
    "movs  r1, #0 \n\t" \
    "adds  r9, r9, r9 \n\t" \
    "adc   r1, r1, #0 \n\t" \
    "adds  r12, r12, r9 \n\t" \
    "adc   r1, r1, #0 \n\t" \
    "stmia r0!, {r11, r12} \n\t" \
    /* Columns 2, 3: doubled, with umaal adding a1^2 + carry inside the adcs chain. */ \
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
    /* Remaining off-diagonal products (columns 4..11 in r8..r12, lr, r2). */ \
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
    /* Double columns 4..10 (top carry in r3), then add a[2..5]^2. */ \
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

/* nw = 7: like FAST_SQUARE_ASM_6, finalizing columns 0..5 two at a time. */
#define FAST_SQUARE_ASM_7 \
    "ldmia r1, {r2, r3, r4, r5, r6, r7, r8} \n\t" \
    /* Columns 0, 1 = a0^2 + 2 * a0 * a1; carry in r1. */ \
    "umull r9, r10, r2, r3 \n\t" \
    "umull r11, r12, r2, r2 \n\t" \
    "movs  r1, #0 \n\t" \
    "adds  r9, r9, r9 \n\t" \
    "adc   r1, r1, #0 \n\t" \
    "adds  r12, r12, r9 \n\t" \
    "adc   r1, r1, #0 \n\t" \
    "stmia r0!, {r11, r12} \n\t" \
    /* Columns 2, 3: doubled, with umaal adding a1^2 + carry inside the adcs chain. */ \
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
    /* Products for columns 4..8; finalize columns 4, 5 with a2^2 + carry. */ \
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
    /* Remaining off-diagonal products (columns 6..13 in r9..r12, lr, r2, r3). */ \
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
    /* Double columns 6..12 (top carry in r4), then add a[3..6]^2. */ \
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

/* nw = 8: like FAST_SQUARE_ASM_7, finalizing columns 0..7 two at a time. a[6]
   and a[7] are loaded into r1 and r10 once the pointer is no longer needed. */
#define FAST_SQUARE_ASM_8 \
    "ldmia r1!, {r2, r3, r4, r5, r6, r7} \n\t" \
    /* Columns 0, 1 = a0^2 + 2 * a0 * a1; carry in r8. */ \
    "umull r9, r10, r2, r3 \n\t" \
    "umull r11, r12, r2, r2 \n\t" \
    "movs  r8, #0 \n\t" \
    "adds  r9, r9, r9 \n\t" \
    "adc   r8, r8, #0 \n\t" \
    "adds  r12, r12, r9 \n\t" \
    "adc   r8, r8, #0 \n\t" \
    "stmia r0!, {r11, r12} \n\t" \
    /* Columns 2, 3: doubled, with umaal adding a1^2 + carry inside the adcs chain. */ \
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
    /* Columns 4, 5 (r1 is pushed to free it for the doubling carry). */ \
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
    /* Load a6, a7 into r1, r10; columns 6, 7. */ \
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
    /* Remaining off-diagonal products (columns 8..15 in lr, r2, r9, r3, r11, r12, r4). */ \
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
    /* Double columns 8..14 (top carry in r5), then add a[4..7]^2. */ \
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
/* Level 3: a single macro for all sizes (num_words in r2, moved to r7). Shares
   columns 0..3 and the a[0..4] products, then branches by size. Every path ends
   at local label 1, which the caller defines. */
#define FAST_SQUARE_ASM_LEVEL3 \
    "mov   r7, r2 \n\t" \
    "ldmia r1!, {r2, r3, r4, r5, r6} \n\t" \
    /* Columns 0..3 and the products among a[0..4] (shared by all sizes). */ \
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
    /* nw = 5: double columns 4..8 and add a[2..4]^2. */ \
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
    /* nw = 6: add row a[5] * a[0..4], double columns 4..10 and add a[2..5]^2. */ \
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
    /* nw = 7: finalize columns 4, 5, add the a[5], a[6] products, then double */ \
    /* and add a[3..6]^2. */ \
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
    /* nw = 8: continue as FAST_SQUARE_ASM_8 from column 4. */ \
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
