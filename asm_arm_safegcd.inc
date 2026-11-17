/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */

/* Thumb-2 / ARM matrix updates for the safegcd modular inversion (see
   uECC_vli_modInv in uECC.c). Naked functions so that every register (including
   r7 / r11) can be used without disturbing a frame pointer.

   Layout: de = { d[9], e[9], mod[9] } and fg = { f[9], g[9] } (signed 30-bit
   limbs); t = { u, v, q, r }. */

#ifndef _UECC_ASM_ARM_SAFEGCD_H_
#define _UECC_ASM_ARM_SAFEGCD_H_

/* Arithmetic shift of the signed 64-bit value hi:lo right by 30. */
#define S30_SHR30(lo, hi) \
    "lsr " lo ", " lo ", #30 \n\t" \
    "orr " lo ", " lo ", " hi ", lsl #2 \n\t" \
    "asr " hi ", " hi ", #30 \n\t"

/* (d, e) = t * (d, e) / 2^30 mod modulus (see s30_update_de). */
__attribute__((naked, noinline))
static void s30_update_de_asm(int32_t *de, const int32_t *t, uint32_t mod_inv30,
                              uint32_t len) {
    __asm__ volatile (
        ".syntax unified \n\t"
        "push {r4-r11, lr} \n\t"
        /* Compute md / me from the signs of d and e. */
        "ldm r1, {r4-r7} \n\t"                 /* u, v, q, r */
        "add r3, r0, r3, lsl #2 \n\t"          /* r3 = &d[len] */
        "ldr r8, [r3, #-4] \n\t"               /* d[len - 1] */
        "ldr r9, [r3, #32] \n\t"               /* e[len - 1] */
        "asr r8, r8, #31 \n\t"                 /* sd */
        "asr r9, r9, #31 \n\t"                 /* se */
        "and r10, r4, r8 \n\t"
        "and r11, r5, r9 \n\t"
        "add r10, r10, r11 \n\t"               /* md = (u & sd) + (v & se) */
        "and r8, r6, r8 \n\t"
        "and r9, r7, r9 \n\t"
        "add r11, r8, r9 \n\t"                 /* me = (q & sd) + (r & se) */
        /* Limb 0: cd / ce, then adjust md / me so that the low 30 bits of
           cd + md * mod0 and ce + me * mod0 are zero. */
        "ldr r12, [r0] \n\t"                   /* d0 */
        "smull r8, r9, r4, r12 \n\t"           /* cd = u * d0 */
        "smull r1, lr, r6, r12 \n\t"           /* ce = q * d0 */
        "ldr r12, [r0, #36] \n\t"              /* e0 */
        "smlal r8, r9, r5, r12 \n\t"           /* cd += v * e0 */
        "smlal r1, lr, r7, r12 \n\t"           /* ce += r * e0 */
        "mla r12, r2, r8, r10 \n\t"
        "bic r12, r12, #0xC0000000 \n\t"
        "sub r10, r10, r12 \n\t"               /* md -= (inv * cd + md) & M30 */
        "mla r12, r2, r1, r11 \n\t"
        "bic r12, r12, #0xC0000000 \n\t"
        "sub r11, r11, r12 \n\t"               /* me -= (inv * ce + me) & M30 */
        "ldr r12, [r0, #72] \n\t"              /* mod0 */
        "smlal r8, r9, r10, r12 \n\t"          /* cd += md * mod0 */
        "smlal r1, lr, r11, r12 \n\t"          /* ce += me * mod0 */
        S30_SHR30("r8", "r9")
        S30_SHR30("r1", "lr")
        "sub r2, r3, #4 \n\t"                  /* r2 = &d[len - 1] */
        /* Limbs 1..len-1: accumulate, store the low 30 bits one limb down,
           and shift the carries. r0 points at d[i - 1]. */
        "1: \n\t"
        "ldr r12, [r0, #4] \n\t"               /* d[i] */
        "smlal r8, r9, r4, r12 \n\t"
        "smlal r1, lr, r6, r12 \n\t"
        "ldr r12, [r0, #40] \n\t"              /* e[i] */
        "smlal r8, r9, r5, r12 \n\t"
        "smlal r1, lr, r7, r12 \n\t"
        "ldr r12, [r0, #76] \n\t"              /* mod[i] */
        "smlal r8, r9, r10, r12 \n\t"
        "smlal r1, lr, r11, r12 \n\t"
        "bic r12, r8, #0xC0000000 \n\t"
        "str r12, [r0] \n\t"                   /* d[i - 1] */
        "bic r12, r1, #0xC0000000 \n\t"
        "str r12, [r0, #36] \n\t"              /* e[i - 1] */
        S30_SHR30("r8", "r9")
        S30_SHR30("r1", "lr")
        "add r0, r0, #4 \n\t"
        "cmp r0, r2 \n\t"
        "bne 1b \n\t"
        /* Top limbs get the remaining signed carries. */
        "str r8, [r0] \n\t"                    /* d[len - 1] */
        "str r1, [r0, #36] \n\t"               /* e[len - 1] */
        "pop {r4-r11, pc} \n\t"
        RESUME_SYNTAX
    );
}

/* (f, g) = t * (f, g) / 2^30 (see s30_update_fg). */
__attribute__((naked, noinline))
static void s30_update_fg_asm(int32_t *fg, const int32_t *t, uint32_t len) {
    __asm__ volatile (
        ".syntax unified \n\t"
        "push {r4-r9, lr} \n\t"
        "ldm r1, {r4-r7} \n\t"                 /* u, v, q, r */
        "add r2, r0, r2, lsl #2 \n\t"
        "sub r2, r2, #4 \n\t"                  /* r2 = &f[len - 1] */
        "ldr r12, [r0] \n\t"                   /* f0 */
        "smull r8, r9, r4, r12 \n\t"           /* cf = u * f0 */
        "smull r1, r3, r6, r12 \n\t"           /* cg = q * f0 */
        "ldr r12, [r0, #36] \n\t"              /* g0 */
        "smlal r8, r9, r5, r12 \n\t"           /* cf += v * g0 */
        "smlal r1, r3, r7, r12 \n\t"           /* cg += r * g0 */
        S30_SHR30("r8", "r9")                  /* Low 30 bits are zero. */
        S30_SHR30("r1", "r3")
        /* Limbs 1..len-1, as in s30_update_de_asm (without the modulus). */
        "1: \n\t"
        "ldr r12, [r0, #4] \n\t"               /* f[i] */
        "smlal r8, r9, r4, r12 \n\t"
        "smlal r1, r3, r6, r12 \n\t"
        "ldr r12, [r0, #40] \n\t"              /* g[i] */
        "smlal r8, r9, r5, r12 \n\t"
        "smlal r1, r3, r7, r12 \n\t"
        "bic r12, r8, #0xC0000000 \n\t"
        "str r12, [r0] \n\t"                   /* f[i - 1] */
        "bic r12, r1, #0xC0000000 \n\t"
        "str r12, [r0, #36] \n\t"              /* g[i - 1] */
        S30_SHR30("r8", "r9")
        S30_SHR30("r1", "r3")
        "add r0, r0, #4 \n\t"
        "cmp r0, r2 \n\t"
        "bne 1b \n\t"
        "str r8, [r0] \n\t"                    /* f[len - 1] */
        "str r1, [r0, #36] \n\t"               /* g[len - 1] */
        "pop {r4-r9, pc} \n\t"
        RESUME_SYNTAX
    );
}

#undef S30_SHR30

#define asm_s30_update 1

#endif /* _UECC_ASM_ARM_SAFEGCD_H_ */
