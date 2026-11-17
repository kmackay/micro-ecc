#!/usr/bin/env python3
"""Generates asm_arm_mmod.inc: constant-time fast reductions for Thumb-2 / ARM."""
import sys

out = []
def emit(s=""):
    out.append(s)

def func(name, comment, body, clobbers):
    """Emits one vli_mmod_fast_<name>() wrapper around the asm body. Body items
    starting with "#" are emitted as preprocessor lines, "/*" as comments, and
    everything else as an instruction string."""
    emit(comment)
    emit(f"static void vli_mmod_fast_{name}(uint32_t *result, uint32_t *product) {{")
    emit('    register uint32_t *r0 __asm__("r0") = result;')
    emit('    register uint32_t *r1 __asm__("r1") = product;')
    emit("    __asm__ volatile (")
    emit('        ".syntax unified \\n\\t"')
    for item in body:
        if isinstance(item, str) and item.startswith("#"):
            emit(item)
        elif isinstance(item, str) and item.startswith("/*"):
            emit("        " + item)
        else:
            emit(f'        "{item} \\n\\t"')
    emit("        RESUME_SYNTAX")
    emit('        : "+r" (r0), "+r" (r1)')
    emit("        :")
    emit("        : " + ", ".join(f'"{c}"' for c in clobbers) + ', "cc", "memory"')
    emit("    );")
    emit("}")
    emit(f"#define asm_mmod_fast_{name} 1")

# ---------------------------------------------------------------------------
# Column accumulation helper (signed column sums with a 2-register carry).
def columns(terms, p_off, L, regs_c, a, res_off=0):
    """terms: list of lists of (coef, idx) for each column (coef in +1,-1,+2).
    L: accumulator low register. regs_c: two registers that alternate as carry.
    a: list of registers holding high words. Returns (lines, final carry reg)."""
    lines = []
    # C is the previous column's (signed) carry; H accumulates this column's.
    C, H = regs_c
    for j, col in enumerate(terms):
        if j == 1:
            lines.append(f"/* Column {j}. The previous carry {C} is signed: add it with its sign\n"
                         f"           extension (asr does not touch the flags, so adc sees the adds carry). */")
        else:
            lines.append(f"/* Column {j}. */")
        lines.append(f"ldr {L}, [r1, #{p_off + 4 * j}]")
        if j == 0:
            lines.append(f"mov {H}, #0")
        else:
            lines.append(f"adds {L}, {L}, {C}")
            lines.append(f"asr {H}, {C}, #31")
            lines.append(f"adc {H}, {H}, #0")
        for coef, i in col:
            if coef == 1:
                lines += [f"adds {L}, {L}, {a[i]}", f"adc {H}, {H}, #0"]
            elif coef == -1:
                lines += [f"subs {L}, {L}, {a[i]}", f"sbc {H}, {H}, #0"]
            elif coef == 2:
                lines += [f"adds {L}, {L}, {a[i]}, lsl #1", f"adc {H}, {H}, {a[i]}, lsr #31"]
            else:
                raise ValueError(coef)
        lines.append(f"str {L}, [r0, #{res_off + 4 * j}]")
        C, H = H, C
    return lines, C

def columns_mla(terms, L, regs_c, a, one, two):
    """Like columns(), but accumulates positive terms with umlal (coefficients
    1, 2 or 3 via registers one/two) and adds the signed carry with smlal.
    Reads product words via r1 and stores the column results back into
    product[0..n-1] (already consumed)."""
    lines = []
    C, H = regs_c
    for j, col in enumerate(terms):
        if j == 1:
            lines.append(f"/* Column {j}. smlal adds the signed previous carry ({C} * 1); umlal by\n"
                         f"           {one} or {two} adds a term once or twice. */")
        else:
            lines.append(f"/* Column {j}. */")
        lines.append(f"ldr {L}, [r1, #{4 * j}]")
        lines.append(f"mov {H}, #0")
        if j > 0:
            lines.append(f"smlal {L}, {H}, {C}, {one}")
        # Merge repeated indices so e.g. 2 a6 + a6 becomes one coefficient 3.
        coefs = {}
        for coef, i in col:
            coefs[i] = coefs.get(i, 0) + coef
        for i, coef in coefs.items():
            if coef > 0:
                rest = coef
                while rest > 0:
                    k = 2 if rest >= 2 else 1
                    lines.append(f"umlal {L}, {H}, {a[i]}, {two if k == 2 else one}")
                    rest -= k
            else:
                assert coef == -1
                lines += [f"subs {L}, {L}, {a[i]}", f"sbc {H}, {H}, #0"]
        lines.append(f"str {L}, [r1, #{4 * j}]")
        C, H = H, C
    return lines, C

# ---------------------------------------------------------------------------
# Generic constant-time finish. fold(c, k): emits r += c * d, carry -> k (k may == c).
def finish(fold, c2, x, t, u, undo=None):
    """c2: register holding first-fold carry (in {-1,0,1}); x, t, u: temps.
    undo(m): optional specialized fold for m in {0, -1} (adds p & m)."""
    lines = [f"/* Fold s = c2 | 1 (subtract p, or add p if c2 == -1). */",
             f"orr {x}, {c2}, #1"]
    lines += fold(x)
    # fold(x) leaves its carry in x. Mask is -1 iff (c2 | carry) == 0.
    lines += [f"/* Undo (add p) if c2 == 0 and the previous fold did not carry: */",
              f"/* mask = ((c2 | carry) == 0) ? -1 : 0, from the sign bit of (v | -v). */",
              f"orr {t}, {c2}, {x}",
              f"rsb {u}, {t}, #0",
              f"orr {t}, {t}, {u}",
              f"mvn {c2}, {t}, asr #31"]
    lines += (undo or fold)(c2)
    return lines

# ===========================================================================
# secp256r1: result = t + 2 s1 + 2 s2 + s3 + s4 - d1 - d2 - d3 - d4
P256_COLS = [
    [(1, 0), (1, 1), (-1, 3), (-1, 4), (-1, 5), (-1, 6)],
    [(1, 1), (1, 2), (-1, 4), (-1, 5), (-1, 6), (-1, 7)],
    [(1, 2), (1, 3), (-1, 5), (-1, 6), (-1, 7)],
    [(2, 3), (2, 4), (1, 5), (-1, 7), (-1, 0), (-1, 1)],
    [(2, 4), (2, 5), (1, 6), (-1, 1), (-1, 2)],
    [(2, 5), (2, 6), (1, 7), (-1, 2), (-1, 3)],
    [(2, 6), (1, 6), (2, 7), (1, 5), (-1, 0), (-1, 1)],
    [(2, 7), (1, 7), (1, 0), (-1, 2), (-1, 3), (-1, 4), (-1, 5)],
]

def p256_body(mla):
    R = ["r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"]
    body = ["/* a0..a7 = product[8..15] -> r2-r9. */",
            "add r12, r1, #32",
            "ldm r12, {r2-r9}"]
    if mla:
        body += ["/* r0 = 1 and r14 = 2 are umlal multipliers, so save the result pointer in\n"
                 "           product[15] (a7 is already in r9). Results go to product[0..7]. */",
                 "str r0, [r1, #60]",
                 "mov r0, #1",
                 "mov r14, #2"]
        cols, c = columns_mla(P256_COLS, "r10", ("r12", "r11"), R, "r0", "r14")
        body += cols
        body += ["/* Reload result words; fold carry: r += c * (2^224 - 2^192 - 2^96 + 1). */",
                 "ldr r0, [r1, #60]",
                 "ldm r1, {r2-r9}"]
    else:
        cols, c = columns(P256_COLS, 0, "r10", ("r12", "r11"), R)
        body += cols
        body += ["/* Reload result words; fold carry: r += c * (2^224 - 2^192 - 2^96 + 1). */",
                 "ldm r0, {r2-r9}"]
    temps = [x for x in ["r1", "r10", "r11", "r12", "r14"] if x != c]
    s, mp, t = temps[0], temps[1], temps[2]
    # Adds the words [c, s, s, s - c, m, m, m - c, c + m] (see the function
    # comment); the carry out (in {-1, 0, 1}) replaces cr.
    def fold(cr):
        return [f"asr {s}, {cr}, #31",
                f"rsb {mp}, {cr}, #0",
                f"asr {mp}, {mp}, #31",
                f"adds {R[0]}, {R[0]}, {cr}",
                f"adcs {R[1]}, {R[1]}, {s}",
                f"adcs {R[2]}, {R[2]}, {s}",
                f"sub {t}, {s}, {cr}",
                f"adcs {R[3]}, {R[3]}, {t}",
                f"adcs {R[4]}, {R[4]}, {mp}",
                f"adcs {R[5]}, {R[5]}, {mp}",
                f"sub {t}, {mp}, {cr}",
                f"adcs {R[6]}, {R[6]}, {t}",
                f"add {t}, {cr}, {mp}",
                f"adcs {R[7]}, {R[7]}, {t}",
                f"adc {cr}, {s}, #0"]
    body += fold(c)
    x = temps[3]
    def undo(m):  # r += p & m; p = [-1, -1, -1, 0, 0, 0, 1, -1].
        return [f"and {t}, {m}, #1",
                f"adds {R[0]}, {R[0]}, {m}",
                f"adcs {R[1]}, {R[1]}, {m}",
                f"adcs {R[2]}, {R[2]}, {m}",
                f"adcs {R[3]}, {R[3]}, #0",
                f"adcs {R[4]}, {R[4]}, #0",
                f"adcs {R[5]}, {R[5]}, #0",
                f"adcs {R[6]}, {R[6]}, {t}",
                f"adc {R[7]}, {R[7]}, {m}"]
    body += finish(fold, c, x, t, mp, undo)
    body += ["stm r0, {r2-r9}"]
    return body

def p256():
    comment = ("/* Computes result = product % p (secp256r1), constant time.\n"
         "   Column sums of t + 2 s1 + 2 s2 + s3 + s4 - d1 - d2 - d3 - d4 (from\n"
         "   http://www.nsa.gov/ia/_files/nist-routines.pdf), then the carry is folded\n"
         "   back using 2^256 = 2^224 - 2^192 - 2^96 + 1 (mod p). Folding c adds the\n"
         "   two's complement words of c * (2^224 - 2^192 - 2^96 + 1):\n"
         "   [c, s, s, s - c, m, m, m - c, c + m] (s = c >> 31, m = -c >> 31).\n"
         "   This version accumulates positive terms with umlal (multipliers 1 and 2\n"
         "   in registers) and uses product[] as scratch. */")
    clob = ["r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r14"]
    emit("#if uECC_ARM_USE_UMAAL")
    func("secp256r1", comment, p256_body(True), clob)
    out.pop()
    emit("#else")
    func("secp256r1", "/* Same as above, accumulating with adds/adc. */", p256_body(False), clob)
    out.pop()
    emit("#endif /* uECC_ARM_USE_UMAAL */")
    emit("#define asm_mmod_fast_secp256r1 1")

# ===========================================================================
P224_COLS = [
    [(-1, 0), (-1, 4)],
    [(-1, 1), (-1, 5)],
    [(-1, 2), (-1, 6)],
    [(1, 0), (1, 4), (-1, 3)],
    [(1, 1), (1, 5), (-1, 4)],
    [(1, 2), (1, 6), (-1, 5)],
    [(1, 3), (-1, 6)],
]

def p224():
    R = ["r2", "r3", "r4", "r5", "r6", "r7", "r8"]
    body = ["/* r = t = product[0..6]; r1 -> a0; a4..a6 -> r9-r11. */",
            "ldm r1!, {r2-r8}",
            "ldr r9, [r1, #16]",
            "ldr r10, [r1, #20]",
            "ldr r11, [r1, #24]",
            "mov r12, #0",
            "/* r -= 2^128 * (a4, a5, a6); r12 = signed top carry. */",
            "subs r6, r6, r9",
            "sbcs r7, r7, r10",
            "sbcs r8, r8, r11",
            "sbc r12, r12, #0",
            "/* B = (a0, a1, a2, a3) + (a4, a5, a6, 0) -> (r9, r10, r11, r14), carry -> r1. */",
            "ldr r14, [r1, #0]",
            "adds r9, r9, r14",
            "ldr r14, [r1, #4]",
            "adcs r10, r10, r14",
            "ldr r14, [r1, #8]",
            "adcs r11, r11, r14",
            "ldr r14, [r1, #12]",
            "mov r1, #0",
            "adcs r14, r14, #0",
            "adc r1, r1, #0",
            "/* r -= B (5 words). */",
            "subs r2, r2, r9",
            "sbcs r3, r3, r10",
            "sbcs r4, r4, r11",
            "sbcs r5, r5, r14",
            "sbcs r6, r6, r1",
            "sbcs r7, r7, #0",
            "sbcs r8, r8, #0",
            "sbc r12, r12, #0",
            "/* r += 2^96 * B. */",
            "adds r5, r5, r9",
            "adcs r6, r6, r10",
            "adcs r7, r7, r11",
            "adcs r8, r8, r14",
            "adc r12, r12, r1",
            "/* Fold carry: r += c * (2^96 - 1). */"]
    c = "r12"
    s, mp, t, x = "r1", "r9", "r10", "r11"
    # Adds the words [-c, m, m, c + m, s, s, s]; carry out replaces cr.
    def fold(cr):
        return [f"asr {s}, {cr}, #31",
                f"rsb {t}, {cr}, #0",
                f"asr {mp}, {t}, #31",
                f"adds {R[0]}, {R[0]}, {t}",
                f"adcs {R[1]}, {R[1]}, {mp}",
                f"adcs {R[2]}, {R[2]}, {mp}",
                f"add {t}, {cr}, {mp}",
                f"adcs {R[3]}, {R[3]}, {t}",
                f"adcs {R[4]}, {R[4]}, {s}",
                f"adcs {R[5]}, {R[5]}, {s}",
                f"adcs {R[6]}, {R[6]}, {s}",
                f"adc {cr}, {s}, #0"]
    def undo(m):  # r += p & m; p = [1, 0, 0, -1, -1, -1, -1].
        return [f"and {t}, {m}, #1",
                f"adds {R[0]}, {R[0]}, {t}",
                f"adcs {R[1]}, {R[1]}, #0",
                f"adcs {R[2]}, {R[2]}, #0",
                f"adcs {R[3]}, {R[3]}, {m}",
                f"adcs {R[4]}, {R[4]}, {m}",
                f"adcs {R[5]}, {R[5]}, {m}",
                f"adc {R[6]}, {R[6]}, {m}"]
    body += fold(c)
    body += finish(fold, c, x, t, "r14", undo)
    body += ["stm r0, {r2-r8}"]
    func("secp224r1",
         "/* Computes result = product % p (secp224r1), constant time.\n"
         "   t + s1 + s2 - d1 - d2 (from http://www.nsa.gov/ia/_files/nist-routines.pdf)\n"
         "   = t + B * (2^96 - 1) - 2^128 * (a4, a5, a6), where\n"
         "   B = (a0, a1, a2, a3) + (a4, a5, a6, 0). The carry is folded back using\n"
         "   2^224 = 2^96 - 1 (mod p): folding c adds the two's complement words of\n"
         "   c * (2^96 - 1): [-c, m, m, c + m, s, s, s] (s = c >> 31, m = -c >> 31). */",
         body, ["r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r14"])

# ===========================================================================
def p192():
    R = ["r2", "r3", "r4", "r5", "r6", "r7"]
    body = ["/* r = t = product[0..5]; a0..a5 = product[6..11] -> r8-r12, r14. */",
            "ldm r1!, {r2-r7}",
            "ldm r1, {r8-r12, r14}",
            "mov r1, #0",
            "/* r += s1 = (a0, a1, a2, a3, a4, a5); carries accumulate in r1. */",
            "adds r2, r2, r8", "adcs r3, r3, r9", "adcs r4, r4, r10",
            "adcs r5, r5, r11", "adcs r6, r6, r12", "adcs r7, r7, r14",
            "adc r1, r1, #0",
            "/* r += s2 = (0, 0, a0, a1, a2, a3). */",
            "adds r4, r4, r8", "adcs r5, r5, r9", "adcs r6, r6, r10", "adcs r7, r7, r11",
            "adc r1, r1, #0",
            "/* r += s3 = (a4, a5, a4, a5, 0, 0). */",
            "adds r2, r2, r12", "adcs r3, r3, r14", "adcs r4, r4, r12", "adcs r5, r5, r14",
            "adcs r6, r6, #0", "adcs r7, r7, #0",
            "adc r1, r1, #0",
            "/* Fold carry: r += c * (2^64 + 1). */"]
    s, t, x, u = "r8", "r9", "r10", "r11"
    # Adds the words [c, s, c + s, s, s, s]; carry out replaces cr. Also used
    # with the mask cr in {0, -1} for the undo step (adds p): then s = cr and
    # the carry out is not needed.
    def fold(cr, undo=False):
        sv = cr if undo else s
        return ([] if undo else [f"asr {s}, {cr}, #31"]) + [
                f"add {t}, {cr}, {sv}",
                f"adds {R[0]}, {R[0]}, {cr}",
                f"adcs {R[1]}, {R[1]}, {sv}",
                f"adcs {R[2]}, {R[2]}, {t}",
                f"adcs {R[3]}, {R[3]}, {sv}",
                f"adcs {R[4]}, {R[4]}, {sv}",
                f"adcs {R[5]}, {R[5]}, {sv}"] + ([] if undo else [
                f"adc {cr}, {s}, #0"])
    body += fold("r1")
    body += finish(fold, "r1", x, t, u, lambda m: fold(m, True))
    body += ["stm r0, {r2-r7}"]
    func("secp192r1",
         "/* Computes result = product % p (secp192r1), constant time.\n"
         "   t + s1 + s2 + s3 (algorithm 5 and 6 from\n"
         "   http://www.isys.uni-klu.ac.at/PDF/2001-0126-MT.pdf), then the carry is\n"
         "   folded back using 2^192 = 2^64 + 1 (mod p). Folding c adds the two's\n"
         "   complement words of c * (2^64 + 1): [c, s, c + s, s, s, s] (s = c >> 31). */",
         body, ["r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r14"])

# ===========================================================================
def p160():
    R = ["r2", "r3", "r4", "r5", "r6"]
    A = ["r7", "r8", "r9", "r10", "r11"]
    body = ["/* r = lo -> r2-r6; hi -> r7-r11. */",
            "ldm r1!, {r2-r6}",
            "ldm r1, {r7-r11}",
            "mov r12, #0",
            "/* r += hi. */",
            "adds r2, r2, r7", "adcs r3, r3, r8", "adcs r4, r4, r9",
            "adcs r5, r5, r10", "adcs r6, r6, r11",
            "adc r12, r12, #0",
            "/* r += hi * 2^31: word j = (a[j] << 31) | (a[j-1] >> 1); a4 >> 1 goes to carry. */",
            "lsl r14, r7, #31",
            "adds r2, r2, r14"]
    for j in range(1, 5):
        body += [f"lsr r14, {A[j-1]}, #1",
                 f"orr r14, r14, {A[j]}, lsl #31",
                 f"adcs {R[j]}, {R[j]}, r14"]
    body += ["lsr r14, r11, #1",
             "adc r12, r12, r14",
             "/* Fold carry (c < 2^31 + 3): r += c * (2^31 + 1) = [c + (c << 31), (c >> 1) + carry]. */",
             "lsl r7, r12, #31",
             "adds r7, r7, r12",
             "lsr r8, r12, #1",
             "adc r8, r8, #0",
             "mov r12, #0",
             "adds r2, r2, r7", "adcs r3, r3, r8", "adcs r4, r4, #0",
             "adcs r5, r5, #0", "adcs r6, r6, #0",
             "adc r12, r12, #0"]
    s, t, x, u = "r7", "r8", "r9", "r10"
    # For cr in {-1, 0, 1}: adds the words [c + (c << 31), s, s, s, s]. For the
    # undo step, cr is a mask in {0, -1}: then s = cr and the carry out is not needed.
    def fold(cr, undo=False):
        sv = cr if undo else s
        return ([] if undo else [f"asr {s}, {cr}, #31"]) + [
                f"add {t}, {cr}, {cr}, lsl #31",
                f"adds {R[0]}, {R[0]}, {t}",
                f"adcs {R[1]}, {R[1]}, {sv}",
                f"adcs {R[2]}, {R[2]}, {sv}",
                f"adcs {R[3]}, {R[3]}, {sv}",
                f"adcs {R[4]}, {R[4]}, {sv}"] + ([] if undo else [
                f"adc {cr}, {s}, #0"])
    body += finish(fold, "r12", x, t, u, lambda m: fold(m, True))
    body += ["stm r0, {r2-r6}"]
    func("secp160r1",
         "/* Computes result = product % p (secp160r1), constant time.\n"
         "   lo + hi + hi * 2^31 using 2^160 = 2^31 + 1 (mod p), then the carry is\n"
         "   folded back the same way. For small c, folding c adds the two's complement\n"
         "   words of c * (2^31 + 1): [c + (c << 31), s, s, s, s] (s = c >> 31). */",
         body, ["r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r14"])

# ===========================================================================
def k1(umaal):
    R = ["r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"]
    body = ["/* r = lo -> r2-r9; r1 -> hi; r10 = 977 (1024 - 47). */",
            "ldm r1!, {r2-r9}",
            "mov r10, #1024",
            "sub r10, r10, #47"]
    if umaal:
        body += ["/* r += hi * 977 (umaal chain, carry in r11) and r += hi * 2^32 (adcs chain).\n"
                 "           umaal does not touch the flags, so the two carry chains interleave. */",
                 "mov r11, #0",
                 "mov r14, #0"]
        for j in range(8):
            body += [f"ldr r12, [r1, #{4 * j}]",
                     f"umaal {R[j]}, r11, r12, r10"]
            if j < 7:
                body += [f"{'adds' if j == 0 else 'adcs'} {R[j+1]}, {R[j+1]}, r12"]
        body += ["adcs r12, r12, r11",
                 "adc r14, r14, #0"]
    else:
        body += ["/* r += hi * 977: umlal gives each high word in r11, added to the next word.\n"
                 "           umlal does not touch the flags, so the adds/adcs carry chain runs across it. */"]
        for j in range(8):
            body += [f"ldr r12, [r1, #{4 * j}]"]
            if j > 0:
                body += [f"{'adds' if j == 1 else 'adcs'} {R[j]}, {R[j]}, r11"]
            body += ["mov r11, #0",
                     f"umlal {R[j]}, r11, r12, r10"]
        body += ["adc r11, r11, #0",
                 "/* r += hi * 2^32; top word + carries -> T = r14:r12. */"]
        for j in range(7):
            body += [f"ldr r12, [r1, #{4 * j}]",
                     f"{'adds' if j == 0 else 'adcs'} {R[j+1]}, {R[j+1]}, r12"]
        body += ["ldr r12, [r1, #28]",
                 "mov r14, #0",
                 "adcs r12, r12, r11",
                 "adc r14, r14, #0"]
    body += ["/* Fold T = r14:r12 (< 2^33): r += T * (2^32 + 977). r11:r2 = r2 + lo(T) * 977,\n"
             "           r11 += hi(T) * 977, then add T at word 1 with r11. */",
             "mov r11, #0",
             "umlal r2, r11, r12, r10",
             "mul r1, r14, r10",
             "add r11, r11, r1",
             "adds r12, r12, r11",
             "adc r14, r14, #0",
             "mov r11, #0",
             "adds r3, r3, r12",
             "adcs r4, r4, r14",
             "adcs r5, r5, #0", "adcs r6, r6, #0", "adcs r7, r7, #0",
             "adcs r8, r8, #0", "adcs r9, r9, #0",
             "adc r11, r11, #0",
             "/* c2 = r11 in {0, 1}: fold +1, i.e. add 2^32 + 977 (subtract p). */",
             "mov r12, #0",
             "adds r2, r2, r10",
             "adcs r3, r3, #1",
             "adcs r4, r4, #0", "adcs r5, r5, #0", "adcs r6, r6, #0",
             "adcs r7, r7, #0", "adcs r8, r8, #0", "adcs r9, r9, #0",
             "adc r12, r12, #0",
             "/* Undo (add p) if c2 == 0 and the fold did not carry: m = (c2 | carry) - 1, then add\n"
             "           m * (2^32 + 977) = [m * 977, 2 * m, m, ...], i.e. -(2^32 + 977) = p mod 2^256. */",
             "orr r11, r11, r12",
             "sub r11, r11, #1",
             "mul r12, r11, r10",
             "add r14, r11, r11",
             "adds r2, r2, r12",
             "adcs r3, r3, r14",
             "adcs r4, r4, r11", "adcs r5, r5, r11", "adcs r6, r6, r11",
             "adcs r7, r7, r11", "adcs r8, r8, r11", "adcs r9, r9, r11",
             "stm r0, {r2-r9}"]
    return body

def secp256k1():
    emit("#if uECC_ARM_USE_UMAAL")
    func_body = k1(True)
    func("secp256k1",
         "/* Computes result = product % p (secp256k1), constant time.\n"
         "   lo + hi * 977 + hi * 2^32 using 2^256 = 2^32 + 977 (mod p); the top\n"
         "   carry is folded back the same way, then p is subtracted once (and added\n"
         "   back if that was not needed). */",
         func_body, ["r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r14"])
    out.pop()  # remove #define; re-emit after #endif
    emit("#else")
    func("secp256k1",
         "/* Same as above, without UMAAL. */",
         k1(False), ["r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r14"])
    out.pop()
    emit("#endif /* uECC_ARM_USE_UMAAL */")
    emit("#define asm_mmod_fast_secp256k1 1")


# ===========================================================================
# Thumb-1 (ARMv6-M): only r0-r7 for arithmetic, no shifted operands and no
# 32x32->64 multiply. Column sums read the product words from memory; the
# folds update the result words in memory.

T1_CLOBBERS = ["r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9"]

def t1_columns(terms, n):
    """Signed column sums. r0 = result, r1 = product, r2 = column low word,
    r3/r4 = carry (alternating), r5 = 0, r6 = loaded term."""
    lines = ["/* r5 = 0 (Thumb-1 adcs/sbcs take no immediate). */",
             "movs r5, #0"]
    C, H = "r4", "r3"
    for j, col in enumerate(terms):
        if j == 1:
            lines.append(f"/* Column {j}. Sign-extend the previous carry {C} into {H} first, since\n"
                         f"           asrs sets the flags and would clobber the adds carry. */")
        else:
            lines.append(f"/* Column {j}. */")
        lines.append(f"ldr r2, [r1, #{4 * j}]")
        if j == 0:
            lines.append(f"movs {H}, #0")
        else:
            lines += [f"asrs {H}, {C}, #31", f"adds r2, r2, {C}", f"adcs {H}, r5"]
        # Merge repeated indices so each high word is loaded only once.
        coefs = {}
        order = []
        for coef, i in col:
            if i not in coefs:
                order.append(i)
            coefs[i] = coefs.get(i, 0) + coef
        for i in order:
            coef = coefs[i]
            lines.append(f"ldr r6, [r1, #{4 * (n + i)}]")
            for _ in range(abs(coef)):
                if coef > 0:
                    lines += ["adds r2, r2, r6", f"adcs {H}, r5"]
                else:
                    lines += ["subs r2, r2, r6", f"sbcs {H}, r5"]
        lines.append(f"str r2, [r0, #{4 * j}]")
        C, H = H, C
    return lines, C

def t1_chain(base, xs, R):
    """Adds the words xs (register names) to the n words at [base], starting
    a fresh carry chain at the first word. Leaves the carry in the flags."""
    lines = []
    for j, x in enumerate(xs):
        lines.append(f"ldr {R}, [{base}, #{4 * j}]")
        lines.append(f"adds {R}, {R}, {x}" if j == 0 else f"adcs {R}, {x}")
        lines.append(f"str {R}, [{base}, #{4 * j}]")
    return lines

def t1_finish(fold, undo, c2, regs):
    """c2: register holding the first-fold carry (in {-1, 0, 1}).
    fold(c, regs) -> (lines, k): fold c using registers regs (not c); k = carry.
    undo(m, regs) -> lines: r += p & m (m in {0, -1})."""
    lines = ["/* Save c2 in r8 (registers are scarce). */",
             "mov r8, " + c2]
    # c2 is saved, so it goes last in the pool and may be reused as a temp.
    pool = [r for r in regs if r != c2] + [c2]
    x = pool[0]
    lines += ["/* Fold s = c2 | 1 (subtract p, or add p if c2 == -1). */",
              f"movs {x}, #1", f"orrs {x}, {c2}"]
    fl, k = fold(x, [r for r in pool if r != x])
    lines += fl
    u, v = [r for r in pool if r != k][:2]
    lines += ["/* Undo (add p) if c2 == 0 and the previous fold did not carry: */",
              "/* mask = ((c2 | carry) == 0) ? -1 : 0, from the sign bit of (v | -v). */",
              f"mov {u}, r8", f"orrs {u}, {k}",
              f"rsbs {v}, {u}, #0", f"orrs {v}, {u}",
              f"asrs {v}, {v}, #31", f"mvns {v}, {v}"]
    lines += undo(v, [r for r in pool if r != v])
    return lines

T1_POOL = ["r1", "r2", "r3", "r4", "r5", "r6", "r7"]

def t1_func(name, comment, body):
    func(name, comment, body, T1_CLOBBERS)

def t1_p256():
    body, c = t1_columns(P256_COLS, 8)
    def fold(cr, regs):
        # Word values [c, s, s, s - c, m, m, m - c, c + m], then add them to
        # result[] in memory; the carry out ends up in s.
        s, m, x3, x6, x7, R = regs[:6]
        return ([f"asrs {s}, {cr}, #31",
                 f"rsbs {m}, {cr}, #0",
                 f"asrs {m}, {m}, #31",
                 f"subs {x3}, {s}, {cr}",
                 f"subs {x6}, {m}, {cr}",
                 f"adds {x7}, {cr}, {m}"]
                + t1_chain("r0", [cr, s, s, x3, m, m, x6, x7], R)
                + [f"movs {R}, #0", f"adcs {s}, {R}"]), s
    def undo(m, regs):
        b, z, R = regs[:3]
        return ([f"movs {b}, #1", f"ands {b}, {m}", f"movs {z}, #0"]
                + t1_chain("r0", [m, m, m, z, z, z, b, m], R))
    body.append("/* Fold carry: r += c * (2^224 - 2^192 - 2^96 + 1). */")
    fl, k = fold(c, [r for r in T1_POOL if r != c])
    body += fl
    body += t1_finish(fold, undo, k, T1_POOL)
    t1_func("secp256r1",
            "/* Computes result = product % p (secp256r1), constant time (Thumb-1).\n"
            "   Same algorithm as the Thumb-2 version. */", body)

def t1_p224():
    body, c = t1_columns(P224_COLS, 7)
    def fold(cr, regs):
        # Word values [-c, m, m, c + m, s, s, s]; carry out in s.
        s, m, x0, x3, R = regs[:5]
        return ([f"asrs {s}, {cr}, #31",
                 f"rsbs {x0}, {cr}, #0",
                 f"asrs {m}, {x0}, #31",
                 f"adds {x3}, {cr}, {m}"]
                + t1_chain("r0", [x0, m, m, x3, s, s, s], R)
                + [f"movs {R}, #0", f"adcs {s}, {R}"]), s
    def undo(m, regs):
        b, z, R = regs[:3]
        return ([f"movs {b}, #1", f"ands {b}, {m}", f"movs {z}, #0"]
                + t1_chain("r0", [b, z, z, m, m, m, m], R))
    body.append("/* Fold carry: r += c * (2^96 - 1). */")
    fl, k = fold(c, [r for r in T1_POOL if r != c])
    body += fl
    body += t1_finish(fold, undo, k, T1_POOL)
    t1_func("secp224r1",
            "/* Computes result = product % p (secp224r1), constant time (Thumb-1).\n"
            "   Column sums of t + s1 + s2 - d1 - d2, then folds as in the Thumb-2\n"
            "   version. */", body)

P192_COLS = [
    [(1, 0), (1, 4)],
    [(1, 1), (1, 5)],
    [(1, 2), (1, 0), (1, 4)],
    [(1, 3), (1, 1), (1, 5)],
    [(1, 4), (1, 2)],
    [(1, 5), (1, 3)],
]

def t1_p192():
    body, c = t1_columns(P192_COLS, 6)
    def fold(cr, regs):
        # Word values [c, s, c + s, s, s, s]; carry out in s.
        s, x2, R = regs[:3]
        return ([f"asrs {s}, {cr}, #31",
                 f"adds {x2}, {cr}, {s}"]
                + t1_chain("r0", [cr, s, x2, s, s, s], R)
                + [f"movs {R}, #0", f"adcs {s}, {R}"]), s
    def undo(m, regs):
        # p = [-1, -1, -2, -1, -1, -1].
        x2, R = regs[:2]
        return [f"adds {x2}, {m}, {m}"] + t1_chain("r0", [m, m, x2, m, m, m], R)
    body.append("/* Fold carry: r += c * (2^64 + 1). */")
    fl, k = fold(c, [r for r in T1_POOL if r != c])
    body += fl
    body += t1_finish(fold, undo, k, T1_POOL)
    t1_func("secp192r1",
            "/* Computes result = product % p (secp192r1), constant time (Thumb-1).\n"
            "   Column sums of t + s1 + s2 + s3, then folds as in the Thumb-2\n"
            "   version. */", body)

def t1_p160():
    body = ["/* r5 = 0. */", "movs r5, #0"]
    C, H = "r4", "r3"
    for j in range(5):
        body.append("/* Column 0: lo[j] + a[j] + ((a[j] << 31) | (a[j-1] >> 1)) + carry. */"
                    if j == 0 else f"/* Column {j}. */")
        body.append(f"ldr r2, [r1, #{4 * j}]")
        body.append(f"movs {H}, #0")
        if j > 0:
            body += [f"adds r2, r2, {C}", f"adcs {H}, r5"]
        body += [f"ldr r6, [r1, #{20 + 4 * j}]",
                 "adds r2, r2, r6", f"adcs {H}, r5",
                 "lsls r6, r6, #31"]
        if j > 0:
            body += [f"ldr r7, [r1, #{20 + 4 * (j - 1)}]", "lsrs r7, r7, #1", "orrs r6, r7"]
        body += ["adds r2, r2, r6", f"adcs {H}, r5",
                 f"str r2, [r0, #{4 * j}]"]
        C, H = H, C
    body += ["/* Top carry c += a4 >> 1. */",
             "ldr r7, [r1, #36]", "lsrs r7, r7, #1", f"adds {C}, {C}, r7",
             "/* Fold carry (c < 2^31 + 3): r += c * (2^31 + 1) = [c + (c << 31), (c >> 1) + carry]. */"]
    x0, x1, R, k = [r for r in ["r1", "r2", "r6", "r7", "r3", "r4"] if r != C][:4]
    body += [f"lsrs {x1}, {C}, #1", f"lsls {x0}, {C}, #31",
             f"adds {x0}, {x0}, {C}", f"adcs {x1}, r5"]
    body += t1_chain("r0", [x0, x1, "r5", "r5", "r5"], R)
    body += [f"movs {k}, #0", f"adcs {k}, {k}"]
    def fold(cr, regs):
        # For cr in {-1, 0, 1}: word values [c + (c << 31), s, s, s, s].
        s, x0, R = regs[:3]
        return ([f"asrs {s}, {cr}, #31",
                 f"lsls {x0}, {cr}, #31",
                 f"adds {x0}, {x0}, {cr}"]
                + t1_chain("r0", [x0, s, s, s, s], R)
                + [f"movs {R}, #0", f"adcs {s}, {R}"]), s
    def undo(m, regs):
        # p = [2^31 - 1, -1, -1, -1, -1].
        b, R = regs[:2]
        return [f"lsrs {b}, {m}, #1"] + t1_chain("r0", [b, m, m, m, m], R)
    body += t1_finish(fold, undo, k, T1_POOL)
    t1_func("secp160r1",
            "/* Computes result = product % p (secp160r1), constant time (Thumb-1).\n"
            "   lo + hi + hi * 2^31, then folds as in the Thumb-2 version. */", body)

def t1_k1():
    # results are accumulated in product[0..7] and copied to result at the end.
    body = ["/* r8 = result pointer; r9 = 977 (0xF4 * 4 + 1); r5 = 0. */",
            "mov r8, r0",
            "movs r2, #0xF4", "lsls r2, r2, #2", "adds r2, r2, #1", "mov r9, r2",
            "movs r5, #0"]
    C, H = "r4", "r3"
    for j in range(8):
        body.append(f"/* Column 0: lo[j] + a[j] * 977 + a[j-1] + carry. a[j] * 977 is built\n"
                    f"           from two 16x16 muls ({H}:r6); its high word joins the carry. */"
                    if j == 0 else f"/* Column {j}. */")
        body += [f"ldr {H}, [r1, #{32 + 4 * j}]",
                 f"uxth r6, {H}",
                 f"lsrs {H}, {H}, #16",
                 "mov r7, r9",
                 "muls r6, r7, r6",
                 f"muls {H}, r7, {H}",
                 f"lsls r7, {H}, #16",
                 f"lsrs {H}, {H}, #16",
                 "adds r6, r6, r7",
                 f"adcs {H}, r5",
                 f"ldr r2, [r1, #{4 * j}]",
                 "adds r2, r2, r6", f"adcs {H}, r5"]
        if j > 0:
            body += [f"ldr r7, [r1, #{32 + 4 * (j - 1)}]",
                     "adds r2, r2, r7", f"adcs {H}, r5",
                     f"adds r2, r2, {C}", f"adcs {H}, r5"]
        body.append(f"str r2, [r1, #{4 * j}]")
        C, H = H, C
    A, B = C, H  # C holds the carry
    body += ["/* T = a7 + carry -> r2:r6 (r2 in {0, 1}). */",
             "ldr r6, [r1, #60]", "movs r2, #0", f"adds r6, r6, {C}", "adcs r2, r5",
             "/* Fold T: r += T * (2^32 + 977). lo(T) * 977 via 16-bit halves; hi(T) * 977 is\n"
             "           977 & -hi(T). Words added: [lo, hi + lo(T), hi(T) + carry]. r0 is a temp. */",
             f"uxth {A}, r6", f"lsrs {B}, r6, #16", "mov r7, r9",
             f"muls {A}, r7, {A}", f"muls {B}, r7, {B}",
             f"lsls r0, {B}, #16", f"lsrs {B}, {B}, #16",
             f"adds {A}, {A}, r0", f"adcs {B}, r5",
             "rsbs r0, r2, #0", "ands r7, r0",
             f"adds {B}, {B}, r7", f"adds {B}, {B}, r6", "adcs r2, r5"]
    body += t1_chain("r1", [A, B, "r2", "r5", "r5", "r5", "r5", "r5"], "r6")
    body += ["movs r0, #0", "adcs r0, r0",
             "/* c2 = r0 in {0, 1}: fold +1, i.e. add 2^32 + 977 (subtract p). */",
             "mov r7, r9", "movs r3, #1"]
    body += t1_chain("r1", ["r7", "r3", "r5", "r5", "r5", "r5", "r5", "r5"], "r6")
    body += ["movs r4, #0", "adcs r4, r4",
             "/* Undo (add p) if c2 == 0 and the fold did not carry: m = (c2 | carry) - 1,\n"
             "           add [m * 977, 2 * m, m, ...] = m * (2^32 + 977). */",
             "orrs r0, r4", "subs r0, r0, #1",
             "muls r7, r0, r7", "adds r3, r0, r0"]  # r7 is still 977.
    body += t1_chain("r1", ["r7", "r3", "r0", "r0", "r0", "r0", "r0", "r0"], "r6")
    body += ["/* Copy product[0..7] to result. */",
             "mov r0, r8",
             "ldm r1!, {r2-r5}", "stm r0!, {r2-r5}",
             "ldm r1!, {r2-r5}", "stm r0!, {r2-r5}"]
    t1_func("secp256k1",
            "/* Computes result = product % p (secp256k1), constant time (Thumb-1).\n"
            "   Column sums of lo + hi * 977 + hi * 2^32 (products via 16-bit halves),\n"
            "   accumulated in product[0..7]; then folds as in the Thumb-2 version. */",
            body)

emit("/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */")
emit("")
emit("/* Constant-time fast reductions (vli_mmod_fast) for ARM, Thumb-2 and Thumb-1.")
emit("   Generated by scripts/mmod_arm.py. Each routine reduces a 2 * num_words")
emit("   product to the fully reduced result in [0, p). result may alias product;")
emit("   some versions also use product as scratch. The final steps are branch-free:")
emit("   after folding the top carry, c2 in {-1, 0, 1} remains; folding +1 is the")
emit("   same as subtracting p (and -1 as adding p) modulo 2^N, so we fold (c2 | 1)")
emit("   and then fold -1 again if c2 was 0 and that fold did not carry (the value")
emit("   was already < p). */")
emit("")
emit("#ifndef _UECC_ASM_ARM_MMOD_H_")
emit("#define _UECC_ASM_ARM_MMOD_H_")
emit("")
emit("#if (uECC_PLATFORM == uECC_arm_thumb)")
for cond, fn in [("uECC_SUPPORTS_secp160r1", t1_p160), ("uECC_SUPPORTS_secp192r1", t1_p192),
                 ("uECC_SUPPORTS_secp224r1", t1_p224), ("uECC_SUPPORTS_secp256r1", t1_p256),
                 ("uECC_SUPPORTS_secp256k1", t1_k1)]:
    emit("")
    emit(f"#if {cond}")
    fn()
    emit(f"#endif /* {cond} */")
emit("")
emit("#else /* uECC_PLATFORM != uECC_arm_thumb */")
for cond, fn in [("uECC_SUPPORTS_secp160r1", p160), ("uECC_SUPPORTS_secp192r1", p192),
                 ("uECC_SUPPORTS_secp224r1", p224), ("uECC_SUPPORTS_secp256r1", p256),
                 ("uECC_SUPPORTS_secp256k1", secp256k1)]:
    emit("")
    emit(f"#if {cond}")
    fn()
    emit(f"#endif /* {cond} */")
emit("")
emit("#endif /* uECC_PLATFORM == uECC_arm_thumb */")
emit("")
emit("#endif /* _UECC_ASM_ARM_MMOD_H_ */")
open(sys.argv[1], "w").write("\n".join(out) + "\n")
