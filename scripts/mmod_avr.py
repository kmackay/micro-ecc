#!/usr/bin/env python3
"""Generates asm_avr_mmod.inc: constant-time fast reductions (vli_mmod_fast) for AVR."""
import sys

LOOPED = "--looped" in sys.argv

out = []
def emit(s=""):
    out.append(s)

# ---------------------------------------------------------------------------
# Curve constants (little-endian 32-bit words).
P = {
    "secp160r1": [0x7FFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF],
    "secp192r1": [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF],
    "secp224r1": [0x00000001, 0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                  0xFFFFFFFF],
    "secp256r1": [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0x00000000, 0x00000000,
                  0x00000001, 0xFFFFFFFF],
    "secp256k1": [0xFFFFFC2F, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                  0xFFFFFFFF, 0xFFFFFFFF],
}

def words_to_bytes(ws):
    return [(w >> (8 * k)) & 0xFF for w in ws for k in range(4)]

def p_bytes(curve):
    return words_to_bytes(P[curve])

def d_bytes(curve):
    n = len(P[curve])
    pv = sum(w << (32 * i) for i, w in enumerate(P[curve]))
    d = (1 << (32 * n)) - pv
    return [(d >> (8 * k)) & 0xFF for k in range(4 * n)]

# ---------------------------------------------------------------------------
# Register conventions: Z (r30:r31) = product, Y (r28:r29) = result, r1 = 0.
# Column accumulator r18..r21 (+ carry/sign r22/r23), term temp r24..r27.
L = ["r18", "r19", "r20", "r21"]
T = ["r24", "r25", "r26", "r27"]

def ld(dst, base, off):
    return f"ld {dst}, {base}" if off == 0 else f"ldd {dst}, {base}+{off}"

def st(base, off, src):
    return f"st {base}, {src}" if off == 0 else f"std {base}+{off}, {src}"

def columns(terms, n, cache=None):
    """Signed column sums of 32-bit words: column j = product[j] + sum(coef *
    product[n + i]). Results are stored to result[j]; returns (lines, carry
    register) where the carry is a small signed byte."""
    cache = cache or {}
    lines = []
    # Words of the high half used in several columns can be preloaded into
    # registers (cache: index -> 4 registers).
    for i, regs in cache.items():
        lines.append(f"/* Preload a[{i}]. */")
        lines += [ld(r, "Z", 4 * (n + i) + k) for k, r in enumerate(regs)]
    # C holds the previous column's signed carry byte, H accumulates this
    # column's carry; they swap roles each column.
    C, H = "r23", "r22"
    for j, col in enumerate(terms):
        desc = "".join(f" {'+' if c > 0 else '-'} {str(abs(c)) + ' ' if abs(c) > 1 else ''}a[{i}]"
                       for c, i in col)
        lines.append(f"/* Column {j}: p[{j}]{desc}{' + carry' if j else ''}. */")
        lines += [ld(r, "Z", 4 * j + k) for k, r in enumerate(L)]
        if j == 0:
            lines.append(f"mov {H}, r1")
        else:
            # Sign-extend the previous carry C (H = 0x00 or 0xFF), add it, and
            # start this column's carry at H + carry-out.
            lines += [f"mov {H}, {C}", f"lsl {H}", f"sbc {H}, {H}",
                      f"add {L[0]}, {C}", f"adc {L[1]}, {H}", f"adc {L[2]}, {H}",
                      f"adc {L[3]}, {H}", f"adc {H}, r1"]
        # Merge repeated terms so each a[i] is loaded once per column.
        coefs, order = {}, []
        for coef, i in col:
            if i not in coefs:
                order.append(i)
            coefs[i] = coefs.get(i, 0) + coef
        for i in order:
            coef = coefs[i]
            if i in cache:
                X = cache[i]
            else:
                X = T
                lines += [ld(r, "Z", 4 * (n + i) + k) for k, r in enumerate(T)]
            # Coefficient k is applied as k repeated adds (or subtracts).
            for _ in range(abs(coef)):
                if coef > 0:
                    lines += [f"add {L[0]}, {X[0]}"] + [f"adc {L[k]}, {X[k]}" for k in (1, 2, 3)]
                    lines.append(f"adc {H}, r1")
                else:
                    lines += [f"sub {L[0]}, {X[0]}"] + [f"sbc {L[k]}, {X[k]}" for k in (1, 2, 3)]
                    lines.append(f"sbc {H}, r1")
        lines += [st("Y", 4 * j + k, r) for k, r in enumerate(L)]
        C, H = H, C
    return lines, C

def chain(xs, t="r24"):
    """result[k] += xs[k] for all bytes (one carry chain); xs = list of byte
    registers (r1 for zero). Leaves the carry in C."""
    lines = []
    for b, x in enumerate(xs):
        lines.append(ld(t, "Y", b))
        lines.append(f"add {t}, {x}" if b == 0 else f"adc {t}, {x}")
        lines.append(st("Y", b, t))
    return lines

def word_bytes(spec):
    """spec: list of (lo, hi) register pairs per 32-bit word -> byte list."""
    res = []
    for lo, hi in spec:
        res += [lo, hi, hi, hi]
    return res

def sign(dst, src):
    return [f"mov {dst}, {src}", f"lsl {dst}", f"sbc {dst}, {dst}"]

def finish(curve, c2, signed, regs):
    """Constant-time final reduction. c2: register with carry in {-1,0,1} (or
    {0,1} if not signed). result + c2 * 2^N is in (-p, 2p).
    Compares result with p (read-only pass), then adds (p & m_add) |
    ((2^N - p) & m_sub) in one pass, where m_add = (c2 < 0) and
    m_sub = (c2 > 0) | (c2 == 0 && result >= p)."""
    pb, db = p_bytes(curve), d_bytes(curve)
    nb = len(pb)
    t = "r24"
    regs = [r for r in regs if r not in (c2, t)]
    bm, pos = regs[0], regs[1]
    lines = ["/* Compare result with p (read-only pass): bm = 0xFF if result >= p. */"]
    for b in range(nb):
        lines.append(ld(t, "Y", b))
        lines.append(f"subi {t}, 0x{pb[b]:02X}" if b == 0 else f"sbci {t}, 0x{pb[b]:02X}")
    lines += [f"sbc {bm}, {bm}", f"com {bm}"]   # bm = 0xFF if result >= p
    if signed:
        neg, msub = regs[2], regs[3]
        lines += ["/* m_add = (c2 < 0), m_sub = (c2 > 0) | (c2 == 0 && result >= p). */"]
        lines += sign(neg, c2)
        lines += [f"mov {pos}, {c2}", f"neg {pos}", f"lsl {pos}", f"sbc {pos}, {pos}",
                  f"mov {msub}, {neg}", f"or {msub}, {pos}", f"com {msub}",   # zero mask
                  f"and {msub}, {bm}", f"or {msub}, {pos}"]
        madd = neg
    else:
        msub = regs[2]
        lines += ["/* m_sub = (c2 == 1) | (result >= p). */",
                  f"mov {msub}, {c2}", f"neg {msub}", f"or {msub}, {bm}"]
        madd = None
    # X bytes: (p_k & m_add) | (d_k & m_sub). Each distinct (p_k, d_k) pair
    # gets one register (computed once); 0x00 and 0xFF bytes reuse r1 or the
    # mask registers directly.
    free = [r for r in regs[4 if signed else 3:] if int(r[1:]) >= 16]
    cachex = {}
    xs = []
    for b in range(nb):
        pk = pb[b] if signed else 0
        dk = db[b]
        key = (pk, dk)
        if key not in cachex:
            if key == (0, 0):
                cachex[key] = "r1"
            elif key == (0xFF, 0):
                cachex[key] = madd
            elif key == (0, 0xFF):
                cachex[key] = msub
            else:
                x = free.pop(0)
                cachex[key] = x
                if pk and dk:
                    y = free.pop(0)
                    lines += [f"mov {x}, {madd}", f"andi {x}, 0x{pk:02X}",
                              f"mov {y}, {msub}", f"andi {y}, 0x{dk:02X}", f"or {x}, {y}"]
                elif pk:
                    lines += [f"mov {x}, {madd}", f"andi {x}, 0x{pk:02X}"]
                else:
                    lines += [f"mov {x}, {msub}", f"andi {x}, 0x{dk:02X}"]
        xs.append(cachex[key])
    lines.append("/* result += (p & m_add) | ((2^N - p) & m_sub). */")
    lines += chain(xs, t)
    return lines

def func(name, comment, body, clobbers, c2=None, signed=False):
    emit(comment)
    if c2 is not None:
        emit(f"static const struct uECC_Curve_t curve_{name};")
    emit(f"static void vli_mmod_fast_{name}(uint8_t *result, uint8_t *product) {{")
    if c2 is not None:
        emit("    uint8_t c2;")
    emit("    __asm__ volatile (")
    for item in body:
        if item.startswith("/*"):
            emit("        " + item)
        else:
            emit(f'        "{item} \\n\\t"')
    if c2 is not None:
        emit(f'        "mov %[c2], {c2} \\n\\t"')
        emit('        : [c2] "=r" (c2)')
    else:
        emit("        :")
    emit('        : "y" (result), "z" (product)')
    emit("        : " + ", ".join(f'"{c}"' for c in clobbers) + ', "memory"')
    emit("    );")
    if c2 is not None:
        emit(f"    vli_mmod_finish_avr(result, curve_{name}.p, c2, {len(P[name]) * 4});")
    emit("}")
    emit(f"#define asm_mmod_fast_{name} 1")

def finish_or_call(curve, c2, signed, regs):
    """Returns (lines, c2_out): inline finish, or nothing (shared looped call).
    --looped trades speed for size by calling vli_mmod_finish_avr instead."""
    if LOOPED:
        if not signed:
            return [], c2
        return [], c2
    return finish(curve, c2, signed, regs), None

FINISH_HELPER = r'''/* Shared looped final reduction: result + c2 * 2^N is in (-p, 2p) with c2 in
   {-1, 0, 1}. Compares result with p, then adds p (c2 < 0), subtracts p
   (c2 > 0, or c2 == 0 and result >= p) or adds 0, in constant time. */
__attribute__((noinline))
static void vli_mmod_finish_avr(uint8_t *result, const uint8_t *p, int8_t c2, uint8_t num_bytes) {
    uint8_t cnt, t, u, bm, m, s;
    __asm__ volatile (
        "movw r26, %[p] \n\t"
        "movw r30, %[r] \n\t"
        "mov %[cnt], %[n] \n\t"
        "clc \n\t"
        "1: \n\t"
        "ld %[t], Z+ \n\t"
        "ld %[u], X+ \n\t"
        "sbc %[t], %[u] \n\t"
        "dec %[cnt] \n\t"
        "brne 1b \n\t"
        "sbc %[bm], %[bm] \n\t"
        "com %[bm] \n\t"             /* bm = 0xFF if result >= p */
        "mov %[m], %[c2] \n\t"       /* m = m_add = (c2 < 0) */
        "lsl %[m] \n\t"
        "sbc %[m], %[m] \n\t"
        "mov %[s], %[c2] \n\t"       /* s = (c2 > 0) */
        "neg %[s] \n\t"
        "lsl %[s] \n\t"
        "sbc %[s], %[s] \n\t"
        "mov %[t], %[m] \n\t"        /* t = (c2 == 0) */
        "or %[t], %[s] \n\t"
        "com %[t] \n\t"
        "and %[t], %[bm] \n\t"
        "or %[s], %[t] \n\t"         /* s = m_sub */
        "or %[m], %[s] \n\t"         /* m = m_add | m_sub */
        "movw r26, %[p] \n\t"
        "movw r30, %[r] \n\t"
        "mov %[cnt], %[n] \n\t"
        "mov %[t], %[s] \n\t"
        "lsr %[t] \n\t"              /* C = 1 if subtracting */
        "2: \n\t"                     /* result += ((p & m) ^ s) + C */
        "ld %[t], Z \n\t"
        "ld %[u], X+ \n\t"
        "and %[u], %[m] \n\t"
        "eor %[u], %[s] \n\t"
        "adc %[t], %[u] \n\t"
        "st Z+, %[t] \n\t"
        "dec %[cnt] \n\t"
        "brne 2b \n\t"
        : [cnt] "=&r" (cnt), [t] "=&r" (t), [u] "=&r" (u), [bm] "=&r" (bm),
          [m] "=&r" (m), [s] "=&r" (s)
        : [r] "r" (result), [p] "r" (p), [c2] "r" (c2), [n] "r" (num_bytes)
        : "r26", "r27", "r30", "r31", "memory"
    );
}
'''


CLOB_LOW = ["r0", "r18", "r19", "r20", "r21", "r22", "r23", "r24", "r25", "r26", "r27"]
FREE = ["r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23", "r25", "r26", "r27"]

# ===========================================================================
P192_COLS = [
    [(1, 0), (1, 4)],
    [(1, 1), (1, 5)],
    [(1, 2), (1, 0), (1, 4)],
    [(1, 3), (1, 1), (1, 5)],
    [(1, 4), (1, 2)],
    [(1, 5), (1, 3)],
]
P224_COLS = [
    [(-1, 0), (-1, 4)],
    [(-1, 1), (-1, 5)],
    [(-1, 2), (-1, 6)],
    [(1, 0), (1, 4), (-1, 3)],
    [(1, 1), (1, 5), (-1, 4)],
    [(1, 2), (1, 6), (-1, 5)],
    [(1, 3), (-1, 6)],
]
P256_COLS = [
    [(1, 0), (1, 1), (-1, 3), (-1, 4), (-1, 5), (-1, 6)],
    [(1, 1), (1, 2), (-1, 4), (-1, 5), (-1, 6), (-1, 7)],
    [(1, 2), (1, 3), (-1, 5), (-1, 6), (-1, 7)],
    [(2, 3), (2, 4), (1, 5), (-1, 7), (-1, 0), (-1, 1)],
    [(2, 4), (2, 5), (1, 6), (-1, 1), (-1, 2)],
    [(2, 5), (2, 6), (1, 7), (-1, 2), (-1, 3)],
    [(3, 6), (2, 7), (1, 5), (-1, 0), (-1, 1)],
    [(3, 7), (1, 0), (-1, 2), (-1, 3), (-1, 4), (-1, 5)],
]

# Solinas reduction: cols lists, per result word j, the (coef, i) terms of
# product[n + i] added to product[j]. fold_spec(c, regs) returns the per-word
# (lo, hi) byte registers of c * (2^N - p), the setup code computing them, and
# the register that receives the final carry.
def solinas(curve, cols, n, fold_spec, comment, cache=None, signed=True):
    body, c = columns(cols, n, cache)
    regs = [r for r in FREE if r != c]
    spec, setup, kreg = fold_spec(c, regs)
    body.append("/* Fold carry: result += c * (2^N - p), c signed. */")
    body += setup
    body += chain(word_bytes(spec))
    body.append(f"adc {kreg}, r1")
    fl, c2out = finish_or_call(curve, kreg, signed, [r for r in FREE if r != kreg])
    body += fl
    clob = CLOB_LOW + ["r16", "r17"]
    if cache:
        clob += sorted({r for regs_ in cache.values() for r in regs_}, key=lambda r: int(r[1:]))
    func(curve, comment, body, sorted(set(clob), key=lambda r: int(r[1:])), c2=c2out)

def p192():
    def fold(c, regs):
        s, x2 = regs[0], regs[1]
        setup = sign(s, c) + [f"mov {x2}, {c}", f"add {x2}, {s}"]
        return [(c, s), (s, s), (x2, s), (s, s), (s, s), (s, s)], setup, s
    solinas("secp192r1", P192_COLS, 6, fold,
            "/* Computes result = product % p (secp192r1), constant time.\n"
            "   Column sums of t + s1 + s2 + s3, then the carry c is folded back using\n"
            "   2^192 = 2^64 + 1 (mod p), and p is added or subtracted once. */",
            signed=False)

def p224():
    def fold(c, regs):
        s, m, x0, x3 = regs[:4]
        setup = (sign(s, c) + [f"mov {x0}, {c}", f"neg {x0}"] + sign(m, x0)
                 + [f"mov {x3}, {c}", f"add {x3}, {m}"])
        return [(x0, m), (m, m), (m, m), (x3, s), (s, s), (s, s), (s, s)], setup, s
    solinas("secp224r1", P224_COLS, 7, fold,
            "/* Computes result = product % p (secp224r1), constant time.\n"
            "   Column sums of t + s1 + s2 - d1 - d2, then the carry c is folded back\n"
            "   using 2^224 = 2^96 - 1 (mod p), and p is added or subtracted once. */")

def p256():
    def fold(c, regs):
        s, m, x3, x6, x7 = regs[:5]
        setup = (sign(s, c) + [f"mov {m}, {c}", f"neg {m}", f"lsl {m}", f"sbc {m}, {m}",
                               f"mov {x3}, {s}", f"sub {x3}, {c}",
                               f"mov {x6}, {m}", f"sub {x6}, {c}",
                               f"mov {x7}, {c}", f"add {x7}, {m}"])
        return [(c, s), (s, s), (s, s), (x3, m), (m, m), (m, m), (x6, m), (x7, s)], setup, s
    cache = {5: ["r2", "r3", "r4", "r5"], 6: ["r6", "r7", "r8", "r9"],
             7: ["r10", "r11", "r12", "r13"], 4: ["r14", "r15", "r16", "r17"]}
    solinas("secp256r1", P256_COLS, 8, fold,
            "/* Computes result = product % p (secp256r1), constant time.\n"
            "   Column sums of t + 2 s1 + 2 s2 + s3 + s4 - d1 - d2 - d3 - d4, then the\n"
            "   carry c is folded back using 2^256 = 2^224 - 2^192 - 2^96 + 1 (mod p), and\n"
            "   p is added or subtracted once. */", cache=cache)

# ===========================================================================
def p160():
    U = ["r24", "r25", "r26", "r27"]       # a[j]
    V = ["r12", "r13", "r14", "r15"]       # a[j - 1]
    body = []
    C, H = "r23", "r22"
    for j in range(5):
        # a * 2^31 is formed on the fly: word j of it is (a[j] << 31) | (a[j-1] >> 1).
        if j == 0:
            body.append("/* Column 0: p[0] + a[0] + (a[0] << 31). */")
        else:
            body.append(f"/* Column {j}: p[{j}] + a[{j}] + ((a[{j}] << 31) | (a[{j-1}] >> 1)) + carry. */")
        body += [ld(r, "Z", 4 * j + k) for k, r in enumerate(L)]
        body.append(f"mov {H}, r1")
        if j > 0:
            body += [f"add {L[0]}, {C}", f"adc {L[1]}, r1", f"adc {L[2]}, r1",
                     f"adc {L[3]}, r1", f"adc {H}, r1"]
        body += [ld(r, "Z", 20 + 4 * j + k) for k, r in enumerate(U)]
        body += [f"add {L[0]}, {U[0]}"] + [f"adc {L[k]}, {U[k]}" for k in (1, 2, 3)] + [f"adc {H}, r1"]
        body += [f"mov r0, {U[0]}", "lsr r0"]   # C = low bit of a[j], shifted into V
        if j == 0:
            body += ["mov r0, r1", "ror r0", f"add {L[3]}, r0", f"adc {H}, r1"]
        else:
            body += [f"ror {V[3]}", f"ror {V[2]}", f"ror {V[1]}", f"ror {V[0]}",
                     f"add {L[0]}, {V[0]}"] + [f"adc {L[k]}, {V[k]}" for k in (1, 2, 3)] + [f"adc {H}, r1"]
        body += [st("Y", 4 * j + k, r) for k, r in enumerate(L)]
        body += [f"mov {V[k]}, {U[k]}" for k in range(4)] if j < 4 else []
        C, H = H, C
    # c = carry + (a4 >> 1) (< 2^31 + 3), in U
    body += ["/* c = carry + (a[4] >> 1). */",
             f"lsr {U[3]}", f"ror {U[2]}", f"ror {U[1]}", f"ror {U[0]}",
             f"add {U[0]}, {C}", f"adc {U[1]}, r1", f"adc {U[2]}, r1", f"adc {U[3]}, r1",
             "/* Fold c: result += c + c * 2^31 = c + ((c & 1) << 31) + (c >> 1) * 2^32. */"]
    Y = ["r12", "r13", "r14", "r15"]
    body += [f"mov {Y[k]}, {U[k]}" for k in range(4)]
    body += [f"lsr {Y[3]}", f"ror {Y[2]}", f"ror {Y[1]}", f"ror {Y[0]}",
             f"mov r0, {U[0]}", "lsr r0", "mov r0, r1", "ror r0",
             f"add {U[3]}, r0"] + [f"adc {Y[k]}, r1" for k in range(4)]
    body += chain([U[0], U[1], U[2], U[3]] + Y + ["r1"] * 12, t="r0")
    body += ["/* c2 = final carry. */", f"mov {L[0]}, r1", f"adc {L[0]}, r1"]
    fl, c2out = finish_or_call("secp160r1", L[0], False, [r for r in FREE if r != L[0]])
    body += fl
    func("secp160r1",
         "/* Computes result = product % p (secp160r1), constant time.\n"
         "   Column sums of lo + hi + hi * 2^31 (2^160 = 2^31 + 1 (mod p)); the carry\n"
         "   is folded back the same way, then p is subtracted once if needed. */",
         body, sorted(set(CLOB_LOW + ["r12", "r13", "r14", "r15", "r16", "r17"]), key=lambda r: int(r[1:])),
         c2=c2out)

def k1():
    # Registers: L r18-r21; carry pairs (r22, r23) / (r16, r17); a[j-1] / a[j]
    # alternate between r2-r5 and r6-r9; product P r10-r15; K0 r24, K1 r25,
    # zero r26 (r1 is clobbered by mul).
    Hs = [("r22", "r23"), ("r16", "r17")]
    As = [["r2", "r3", "r4", "r5"], ["r6", "r7", "r8", "r9"]]
    Pr = ["r10", "r11", "r12", "r13", "r14", "r15"]
    K0, K1, Zr = "r24", "r25", "r26"
    # p[0..5] = b[0..3] * 977 (K1:K0 = 0x03:0xD1), 8 mul instructions.
    def mul977(b, p):
        return [f"mul {b[0]}, {K0}", f"movw {p[0]}, r0",
                f"mul {b[2]}, {K0}", f"movw {p[2]}, r0",
                f"mul {b[1]}, {K0}", f"add {p[1]}, r0", f"adc {p[2]}, r1", f"adc {p[3]}, {Zr}",
                f"mul {b[3]}, {K0}", f"add {p[3]}, r0", f"mov {p[4]}, r1", f"adc {p[4]}, {Zr}",
                f"mov {p[5]}, {Zr}",
                f"mul {b[0]}, {K1}", f"add {p[1]}, r0", f"adc {p[2]}, r1", f"adc {p[3]}, {Zr}",
                f"adc {p[4]}, {Zr}", f"adc {p[5]}, {Zr}",
                f"mul {b[1]}, {K1}", f"add {p[2]}, r0", f"adc {p[3]}, r1", f"adc {p[4]}, {Zr}",
                f"adc {p[5]}, {Zr}",
                f"mul {b[2]}, {K1}", f"add {p[3]}, r0", f"adc {p[4]}, r1", f"adc {p[5]}, {Zr}",
                f"mul {b[3]}, {K1}", f"add {p[4]}, r0", f"adc {p[5]}, r1"]
    body = ["/* K = 977; Zr = 0 (mul clobbers r1). */",
            f"ldi {K0}, 0xD1", f"ldi {K1}, 0x03", f"mov {Zr}, r1"]
    for j in range(8):
        C, H = Hs[j % 2], Hs[(j + 1) % 2]
        A, B = As[j % 2], As[(j + 1) % 2]   # A = a[j-1], B = a[j]
        if j == 0:
            body.append("/* Column 0: p[0] + a[0] * 977. */")
        else:
            body.append(f"/* Column {j}: p[{j}] + a[{j}] * 977 + a[{j-1}] + carry. */")
        body += [ld(r, "Z", 4 * j + k) for k, r in enumerate(L)]
        body += [f"mov {H[0]}, {Zr}", f"mov {H[1]}, {Zr}"]
        if j > 0:
            body += [f"add {L[0]}, {C[0]}", f"adc {L[1]}, {C[1]}", f"adc {L[2]}, {Zr}",
                     f"adc {L[3]}, {Zr}", f"adc {H[0]}, {Zr}",
                     f"add {L[0]}, {A[0]}"] + [f"adc {L[k]}, {A[k]}" for k in (1, 2, 3)] + [f"adc {H[0]}, {Zr}"]
        body += [ld(r, "Z", 32 + 4 * j + k) for k, r in enumerate(B)]
        body += mul977(B, Pr)
        body += [f"add {L[0]}, {Pr[0]}"] + [f"adc {L[k]}, {Pr[k]}" for k in (1, 2, 3)]
        body += [f"adc {H[0]}, {Pr[4]}", f"adc {H[1]}, {Pr[5]}"]
        body += [st("Y", 4 * j + k, r) for k, r in enumerate(L)]
    C = Hs[0]
    A = As[0]   # a7 (B of column 7 = As[0])
    # T = a7 + carry (33 bits) in A[0..3], T4
    T4 = "r27"
    body += ["/* T = a[7] + carry (< 2^33). */",
             f"mov {T4}, {Zr}",
             f"add {A[0]}, {C[0]}", f"adc {A[1]}, {C[1]}", f"adc {A[2]}, {Zr}", f"adc {A[3]}, {Zr}",
             f"adc {T4}, {Zr}",
             "/* X = T * (2^32 + 977) = T * 977 + T4 * 977 * 2^32 + T * 2^32. */"]
    body += mul977(A, Pr)
    # X bytes: Pr0..Pr3 | (Pr4 + Pr5*256 + T + T4*977) at bytes 4..7 | carry at byte 8
    X4 = As[1]  # reuse r6-r9 for bytes 4..7, H pair for T4*977
    body += [f"mov {C[0]}, {T4}", f"neg {C[0]}",          # 0xFF if T4
             f"mov {C[1]}, {C[0]}", f"and {C[0]}, {K0}", f"and {C[1]}, {K1}",  # T4 * 977
             f"mov {X4[0]}, {A[0]}", f"mov {X4[1]}, {A[1]}", f"mov {X4[2]}, {A[2]}", f"mov {X4[3]}, {A[3]}",
             f"add {X4[0]}, {Pr[4]}", f"adc {X4[1]}, {Pr[5]}", f"adc {X4[2]}, {Zr}", f"adc {X4[3]}, {Zr}",
             f"adc {T4}, {Zr}",
             f"add {X4[0]}, {C[0]}", f"adc {X4[1]}, {C[1]}", f"adc {X4[2]}, {Zr}", f"adc {X4[3]}, {Zr}",
             f"adc {T4}, {Zr}",
             "clr r1",
             "/* result += X; c2 = final carry. */"]
    body += chain(Pr[:4] + X4 + [T4] + ["r1"] * 23)
    body += [f"mov {L[0]}, r1", f"adc {L[0]}, r1"]
    fl, c2out = finish_or_call("secp256k1", L[0], False, [r for r in FREE if r != L[0]])
    body += fl
    clob = CLOB_LOW + [f"r{i}" for i in range(2, 18)]
    func("secp256k1",
         "/* Computes result = product % p (secp256k1), constant time.\n"
         "   Column sums of lo + hi * 977 + hi * 2^32 (2^256 = 2^32 + 977 (mod p)); the\n"
         "   top carry is folded back the same way, then p is subtracted once if\n"
         "   needed. */",
         body, sorted(set(clob), key=lambda r: int(r[1:])), c2=c2out)

emit("/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */")
emit("")
emit("/* Constant-time fast reductions (vli_mmod_fast) for AVR.")
emit("   Generated by scripts/mmod_avr.py. Each routine reduces a 2 * num_words")
emit("   product to the fully reduced result in [0, p); result may alias product.")
emit("   The top carry is folded back first, leaving result + c2 * 2^N with c2 in")
emit("   {-1, 0, 1} and a value in (-p, 2p); then a read-only pass compares result")
emit("   with p and a final pass adds p, subtracts p (adds 2^N - p) or adds 0,")
emit("   selected with masks. */")
emit("")
emit("#ifndef _UECC_ASM_AVR_MMOD_H_")
emit("#define _UECC_ASM_AVR_MMOD_H_")
if LOOPED:
    emit("")
    emit(FINISH_HELPER)
for cond, fn in [("uECC_SUPPORTS_secp160r1", p160), ("uECC_SUPPORTS_secp192r1", p192),
                 ("uECC_SUPPORTS_secp224r1", p224), ("uECC_SUPPORTS_secp256r1", p256),
                 ("uECC_SUPPORTS_secp256k1", k1)]:
    emit("")
    emit(f"#if {cond}")
    fn()
    emit(f"#endif /* {cond} */")
emit("")
emit("#endif /* _UECC_ASM_AVR_MMOD_H_ */")
open([a for a in sys.argv[1:] if not a.startswith("--")][0], "w").write("\n".join(out) + "\n")
