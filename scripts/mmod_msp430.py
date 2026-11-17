#!/usr/bin/env python3
"""Generates asm_msp430_mmod.inc: constant-time fast reductions (vli_mmod_fast) for
the classic MSP430 with 16-bit words (uECC_WORD_SIZE == 2)."""
import re
import sys

out = []
def emit(s=""):
    out.append(s)

# ---------------------------------------------------------------------------
# Curve constants (little-endian 16-bit words).
def words16(v, n):
    return [(v >> (16 * k)) & 0xFFFF for k in range(n)]

PV = {
    "secp160r1": (1 << 160) - (1 << 31) - 1,
    "secp192r1": (1 << 192) - (1 << 64) - 1,
    "secp224r1": (1 << 224) - (1 << 96) + 1,
    "secp256r1": (1 << 256) - (1 << 224) + (1 << 192) + (1 << 96) - 1,
    "secp256k1": (1 << 256) - (1 << 32) - 977,
}
NW = {"secp160r1": 10, "secp192r1": 12, "secp224r1": 14, "secp256r1": 16, "secp256k1": 16}

def p_words(curve):
    return words16(PV[curve], NW[curve])

def d_words(curve):
    n = NW[curve]
    return words16((1 << (16 * n)) - PV[curve], n)

# Hardware multiplier (MPY16) registers.
# (C string macros from asm_msp430.inc, so that uECC_MSP430_MPY_BASE applies.)
MPY, OP2 = '" uECC_MSP430_MPY "', '" uECC_MSP430_OP2 "'
RESLO, RESHI = '" uECC_MSP430_RESLO "', '" uECC_MSP430_RESHI "'
# MPY32 (uECC_MSP430_HWMULT == 32).
OP2L, OP2H = '" uECC_MSP430_OP2L "', '" uECC_MSP430_OP2H "'
RES0, RES1, RES2 = '" uECC_MSP430_RES0 "', '" uECC_MSP430_RES1 "', '" uECC_MSP430_RES2 "'

def imm(v):
    """Immediate operand; 0xFFFF etc. as negative values so that the assembler
    uses the constant generator where possible."""
    v &= 0xFFFF
    return f"#{v - 0x10000}" if v >= 0x8000 else f"#{v}"

def mem(base, off):
    return f"{off}({base})"

# ---------------------------------------------------------------------------
# Result word locations: result word k is in memory at 2k(r12) unless it is
# still held in a register (top word only).
class Words:
    def __init__(self, n, regs=None):
        self.n = n
        self.regs = regs or {}

    def loc(self, k):
        return self.regs.get(k, mem("r12", 2 * k))

    def src(self, k):
        """Source operand form (@r12 is one cycle cheaper than 0(r12))."""
        if k in self.regs:
            return self.regs[k]
        return "@r12" if k == 0 else mem("r12", 2 * k)

def add_chain(words, xs):
    """result += xs (one carry chain over all words; xs[k] is a register or
    None for zero). Leading zero words are skipped. Leaves the carry in C."""
    lines, started = [], False
    for k, x in enumerate(xs):
        if not started and x is None:
            continue
        if x is None:
            lines.append(f"adc {words.loc(k)}")
        else:
            lines.append(f"{'addc' if started else 'add'} {x}, {words.loc(k)}")
        started = True
    return lines

def ro_top(words, xs, ext, t, u=None):
    """t = ext + carry out of result + xs, without writing result (ext is a
    register holding the top word of the addend, or None for 0). With a spare
    register u and ext = 0, the carry through the top run of zero addend words
    is computed as (AND of those words) + carry, since it only propagates if
    they are all 0xFFFF (and costs 3 cycles per word instead of 4)."""
    n = len(xs)
    lines, started = [], False
    top = n
    if u is not None and ext is None:
        while top > 0 and xs[top - 1] is None:
            top -= 1
        if n - top < 2:
            top = n
    if top < n:
        lines.append(f"/* u = AND of result words {top}..{n - 1}. */")
        lines.append(f"mov {words.src(top)}, {u}")
        lines += [f"and {words.src(k)}, {u}" for k in range(top + 1, n)]
    for k in range(top):
        x = xs[k]
        if not started and x is None:
            continue
        lines.append(f"mov {words.src(k)}, {t}")
        if x is None:
            lines.append(f"adc {t}")
        else:
            lines.append(f"{'addc' if started else 'add'} {x}, {t}")
        started = True
    if top < n:
        lines.append(f"adc {u}")
    lines.append(f"mov {ext if ext else '#0'}, {t}")   # mov keeps C
    lines.append(f"adc {t}")
    return lines

def store_cached(words):
    return [f"mov {r}, {mem('r12', 2 * k)}" for k, r in sorted(words.regs.items())]

# The final reduction. After the column sums, the value is W = r + c * d with
# d = 2^N - p, r the N-bit low part and W in (-p, 2p). Since -p = d (mod 2^N),
# the reduced result is (r + (c + e) * d) mod 2^N, where e = 1 if W >= p,
# e = -1 if W < 0 and e = 0 otherwise. W >= p is only possible if c >= 0 and
# W < 0 only if c < 0, so e is the top word of r + (c + (c >= 0)) * d, which a
# read-only pass computes. One more pass then adds (c + e) * d.
FINAL_COMMENT = ("/* e = top word of result + (c + (c >= 0)) * d (read-only pass): 1 if the\n"
                 "           value is >= p, -1 if it is < 0, else 0. */")

# ---------------------------------------------------------------------------
# Function wrapper. r12 = result, r13 = product. The body saves the callee-saved
# registers it uses itself (so it never clobbers the frame pointer r4 behind the
# compiler's back) and lists the call-clobbered ones.
REG = re.compile(r"\br(1[0-5]|[4-9])\b")
ONE_OP = {"rrc", "rra", "rla", "rlc", "swpb", "sxt", "inv", "inc", "dec", "clr",
          "adc", "sbc", "pop", "incd", "decd"}

def written(lines):
    regs = set()
    for l in lines:
        if l.startswith("/*") or l.startswith("#"):
            continue
        op, _, args = l.partition(" ")
        op = op.split(".")[0]
        a = [x.strip() for x in args.split(",")] if args else []
        for x in a:
            m = re.fullmatch(r"@r(\d+)\+", x)
            if m:
                regs.add(f"r{m.group(1)}")
        if op in ("cmp", "bit", "push", "dint", "eint", "nop", "clrc", "setc"):
            continue
        dst = a[0] if op in ONE_OP else (a[-1] if a else "")
        if re.fullmatch(r"r\d+", dst):
            regs.add(dst)
    return regs

def used(lines):
    regs = set()
    for l in lines:
        if not (l.startswith("/*") or l.startswith("#")):
            regs |= {f"r{m}" for m in REG.findall(l)}
    return regs

def func(name, comment, body):
    w = written(body)
    u = used(body)
    saved = sorted({r for r in u if int(r[1:]) <= 10}, key=lambda r: int(r[1:]))
    clob = sorted({r for r in u if r in ("r11", "r14", "r15")}, key=lambda r: int(r[1:]))
    if saved:
        # CPUX: one PUSHM/POPM for r<lo>..r10 (20 bits in the large model).
        lo = int(saved[0][1:])
        k = 11 - lo
        full = (["#if uECC_MSP430_CPUX",
                 f"/* Save r{lo}..r10 (callee-saved). */",
                 f"@RAW uECC_MSP430_MMOD_PUSHM \" #{k}, r10 \\n\\t\"",
                 "#else",
                 "/* Save the callee-saved registers used below. */"]
                + [f"push {r}" for r in saved] + ["#endif"]
                + body
                + ["#if uECC_MSP430_CPUX",
                   f"@RAW uECC_MSP430_MMOD_POPM \" #{k}, r10 \\n\\t\"",
                   "#else"]
                + [f"pop {r}" for r in reversed(saved)] + ["#endif"])
    else:
        full = body
    emit(comment)
    emit("__attribute__((noinline))")
    emit(f"static void vli_mmod_fast_{name}(uint16_t *result, uint16_t *product) {{")
    emit('    register uint16_t *r12 __asm__("r12") = result;')
    emit('    register uint16_t *r13 __asm__("r13") = product;')
    emit("    __asm__ volatile (")
    for item in full:
        if item.startswith("#"):
            emit(item)
        elif item.startswith("@RAW "):
            emit("        " + item[5:])
        elif item.startswith("/*"):
            emit("        " + item)
        else:
            emit(f'        "{item} \\n\\t"')
    o12 = '"+r"' if "r12" in w else '"r"'
    o13 = '"+r"' if "r13" in w else '"r"'
    outs = [f'{o} (r{n})' for o, n in ((o12, 12), (o13, 13)) if o == '"+r"']
    ins = [f'{o} (r{n})' for o, n in ((o12, 12), (o13, 13)) if o == '"r"']
    emit("        : " + ", ".join(outs))
    emit("        : " + ", ".join(ins))
    emit("        : " + "".join(f'"{c}", ' for c in clob) + '"cc", "memory"')
    emit("    );")
    emit("}")
    emit(f"#define asm_mmod_fast_{name} 1")

# ---------------------------------------------------------------------------
# NIST curves (Solinas). cols32 lists, per 32-bit result word w, the (coef, i)
# terms of the high 32-bit word a[i] added to the low half; 16-bit column 2w + h
# uses half h of each term. digits: d = 2^N - p = sum(digits[k] * 2^(16 k)).
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
DIGITS = {
    "secp192r1": {0: 1, 4: 1},                  # 2^192 = 2^64 + 1
    "secp224r1": {0: -1, 6: 1},                 # 2^224 = 2^96 - 1
    "secp256r1": {0: 1, 6: -1, 12: -1, 14: 1},  # 2^256 = 2^224 - 2^192 - 2^96 + 1
}
CACHE_REGS = ["r4", "r5", "r6", "r7", "r8", "r9", "r10"]

def merged(col):
    coefs, order = {}, []
    for coef, i in col:
        if i not in coefs:
            order.append(i)
        coefs[i] = coefs.get(i, 0) + coef
    return [(coefs[i], i) for i in order if coefs[i]]

def nist(curve, cols32, signed, comment):
    n = NW[curve]
    cols = [merged(col) for col in cols32]
    # 32-bit columns with a 48-bit accumulator (A0, A1, B). The most used high
    # words are cached in registers (16-bit halves): a 32-bit term costs 3 cycles
    # with both halves in registers and 7 with both in memory.
    uses = {}
    for col in cols:
        for c, i in col:
            uses[i] = uses.get(i, 0) + abs(c)
    cregs = list(CACHE_REGS)
    cache = {}
    for i in sorted(uses, key=lambda i: (-uses[i], i)):
        for h in (0, 1):
            if cregs and uses[i] >= 3:
                cache[2 * i + h] = cregs.pop(0)
    loaded = set()
    A0, A1, B = "r14", "r15", "r11"
    body = []
    ptr = 0   # r13 - product, in bytes (r13 walks the low half with @r13+)
    def src(k):
        """High 16-bit word k (a register if cached)."""
        if k in cache:
            if k not in loaded:
                body.append(f"mov {mem('r13', 2 * (n + k) - ptr)}, {cache[k]}")
                loaded.add(k)
            return cache[k]
        return mem("r13", 2 * (n + k) - ptr)
    for w, col in enumerate(cols):
        desc = "".join(f" {'+' if c > 0 else '-'} {str(abs(c)) + ' ' if abs(c) > 1 else ''}a[{i}]"
                       for c, i in col)
        body.append(f"/* Column {w}: l[{w}]{desc}{' + carry' if w else ''} (32-bit words). */")
        if w == 0:
            body += [f"mov @r13+, {A0}", f"mov @r13+, {A1}", f"mov #0, {B}"]
        else:
            # The previous carry B (small) is the new low word; the higher words
            # start at its sign extension.
            nA0, nA1, nB = B, A0, A1
            if signed:
                body += [f"mov {nA0}, {nA1}", f"swpb {nA1}", f"sxt {nA1}", f"mov {nA1}, {nB}"]
            else:
                body += [f"mov #0, {nA1}", f"mov #0, {nB}"]
            A0, A1, B = nA0, nA1, nB
            body += [f"add @r13+, {A0}", f"addc @r13+, {A1}", f"adc {B}"]
        ptr += 4
        for coef, i in col:
            op, op2, cop = ("add", "addc", "adc") if coef > 0 else ("sub", "subc", "sbc")
            for _ in range(abs(coef)):
                body += [f"{op} {src(2 * i)}, {A0}", f"{op2} {src(2 * i + 1)}, {A1}", f"{cop} {B}"]
        if w < n // 2 - 1:
            body += [f"mov {A0}, {mem('r12', 4 * w)}", f"mov {A1}, {mem('r12', 4 * w + 2)}"]
    # The top two result words stay in A0, A1; c = B (signed small carry).
    words = Words(n, {n - 2: A0, n - 1: A1})
    A = None
    c = B
    pool0 = [r for r in ["r13"] + CACHE_REGS if r not in (A0, A1, B)]
    digits = DIGITS[curve]

    def addend(c, pool):
        """c * d as an N-bit two's complement addend: word k is digits[k] * c plus
        the sign extension of the lower digits' sum (0, s = sign(c) or
        m = sign(-c)); the part above 2^N is the extension of the top digit.
        Returns (setup lines, words, extension register)."""
        pool = list(pool)
        regs, setup = {}, []
        def get(name):
            if name in regs:
                return regs[name]
            r = pool.pop(0)
            regs[name] = r
            if name == "s":
                setup.extend([f"mov {c}, {r}", f"swpb {r}", f"sxt {r}"])
            elif name == "nc":
                setup.extend([f"mov {c}, {r}", f"inv {r}", f"inc {r}"])
            elif name == "m":
                setup.extend([f"mov {get('nc')}, {r}", f"swpb {r}", f"sxt {r}"])
            else:   # "c+s", "c+m", "s-c", "m-c"
                x, y = name.split("+") if "+" in name else name.split("-")
                if "+" in name:
                    setup.extend([f"mov {c}, {r}", f"add {get(y)}, {r}"])
                else:
                    setup.extend([f"mov {get(x)}, {r}", f"sub {c}, {r}"])
            return r
        xs, ext = [], None
        for k in range(n):
            e = digits.get(k, 0)
            if e == 0:
                xs.append(None if ext is None else get(ext))
            else:
                if ext is None:
                    name = "c" if e > 0 else "nc"
                else:
                    name = f"c+{ext}" if e > 0 else f"{ext}-c"
                xs.append(c if name == "c" else get(name))
                if signed:
                    ext = "s" if e > 0 else "m"
        return setup, xs, (get(ext) if ext else None)

    cc, t = pool0[0], pool0[1]
    body.append(FINAL_COMMENT)
    if signed:
        # cc = c + (c >= 0) = c + 1 + sign(c).
        body += [f"mov {c}, {cc}", f"swpb {cc}", f"sxt {cc}", f"add {c}, {cc}", f"inc {cc}"]
    else:
        body += [f"mov {c}, {cc}", f"inc {cc}"]
    setup, xs, ext = addend(cc, pool0[2:])
    body += setup + ro_top(words, xs, ext, t, None if signed else pool0[-1])
    body += ["/* result += (c + e) * d (mod 2^N). */", f"add {t}, {c}"]
    setup, xs, ext = addend(c, pool0)
    body += setup + add_chain(words, xs) + store_cached(words)
    func(curve, comment, body)

def final_small(words, xs, dw, t, temps, u):
    """Final reduction for c >= 0 when X = c * d is already in registers xs
    (low words first; d = dw, both short). Adds d (X + d = (c + 1) * d), reads
    e, subtracts d again unless e = 1, then adds X to result."""
    n = words.n
    lines = [FINAL_COMMENT]
    lines += [f"{'addc' if k else 'add'} {imm(dw[k]) if k < len(dw) else '#0'}, {x}"
              for k, x in enumerate(xs)]
    full = xs + [None] * (n - len(xs))
    lines += ro_top(words, full, None, t, u)
    lines += ["/* X -= d & (e - 1); result += X (mod 2^N). */", f"dec {t}"]
    us = []
    for k, dk in enumerate(dw):
        if dk == 0:
            us.append(None)
        elif k == len(dw) - 1:
            lines.append(f"and {imm(dk)}, {t}")
            us.append(t)
        else:
            u = temps[k]
            lines += [f"mov {t}, {u}", f"and {imm(dk)}, {u}"]
            us.append(u)
    lines += [f"{'subc' if k else 'sub'} {us[k]}, {x}" if k < len(us) and us[k] else f"sbc {x}"
              for k, x in enumerate(xs)]
    lines += add_chain(words, full) + store_cached(words)
    return lines

# ---------------------------------------------------------------------------
def p160():
    n = 10
    A, B = "r14", "r15"
    H = ["r11", "r10", "r9"]   # h[i] in H[i % 3]
    body = []
    ptr = 0
    def hi(i):
        return mem("r13", 2 * (n + i) - ptr)
    for i in range(n):
        if i == 0:
            body += ["/* Column 0: l[0] + h[0]. */", f"mov @r13+, {A}", f"mov #0, {B}"]
        else:
            body += [f"/* Column {i}: l[{i}] + h[{i}] + ((h[{i-1}] << 15) | (h[{i-2}] >> 1)) + carry. */",
                     f"mov #0, {A}", f"add @r13+, {B}", f"adc {A}"]
            A, B = B, A
        ptr += 2
        h = H[i % 3]
        body += [f"mov {hi(i)}, {h}", f"add {h}, {A}", f"adc {B}"]
        if i >= 1:
            # (h[i-1] << 15) | (h[i-2] >> 1): shift bit 0 of h[i-1] into h[i-2].
            if i == 1:
                z = H[2]
                body.append(f"mov #0, {z}")
            else:
                z = H[(i - 2) % 3]
            body += [f"bit #1, {H[(i - 1) % 3]}", f"rrc {z}", f"add {z}, {A}", f"adc {B}"]
        if i < n - 1:
            body.append(f"mov {A}, {mem('r12', 2 * i)}")
    words = Words(n, {n - 1: A})
    h8, h9 = H[8 % 3], H[9 % 3]
    c0, c1 = B, h9
    body += ["/* c = carry + (h >> 1) * 2^16 + (h[9] << 15 | h[8] >> 1), the part above 2^160. */",
             f"bit #1, {h9}", f"rrc {h8}", "clrc", f"rrc {h9}", f"add {h8}, {c0}", f"adc {c1}"]
    # c * 2^31 = Y1 * 2^16 + Y2 * 2^32 + Y3 * 2^48.
    Y1, Y2, Y3 = "r13", h8, H[7 % 3]
    body += ["/* X = c + c * 2^31 = c * d. */",
             f"mov {c1}, {Y3}", f"mov {c0}, {Y2}", f"mov #0, {Y1}",
             "clrc", f"rrc {Y3}", f"rrc {Y2}", f"rrc {Y1}",
             f"add {Y1}, {c1}", f"adc {Y2}", f"adc {Y3}"]
    body += final_small(words, [c0, c1, Y2, Y3], [1, 0x8000], "r13", ["r8"], "r8")
    func("secp160r1",
         "/* Computes result = product % p (secp160r1), constant time.\n"
         "   Column sums of lo + hi + hi * 2^31 (2^160 = 2^31 + 1 (mod p)); the carry\n"
         "   is folded back the same way, then p is subtracted once if needed. */", body)

# ---------------------------------------------------------------------------
def mul977_acc(x, A, B, T, U, T2, U2):
    """(B:A) += x * 977 = x * (2^10 - 2^6 + 2^4 + 1), without a multiplier."""
    return [f"mov {x}, {T}", f"swpb {T}", f"mov.b {T}, {U}", f"xor {U}, {T}",   # x << 8
            f"mov {T}, {T2}", f"mov {U}, {U2}",
            f"rla {T}", f"rlc {U}", f"rla {T}", f"rlc {U}",                     # x << 10
            f"add {T}, {A}", f"addc {U}, {B}",
            f"rra {U2}", f"rrc {T2}", f"rra {U2}", f"rrc {T2}",                 # x << 6
            f"sub {T2}, {A}", f"subc {U2}, {B}",
            f"rra {U2}", f"rrc {T2}", f"rra {U2}", f"rrc {T2}",                 # x << 4
            f"add {T2}, {A}", f"addc {U2}, {B}",
            f"add {x}, {A}", f"adc {B}"]

GIE_OFF = ["#if uECC_MSP430_DISABLE_INTERRUPTS",
           "/* Disable interrupts while the multiplier is in use. */",
           "push r2", "dint", "nop",
           "#endif"]
GIE_ON = ["#if uECC_MSP430_DISABLE_INTERRUPTS",
          "/* Restore the interrupt state. */",
          "pop r2", "nop",
          "#endif"]

def k1(hw):
    n = 16
    A, B = "r14", "r15"
    H = ["r11", "r10", "r9"]   # h[j] in H[j % 3]
    TMP = ["r8", "r7", "r6", "r5"]
    body = []
    if hw:
        body += GIE_OFF + ["/* OP1 = 977 (unsigned multiply). */", f"mov #977, {MPY}"]
    ptr = 0
    def hi(i):
        return mem("r13", 2 * (n + i) - ptr)
    for j in range(n):
        h = H[j % 3]
        body.append(f"/* Column {j}: l[{j}] + h[{j}] * 977{f' + h[{j-2}]' if j >= 2 else ''}"
                    f"{' + carry' if j else ''}. */")
        body.append(f"mov {hi(j)}, {h}")
        if hw:
            body.append(f"mov {h}, {OP2}")
        if j == 0:
            body += [f"mov @r13+, {A}", f"mov #0, {B}"]
        else:
            body += [f"mov #0, {A}", f"add @r13+, {B}", f"adc {A}"]
            A, B = B, A
        ptr += 2
        if j >= 2:
            body += [f"add {H[(j - 2) % 3]}, {A}", f"adc {B}"]
        if hw:
            body += [f"add {RESLO}, {A}", f"addc {RESHI}, {B}"]
        else:
            body += mul977_acc(h, A, B, *TMP)
        body.append(f"mov {A}, {mem('r12', 2 * j)}")
    # Columns 16..18: c = carry + h[14] + h[15] * 2^16 (< 2^32 + 978).
    c0, c1, c2 = B, A, H[1]   # H[1] = h[13] is free
    body += ["/* c = carry + h[14] + h[15] * 2^16, the part above 2^256 (c2 <= 1). */",
             f"add {H[14 % 3]}, {c0}", f"mov #0, {c1}", f"adc {c1}",
             f"add {H[15 % 3]}, {c1}", f"mov #0, {c2}", f"adc {c2}"]
    # X = c * (2^32 + 977) in X0..X4 (X3 = c1, X4 = c2 in place).
    X0, X1, X2 = "r13", H[0], H[2]
    body.append("/* X = c * 977 + c * 2^32. */")
    if hw:
        body += [f"mov {c0}, {OP2}", f"mov {RESLO}, {X0}", f"mov {RESHI}, {X1}",
                 f"mov {c1}, {OP2}", f"add {RESLO}, {X1}", f"mov {RESHI}, {X2}", f"adc {X2}"]
        body += GIE_ON
    else:
        body += [f"mov #0, {X0}", f"mov #0, {X1}", f"mov #0, {X2}"]
        body += mul977_acc(c0, X0, X1, *TMP)
        body += mul977_acc(c1, X1, X2, *TMP)
    t = TMP[0]
    body += [f"add {c0}, {X2}", f"adc {c1}", f"adc {c2}",
             f"mov {c2}, {t}", f"inv {t}", f"inc {t}", f"and #977, {t}",   # c2 * 977
             f"add {t}, {X2}", f"adc {c1}", f"adc {c2}"]
    words = Words(n)
    body += final_small(words, [X0, X1, X2, c1, c2], [977, 0, 1], c0, [TMP[0]], TMP[0])
    func("secp256k1",
         "/* Computes result = product % p (secp256k1), constant time.\n"
         "   Column sums of lo + hi * 977 + hi * 2^32 (2^256 = 2^32 + 977 (mod p)); the\n"
         "   top carry is folded back the same way, then p is subtracted once if\n"
         f"   needed. {'Uses the hardware multiplier.' if hw else 'Software multiplication by 977.'} */",
         body)


def k1_mpy32():
    """secp256k1 with MPY32: 32-bit columns, one 16 x 32-bit multiply (977 * a[w])
    per column."""
    n = 16
    A0, A1, B = "r14", "r15", "r11"
    H = [["r10", "r9"], ["r8", "r7"]]   # a[w] (32-bit high-half word) in H[w % 2]
    body = GIE_OFF + ["/* OP1 = 977 (unsigned; 16 x 32-bit multiplies). */", f"mov #977, {MPY}"]
    ptr = 0
    def hi(k):
        return mem("r13", 2 * (n + k) - ptr)
    for w in range(8):
        h, q = H[w % 2], H[(w + 1) % 2]
        body.append(f"/* Column {w}: l[{w}] + a[{w}] * 977{f' + a[{w-1}]' if w else ''}"
                    f"{' + carry' if w else ''} (32-bit words). */")
        body += [f"mov {hi(2 * w)}, {h[0]}", f"mov {hi(2 * w + 1)}, {h[1]}",
                 f"mov {h[0]}, {OP2L}", f"mov {h[1]}, {OP2H}"]
        if w == 0:
            body += [f"mov @r13+, {A0}", f"mov @r13+, {A1}", f"mov #0, {B}"]
        else:
            nA0, nA1, nB = B, A0, A1
            body += [f"mov #0, {nA1}", f"mov #0, {nB}",
                     f"add @r13+, {nA0}", f"addc @r13+, {nA1}", f"adc {nB}"]
            A0, A1, B = nA0, nA1, nB
            body += [f"add {q[0]}, {A0}", f"addc {q[1]}, {A1}", f"adc {B}"]
        ptr += 4
        # RES0 / RES1 / RES2 are read at least 3 / 5 / 6 cycles after the OP2H write.
        body += [f"add {RES0}, {A0}", f"addc {RES1}, {A1}", f"addc {RES2}, {B}"]
        if w < 7:
            body += [f"mov {A0}, {mem('r12', 4 * w)}", f"mov {A1}, {mem('r12', 4 * w + 2)}"]
    words = Words(n, {n - 2: A0, n - 1: A1})
    h7 = H[1]
    c0, c1, c2, t = B, h7[1], "r10", "r9"
    body += ["/* c = carry + a[7], the part above 2^256 (< 2^32 + 2^11; c2 <= 1). */",
             f"add {h7[0]}, {c0}", f"adc {c1}", f"mov #0, {c2}", f"adc {c2}",
             "/* X = c * 977 + c * 2^32 (c2 * 977 with a mask). */",
             f"mov {c0}, {OP2L}", f"mov {c1}, {OP2H}",
             f"mov {c2}, {t}", f"inv {t}", f"inc {t}", f"and #977, {t}",
             f"mov {RES0}, r13", f"mov {RES1}, {h7[0]}", f"add {RES2}, {t}"]
    body += GIE_ON
    body += [f"add {c0}, {t}", f"adc {c1}", f"adc {c2}"]
    body += final_small(words, ["r13", h7[0], t, c1, c2], [977, 0, 1], c0, ["r6"], "r6")
    func("secp256k1",
         "/* Computes result = product % p (secp256k1), constant time.\n"
         "   Column sums of lo + hi * 977 + hi * 2^32 (2^256 = 2^32 + 977 (mod p)) over\n"
         "   32-bit words, with one 16 x 32-bit MPY32 multiply per column; the top carry\n"
         "   is folded back the same way, then p is subtracted once if needed. */",
         body)

# ---------------------------------------------------------------------------
emit("/* Copyright 2026, Kenneth MacKay. Licensed under the BSD 2-clause license. */")
emit("")
emit("/* Constant-time fast reductions (vli_mmod_fast) for the classic MSP430.")
emit("   Generated by scripts/mmod_msp430.py. Each routine reduces a 2 * num_words")
emit("   product to the fully reduced result in [0, p); result may alias product.")
emit("   The high half is folded into the low half with column sums, giving")
emit("   r + c * 2^N = r + c * (2^N - p) (mod p); a read-only pass finds the final")
emit("   correction e in {-1, 0, 1} and one pass adds (c + e) * (2^N - p) (see")
emit("   FINAL_COMMENT in the script). Classic MSP430 and MSP430X (CPUX), small and")
emit("   large memory model (20-bit pointers; data may be above 64 KB). */")
emit("")
emit("#ifndef _UECC_ASM_MSP430_MMOD_H_")
emit("#define _UECC_ASM_MSP430_MMOD_H_")
emit("")
emit("/* CPUX: callee-saved registers are saved with one PUSHM / POPM (20 bits in the")
emit("   large model). */")
emit("#if uECC_MSP430_LARGE")
emit('    #define uECC_MSP430_MMOD_PUSHM "pushm.a"')
emit('    #define uECC_MSP430_MMOD_POPM "popm.a"')
emit("#else")
emit('    #define uECC_MSP430_MMOD_PUSHM "pushm.w"')
emit('    #define uECC_MSP430_MMOD_POPM "popm.w"')
emit("#endif")
CURVES = [
    ("uECC_SUPPORTS_secp160r1", p160),
    ("uECC_SUPPORTS_secp192r1", lambda: nist(
        "secp192r1", P192_COLS, False,
        "/* Computes result = product % p (secp192r1), constant time.\n"
        "   Column sums of t + s1 + s2 + s3, then the carry c is folded back using\n"
        "   2^192 = 2^64 + 1 (mod p), and p is subtracted once if needed. */")),
    ("uECC_SUPPORTS_secp224r1", lambda: nist(
        "secp224r1", P224_COLS, True,
        "/* Computes result = product % p (secp224r1), constant time.\n"
        "   Column sums of t + s1 + s2 - d1 - d2, then the carry c is folded back\n"
        "   using 2^224 = 2^96 - 1 (mod p), and p is added or subtracted once. */")),
    ("uECC_SUPPORTS_secp256r1", lambda: nist(
        "secp256r1", P256_COLS, True,
        "/* Computes result = product % p (secp256r1), constant time.\n"
        "   Column sums of t + 2 s1 + 2 s2 + s3 + s4 - d1 - d2 - d3 - d4, then the\n"
        "   carry c is folded back using 2^256 = 2^224 - 2^192 - 2^96 + 1 (mod p), and\n"
        "   p is added or subtracted once. */")),
]
for cond, fn in CURVES:
    emit("")
    emit(f"#if {cond}")
    fn()
    emit(f"#endif /* {cond} */")
emit("")
emit("#if uECC_SUPPORTS_secp256k1")
emit("#if (uECC_MSP430_HWMULT == 32)")
k1_mpy32()
emit("#elif (uECC_MSP430_HWMULT != 0)")
k1(True)
emit("#else")
k1(False)
emit("#endif")
emit("#endif /* uECC_SUPPORTS_secp256k1 */")
emit("")
emit("#endif /* _UECC_ASM_MSP430_MMOD_H_ */")
open([a for a in sys.argv[1:] if not a.startswith("--")][0], "w").write("\n".join(out) + "\n")
