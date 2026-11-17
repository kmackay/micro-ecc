#!/usr/bin/env python3
"""Generates asm_msp430_mult_square.inc: unrolled MSP430 (classic CPU) multiplication and
squaring for uECC_OPTIMIZATION_LEVEL >= 3.

usage: mult_msp430.py <output file>

Hardware multiplier (MPY16) code, product scanning. Each column k accumulates its products
in RESHI:RESLO: the first with MPY (which overwrites RES), the rest with MAC, adding the
carries out of RES (SUMEXT) to the column's carry word. At the end of the column RES is
added to a 3-word window of registers (w0, w1, w2), w0 is stored and the window shifts.
SLAU049 needs one instruction between an OP2 write and an indirect (@Rn) read of SUMEXT,
so each SUMEXT read is placed after the MAC write of the next product, or after the first
RES read at the end of the column.
"""
import sys

MPY, MAC, OP2 = "&0x0130", "&0x0134", "&0x0138"
RESLO, RESHI, SUMEXT = "&0x013A", "&0x013C", "&0x013E"


class Window:
    """3-word accumulator window (w0, w1, w2) with known-zero tracking."""

    def __init__(self, regs):
        self.r = list(regs)
        self.zero = [True, True, True]

    def rotate(self):
        self.r = self.r[1:] + self.r[:1]
        self.zero = self.zero[1:] + [True]


def adjust(e, reg, cur, want):
    """Moves a word pointer from word cur to word want."""
    d = 2 * (want - cur)
    if d > 0:
        e("add #%d, %s" % (d, reg))
    elif d < 0:
        e("sub #%d, %s" % (-d, reg))


def ps_mult(e, m, xop, ry, out, wregs, se, before=None):
    """Product scanning m x m multiply: X words from xop(i), Y streamed with @ry+ (ry
    points at y[0]; it is left pointing somewhere in Y), result word k stored to out(k).
    se is the SUMEXT operand ('@rN' with rN = 0x013E, or the absolute address).
    before(k), if given, is called at the start of column k."""
    win = Window(wregs)
    ypos = 0
    for k in range(2 * m - 1):
        if before:
            before(k)
        ihi, ilo = min(k, m - 1), max(0, k - m + 1)
        pairs = [(i, k - i) for i in range(ihi, ilo - 1, -1)]  # y ascending
        adjust(e, ry, ypos, pairs[0][1])
        w0, w1, w2 = win.r
        for idx, (i, j) in enumerate(pairs):
            e("mov %s, %s" % (xop(i), MPY if idx == 0 else MAC))
            if idx >= 2:  # carry of the previous MAC
                e("%s %s, %s" % ("mov" if win.zero[2] else "add", se, w2))
                win.zero[2] = False
            e("mov @%s+, %s" % (ry, OP2))
        ypos = pairs[-1][1] + 1
        last_se = len(pairs) >= 2
        if win.zero[0]:  # column 0
            e("mov %s, %s" % (RESLO, out(k)))
            e("mov %s, %s" % (RESHI, w1))
            win.zero[1] = False
            win.rotate()
            continue
        # (w2:w1:w0) += RES, + the carry of the last MAC
        e("add %s, %s" % (RESLO, w0))
        if last_se and win.zero[2]:  # mov keeps C
            e("mov %s, %s" % (se, w2))
            win.zero[2] = False
            last_se = False
        if win.zero[1]:
            e("mov %s, %s" % (RESHI, w1))
            e("adc %s" % w1)
            win.zero[1] = False
            carry = len(pairs) >= 2  # a single product has RESHI <= 0xFFFE
        else:
            e("addc %s, %s" % (RESHI, w1))
            carry = True
        if carry:
            if win.zero[2]:
                e("clr %s" % w2)
                win.zero[2] = False
            e("adc %s" % w2)
        if last_se:
            e("add %s, %s" % (se, w2))
        e("mov %s, %s" % (w0, out(k)))
        win.rotate()
    e("mov %s, %s" % (win.r[0], out(2 * m - 1)))


def ps_square(e, n, xop, ry, out, wregs, tc, se, before=None):
    """Product scanning squaring: per column, the cross products x_i * x_j (i < j) are
    summed in RES (their carries in tc), RES is added twice to the window and tc twice,
    then x_{k/2}^2 is added. X words from xop(i), also streamed with @ry+. before(k), if
    given, is called at the start of column k."""
    win = Window(wregs)

    def add_res(carry=True):
        w0, w1, w2 = win.r
        e("add %s, %s" % (RESLO, w0))
        if win.zero[1]:
            e("mov %s, %s" % (RESHI, w1))
            e("adc %s" % w1)
            win.zero[1] = False
        else:
            e("addc %s, %s" % (RESHI, w1))
        if carry:
            if win.zero[2]:
                e("clr %s" % w2)
                win.zero[2] = False
            e("adc %s" % w2)

    ypos = 0
    for k in range(2 * n - 1):
        if before:
            before(k)
        ilo = max(0, k - n + 1)
        pairs = [(i, k - i) for i in range(ilo, n) if i < k - i]
        pairs.reverse()  # y ascending
        tc_live = False
        if pairs:
            adjust(e, ry, ypos, pairs[0][1])
            for idx, (i, j) in enumerate(pairs):
                e("mov %s, %s" % (xop(i), MPY if idx == 0 else MAC))
                if idx >= 2:
                    e("%s %s, %s" % ("add" if tc_live else "mov", se, tc))
                    tc_live = True
                e("mov @%s+, %s" % (ry, OP2))
            ypos = pairs[-1][1] + 1
            add_res()
            if len(pairs) >= 2:  # not right after the OP2 write
                e("%s %s, %s" % ("add" if tc_live else "mov", se, tc))
                tc_live = True
            add_res()
            if tc_live:
                e("rla %s" % tc)
                e("add %s, %s" % (tc, win.r[2]))
        if k % 2 == 0:
            x = xop(k // 2)
            e("mov %s, %s" % (x, MPY))
            e("mov %s, %s" % (x, OP2))
            if win.zero[0]:  # column 0
                e("mov %s, %s" % (RESLO, out(0)))
                e("mov %s, %s" % (RESHI, win.r[1]))
                win.zero[1] = False
                win.rotate()
                continue
            add_res(carry=not (win.zero[1] and win.zero[2]))
        e("mov %s, %s" % (win.r[0], out(k)))
        win.rotate()
    e("mov %s, %s" % (win.r[0], out(2 * n - 1)))


# ---- Level 3: direct product scanning ----
# On entry r12 = result, r13 = left, r14 = right; r4..r11 and r15 are free.

class XCache:
    """Keeps some left words in registers: before each column, a register is reloaded
    when another word is used enough more often in this column and the next one (an
    indexed operand costs 2 cycles more than a register, a reload 3)."""

    def __init__(self, e, regs, uses):
        self.e, self.uses = e, uses
        self.reg = {}
        for i, r in enumerate(regs):
            e("mov %d(r13), %s" % (2 * i, r))
            self.reg[i] = r

    def before(self, k):
        u = {}
        for kk in (k, k + 1):
            for i in self.uses(kk):
                u[i] = u.get(i, 0) + 1
        want = sorted(u, key=lambda i: (-u[i], i))[:len(self.reg)]
        for old in sorted([i for i in self.reg if i not in want], key=lambda i: u.get(i, 0)):
            cand = [i for i in want if i not in self.reg]
            if not cand:
                break
            new = max(cand, key=lambda i: u[i])
            if 2 * (u[new] - u.get(old, 0)) > 3:
                r = self.reg.pop(old)
                self.e("mov %d(r13), %s" % (2 * new, r))
                self.reg[new] = r

    def op(self, i):
        return self.reg[i] if i in self.reg else "%d(r13)" % (2 * i)


def direct_mult(n):
    """r15 = SUMEXT, window r9..r11, up to 5 left words in r4..r8."""
    L = []
    e = L.append
    e("mov #0x013E, r15")

    def uses(k):
        return range(max(0, k - n + 1), min(k, n - 1) + 1) if k < 2 * n - 1 else []

    c = XCache(e, ["r4", "r5", "r6", "r7", "r8"], uses)
    ps_mult(e, n, c.op, "r14", lambda k: "%d(r12)" % (2 * k), ["r9", "r10", "r11"], "@r15",
            c.before)
    return L


def direct_square(n):
    """r15 = SUMEXT, window r9..r11, tc = r8, up to 4 left words in r4..r7, r14 streams."""
    L = []
    e = L.append
    e("mov r13, r14")
    e("mov #0x013E, r15")

    def uses(k):
        if k >= 2 * n - 1:
            return []
        u = [i for i in range(max(0, k - n + 1), n) if i < k - i]
        return u + ([k // 2, k // 2] if k % 2 == 0 else [])

    c = XCache(e, ["r4", "r5", "r6", "r7"], uses)
    ps_square(e, n, c.op, "r14", lambda k: "%d(r12)" % (2 * k), ["r9", "r10", "r11"], "r8",
              "@r15", c.before)
    return L


# ---- Software multiplication (no hardware multiplier) ----
# Subroutines inside one asm statement, called with r12 = out, r13 = a, r14 = b; they
# clobber r4..r15. Labels: 100 + n for the n-word Karatsuba step, 200 + w for leaves.
SWA = ["r4", "r5", "r6", "r7"]
SWH = ["r8", "r9", "r10", "r11"]


def sw_leaf(e, w, unroll):
    """w x w schoolbook multiply by rows: (H:L) = A * b[j] + H for the w-word multiplicand A
    (registers), with a constant-time shift-and-add over the bits of b[j] in L. Each step
    adds A to H if the low bit of L is set (bic with the inverted mask keeps C), then
    shifts (C:H:L) right; after 16 steps L = out[j] and H is the running sum."""
    A, H = SWA[:w], SWH[:w]
    e("%d:" % (200 + w))
    e("push r12")  # out at 2(r1)
    e("push r14")  # b at 0(r1)
    for i in range(w):
        e("mov %d(r13), %s" % (2 * i, A[i]))
        e("clr %s" % H[i])

    def step():
        e("bit #1, r12")    # C = low bit of L
        e("subc r13, r13")  # r13 = C - 1: 0 if the bit is set
        for i in range(w):
            e("mov %s, r14" % A[i])
            e("bic r13, r14")
            e("%s r14, %s" % ("add" if i == 0 else "addc", H[i]))
        for i in reversed(range(w)):
            e("rrc %s" % H[i])
        e("rrc r12")

    for j in range(w):
        e("mov @r1, r14")
        e("mov %d(r14), r12" % (2 * j))
        if unroll:
            for _ in range(16):
                step()
        else:
            e("mov #4, r15")
            e("1:")
            for _ in range(4):
                step()
            e("dec r15")
            e("jnz 1b")
        e("mov 2(r1), r14")
        e("mov r12, %d(r14)" % (2 * j))
    for i in range(w):
        e("mov %s, %d(r14)" % (H[i], 2 * (w + i)))
    e("add #4, r1")
    e("ret")


def sw_absdiff(e, src, m, h, mreg, treg):
    """r4.. = |src[0..m-1] - src[m..m+h-1]|, mreg = 0xFFFF if negative (else 0)."""
    R = (SWA + SWH)[:m]
    for i in range(m):
        e("mov %d(%s), %s" % (2 * i, src, R[i]))
        if i < h:
            e("%s %d(%s), %s" % ("sub" if i == 0 else "subc", 2 * m + 2 * i, src, R[i]))
        else:
            e("sbc %s" % R[i])
    e("subc %s, %s" % (mreg, mreg))  # C - 1: 0xFFFF on borrow
    for i in range(m):
        e("xor %s, %s" % (mreg, R[i]))
    e("mov %s, %s" % (mreg, treg))
    e("rra %s" % treg)
    for i in range(m):
        e("adc %s" % R[i])
    return R


def sw_karatsuba(e, n, sub):
    """One subtractive Karatsuba step on n words (m = ceil(n/2), h = n - m), calling
    sub(k) for the k-word products. Frame: |A_l - A_h|, |B_l - B_h| (m words each), Z1
    (2m words), N, and the saved out, a and b pointers."""
    m, h = (n + 1) // 2, n // 2
    DA, DB, Z1 = 0, 2 * m, 4 * m
    NS = 8 * m
    SO, SA, SB = NS + 2, NS + 4, NS + 6
    frame = NS + 8
    e("%d:" % (100 + n))
    e("sub #%d, r1" % frame)
    e("mov r12, %d(r1)" % SO)
    e("mov r13, %d(r1)" % SA)
    e("mov r14, %d(r1)" % SB)
    # L = A_l * B_l -> out[0 .. 2m-1]
    e("call #%s" % sub(m))
    # H = A_h * B_h -> out[2m .. 2n-1]
    e("mov %d(r1), r12" % SO)
    e("add #%d, r12" % (4 * m))
    e("mov %d(r1), r13" % SA)
    e("add #%d, r13" % (2 * m))
    e("mov %d(r1), r14" % SB)
    e("add #%d, r14" % (2 * m))
    e("call #%s" % sub(h))
    # |B_l - B_h| (mask Mb in r12), |A_l - A_h| (mask Ma in r13)
    e("mov %d(r1), r14" % SB)
    R = sw_absdiff(e, "r14", m, h, "r12", "r13")
    for i in range(m):
        e("mov %s, %d(r1)" % (R[i], DB + 2 * i))
    e("mov %d(r1), r14" % SA)
    R = sw_absdiff(e, "r14", m, h, "r13", "r15")
    for i in range(m):
        e("mov %s, %d(r1)" % (R[i], DA + 2 * i))
    e("xor r12, r13")
    e("inv r13")
    e("mov r13, %d(r1)" % NS)  # N = 0xFFFF if the signs are equal (subtract)
    # Z1 = |A_l - A_h| * |B_l - B_h|
    e("mov r1, r12")
    e("add #%d, r12" % Z1)
    e("mov r1, r13")
    e("add #%d, r13" % DA)
    e("mov r1, r14")
    e("add #%d, r14" % DB)
    e("call #%s" % sub(m))
    sw_combine(e, n, m, h, Z1, NS, SO)
    e("add #%d, r1" % frame)
    e("ret")


def sw_combine(e, n, m, h, Z1, NS, SO):
    """out[m..] += L + H - (-1)^s Z1: Z1 = (Z1 ^ N) + N_bit0 + L (carry kept in r12, as
    xor changes C), Z1 += H, then out[m..] += Z1 and the top word."""
    e("mov %d(r1), r15" % SO)
    e("mov %d(r1), r14" % NS)
    e("mov r14, r12")
    e("and #1, r12")
    for i in range(2 * m):
        e("mov %d(r1), r13" % (Z1 + 2 * i))
        e("xor r14, r13")
        e("rra r12")
        e("addc %d(r15), r13" % (2 * i))
        e("adc r12")
        e("mov r13, %d(r1)" % (Z1 + 2 * i))
    e("add r12, r14")  # top word of the middle term: N + carries
    for i in range(2 * h):
        e("%s %d(r15), %d(r1)" % ("add" if i == 0 else "addc", 4 * m + 2 * i, Z1 + 2 * i))
    for i in range(2 * h, 2 * m):
        e("adc %d(r1)" % (Z1 + 2 * i))
    e("adc r14")
    for i in range(2 * m):
        e("%s %d(r1), %d(r15)" % ("add" if i == 0 else "addc", Z1 + 2 * i, 2 * (m + i)))
    if 3 * m < 2 * n:
        e("addc r14, %d(r15)" % (2 * 3 * m))
        for i in range(3 * m + 1, 2 * n):
            e("adc %d(r15)" % (2 * i))


def sw_mult(n, unroll):
    """n-word multiply: Karatsuba steps down to leaves of at most 4 words, each a
    subroutine (the larger first, so that every call is a forward reference)."""
    need = []

    def sub(k):
        if k not in need:
            need.append(k)
        return "%df" % ((200 + k) if k <= 4 else (100 + k))

    L = []
    e = L.append
    e("call #%s" % sub(n))
    e("br #99f")
    done = 0
    while done < len(need):  # need grows while the steps are emitted
        k = need[done]
        done += 1
        if k > 4:
            sw_karatsuba(e, k, sub)
    for k in sorted([k for k in need if k <= 4], reverse=True):
        sw_leaf(e, k, unroll)
    e("99:")
    return L


def macro(name, lines):
    out = ["#define %s \\" % name]
    for l in lines:
        out.append('    "%s \\n\\t" \\' % l)
    out.append("")
    return "\n".join(out) + "\n"


HEADER = """/* Copyright 2026, Kenneth MacKay. Licensed under the BSD 2-clause license. */

/* Generated by scripts/mult_msp430.py - do not edit. */

/* Unrolled MSP430 multiply/square bodies for uECC_OPTIMIZATION_LEVEL >= 3. On entry
 * r12 = result, r13 = left (r14 = right for multiply); they use r4..r15.
 * Hardware multiplier (MPY16), product scanning with RES as the column accumulator:
 *   uECC_MSP430_MULT_N: up to 5 left words in registers (reloaded as the columns move
 *     on), right streamed with @r14+.
 *   uECC_MSP430_SQUARE_N: each column sums its cross products once and adds them twice.
 * Software multiplication (constant-time shift and add):
 *   uECC_MSP430_SWMULT_N: subtractive Karatsuba,
 *     A * B = L + W^m (L + H - (-1)^s |A_l - A_h| |B_l - B_h|) + W^2m H,
 *     down to 2..4-word schoolbook leaves that multiply the whole left operand (in
 *     registers) by one bit of right[j] per step. Uses a stack frame per level.
 *   uECC_MSP430_SWMULT_UNROLLED_N: the same with the bit loops of the leaves unrolled.
 */

"""

if __name__ == "__main__":
    s = HEADER
    for n in (10, 12, 14, 16):
        s += macro("uECC_MSP430_MULT_%d" % n, direct_mult(n))
        s += macro("uECC_MSP430_SQUARE_%d" % n, direct_square(n))
        s += macro("uECC_MSP430_SWMULT_%d" % n, sw_mult(n, False))
        s += macro("uECC_MSP430_SWMULT_UNROLLED_%d" % n, sw_mult(n, True))
    with open(sys.argv[1], "w") as f:
        f.write(s)
