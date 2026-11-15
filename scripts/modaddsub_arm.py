#!/usr/bin/env python3
"""Generates asm_arm_modaddsub.inc: fused constant-time uECC_vli_modAdd /
uECC_vli_modSub for Thumb-2 and ARM (5..8 words). Used at optimization level 4
(see asm_arm.inc): about 1 KB larger than the generic C versions."""
import sys

out = []
def emit(s=""):
    out.append(s)

def R(n):
    return [f"r{4 + i}" for i in range(n)]

def rl(regs):
    """Register list for ldm/stm (must be ascending)."""
    key = lambda r: 14 if r == "lr" else int(r[1:])
    return "{" + ", ".join(sorted(regs, key=key)) + "}"

def sortregs(regs):
    key = lambda r: 14 if r == "lr" else int(r[1:])
    return sorted(regs, key=key)

def pass1(n, op):
    """result regs = left op right, carry chain left in the flags.
    Returns lines. Uses r1 (left), r2 (right)."""
    Rg = R(n)
    lines = [f"ldmia r1!, {rl(Rg[:4])}",
             f"ldmia r2!, {rl(['r8', 'r9', 'r10', 'r11'])}"]
    first, rest = ("adds", "adcs") if op == "add" else ("subs", "sbcs")
    for i, t in enumerate(["r8", "r9", "r10", "r11"]):
        lines.append(f"{first if i == 0 else rest} {Rg[i]}, {Rg[i]}, {t}")
    k = n - 4
    pool = [f"r{4 + n + i}" for i in range(8 - n)] + ["r12", "lr", "r1", "r2"]
    temps = sortregs(pool[:k])
    if k == 1:
        lines += [f"ldr {Rg[4]}, [r1]", f"ldr {temps[0]}, [r2]"]
    else:
        lines += [f"ldmia r1, {rl(Rg[4:])}", f"ldmia r2, {rl(temps)}"]
    for i in range(k):
        lines.append(f"{rest} {Rg[4 + i]}, {Rg[4 + i]}, {temps[i]}")
    return lines

def mod_pass(n, op, temps, mask=None):
    """result regs op= (mod [& mask]), chaining from a fresh carry. Loads mod
    via r3 (advanced). Returns lines."""
    Rg = R(n)
    lines = []
    first, rest = ("adds", "adcs") if op == "add" else ("subs", "sbcs")
    i = 0
    while i < n:
        k = min(len(temps), n - i)
        ts = sortregs(temps[:k])
        if k == 1:
            lines.append(f"ldr {ts[0]}, [r3], #4")
        else:
            lines.append(f"ldmia r3!, {rl(ts)}")
        if mask:
            lines += [f"and {t}, {t}, {mask}" for t in ts]
        for j, t in enumerate(ts):
            lines.append(f"{first if i + j == 0 else rest} {Rg[i + j]}, {Rg[i + j]}, {t}")
        i += k
    return lines

def free_after(n, exclude):
    pool = ["r1", "r2", "r12", "lr"] + [f"r{4 + n + i}" for i in range(8 - n)]
    return [r for r in pool if r not in exclude]

def modsub(n):
    m = "r12"
    lines = pass1(n, "sub")
    lines += [f"sbc {m}, {m}, {m}",             # m = -borrow
              "/* result += mod & m */"]
    lines += mod_pass(n, "add", free_after(n, [m]), m)
    lines.append(f"stmia r0, {rl(R(n))}")
    return lines

def modadd(n):
    c = "r12"
    lines = pass1(n, "add")
    lines += [f"sbc {c}, {c}, {c}",             # c = carry - 1
              "/* result -= mod */"]
    temps = free_after(n, [c])
    lines += mod_pass(n, "sub", temps)
    b = "r1"
    lines += [f"sbc {b}, {b}, {b}",             # b = -borrow
              f"and {b}, {b}, {c}",             # add mod back iff borrow and no carry
              f"sub r3, r3, #{4 * n}",
              "/* result += mod & mask */"]
    lines += mod_pass(n, "add", [t for t in temps if t != b], b)
    lines.append(f"stmia r0, {rl(R(n))}")
    return lines


def asm_block(lines, indent="        "):
    res = [f'{indent}__asm__ volatile (', f'{indent}    ".syntax unified \\n\\t"']
    for l in lines:
        if l.startswith("/*"):
            res.append(f"{indent}    {l}")
        else:
            res.append(f'{indent}    "{l} \\n\\t"')
    res += [f"{indent}    RESUME_SYNTAX",
            f'{indent}    : "+r" (r0), "+r" (r1), "+r" (r2), "+r" (r3)',
            f"{indent}    :",
            f'{indent}    : "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r14", "cc", "memory"',
            f"{indent});"]
    return res

SIZES = [(8, "(uECC_SUPPORTS_secp256r1 || uECC_SUPPORTS_secp256k1)"),
         (7, "uECC_SUPPORTS_secp224r1"),
         (6, "(uECC_SUPPORTS_secp160r1 || uECC_SUPPORTS_secp192r1)"),
         (5, "uECC_SUPPORTS_secp160r1")]

def func(name, gen, comment):
    emit(comment)
    emit(f"uECC_VLI_API void uECC_vli_{name}(uECC_word_t *result,")
    emit(" " * (len("uECC_VLI_API void uECC_vli_(") + len(name)) + "const uECC_word_t *left,")
    emit(" " * (len("uECC_VLI_API void uECC_vli_(") + len(name)) + "const uECC_word_t *right,")
    emit(" " * (len("uECC_VLI_API void uECC_vli_(") + len(name)) + "const uECC_word_t *mod,")
    emit(" " * (len("uECC_VLI_API void uECC_vli_(") + len(name)) + "wordcount_t num_words) {")
    emit('    register uECC_word_t *r0 __asm__("r0") = result;')
    emit('    register const uECC_word_t *r1 __asm__("r1") = left;')
    emit('    register const uECC_word_t *r2 __asm__("r2") = right;')
    emit('    register const uECC_word_t *r3 __asm__("r3") = mod;')
    for n, cond in SIZES:
        emit(f"#if {cond}")
        emit(f"#if (uECC_MIN_WORDS < {n})")
        emit(f"    if (num_words == {n})")
        emit("#endif")
        emit("    {")
        for l in asm_block(gen(n)):
            emit(l)
        emit("        return;")
        emit("    }")
        emit("#endif")
    emit("}")
    emit(f"#define asm_{name} 1")
    emit("")

emit("/* Copyright 2015, Kenneth MacKay. Licensed under the BSD 2-clause license. */")
emit("")
emit("/* Fused constant-time modular addition / subtraction for Thumb-2 and ARM.")
emit("   Generated by scripts/modaddsub_arm.py. The smallest supported size")
emit("   (uECC_MIN_WORDS) is handled unconditionally as the last case. */")
emit("")
emit("#ifndef _UECC_ASM_ARM_MODADDSUB_H_")
emit("#define _UECC_ASM_ARM_MODADDSUB_H_")
emit("")
COMMENT_ADD = ("/* Computes result = (left + right) % mod in constant time: result = left +\n"
               "   right, then result -= mod, then mod is added back if the sum was < mod (the\n"
               "   subtraction borrowed and the addition did not carry).\n"
               "   Assumes that left < mod and right < mod, and that result does not overlap\n"
               "   mod. */")
COMMENT_SUB = ("/* Computes result = (left - right) % mod in constant time: result = left -\n"
               "   right, then (mod & -borrow) is added back.\n"
               "   Assumes that left < mod and right < mod, and that result does not overlap\n"
               "   mod. */")
func("modAdd", modadd, COMMENT_ADD)
func("modSub", modsub, COMMENT_SUB)
emit("#endif /* _UECC_ASM_ARM_MODADDSUB_H_ */")
open(sys.argv[1], "w").write("\n".join(out) + "\n")
