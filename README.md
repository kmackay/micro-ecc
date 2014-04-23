micro-ecc
==========

A small ECDH and ECDSA implementation for 8-bit, 32-bit, and 64-bit processors.

Features
--------

 * Resistant to known side-channel attacks.
 * Written in C, with optional inline assembly for AVR, ARM and Thumb platforms.
 * Support for 3 standard curves: secp160r1, secp192r1, and secp256r1.
 * BSD 2-clause license.

Usage Notes
-----------
#### Using the Code ####

I recommend just copying (or symlink) uECC.h and uECC.c into your project. Then just `#include "uECC.h"` to use the micro-ecc functions.

See uECC.h for documentation for each function.

#### Compilation Notes ####

 * When compiling for a Thumb-1 platform with inline assembly enabled (ie, `uECC_ASM` is defined to `uECC_asm_small` or `uECC_asm_fast`), you must use the `-fomit-frame-pointer` GCC option (this is enabled by default when compiling with `-O1` or higher).
 * When compiling for AVR with `uECC_ASM` defined to `uECC_asm_fast`, you must have optimizations enabled (compile with `-O1` or higher).
