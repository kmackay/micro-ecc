micro-ecc
==========

A small and fast ECDH and ECDSA implementation for 8-bit, 32-bit, and 64-bit processors.

The old version of micro-ecc can be found in the "old" branch.

Features
--------

 * Resistant to known side-channel attacks.
 * Written in C, with optional GCC inline assembly for AVR, ARM and Thumb platforms.
 * Supports 8, 32, and 64-bit architectures.
 * Small code size.
 * No dynamic memory allocation.
 * Support for 4 standard curves: secp160r1, secp192r1, secp256r1, and secp256k1.
 * BSD 2-clause license.

Usage Notes
-----------
#### Point Representation ####
Compressed points are represented in the standard format as defined in http://www.secg.org/collateral/sec1_final.pdf; uncompressed points are represented in standard format, but without the `0x04` prefix. `uECC_make_key()`, `uECC_shared_secret()`, `uECC_sign()`, and `uECC_verify()` only handle uncompressed points; you can use `uECC_compress()` and `uECC_decompress()` to convert between compressed and uncompressed point representations.

Private keys are represented in the standard format.

#### Using the Code ####

I recommend just copying (or symlink) uECC.h, uECC.c, and the appropriate asm\_&lt;arch&gt;\_.inc (if any) into your project. Then just `#include "uECC.h"` to use the micro-ecc functions.

See uECC.h for documentation for each function.

#### Compilation Notes ####

 * Should compile with any C/C++ compiler that supports stdint.h (this includes Visual Studio 2013).
 * When compiling for a Thumb-1 platform with inline assembly enabled (ie, `uECC_ASM` is defined to `uECC_asm_small` or `uECC_asm_fast`), you must use the `-fomit-frame-pointer` GCC option (this is enabled by default when compiling with `-O1` or higher).
 * When compiling for AVR with inline assembly enabled, you must have optimizations enabled (compile with `-O1` or higher).
 * When building on Windows, you will need to link in the `advapi32.lib` system library.
