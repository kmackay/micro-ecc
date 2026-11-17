micro-ecc
==========

A small and fast ECDH and ECDSA implementation for 8-bit, 32-bit, and 64-bit processors.

The static version of micro-ecc (ie, where the curve was selected at compile-time) can be found in the "static" branch.

Features
--------

 * Resistant to known side-channel attacks.
 * Written in C, with optional GCC inline assembly for AVR, ARM and Thumb platforms.
 * Supports 8, 32, and 64-bit architectures.
 * Small code size.
 * No dynamic memory allocation.
 * Support for 5 standard curves: secp160r1, secp192r1, secp224r1, secp256r1, and secp256k1.
 * BSD 2-clause license.

Usage Notes
-----------
### Point Representation ###
Compressed points are represented in the standard format as defined in http://www.secg.org/sec1-v2.pdf; uncompressed points are represented in standard format, but without the `0x04` prefix. All functions except `uECC_decompress()` only accept uncompressed points; use `uECC_compress()` and `uECC_decompress()` to convert between compressed and uncompressed point representations.

Private keys are represented in the standard format.

### Using the Code ###

I recommend just copying (or symlink) the uECC files into your project. Then just `#include "uECC.h"` to use the micro-ecc functions.

For use with Arduino, you can use the Library Manager to download micro-ecc (**Sketch**=>**Include Library**=>**Manage Libraries**). You can then use uECC just like any other Arduino library (uECC should show up in the **Sketch**=>**Import Library** submenu).

See uECC.h for documentation for each function.

### Optimization Levels ###

`uECC_OPTIMIZATION_LEVEL` (0 - 4, default 2) trades code size for speed. Each level produces code that is larger and faster than the level below it:

 * **0** - Smallest code. Uses generic (bit-by-bit) modular reduction for all arithmetic; unusably slow for most applications.
 * **1** - Adds curve-specific fast reductions written in C. On non-ARM platforms, also adds Montgomery multiplication for the ECDSA arithmetic mod n and, on 16-bit, 32-bit and 64-bit platforms, constant-time (safegcd) modular inversion. On ARM, level 1 favors code size: it keeps the generic mod n arithmetic and the binary extended Euclidean inversion, uses smaller (variable-time) C reductions, and on Thumb-1 uses a compact assembly multiply.
   * On MSP430 and MSP430X, adds assembly loops for addition, subtraction, right shift, multiplication and squaring (product scanning with the hardware multiplier, on 32-bit digits with MPY32, or constant-time software multiplication).
 * **2** - On AVR, adds unrolled assembly addition/subtraction and constant-time assembly reductions. On ARM, adds constant-time assembly reductions and Montgomery multiplication mod n; on Thumb-2 and ARM it also adds constant-time (safegcd) inversion and unrolled assembly addition/subtraction and right shift.
   * On MSP430, adds unrolled assembly addition, subtraction and right shift, and constant-time assembly reductions.
 * **3** - Adds unrolled assembly multiplication and squaring on AVR, Thumb-2 and ARM. On Thumb-1, adds unrolled assembly addition/subtraction and right shift.
   * On MSP430, adds assembly multiplication for each curve size: unrolled product scanning with the hardware multiplier (16-bit digits with MPY16; 32-bit digits with MPY32; squaring computes the cross products once), or Karatsuba multiplication with software multiplication.
 * **4** - Adds size-specialized assembly multiplication and squaring. On Thumb-2 and ARM, also inlines addition/subtraction and uses fused constant-time modular addition/subtraction. On AVR, uses fully unrolled multi-level Karatsuba multiplication for 192 and 256-bit curves (instead of 1-level Karatsuba with a shared sub-multiply).
   * On MSP430 without a hardware multiplier, unrolls the software multiplication loops.

### Compilation Notes ###

 * Should compile with any C/C++ compiler that supports stdint.h (this includes Visual Studio 2013).
 * If you want to change the defaults for any of the uECC compile-time options (such as `uECC_OPTIMIZATION_LEVEL`), you must change them in your Makefile or similar so that uECC.c is compiled with the desired values (ie, compile uECC.c with `-DuECC_OPTIMIZATION_LEVEL=3` or whatever).
 * `uECC_WORD_SIZE` (1, 2, 4 or 8 bytes) is set automatically based on the platform if not defined; 2 (16-bit words) is intended for 16-bit platforms.
 * `uECC_SAFEGCD` selects the modular inversion algorithm: 0 for the binary extended Euclidean algorithm (variable-time), 1 for constant-time safegcd with 30-bit limbs (needs a fast 32x32->64-bit multiply; not supported with avr-gcc), or 2 for constant-time safegcd with 14-bit limbs (only needs 16x16->32-bit multiplies; slower than 0 on AVR and Thumb-1, but constant-time). The default is 1, except at optimization level 0, at level 1 on ARM, and on AVR and Thumb-1, where it is 0, and with `uECC_WORD_SIZE` 2, where it is 2.
 * When compiling for a Thumb-1 platform, you must use the `-fomit-frame-pointer` GCC option (this is enabled by default when compiling with `-O1` or higher).
 * When compiling for an ARM/Thumb-2 platform with `uECC_OPTIMIZATION_LEVEL` >= 2, you must use the `-fomit-frame-pointer` GCC option (this is enabled by default when compiling with `-O1` or higher).
 * When compiling for AVR, you must have optimizations enabled (compile with `-O1` or higher).
 * On MSP430 (classic CPU and MSP430X, small and large memory models), `uECC_MSP430_HWMULT` selects the hardware multiplier (32 for MPY32, 16 for MPY16) or constant-time software multiplication (0). By default it is detected from the device header selected with `-mmcu` (0 if there is no device header, e.g. with only `-mcpu`); GCC's `-mhwmult` option does not affect it, so to avoid the hardware multiplier on a device that has one, define `uECC_MSP430_HWMULT=0`. The multiplier address (`uECC_MSP430_MPY_BASE`) also comes from the device header by default. `uECC_MSP430_DISABLE_INTERRUPTS` (default 1) disables interrupts while uECC uses the hardware multiplier. See uECC.h for details. When compiling the MSP430 assembly with clang, use `-fno-integrated-as` (clang's integrated MSP430 assembler rejects some valid instructions); clang does not support the MSP430X large memory model (`-mlarge`).
 * On MSP430, the uECC functions need the following amount of stack (peak over `uECC_make_key()`, `uECC_shared_secret()`, `uECC_sign()` and `uECC_verify()`, measured with msp430-elf-gcc 9.3 `-Os`; the largest enabled curve determines the usage):

   | | secp160r1 | 256-bit curves |
   |---|---|---|
   | MSP430 and MSP430X small model, levels 1-4 | 660 bytes | 770 bytes (800 bytes at levels 3 and 4 without a hardware multiplier) |
   | MSP430X large model (`-mlarge`), levels 1-4 | 760 bytes | 870 bytes (920 bytes at levels 3 and 4 without a hardware multiplier) |
   | Level 0 | 670 bytes (810 large) | 840 bytes (970 large) |

   Add the stack needed by your own code (including buffers passed to uECC) and by interrupt handlers. Devices with 512 bytes of RAM or less are too small.
 * When building for Windows, you will need to link in the `advapi32.lib` system library.
