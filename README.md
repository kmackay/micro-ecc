micro-ecc
==========

A small and fast ECDH, ECDSA, and EC-SDSA (Schnorr) implementation for 8-bit, 32-bit, and 64-bit processors.

The static version of micro-ecc (ie, where the curve was selected at compile-time) can be found in the "static" branch.

Features
--------

 * Resistant to known side-channel attacks.
 * Written in C, with optional GCC inline assembly for AVR, ARM and Thumb platforms.
 * Supports 8, 32, and 64-bit architectures.
 * Small code size.
 * No dynamic memory allocation.
 * Support for 5 standard curves: secp160r1, secp192r1, secp224r1, secp256r1, and secp256k1.
 * ECDSA sign and verify.
 * EC-SDSA (Schnorr) sign and verify — two variants (optimized and standard), each independently gated by a preprocessor macro.
 * Optional built-in SHA-256 and SHA-512 hash adapters (`uECC_hash.h` / `uECC_hash.c`), disabled by default so projects with an existing hash library incur zero extra code size.
 * BSD 2-clause license.

Usage Notes
-----------
### Point Representation ###
Compressed points are represented in the standard format as defined in http://www.secg.org/sec1-v2.pdf; uncompressed points are represented in standard format, but without the `0x04` prefix. All functions except `uECC_decompress()` only accept uncompressed points; use `uECC_compress()` and `uECC_decompress()` to convert between compressed and uncompressed point representations.

Private keys are represented in the standard format.

### Using the Code ###

I recommend just copying (or symlink) the uECC files into your project. Then just `#include "uECC.h"` to use the micro-ecc functions.

For EC-SDSA, also copy `uECC_hash.h` and `uECC_hash.c` if you want the built-in hash adapters (see [Hash Support](#hash-support) below).

For use with Arduino, you can use the Library Manager to download micro-ecc (**Sketch**=>**Include Library**=>**Manage Libraries**). You can then use uECC just like any other Arduino library (uECC should show up in the **Sketch**=>**Import Library** submenu).

See `uECC.h` for documentation for each function.

### EC-SDSA (Schnorr Signatures) ###

EC-SDSA is a Schnorr signature scheme. Two variants are provided; they differ only in what is hashed to produce the challenge value R:

| Variant | Hash input | API |
|---------|-----------|-----|
| Optimized | `Hash(x1 \|\| M)` — x-coordinate only | `uECC_ecsdsa_sign/verify_optimized` |
| Standard  | `Hash(x1 \|\| y1 \|\| M)` — both coordinates | `uECC_ecsdsa_sign/verify_standard` |

Both functions accept any hash algorithm through the `uECC_HashContext` interface, so they work with SHA-256, SHA-512, or any other hash you supply.

**Signature format:** `R || S` where `|R| = hash_context->result_size` and `|S| = curve->num_bytes`.

**Quick example (optimized variant, SHA-256):**

```c
#define uECC_SUPPORTS_SHA256 1
#include "uECC_hash.h"

uECC_SHA256_HashContext hctx;
uECC_SHA256_HashContext_init(&hctx);

uint8_t sig[32 + 32]; /* R(32) + S(32) for secp256r1 + SHA-256 */
uECC_ecsdsa_sign_optimized(private_key, message, message_len, &hctx.uECC, sig, curve);
uECC_ecsdsa_verify_optimized(public_key, message, message_len, &hctx.uECC, sig, curve);
```

### Hash Support ###

`uECC_hash.h` and `uECC_hash.c` provide self-contained SHA-256 and SHA-512 `uECC_HashContext` adapters. Both are **disabled by default** — projects that already include a hash library (mbed TLS, wolfCrypt, etc.) should implement the three `uECC_HashContext` callbacks against their existing library instead.

To enable the built-in implementations, define the relevant macro **before** including `uECC_hash.h`:

```c
#define uECC_SUPPORTS_SHA256 1   /* enable built-in SHA-256 */
#define uECC_SUPPORTS_SHA512 1   /* enable built-in SHA-512 */
#include "uECC_hash.h"
```

You must also compile `uECC_hash.c` (with the same macros defined) and link it into your project.

Each context type (`uECC_SHA256_HashContext`, `uECC_SHA512_HashContext`) embeds its own scratch buffer, so no heap allocation is required.

### Preprocessor Feature Gates ###

All features are individually enabled or disabled at compile time:

| Macro | Default | Controls |
|-------|---------|----------|
| `uECC_SUPPORTS_ECDSA` | `1` | `uECC_sign`, `uECC_sign_deterministic`, `uECC_verify` |
| `uECC_SUPPORTS_ECSDSA` | `1` | Master switch for both EC-SDSA variants |
| `uECC_SUPPORTS_ECSDSA_OPTIMIZED` | `uECC_SUPPORTS_ECSDSA` | Optimized variant only |
| `uECC_SUPPORTS_ECSDSA_STANDARD` | `uECC_SUPPORTS_ECSDSA` | Standard variant only |
| `uECC_SUPPORTS_SHA256` | `0` | Built-in SHA-256 adapter (in `uECC_hash.c`) |
| `uECC_SUPPORTS_SHA512` | `0` | Built-in SHA-512 adapter (in `uECC_hash.c`) |

Setting a macro to `0` strips the corresponding code entirely from the build, which is useful on flash-constrained targets.

### Compilation Notes ###

 * Should compile with any C/C++ compiler that supports stdint.h (this includes Visual Studio 2013).
 * If you want to change the defaults for any of the uECC compile-time options (such as `uECC_OPTIMIZATION_LEVEL`), you must change them in your Makefile or similar so that uECC.c is compiled with the desired values (ie, compile uECC.c with `-DuECC_OPTIMIZATION_LEVEL=3` or whatever).
 * When compiling for a Thumb-1 platform, you must use the `-fomit-frame-pointer` GCC option (this is enabled by default when compiling with `-O1` or higher).
 * When compiling for an ARM/Thumb-2 platform with `uECC_OPTIMIZATION_LEVEL` >= 3, you must use the `-fomit-frame-pointer` GCC option (this is enabled by default when compiling with `-O1` or higher).
 * When compiling for AVR, you must have optimizations enabled (compile with `-O1` or higher).
 * When building for Windows, you will need to link in the `advapi32.lib` system library.
