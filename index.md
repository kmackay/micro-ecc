---
title: micro-ecc
layout: default
---

micro-ecc
==========

A small ECDH and ECDSA implementation for 32-bit microcontrollers. See [easy-ecc](https://github.com/kmackay/easy-ecc) for a fast and secure pure-C implementation for \*nix and Windows.

Features
--------

 * Resistant to known side-channel attacks.
 * Written in C, with optional inline assembly for ARM and Thumb platforms.
 * Small code size: ECDH in as little as 2KB, ECDH + ECDSA in as little as 3KB when compiled for Thumb (eg, Cortex-M0).
 * No dynamic memory allocation.
 * Reasonably fast: on an LPC1114 at 48MHz (ARM Cortex-M0, 32-cycle 32x32 bit multiply), 192-bit ECDH shared secret calculation takes as little as ~175ms (depending on selected optimizations).
 * Support for 4 standard curves: secp128r1, secp192r1, secp256r1, and secp384r1
 * BSD 2-clause license.

Usage Notes
-----------

#### Integer Representation ####

To reduce code size, all large integers are represented using little-endian words - so the least significant word is first. For example, the standard representation of the prime modulus for the curve secp128r1 is `FFFFFFFD FFFFFFFF FFFFFFFF FFFFFFFF`; in micro-ecc, this would be represented as `uint32_t p[4] = {0xffffffff, 0xffffffff, 0xffffffff, 0xfffffffd};`.

You can use the `ecc_bytes2native()` and `ecc_native2bytes()` functions to convert between the native integer representation and the standardized octet representation.

#### Generating Keys ####

You can use the `makekeys` program in the `apps` directory to generate keys (on Linux or OS X). You can run `make` in that directory to build for your native platform (or use [emk](http://kmackay.ca/emk)). To generate a single public/private key pair, run `makekeys`. It will print out the public and private keys in a representation suitable to be copied into your source code. You can generate multiple key pairs at once using `makekeys <n>` to generate n keys.

#### Using the Code ####

I recommend just copying (or symlink) ecc.h and ecc.c into your project. Then just `#include "ecc.h"` to use the micro-ecc functions.

See ecc.h for documentation for each function.

Speed and Size
--------------

Available optimizations are:
 * `ECC_SQUARE_FUNC` - Use a separate function for squaring.
 * `ECC_ASM` - Choose the type of inline assembly to use. The available options are `ecc_asm_none`, `ecc_asm_thumb`, `ecc_asm_thumb2`, and `ecc_asm_arm`.

All tests were performed on an LPC1114 running at 48MHz. The listed code sizes include all code and data required by the micro-ecc library (including `aebi_lmul` when not using assembly),
but do not include the sizes of the code using the library functions.

The following compiler options were used (using gcc 4.8):
 * Compile: `-mcpu=cortex-m0 -mthumb -ffunction-sections -fdata-sections -Os`
 * Link: `-mcpu=cortex-m0 -mthumb -nostartfiles -nostdlib -Wl,--gc-sections`

### Effect of optimization settings ###

These tests were performed using the curve secp192r1. Only ECDH code was used (no ECDSA). When enabled, ECC_ASM was defined to `ecc_asm_thumb`.

<table>
	<tr>
		<th>Optimizations</th>
		<th>ECDH time (ms)</th>
		<th>Code size (bytes)</th>
	</tr>
	<tr>
		<td>none</td>
		<td>438.9</td>
		<td>2212</td>
	</tr>
	<tr>
		<td>ECC_SQUARE_FUNC</td>
		<td>406.7</td>
		<td>2412</td>
	</tr>
	<tr>
		<td>ECC_ASM</td>
		<td>186.3</td>
		<td>2036</td>
	</tr>
	<tr>
		<td>both</td>
		<td>175.7</td>
		<td>2170</td>
	</tr>
</table>

### ECDH for different curves ###

In these tests, `ECC_ASM` was defined to `ecc_asm_thumb` and `ECC_SQUARE_FUNC` was defined to `1` in all cases.

<table>
	<tr>
		<th></th>
		<th>secp128r1</th>
		<th>secp192r1</th>
		<th>secp256r1</th>
		<th>secp384r1</th>
	</tr>
	<tr>
		<td><em>ECDH time (ms):</em></td>
		<td>89.9</td>
		<td>175.7</td>
		<td>465.1</td>
		<td>1370.3</td>
	</tr>
	<tr>
		<td><em>Code size (bytes):</em></td>
		<td>2324</td>
		<td>2170</td>
		<td>2512</td>
		<td>2244</td>
	</tr>
</table>

### ECDSA speed and combined code size ###

In these tests, the measured speed is the time to verify an ECDSA signature. The measured code size is the combined code size for ECDH and ECDSA. `ECC_ASM` was defined to `ecc_asm_thumb` and `ECC_SQUARE_FUNC` was defined to `1` in all cases.

<table>
	<tr>
		<th></th>
		<th>secp128r1</th>
		<th>secp192r1</th>
		<th>secp256r1</th>
		<th>secp384r1</th>
	</tr>
	<tr>
		<td><em>ECDSA verify time (ms):</em></td>
		<td>106.7</td>
		<td>217.1</td>
		<td>555.2</td>
		<td>1576.1</td>
	</tr>
	<tr>
		<td><em>Code size (bytes):</em></td>
		<td>3138</td>
		<td>3014</td>
		<td>3334</td>
		<td>3158</td>
	</tr>
</table>

### Maximum stack usage ###

In these tests, `ECC_ASM` was defined to `ecc_asm_thumb` and `ECC_SQUARE_FUNC` was defined to `1` in all cases. The table values are the maximum possible stack usage for each function, in bytes.

<table>
	<tr>
		<th></th>
		<th>secp128r1</th>
		<th>secp192r1</th>
		<th>secp256r1</th>
		<th>secp384r1</th>
	</tr>
	<tr>
		<td><em>ecc_make_key()</em></td>
		<td>336</td>
		<td>416</td>
		<td>528</td>
		<td>728</td>
	</tr>
	<tr>
		<td><em>ecc_valid_public_key()</em></td>
		<td>192</td>
		<td>232</td>
		<td>304</td>
		<td>424</td>
	</tr>
	<tr>
		<td><em>ecdh_shared_secret()</em></td>
		<td>360</td>
		<td>456</td>
		<td>584</td>
		<td>816</td>
	</tr>
	<tr>
		<td><em>ecdsa_sign()</em></td>
		<td>400</td>
		<td>504</td>
		<td>640</td>
		<td>888</td>
	</tr>
	<tr>
		<td><em>ecdsa_verify()</em></td>
		<td>400</td>
		<td>520</td>
		<td>672</td>
		<td>952</td>
	</tr>
</table>
