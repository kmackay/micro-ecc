#ifndef uECC_PLATFORM
    #if __AVR__
        #define uECC_PLATFORM uECC_avr
    #elif defined(__thumb2__) || defined(_M_ARMT) /* I think MSVC only supports Thumb-2 targets */
        #define uECC_PLATFORM uECC_arm_thumb2
    #elif defined(__thumb__)
        #define uECC_PLATFORM uECC_arm_thumb
    #elif defined(__arm__) || defined(_M_ARM)
        #define uECC_PLATFORM uECC_arm
    #elif defined(__i386__) || defined(_M_IX86) || defined(_X86_) || defined(__I86__)
        #define uECC_PLATFORM uECC_x86
    #elif defined(__amd64__) || defined(_M_X64)
        #define uECC_PLATFORM uECC_x86_64
    #else
        #define uECC_PLATFORM uECC_arch_other
    #endif
#endif

#ifndef uECC_WORD_SIZE
    #if uECC_PLATFORM == uECC_avr
        #define uECC_WORD_SIZE 1
    #elif (uECC_PLATFORM == uECC_x86_64)
        #define uECC_WORD_SIZE 8
    #else
        #define uECC_WORD_SIZE 4
    #endif
#endif

#if (uECC_WORD_SIZE != 1) && (uECC_WORD_SIZE != 4) && (uECC_WORD_SIZE != 8)
    #error "Unsupported value for uECC_WORD_SIZE"
#endif

#if (uECC_ASM && (uECC_PLATFORM == uECC_avr) && (uECC_WORD_SIZE != 1))
    #pragma message ("uECC_WORD_SIZE must be 1 when using AVR asm")
    #undef uECC_WORD_SIZE
    #define uECC_WORD_SIZE 1
#endif

#if (uECC_ASM && \
     (uECC_PLATFORM == uECC_arm || uECC_PLATFORM == uECC_arm_thumb) && \
     (uECC_WORD_SIZE != 4))
    #pragma message ("uECC_WORD_SIZE must be 4 when using ARM asm")
    #undef uECC_WORD_SIZE
    #define uECC_WORD_SIZE 4
#endif

#if defined(__SIZEOF_INT128__) || ((__clang_major__ * 100 + __clang_minor__) >= 302)
    #define SUPPORTS_INT128 1
#else
    #define SUPPORTS_INT128 0
#endif

#if (uECC_WORD_SIZE == 1)

typedef uint8_t uECC_word_t;
typedef uint16_t uECC_dword_t;
typedef uint8_t wordcount_t;
typedef int8_t swordcount_t;
typedef int16_t bitcount_t;
typedef int8_t cmpresult_t;

#define HIGH_BIT_SET 0x80
#define uECC_WORD_BITS 8
#define uECC_WORD_BITS_SHIFT 3
#define uECC_WORD_BITS_MASK 0x07

#elif (uECC_WORD_SIZE == 4)

typedef uint32_t uECC_word_t;
typedef uint64_t uECC_dword_t;
typedef unsigned wordcount_t;
typedef int swordcount_t;
typedef int bitcount_t;
typedef int cmpresult_t;

#define HIGH_BIT_SET 0x80000000
#define uECC_WORD_BITS 32
#define uECC_WORD_BITS_SHIFT 5
#define uECC_WORD_BITS_MASK 0x01F

#elif (uECC_WORD_SIZE == 8)

typedef uint64_t uECC_word_t;
#if SUPPORTS_INT128
typedef unsigned __int128 uECC_dword_t;
#endif
typedef unsigned wordcount_t;
typedef int swordcount_t;
typedef int bitcount_t;
typedef int cmpresult_t;

#define HIGH_BIT_SET 0x8000000000000000ull
#define uECC_WORD_BITS 64
#define uECC_WORD_BITS_SHIFT 6
#define uECC_WORD_BITS_MASK 0x03F

#endif /* uECC_WORD_SIZE */

#if (defined(_WIN32) || defined(_WIN64))
/* Windows */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <wincrypt.h>

static int default_RNG(uint8_t *dest, unsigned size) {
    HCRYPTPROV prov;
    if (!CryptAcquireContext(&prov, NULL, NULL, PROV_RSA_FULL, CRYPT_VERIFYCONTEXT)) {
        return 0;
    }

    CryptGenRandom(prov, size, (BYTE *)dest);
    CryptReleaseContext(prov, 0);
    return 1;
}

#elif defined(unix) || defined(__linux__) || defined(__unix__) || defined(__unix) || \
    (defined(__APPLE__) && defined(__MACH__)) || defined(uECC_POSIX)

/* Some POSIX-like system with /dev/urandom or /dev/random. */
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

#ifndef O_CLOEXEC
    #define O_CLOEXEC 0
#endif

static int default_RNG(uint8_t *dest, unsigned size) {
    int fd = open("/dev/urandom", O_RDONLY | O_CLOEXEC);
    if (fd == -1) {
        fd = open("/dev/random", O_RDONLY | O_CLOEXEC);
        if (fd == -1) {
            return 0;
        }
    }
    
    char *ptr = (char *)dest;
    size_t left = size;
    while (left > 0) {
        ssize_t bytes_read = read(fd, ptr, left);
        if (bytes_read <= 0) { // read failed
            close(fd);
            return 0;
        }
        left -= bytes_read;
        ptr += bytes_read;
    }
    
    close(fd);
    return 1;
}

#else /* Some other platform */

static int default_RNG(uint8_t *dest, unsigned size) {
    return 0;
}

#endif

// #ifdef __GNUC__ /* Only support GCC inline asm for now */
//     #if (uECC_ASM && (uECC_PLATFORM == uECC_avr))
//         #include "asm_avr.inc"
//     #endif
// 
//     #if (uECC_ASM && (uECC_PLATFORM == uECC_arm || uECC_PLATFORM == uECC_arm_thumb || \
//                       uECC_PLATFORM == uECC_arm_thumb2))
//         #include "asm_arm.inc"
//     #endif
// #endif
