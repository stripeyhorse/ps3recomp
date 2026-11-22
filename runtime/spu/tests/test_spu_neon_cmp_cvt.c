/*
 * The vector float compares (fceq, fcgt, fcmeq, fcmgt) and int<->float conversions
 * (cflts, cfltu, csflt, cuflt) against their scalar references (spu_*_ref in
 * spu_helpers.h), lane for lane. On ARM64 that is the NEON forms; on x86 the SSE2
 * compares and the AVX2 or reference conversions; with SPU_SCALAR_HELPERS the
 * references themselves.
 *
 * Inputs: every pair of a table of special words (both zeros, denormals, the
 * normal/extended-range boundaries, exponent 255), random words drawn from every
 * exponent band, integers of every magnitude, and every scale immediate. On ARM64
 * everything runs once in the default FPCR and once under round toward zero with
 * flush-to-zero, the mode SPU_FLOAT_RZ runs lifted code in. Prints ns per vector
 * for the vector form and for the reference.
 *
 *   cc -std=gnu17 -O2 -I include -I runtime/spu runtime/spu/tests/test_spu_neon_cmp_cvt.c -o t && ./t
 */
#include "spu_helpers.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static uint64_t s_rng = 0xD1B54A32D192ED03ull;
static uint32_t rnd(void) { s_rng ^= s_rng << 13; s_rng ^= s_rng >> 7; s_rng ^= s_rng << 17; return (uint32_t)(s_rng >> 16); }

static const uint32_t k_special[] = {
    0x00000000u, 0x80000000u, 0x00000001u, 0x80000001u, 0x007FFFFFu, 0x807FFFFFu,
    0x00800000u, 0x80800000u, 0x00800001u, 0x80800001u, 0x3F800000u, 0xBF800000u,
    0x3F7FFFFFu, 0xBF7FFFFFu, 0x4F000000u, 0xCF000000u, 0x4F7FFFFFu, 0x4F800000u,
    0x7F7FFFFFu, 0xFF7FFFFFu, 0x7F800000u, 0xFF800000u, 0x7F800001u, 0xFF800001u,
    0x7FC00000u, 0xFFC00000u, 0x7FFFFFFFu, 0xFFFFFFFFu, 0x5E800000u, 0x2F800000u,
};
#define NSPECIAL ((int)(sizeof k_special / sizeof k_special[0]))
static const uint32_t k_special_int[] = {
    0u, 1u, 2u, 3u, 0x7Fu, 0x80u, 0xFFFFFFu, 0x1000000u, 0x1000001u, 0x1FFFFFFu,
    0x7FFFFFFFu, 0x80000000u, 0x80000001u, 0xFFFFFFFFu, 0xFFFFFFFEu, 0xFF000000u,
};
#define NSPECIAL_INT ((int)(sizeof k_special_int / sizeof k_special_int[0]))

static uint32_t rnd_float(void)
{
    const uint32_t sign = (rnd() & 1u) << 31, man = rnd() & 0x7FFFFFu;
    uint32_t e;
    switch (rnd() % 10) {
    case 0: e = 0; break;                                  /* zero / denormal */
    case 1: e = 1 + rnd() % 4; break;                      /* bottom of the normal range */
    case 2: e = 248 + rnd() % 8; break;                    /* top, into extended range */
    case 3: return rnd() ^ (rnd() << 16);                  /* anything */
    case 4: return sign;                                   /* +-0 */
    case 5: return k_special[rnd() % NSPECIAL];
    default: e = 100 + rnd() % 60; break;                  /* ordinary, around the scales */
    }
    return sign | (e << 23) | man;
}
static uint32_t rnd_int(void)
{
    switch (rnd() % 7) {
    case 0: return 0;
    case 1: return 0x80000000u >> (rnd() % 32);
    case 2: return 0xFFFFFFFFu >> (rnd() % 32);
    case 3: return (uint32_t)-(int32_t)((rnd() ^ (rnd() << 16)) >> (rnd() % 32));
    case 4: return k_special_int[rnd() % NSPECIAL_INT];
    default: return (rnd() ^ (rnd() << 16)) >> (rnd() % 32);
    }
}

static unsigned long long g_n, g_bad;
static void cmp(const char* what, int i8, u128 a, u128 b, u128 got, u128 want)
{
    g_n++;
    if (!memcmp(&got, &want, 16)) return;
    if (g_bad++ < 16)
        fprintf(stderr, "FAIL %s i8=%d a=%08X %08X %08X %08X b=%08X %08X %08X %08X\n"
                "  got  %08X %08X %08X %08X\n  want %08X %08X %08X %08X\n", what, i8,
                a._u32[0], a._u32[1], a._u32[2], a._u32[3], b._u32[0], b._u32[1], b._u32[2], b._u32[3],
                got._u32[0], got._u32[1], got._u32[2], got._u32[3],
                want._u32[0], want._u32[1], want._u32[2], want._u32[3]);
}
#define C2(op, a, b) cmp(#op, 0, a, b, spu_##op(a, b), spu_##op##_ref(a, b))
#define CI(op, a, i) cmp(#op, i, a, a, spu_##op(a, i), spu_##op##_ref(a, i))

static void compares(u128 a, u128 b)
{
    C2(fceq, a, b); C2(fcgt, a, b); C2(fcmeq, a, b); C2(fcmgt, a, b);
    C2(fcgt, b, a); C2(fcmgt, b, a);
}
static void conversions(u128 f, u128 n, int i8)
{
    CI(cflts, f, i8); CI(cfltu, f, i8); CI(csflt, n, i8); CI(cuflt, n, i8);
}

static void run(void)
{
    /* every ordered pair of special words, rotated through the four lanes */
    for (int i = 0; i < NSPECIAL; i++)
        for (int j = 0; j < NSPECIAL; j++) {
            u128 a, b;
            for (int k = 0; k < 4; k++) {
                a._u32[k] = k_special[(i + k) % NSPECIAL];
                b._u32[k] = k_special[(j + 3 * k) % NSPECIAL];
            }
            a._u32[0] = k_special[i]; b._u32[0] = k_special[j];
            compares(a, b);
        }
    /* every immediate against every special word and integer, and a random spread */
    for (int i8 = 0; i8 < 256; i8++) {
        for (int i = 0; i < NSPECIAL; i += 4) {
            u128 f, n;
            for (int k = 0; k < 4; k++) {
                f._u32[k] = k_special[(i + k) % NSPECIAL];
                n._u32[k] = k_special_int[(i + k) % NSPECIAL_INT];
            }
            conversions(f, n, i8);
        }
        for (int it = 0; it < 4000; it++) {
            u128 f, n;
            for (int k = 0; k < 4; k++) { f._u32[k] = rnd_float(); n._u32[k] = rnd_int(); }
            conversions(f, n, i8);
        }
    }
    for (long it = 0; it < 1000000; it++) {
        u128 a, b, n;
        for (int k = 0; k < 4; k++) { a._u32[k] = rnd_float(); b._u32[k] = rnd_float(); n._u32[k] = rnd_int(); }
        if (it & 1) b._u32[rnd() & 3] = a._u32[rnd() & 3];          /* equal lanes */
        if (!(it & 6)) b._u32[rnd() & 3] = a._u32[rnd() & 3] ^ 0x80000000u;  /* sign flips */
        compares(a, b);
        conversions(a, n, (int)(rnd() & 0xFF));
    }
}

/* ns per operation, the way lifted code uses one: operands loaded from and the result stored
 * to memory (ctx->gpr), with a compiler barrier between operations so neither form is
 * vectorised or folded across them. */
enum { TN = 1024 };
u128 g_ta[TN], g_tb[TN], g_tr[TN];   /* external, so the stores are kept */
#define TIME1(label, expr) do { \
    clock_t t0 = clock(); \
    for (long rep_ = 0; rep_ < R; rep_++) \
        for (int i_ = 0; i_ < TN; i_++) { const u128 x = g_ta[i_], y = g_tb[i_]; (void)y; \
            g_tr[i_] = (expr); __asm__ volatile("" : : "r"(&g_tr[i_]) : "memory"); } \
    clock_t t1 = clock(); \
    printf("  %-22s %6.2f ns\n", label, 1e9 * (double)(t1 - t0) / CLOCKS_PER_SEC / ((double)R * TN)); } while (0)

int main(int argc, char** argv)
{
    (void)argv;
#if defined(__aarch64__)
    run();
    uint64_t fpcr; __asm__ volatile("mrs %0, fpcr" : "=r"(fpcr));
    __asm__ volatile("msr fpcr, %0" :: "r"(fpcr | (3ull << 22) | (1ull << 24)));   /* RZ + FZ */
    run();
    __asm__ volatile("msr fpcr, %0" :: "r"(fpcr));
#else
    run();
#endif
    printf("compares and conversions vs reference: %llu vectors, %llu mismatched\n", g_n, g_bad);
    if (argc > 1) {
        const long R = 20000;
        for (int i = 0; i < TN; i++)
            for (int k = 0; k < 4; k++) { g_ta[i]._u32[k] = rnd_float(); g_tb[i]._u32[k] = rnd_float(); }
        TIME1("fcgt", spu_fcgt(x, y));
        TIME1("fcgt_ref", spu_fcgt_ref(x, y));
        TIME1("fcmgt", spu_fcmgt(x, y));
        TIME1("fcmgt_ref", spu_fcmgt_ref(x, y));
        TIME1("cfltu(0x9D)", spu_cfltu(x, 0x9D));
        TIME1("cfltu_ref(0x9D)", spu_cfltu_ref(x, 0x9D));
        TIME1("cflts(0x9D)", spu_cflts(x, 0x9D));
        TIME1("cflts_ref(0x9D)", spu_cflts_ref(x, 0x9D));
        TIME1("csflt(0x9B)", spu_csflt(x, 0x9B));
        TIME1("csflt_ref(0x9B)", spu_csflt_ref(x, 0x9B));
        TIME1("cuflt(0x9B)", spu_cuflt(x, 0x9B));
        TIME1("cuflt_ref(0x9B)", spu_cuflt_ref(x, 0x9B));
    }
    return g_bad != 0;
}
