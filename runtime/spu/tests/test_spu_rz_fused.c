/* The round-toward-zero forms of fm and fma/fms/fnms (spu__rz_fm, spu__rz_fma in spu_helpers.h,
 * which the native float path takes when FPCR is round toward zero with flush-to-zero) against
 * the exact references, lane for lane, in that FPCR: every triple of edge encodings (zeros,
 * denormals, the normal boundaries, FLT_MAX, exponent 255 with and without fraction) rotated
 * through the lanes, then random words mixed with edges. arm64 only.
 *   cc -std=gnu17 -O2 -I include -I runtime/spu runtime/spu/tests/test_spu_rz_fused.c -lm */
#include "spu_helpers.h"
#include "../../../tests/spu_ls_watch_fixture.h"
#include <stdio.h>
#include <stdlib.h>

#if defined(__aarch64__) && defined(__ARM_NEON) && !defined(SPU_SCALAR_HELPERS)
static uint32_t s_seed = 590384217u;
static uint32_t next(void) { s_seed ^= s_seed << 13; s_seed ^= s_seed >> 17; s_seed ^= s_seed << 5; return s_seed; }
static unsigned long s_n, s_bad;

static void same(const char* op, u128 got, u128 want, u128 a, u128 b, u128 c)
{
    s_n++;
    for (unsigned lane = 0; lane < 4; ++lane) {
        if (got._u32[lane] == want._u32[lane]) continue;
        if (s_bad++ < 10)
            fprintf(stderr, "FAIL %s lane %u: a=%08x b=%08x c=%08x want %08x got %08x\n",
                    op, lane, a._u32[lane], b._u32[lane], c._u32[lane], want._u32[lane], got._u32[lane]);
        return;
    }
}

__attribute__((noinline)) static void compare(u128 a, u128 b, u128 c)
{
    same("fm", spu__rz_fm(a, b), spu_fm_ref(a, b), a, b, c);
    same("fma", spu__rz_fma(a, b, c, 0, 0), spu_fma_ref(a, b, c), a, b, c);
    same("fms", spu__rz_fma(a, b, c, 0, 0x80000000u), spu_fms_ref(a, b, c), a, b, c);
    same("fnms", spu__rz_fma(a, b, c, 0x80000000u, 0), spu_fnms_ref(a, b, c), a, b, c);
    same("fm (public)", spu_fm(a, b), spu_fm_ref(a, b), a, b, c);
    same("fma (public)", spu_fma(a, b, c), spu_fma_ref(a, b, c), a, b, c);
}

static const uint32_t k_edge[] = {
    0, 0x80000000u, 1, 0x80000001u, 0x7FFFFFu, 0x807FFFFFu, 0x00800000u, 0x80800000u,
    0x00800001u, 0x3EAAAAABu, 0x3F800000u, 0xBF800000u, 0x3F800001u, 0x3F7FFFFFu,
    0x40000000u, 0xC0000000u, 0x1F800000u, 0x5F800000u, 0x7F7FFFFEu, 0x7F7FFFFFu,
    0xFF7FFFFFu, 0x7F800000u, 0xFF800000u, 0x7F800001u, 0xFFC00000u, 0x7FFFFFFFu, 0xFFFFFFFFu,
};

int main(void)
{
    uint64_t saved;
    __asm__ volatile("mrs %0, fpcr" : "=r"(saved));
    __asm__ volatile("msr fpcr, %0; isb" :: "r"((uint64_t)SPU_FPCR_RZFZ) : "memory");
    const unsigned n = sizeof k_edge / sizeof *k_edge;
    for (unsigned i = 0; i < n; ++i) for (unsigned j = 0; j < n; ++j) for (unsigned k = 0; k < n; ++k) {
        u128 a, b, c;
        for (unsigned lane = 0; lane < 4; ++lane) {
            a._u32[lane] = k_edge[(i + lane) % n];
            b._u32[lane] = k_edge[(j + 2 * lane) % n];
            c._u32[lane] = k_edge[(k + 3 * lane) % n];
        }
        compare(a, b, c);
    }
    for (unsigned i = 0; i < 1000000; ++i) {
        u128 a, b, c;
        for (unsigned lane = 0; lane < 4; ++lane) {
            a._u32[lane] = i % 5 ? next() : k_edge[next() % n];
            b._u32[lane] = i % 7 ? next() : k_edge[next() % n];
            c._u32[lane] = i % 11 ? next() : k_edge[next() % n];
            if (i % 3 == 0) a._u32[lane] = (a._u32[lane] & 0x807FFFFFu) | ((100u + next() % 56u) << 23);  /* ordinary */
            if (i % 3 == 0) b._u32[lane] = (b._u32[lane] & 0x807FFFFFu) | ((100u + next() % 56u) << 23);
        }
        compare(a, b, c);
    }
    __asm__ volatile("msr fpcr, %0; isb" :: "r"(saved) : "memory");
    printf("spu__rz_fm/spu__rz_fma vs reference under RZ+FZ: %lu vector ops, %lu mismatched\n", s_n, s_bad);
    return s_bad != 0;
}
#else
int main(void) { puts("SKIP: the round-toward-zero forms are arm64 only"); return 0; }
#endif
