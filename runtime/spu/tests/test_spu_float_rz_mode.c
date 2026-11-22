/* SPU_FLOAT_RZ: spu_run_with_halt runs SPU code with FPCR at round toward zero with
 * flush-to-zero and puts the caller's FPCR back on the way out -- after a normal return, after a
 * halt (longjmp), and around a nested run. A run can happen synchronously on a PPU thread, whose
 * floating point must keep round to nearest with denormals. Without SPU_FLOAT_RZ=1 FPCR is never
 * touched -- unless a TU was built SPU_FLOAT_RZ_ONLY (tools/test_spu_float_rz_mode.py builds this
 * file both ways), which turns the mode on for every run whatever SPU_FLOAT_RZ says. arm64 only. */
#include "../spu_helpers.h"
#include "../spu_interp.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8_t* vm_base;
uint32_t ppu_vm_size;
extern int spu_run_with_halt(void (*)(spu_context*), spu_context*);
extern void spu_halt(spu_context*);

#if defined(__aarch64__) && defined(__ARM_NEON) && !defined(SPU_SCALAR_HELPERS)
static uint64_t fpcr_get(void) { uint64_t v; __asm__ volatile("mrs %0, fpcr" : "=r"(v)); return v; }
static void fpcr_set(uint64_t v) { __asm__ volatile("msr fpcr, %0" :: "r"(v) : "memory"); }

static spu_context ctx, inner_ctx;
static uint64_t seen_inner, seen_outer_after;
static int rz;

static void leaf_return(spu_context* c) { (void)c; seen_inner = fpcr_get(); }
static void leaf_halt(spu_context* c) { seen_inner = fpcr_get(); c->status = SPU_STATUS_STOPPED_BY_STOP; spu_halt(c); }
static void outer_nested(spu_context* c)
{
    (void)c;
    spu_context_init(&inner_ctx, 1);
    spu_run_with_halt(leaf_halt, &inner_ctx);
    seen_outer_after = fpcr_get();
}

static void run(void (*entry)(spu_context*), uint64_t caller, const char* what)
{
    fpcr_set(caller);
    seen_inner = seen_outer_after = ~0ull;
    spu_context_init(&ctx, 0);
    spu_run_with_halt(entry, &ctx);
    const uint64_t after = fpcr_get();
    fpcr_set(0);
    const uint64_t want_inner = rz ? SPU_FPCR_RZFZ : caller;
    if (seen_inner != want_inner || after != caller ||
        (entry == outer_nested && seen_outer_after != want_inner)) {
        fprintf(stderr, "FAIL %s (SPU_FLOAT_RZ=%d, caller FPCR %llX): inside %llX (want %llX), "
                "after %llX, outer after nested %llX\n", what, rz, (unsigned long long)caller,
                (unsigned long long)seen_inner, (unsigned long long)want_inner,
                (unsigned long long)after, (unsigned long long)seen_outer_after);
        exit(1);
    }
}

int main(void)
{
    const char* e = getenv("SPU_FLOAT_RZ");
    extern int spu_float_rz_required;
    rz = (e && !strcmp(e, "1")) || spu_float_rz_required;
#if defined(SPU_FLOAT_RZ_ONLY)
    if (!spu_float_rz_required) { fputs("FAIL: SPU_FLOAT_RZ_ONLY TU did not register\n", stderr); return 1; }
    /* every native form, no exact one: a truncating add must truncate */
    fpcr_set(SPU_FPCR_RZFZ);
    volatile uint32_t w_one = 0x3F800000u, w_tiny = 0x33800001u;   /* 1 + 2^-24(1 + 2^-23): not foldable */
    const u128 one = spu_splat_u32(w_one), tiny = spu_splat_u32(w_tiny);
    if (spu_fa(one, tiny)._u32[0] != 0x3F800000u) { fputs("FAIL: RZ-only fa did not truncate\n", stderr); return 1; }
    /* and it is the native form: outside the mode it rounds to nearest, which is why the
     * runtime must establish the mode for such a TU */
    fpcr_set(0);
    const u128 one2 = spu_splat_u32(w_one), tiny2 = spu_splat_u32(w_tiny);
    volatile uint32_t probe = spu_fa(one2, tiny2)._u32[0];
    if (probe != 0x3F800001u) { fputs("FAIL: RZ-only fa is not the native form\n", stderr); return 1; }
#endif
    const uint64_t callers[] = { 0, 0x00400000ull /* round toward +inf */, SPU_FPCR_RZFZ };
    for (unsigned i = 0; i < sizeof callers / sizeof callers[0]; i++) {
        run(leaf_return, callers[i], "return");
        run(leaf_halt, callers[i], "halt");
        run(outer_nested, callers[i], "nested");
    }
    printf("PASS: SPU_FLOAT_RZ=%s%s: FPCR inside runs and restored after return, halt and nesting\n",
           e ? e : "(unset)", spu_float_rz_required ? " with an RZ-only TU" : "");
    return 0;
}
#else
int main(void) { puts("SKIP: SPU_FLOAT_RZ is arm64 only"); return 0; }
#endif

/* Link-only boundaries outside this CPU-only fixture. Fail if any is reached. */
void spu_thread_publish_ctx(uint32_t tid, void* c) { (void)tid; (void)c; abort(); }
void ppu_resv_break_store(uint64_t ea) { (void)ea; abort(); }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; abort(); }
unsigned long long ps3_ms_now(void) { abort(); }
uint32_t g_ww_lo = 0xFFFFFFFFu, g_ww_hi;
int g_resv_store_active;
uint32_t g_barrier_sync_watch;
uint32_t g_ydkj_real_spurs_ea, g_ydkj_real_taskset_ea, g_ydkj_real_taskid;
void spurs_ef_set_from_spu(uint32_t ea, uint16_t bits) { (void)ea; (void)bits; abort(); }
void ydkj_wake_all_event_flags(void) { abort(); }
void spurs_pm_build_context(spu_context* c, uint32_t a, uint32_t b, uint32_t d) {
    (void)c; (void)a; (void)b; (void)d; abort();
}
uint32_t g_spu_image_src_ea, g_spu_image_ls_start, g_spu_image_span;
