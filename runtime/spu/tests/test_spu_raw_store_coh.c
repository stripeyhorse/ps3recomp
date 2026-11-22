/*
 * PPU raw stores against SPU PUTLLC: the lost update the lifted code's inline
 * raw store paths allowed (tools/ppu_lifter.py: VM_RAW_ST, vm_raw_st16,
 * vm_dcbz -- stwbrx/sthbrx/stdbrx, stvx/stvebx/stvehx/stvewx, dcbz).
 *
 * Those stores bypass vm_write*, so they never looked at the SPU
 * reserved-line bitmap and never took the lock-line lock. A PUTLLC compares
 * its snapshot with guest memory and then copies its 128 bytes back, both
 * under that lock; a PPU store that lands between the compare and the copy is
 * written straight back over with the SPU's stale copy of those bytes, and the
 * PUTLLC still reports success. Nothing notices: the PPU's write is simply
 * gone.
 *
 * The window is a 128-byte memcpy, so this is a stress test rather than a
 * constructed interleaving (the runtime exposes no hook between the compare
 * and the copy). SPU threads loop GETLLAR / bump word 0 / PUTLLC on one line;
 * a PPU thread stores a fresh sequence number into another word of the same
 * line through the generated header's raw store, and before each new store
 * checks that the previous one is still there. Only the PPU ever changes that
 * word, so any other value there is a lost PPU store. It also checks that no
 * SPU increment of word 0 was lost (the reverse direction, which the lock
 * already prevents; a regression guard).
 *
 * The raw store helpers are compiled from the real generated header
 * (raw_tu.cpp, built against a ppu_recomp.h this lifter emits), not copied.
 *
 *   PS3_RAW_STORE_COH unset: expect 0 lost stores.
 *   PS3_RAW_STORE_COH=0:     the old unlocked store; expect lost stores
 *                            (exit 1), which is the reproduction.
 *
 * Build (from ps3recomp/). HDR = a directory holding a ppu_recomp.h from this
 * lifter, e.g. `python3 -c "import sys; sys.path.insert(0,'tools'); import
 * ppu_lifter as L; open('$HDR/ppu_recomp.h','w').write(L.HEADER_PREAMBLE)"`:
 *   g++ -std=gnu++17 -O2 -c -I $HDR -o raw_tu.o runtime/spu/tests/raw_store_coh_tu.cpp
 *   for f in spu_drain spu_lockstep spu_coherency spu_coh_raw spu_workload spurs_policy \
 *            spurs_job spu_tsp_weak spu_vm_pagemap spurs_policy_blob_weak spu_interp; do
 *     gcc -std=gnu17 -O2 -w -I include -I runtime/spu -I runtime/platform -c runtime/spu/$f.c; done
 *   gcc -std=gnu17 -O2 -w -I include -c runtime/platform/win32_compat.c
 *   gcc -std=gnu17 -O2 -w -I include -I runtime/spu -I runtime/platform -c \
 *       runtime/spu/tests/test_spu_raw_store_coh.c
 *   g++ -o test_spu_raw_store_coh *.o -lpthread -lm
 *   ./test_spu_raw_store_coh 3 stw; PS3_RAW_STORE_COH=0 ./test_spu_raw_store_coh 3 stw   # 2nd exits 1
 *
 * Arguments: [seconds] [mode]   mode: stw (default), stvx, dcbz
 */

#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static uint8_t g_guest_mem[1u << 20];
uint8_t* vm_base = g_guest_mem;
uint32_t ppu_vm_size = 0;

#include "../spu_channels.c"

/* The runtime beyond the SPU channel layer, stubbed (as in
 * test_spu_lockline_peer.c). None of it is on the path this test drives. */
void ppu_resv_break_store(uint64_t ea)                 { (void)ea; }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; }
unsigned long long ps3_ms_now(void)                    { return 0; }
uint32_t g_ww_lo = 0xFFFFFFFFu, g_ww_hi = 0;
int      g_resv_store_active = 0;
uint32_t g_barrier_sync_watch = 0;
void (*g_spurs_kernel_hook)(uint32_t) = 0;
uint32_t g_ydkj_spurs_ctx_ea = 0;
uint32_t g_ydkj_real_spurs_ea = 0;
uint32_t g_ydkj_real_taskset_ea = 0;
uint32_t g_ydkj_real_taskid = 0;
void spurs_ef_set_from_spu(uint32_t ea, uint16_t bits) { (void)ea; (void)bits; }
void ydkj_wake_all_event_flags(void)                   { }
void spurs_pm_build_context(spu_context* c, uint32_t a, uint32_t b, uint32_t d) { (void)c; (void)a; (void)b; (void)d; }
uint32_t g_spu_image_src_ea, g_spu_image_ls_start, g_spu_image_span;
void spu_thread_publish_ctx(uint32_t tid, void* ctx) { (void)tid; (void)ctx; }

/* raw_store_coh_tu.cpp: the generated header's store paths. */
void fx_stw_raw(uint32_t ea, uint32_t v);
void fx_stvx(uint32_t ea, const void* src);
void fx_dcbz(uint32_t ea);

#define LINE     0x00030000u
#define PPU_OFF  64u          /* the PPU's word (and quadword) in the line */
#define LSA      0x00001000u
#define NSPU     3

static spu_context g_spu[NSPU];
static atomic_int g_stop;
static unsigned long g_spu_commits[NSPU];

static void wrch(spu_context* ctx, uint32_t channel, uint32_t v)
{
    spu_wrch(ctx, channel, spu_make_preferred_u32(v));
}
static void spu_atomic(spu_context* ctx, uint32_t cmd)
{
    wrch(ctx, MFC_LSA, LSA);
    wrch(ctx, MFC_EAH, 0);
    wrch(ctx, MFC_EAL, LINE);
    wrch(ctx, MFC_Cmd, cmd);
}

static void* spu_thread(void* p)
{
    spu_context* ctx = (spu_context*)p;
    const int me = (int)(ctx - g_spu);
    while (!atomic_load_explicit(&g_stop, memory_order_relaxed)) {
        spu_atomic(ctx, MFC_GETLLAR_CMD);
        uint32_t w;
        memcpy(&w, &ctx->ls[LSA], 4);
        w = __builtin_bswap32(__builtin_bswap32(w) + 1u);
        memcpy(&ctx->ls[LSA], &w, 4);
        spu_atomic(ctx, MFC_PUTLLC_CMD);
        if (ctx->atomic_stat == 0) g_spu_commits[me]++;
        SPU_EVENT_CLEAR(ctx, SPU_EVENT_LR);
    }
    return NULL;
}

static uint32_t ld32(uint32_t ea)
{
    return __atomic_load_n((const uint32_t*)(g_guest_mem + ea), __ATOMIC_RELAXED);
}

int main(int argc, char** argv)
{
    const double secs = argc > 1 ? atof(argv[1]) : 3.0;
    const char* mode = argc > 2 ? argv[2] : "stw";
    const char* env = getenv("PS3_RAW_STORE_COH");
    memset(g_guest_mem + LINE, 0, 128);
    for (int i = 0; i < NSPU; i++) { spu_context_init(&g_spu[i], 0); g_spu[i].spu_id = (uint32_t)i; }
    /* One GETLLAR first, so the line is marked before the PPU starts. */
    spu_atomic(&g_spu[0], MFC_GETLLAR_CMD);

    pthread_t th[NSPU];
    for (int i = 0; i < NSPU; i++) pthread_create(&th[i], NULL, spu_thread, &g_spu[i]);

    struct timespec t0, t;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    unsigned long stores = 0, lost = 0;
    uint32_t seq = 0;
    uint8_t q[16];
    for (;;) {
        if ((stores & 1023) == 0) {
            clock_gettime(CLOCK_MONOTONIC, &t);
            if ((double)(t.tv_sec - t0.tv_sec) + (t.tv_nsec - t0.tv_nsec) * 1e-9 > secs) break;
        }
        /* The previous store must still be there. */
        const uint32_t want = __builtin_bswap32(seq);
        if (strcmp(mode, "dcbz") == 0) {
            /* dcbz zeroes the whole line, SPU word 0 included, so the check is
             * that the line's PPU word reads 0 right after each zeroing -- a
             * stale PUTLLC brings the old nonzero marker back. */
            if (seq && ld32(LINE + PPU_OFF) != 0) lost++;
            fx_stw_raw(LINE + PPU_OFF, 0xA5A5A5A5u);   /* marker for the next round */
            for (volatile int k = 0; k < 64; k++) { }
            fx_dcbz(LINE);
        } else {
            if (ld32(LINE + PPU_OFF) != want) lost++;
            seq++;
            const uint32_t be = __builtin_bswap32(seq);
            if (strcmp(mode, "stvx") == 0) {
                for (int k = 0; k < 16; k += 4) memcpy(q + k, &be, 4);
                fx_stvx(LINE + PPU_OFF, q);
            } else {
                fx_stw_raw(LINE + PPU_OFF, be);   /* raw = guest byte order */
            }
        }
        if (strcmp(mode, "dcbz") == 0) seq++;
        stores++;
        for (volatile int k = 0; k < 32; k++) { }   /* let a PUTLLC overlap */
    }
    atomic_store(&g_stop, 1);
    for (int i = 0; i < NSPU; i++) pthread_join(th[i], NULL);

    unsigned long commits = 0;
    for (int i = 0; i < NSPU; i++) commits += g_spu_commits[i];
    const uint32_t w0 = __builtin_bswap32(ld32(LINE));
    const int spu_lost = strcmp(mode, "dcbz") != 0 && w0 != (uint32_t)commits;
    printf("mode=%s PS3_RAW_STORE_COH=%s: %lu PPU raw stores, %lu lost; %lu SPU PUTLLC commits, word0=%u%s\n",
           mode, env ? env : "(unset)", stores, lost, commits, w0, spu_lost ? " (SPU UPDATE LOST)" : "");
    return (lost || spu_lost) ? 1 : 0;
}
