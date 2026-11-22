/* MFC engine binding (PS3_SPU_MFC_BIND) and the collapsed MFC diagnostics
 * (PS3_SPU_MFC_FAST): lifetime rules and guest-visible equivalence.
 *
 *   ./fixture lifetime   bound-mode lifetime checks (needs PS3_SPU_MFC_BIND unset or 1)
 *   ./fixture diff N     run a seeded random channel/DMA program of N steps and
 *                        print one hash of everything the guest could observe;
 *                        the driver runs it under every switch combination and
 *                        requires one hash
 *   ./fixture stress     concurrent claim/release/lookup (for TSan)
 *
 * tools/test_spu_mfc_bind.py builds and drives it (plain, ASan+UBSan, TSan). */
#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include "../../platform/win32_compat.h"

/* Count registry lock acquisitions per thread, the way the older registry
 * fixtures do: hot-path lookups must take none once bound. */
static SRWLOCK* fixture_lock;
static _Thread_local unsigned t_shared_locks, t_exclusive_locks;
static void fixture_shared(SRWLOCK* l) { if (l == fixture_lock) t_shared_locks++; AcquireSRWLockShared(l); }
static void fixture_exclusive(SRWLOCK* l) { if (l == fixture_lock) t_exclusive_locks++; AcquireSRWLockExclusive(l); }
#define AcquireSRWLockShared fixture_shared
#define AcquireSRWLockExclusive fixture_exclusive

static uint8_t guest[1u << 22];
uint8_t* vm_base = guest;
uint32_t ppu_vm_size;
#include "../spu_channels.c"
#undef AcquireSRWLockShared
#undef AcquireSRWLockExclusive

static atomic_int g_fail;
#define CHECK(c) do { if (!(c)) { g_fail++; fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

static u128 V(uint32_t v) { u128 r; memset(&r, 0, sizeof r); r._u32[0] = v; return r; }
static void put_be32(uint8_t* p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }

static void issue(spu_context* c, uint32_t cmd, uint32_t lsa, uint32_t ea, uint32_t size, uint32_t tag)
{
    spu_wrch(c, MFC_LSA, V(lsa));
    spu_wrch(c, MFC_EAH, V(0));
    spu_wrch(c, MFC_EAL, V(ea));
    spu_wrch(c, MFC_Size, V(size));
    spu_wrch(c, MFC_TagID, V(tag));
    spu_wrch(c, MFC_Cmd, V(cmd));
}
static uint32_t tag_status(spu_context* c, uint32_t mask)
{
    spu_wrch(c, MFC_WrTagMask, V(mask));
    spu_wrch(c, MFC_WrTagUpdate, V(0));   /* immediate */
    return spu_rdch(c, MFC_RdTagStat)._u32[0];
}
/* A two-element GETL whose first element stalls: the tag stays incomplete
 * until MFC_WrListStallAck, i.e. a completion still to be delivered. */
static void stalled_list(spu_context* c, uint32_t tag, uint32_t ea)
{
    uint8_t* l = c->ls + 0x3000;
    put_be32(l + 0, 0x80000000u | 32u); put_be32(l + 4, ea);
    put_be32(l + 8, 32u);               put_be32(l + 12, ea + 32u);
    issue(c, MFC_GETL_CMD, 0x4000, 0x3000, 16, tag);
}

static uint32_t live_slots(void)
{
    AcquireSRWLockShared(&s_mfc_claim_lock);
    uint32_t n = s_mfc_bound_live;
    ReleaseSRWLockShared(&s_mfc_claim_lock);
    return n;
}

/* ---------------------------------------------------------------------------
 * Lifetime
 * -----------------------------------------------------------------------*/
static spu_context* nested_ctx;
static mfc_engine* nested_engine;
static void nested_entry(spu_context* c)
{
    CHECK(c == nested_ctx);
    issue(c, MFC_GET_CMD, 0x100, 0x20000, 16, 3);
    nested_engine = mfc_for(c);
    spu_halt(c);                       /* leave through the landing pad */
}
static spu_context* outer_ctx;
static mfc_engine* outer_engine;
static void outer_entry(spu_context* c)
{
    issue(c, MFC_GET_CMD, 0x100, 0x20000, 16, 1);
    outer_engine = mfc_for(c);
    outer_engine->tag_completed = 0x12345678u;
    CHECK(spu_run_with_halt(nested_entry, nested_ctx) == 1);
    CHECK(nested_engine != outer_engine);
    CHECK(mfc_for(c) == outer_engine && outer_engine->tag_completed == 0x12345678u);
    outer_engine->tag_completed = 0xFFFFFFFFu;
}

struct retire_job { spu_context** ctxs; unsigned n, rounds; atomic_int* stop; };
static void* retire_unrelated(void* p)
{
    struct retire_job* j = p;
    for (unsigned r = 0; r < j->rounds; r++)
        for (unsigned i = 0; i < j->n; i++) {
            issue(j->ctxs[i], MFC_GET_CMD, 0x200, 0x30000, 16, 2);
            CHECK(tag_status(j->ctxs[i], 4) == 4);
            spu_mfc_release(j->ctxs[i]);
        }
    atomic_store(j->stop, 1);
    return NULL;
}

/* One async-task lifetime on a fixed address, as spu_run_lifted_job_abi runs
 * it: init, DMA, a completion left undelivered, then release (or not). */
struct task_slot { spu_context* ctx; pthread_mutex_t m; pthread_cond_t cv; unsigned turn; };
struct task_arg { struct task_slot* slot; unsigned id, tasks, threads; int release; };
static void* task_thread(void* p)
{
    struct task_arg* a = p;
    struct task_slot* s = a->slot;
    for (unsigned k = a->id; k < a->tasks; k += a->threads) {
        pthread_mutex_lock(&s->m);
        while (s->turn != k) pthread_cond_wait(&s->cv, &s->m);
        pthread_mutex_unlock(&s->m);

        spu_context* c = s->ctx;
        const uint32_t prev_hint = c->mfc_slot_hint;
        spu_context_init(c, k);
        if (k & 1) c->mfc_slot_hint = prev_hint;   /* a frame that is not re-zeroed */
        const uint32_t ea = 0x40000u + (k & 15u) * 0x100u;
        memset(guest + ea, (int)(k & 0xFF), 0x80);
        /* The previous task left tag 5 incomplete. A fresh engine reports
         * every tag complete; a leaked one would not. */
        if (a->release) CHECK(tag_status(c, 1u << 5) == (1u << 5));
        else if (k) CHECK(tag_status(c, 1u << 5) == 0);   /* the leaked engine, as before */
        issue(c, MFC_GET_CMD, 0x1000, ea, 64, 1);
        CHECK(tag_status(c, 2) == 2);
        CHECK(c->ls[0x1000] == (uint8_t)(k & 0xFF) && c->ls[0x103F] == (uint8_t)(k & 0xFF));
        stalled_list(c, 5, ea);
        CHECK(tag_status(c, 1u << 5) == 0);          /* pending at task end */
        if (a->release) spu_mfc_release(c);

        pthread_mutex_lock(&s->m);
        s->turn = k + 1;
        pthread_cond_broadcast(&s->cv);
        pthread_mutex_unlock(&s->m);
    }
    return NULL;
}
static void run_tasks(int release, unsigned threads, unsigned tasks)
{
    struct task_slot s = { .ctx = calloc(1, sizeof(spu_context)) };
    pthread_mutex_init(&s.m, NULL); pthread_cond_init(&s.cv, NULL);
    pthread_t th[8]; struct task_arg a[8];
    for (unsigned i = 0; i < threads; i++) {
        a[i] = (struct task_arg){ &s, i, tasks, threads, release };
        pthread_create(&th[i], NULL, task_thread, &a[i]);
    }
    for (unsigned i = 0; i < threads; i++) pthread_join(th[i], NULL);
    spu_mfc_release(s.ctx);
    free(s.ctx);
}

static int lifetime(void)
{
    fixture_lock = &s_mfc_claim_lock;
    CHECK(spu_mfc_bound());
    CHECK(live_slots() == 0);

    /* Lazy allocation: parameter latches and MFC reads never allocate. */
    spu_context* a = calloc(1, sizeof *a); spu_context_init(a, 1);
    spu_wrch(a, MFC_LSA, V(0x100)); spu_wrch(a, MFC_WrTagMask, V(1));
    (void)spu_rdch(a, MFC_RdTagStat); (void)spu_rdch(a, MFC_RdAtomicStat);
    CHECK(live_slots() == 0);

    /* First engine use binds; then the hot path takes no registry lock. */
    memset(guest + 0x20000, 0xA5, 0x100);
    issue(a, MFC_GET_CMD, 0x100, 0x20000, 64, 0);
    CHECK(live_slots() == 1);
    mfc_engine* ea = mfc_for(a);
    t_shared_locks = t_exclusive_locks = 0;
    for (unsigned i = 0; i < 10000; i++) {
        issue(a, MFC_GET_CMD, 0x100 + (i & 3) * 64, 0x20000, 64, i & 31);
        CHECK(tag_status(a, 1u << (i & 31)) == 1u << (i & 31));
    }
    CHECK(t_shared_locks == 0 && t_exclusive_locks == 0);
    CHECK(mfc_for(a) == ea && a->ls[0x100] == 0xA5);
    printf("10000 bound DMAs: %u shared, %u exclusive registry locks\n", t_shared_locks, t_exclusive_locks);

    /* Retirement of unrelated contexts while this one is mid-run: its lookups
     * stay lock-free and keep returning its own engine and state. */
    enum { NOTHER = 32 };
    spu_context* other[NOTHER];
    for (unsigned i = 0; i < NOTHER; i++) { other[i] = calloc(1, sizeof(spu_context)); spu_context_init(other[i], 100 + i); }
    atomic_int stop = 0;
    struct retire_job rj = { other, NOTHER, 64, &stop };
    pthread_t rt; pthread_create(&rt, NULL, retire_unrelated, &rj);
    ea->tag_completed = 0x0F0F0F0Fu;
    t_shared_locks = t_exclusive_locks = 0;
    unsigned spins = 0;
    while (!atomic_load(&stop) || spins < 100000) {
        CHECK(mfc_for(a) == ea);
        CHECK(ea->tag_completed == 0x0F0F0F0Fu);
        spins++;
    }
    pthread_join(rt, NULL);
    CHECK(t_shared_locks == 0 && t_exclusive_locks == 0);
    printf("%u lookups during %u unrelated retirements: %u registry locks\n", spins, NOTHER * 64, t_shared_locks + t_exclusive_locks);
    ea->tag_completed = 0xFFFFFFFFu;

    /* Release, then the same address again WITHOUT a reset: the stale hint
     * names a slot that is free or reassigned, so it is not trusted. */
    const uint32_t old_hint = a->mfc_slot_hint;
    ea->tag_completed = 0;                       /* state only the old engine has */
    spu_mfc_release(a);
    CHECK(a->mfc_slot_hint == old_hint);         /* release never touches the context */
    spu_context* b = calloc(1, sizeof *b); spu_context_init(b, 2);
    issue(b, MFC_GET_CMD, 0x100, 0x20000, 16, 0);   /* likely reclaims that slot */
    CHECK(mfc_for(a) != mfc_for(b));
    CHECK(mfc_for(a)->tag_completed == 0xFFFFFFFFu);

    /* A hint that names ANOTHER live context's slot, and garbage hints. */
    mfc_engine* eb = mfc_for(b);
    eb->tag_completed = 0x5A5A5A5Au;
    a->mfc_slot_hint = b->mfc_slot_hint;
    CHECK(mfc_for(a) != eb && mfc_for(a)->tag_completed == 0xFFFFFFFFu);
    const uint32_t garbage[] = { 0xFFFFFFFFu, 0x80000000u, 12345u, 63u, 64u, 65535u, 65536u };
    for (unsigned i = 0; i < sizeof garbage / sizeof garbage[0]; i++) {
        mfc_engine* want = mfc_for(a);
        a->mfc_slot_hint = garbage[i];
        CHECK(mfc_for(a) == want);
    }
    CHECK(eb->tag_completed == 0x5A5A5A5Au);

    /* A struct copy of a bound context (the copy's hint names the original's
     * slot) gets its own engine; the original keeps its own. */
    spu_context* copy = malloc(sizeof *copy);
    memcpy(copy, b, sizeof *copy);
    CHECK(mfc_for(copy) != eb && mfc_for(copy)->tag_completed == 0xFFFFFFFFu);
    CHECK(mfc_for(b) == eb);

    /* A context reset (spu_context_init) without a release keeps the engine,
     * exactly as the address-keyed registry always has. */
    spu_context_init(b, 2);
    CHECK(mfc_for(b) == eb && eb->tag_completed == 0x5A5A5A5Au);
    eb->tag_completed = 0xFFFFFFFFu;

    /* Nested halt: an inner context bound inside an outer run leaves the
     * outer binding and state untouched (there is no scope to restore). */
    outer_ctx = calloc(1, sizeof(spu_context)); spu_context_init(outer_ctx, 3);
    nested_ctx = calloc(1, sizeof(spu_context)); spu_context_init(nested_ctx, 4);
    CHECK(spu_run_with_halt(outer_entry, outer_ctx) == 0);
    CHECK(mfc_for(outer_ctx) == outer_engine && mfc_for(nested_ctx) == nested_engine);

    /* Async task replacement at one context address, on alternating host
     * threads, each task leaving a list stalled (tag incomplete). Released:
     * every task starts with all tags complete. */
    run_tasks(1, 3, 300);
    /* Unreleased (a leak): every task meets the previous task's engine, as
     * with the address-keyed list; the diff mode checks the two agree. */
    run_tasks(0, 2, 20);

    /* Thread-exit cleanup of unreadable context memory reads only addresses. */
    void* dead = mmap(NULL, sizeof(spu_context), PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    CHECK(dead != MAP_FAILED);
    spu_context* d = dead; spu_context_init(d, 9);
    issue(d, MFC_GET_CMD, 0x100, 0x20000, 16, 0);
    const uint32_t before = live_slots();
    CHECK(mprotect(dead, sizeof(spu_context), PROT_NONE) == 0);
    spu_mfc_forget_range((uintptr_t)dead, (uintptr_t)dead + sizeof(spu_context));
    CHECK(live_slots() == before - 1);
    spu_mfc_release(d);                          /* idempotent, reads nothing */
    CHECK(munmap(dead, sizeof(spu_context)) == 0);

    spu_mfc_release(a); spu_mfc_release(b); spu_mfc_release(copy);
    spu_mfc_release(outer_ctx); spu_mfc_release(nested_ctx);
    for (unsigned i = 0; i < NOTHER; i++) { spu_mfc_release(other[i]); free(other[i]); }
    CHECK(live_slots() == 0);
    free(a); free(b); free(copy); free(outer_ctx); free(nested_ctx);
    printf("MFC binding lifetime: %s\n", g_fail ? "FAILED" : "all checks passed");
    return g_fail != 0;
}

/* ---------------------------------------------------------------------------
 * Differential program: same hash in every mode
 * -----------------------------------------------------------------------*/
static uint64_t g_rng = 0x9E3779B97F4A7C15ull;
static uint32_t rnd(void) { g_rng ^= g_rng << 13; g_rng ^= g_rng >> 7; g_rng ^= g_rng << 17; return (uint32_t)(g_rng >> 16); }
static uint64_t g_hash = 1469598103934665603ull;
static void mix(uint64_t v) { for (int i = 0; i < 8; i++) { g_hash ^= (v >> (i * 8)) & 0xFF; g_hash *= 1099511628211ull; } }
static void mix_bytes(const uint8_t* p, size_t n) { for (size_t i = 0; i < n; i++) { g_hash ^= p[i]; g_hash *= 1099511628211ull; } }

static int diff(unsigned steps)
{
    enum { NC = 4 };
    spu_context* c[NC];
    for (unsigned i = 0; i < NC; i++) { c[i] = calloc(1, sizeof(spu_context)); spu_context_init(c[i], i); c[i]->image_id = 7; }
    for (uint32_t i = 0; i < sizeof guest; i++) guest[i] = (uint8_t)(i * 31u + (i >> 9));
    static const uint32_t sizes[] = { 1, 2, 4, 8, 16, 32, 48, 64, 80, 96, 112, 128, 144, 256, 512, 1024, 4096, 16384, 3, 20, 0, 32768 };
    static const uint32_t cmds[] = { 0x40, 0x41, 0x42, 0x20, 0x21, 0x22, 0x40, 0x20, 0x44, 0x24, 0xC0, 0x89, 0xD0, 0xB4, 0xB0 };
    for (unsigned s = 0; s < steps; s++) {
        spu_context* x = c[rnd() % NC];
        const uint32_t op = rnd() % 16;
        if (op < 9) {
            uint32_t cmd = cmds[rnd() % (sizeof cmds / sizeof cmds[0])];
            uint32_t size = sizes[rnd() % (sizeof sizes / sizeof sizes[0])];
            uint32_t lsa = (rnd() & 0x3FFF0u) | (size < 16 ? (rnd() & 15u) & ~(size - 1u) : 0u);
            uint32_t ea = 0x10000u + ((rnd() % 0x3E0000u) & ~15u) + (lsa & 15u);
            if (rnd() % 64 == 0) ea = rnd() % (sizeof guest - 0x8000u); /* unaligned, any size */
            if (rnd() % 32 == 0) ea = (rnd() & 0xFFF0u) | (lsa & 15u); /* null page */
            if (cmd == 0x44 || cmd == 0x24) {                        /* list */
                uint32_t n = 1 + rnd() % 4, l = 0x3F000u + (rnd() % 8) * 64;
                for (uint32_t k = 0; k < n; k++) {
                    uint32_t es = sizes[rnd() % 12];
                    put_be32(x->ls + l + k * 8, (rnd() % 5 == 0 ? 0x80000000u : 0) | es);
                    put_be32(x->ls + l + k * 8 + 4, 0x10000u + ((rnd() % 0x3E0000u) & ~15u));
                }
                lsa &= 0x1FFF0u;
                issue(x, cmd, lsa, l, n * 8, rnd() & 31);
            } else if (cmd >= 0xB0 && cmd != 0xC0) {
                issue(x, cmd, (rnd() & 0x3FF80u), 0x10000u + ((rnd() % 0x3E0000u) & ~127u), 128, rnd() & 31);
            } else if (cmd == 0x89) {
                issue(x, cmd, 0, 0x10000u + ((rnd() % 0x3E0000u) & ~127u), 128u * (1 + rnd() % 4), rnd() & 31);
            } else {
                issue(x, cmd, lsa, ea, size, rnd() & 31);
            }
        } else if (op < 11) {
            mix(tag_status(x, rnd()));
            spu_wrch(x, MFC_WrTagUpdate, V(1 + rnd() % 2));
            mix(spu_rdch(x, MFC_RdTagStat)._u32[0]);
        } else if (op == 11) {
            uint32_t m = spu_rdch(x, MFC_RdListStallStat)._u32[0];
            mix(m);
            for (uint32_t t = 0; t < 32; t++) if (m & (1u << t)) spu_wrch(x, MFC_WrListStallAck, V(t));
        } else if (op == 12) {
            mix(spu_rdch(x, MFC_RdAtomicStat)._u32[0]);
            mix(spu_rchcnt(x, MFC_Cmd));
            mix(spu_rdch(x, MFC_RdTagMask)._u32[0]);
        } else if (op == 13) {
            /* task replacement at the same address: release, maybe reset */
            if (rnd() & 1) spu_mfc_release(x);
            if (rnd() & 1) { uint32_t keep = x->mfc_slot_hint; spu_context_init(x, 1); x->image_id = 7; if (rnd() & 1) x->mfc_slot_hint = keep; }
        } else if (op == 14) {
            x->mfc_slot_hint = rnd();                                 /* hostile hint */
        } else {
            spu_wrch(x, SPU_WrEventMask, V(rnd() & 3));
            mix(spu_rdch(x, SPU_RdEventMask)._u32[0]);
        }
        mix(x->mfc_tag_status); mix(x->list_stall_mask); mix(x->atomic_stat); mix(x->event_status);
    }
    for (unsigned i = 0; i < NC; i++) { mix_bytes(c[i]->ls, SPU_LS_SIZE); mix(tag_status(c[i], 0xFFFFFFFFu)); }
    mix_bytes(guest, sizeof guest);
    printf("diff hash %016llx steps %u\n", (unsigned long long)g_hash, steps);
    for (unsigned i = 0; i < NC; i++) { spu_mfc_release(c[i]); free(c[i]); }
    return 0;
}

/* ---------------------------------------------------------------------------
 * Concurrent claim / release / lookup, for ThreadSanitizer
 * -----------------------------------------------------------------------*/
static void* stress_thread(void* p)
{
    const unsigned id = (unsigned)(uintptr_t)p;
    spu_context* mine[4];
    for (int i = 0; i < 4; i++) { mine[i] = calloc(1, sizeof(spu_context)); spu_context_init(mine[i], id * 4 + i); }
    for (unsigned r = 0; r < 2000; r++) {
        spu_context* x = mine[r & 3];
        issue(x, MFC_PUT_CMD, 0x100, 0x100000u + id * 0x1000u, 32, r & 31);
        if (tag_status(x, 1u << (r & 31)) != 1u << (r & 31)) g_fail++;
        if ((r & 7) == 7) spu_mfc_release(x);
        if ((r & 63) == 63) spu_mfc_forget_range((uintptr_t)mine[0] - 1, (uintptr_t)mine[0]);  /* misses */
    }
    for (int i = 0; i < 4; i++) { spu_mfc_release(mine[i]); free(mine[i]); }
    return NULL;
}
static int stress(void)
{
    /* The transfer path's existing lazily read env gates (SPU_DMA_LAX, the
     * repeat watchdog's limits, SPU_POISON_DMA_WATCH) are plain statics that
     * race benignly on first use from several threads; that is unchanged by
     * this work. Read them once here so TSan sees only the registry. */
    { spu_context* w = calloc(1, sizeof(spu_context)); spu_context_init(w, 99);
      issue(w, MFC_PUT_CMD, 0x100, 0x100000u, 32, 0); (void)tag_status(w, 1);
      spu_mfc_release(w); free(w); }
    pthread_t th[6];
    for (uintptr_t i = 0; i < 6; i++) pthread_create(&th[i], NULL, stress_thread, (void*)i);
    for (int i = 0; i < 6; i++) pthread_join(th[i], NULL);
    printf("stress: %s, %u live slots left\n", g_fail ? "FAILED" : "ok", live_slots());
    return g_fail != 0 || live_slots() != 0;
}

int main(int argc, char** argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    if (argc > 1 && !strcmp(argv[1], "diff")) return diff(argc > 2 ? (unsigned)atoi(argv[2]) : 20000);
    if (argc > 1 && !strcmp(argv[1], "stress")) return stress();
    return lifetime();
}

/* Link stubs, as in test_spu_dma_repeat.c. */
unsigned long long ps3_ms_now(void) { return 1000; }
void ppu_resv_break_store(uint64_t ea) { (void)ea; }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; }
uint32_t g_ww_lo = 0xFFFFFFFFu, g_ww_hi = 0;
int      g_resv_store_active = 0;
uint32_t g_barrier_sync_watch = 0;
void (*g_spurs_kernel_hook)(uint32_t) = 0;
uint32_t g_ydkj_spurs_ctx_ea = 0;
uint32_t g_ydkj_real_spurs_ea = 0;
uint32_t g_ydkj_real_taskset_ea = 0;
uint32_t g_ydkj_real_taskid = 0;
void spurs_ef_set_from_spu(uint32_t ea, uint16_t bits) { (void)ea; (void)bits; }
void ydkj_wake_all_event_flags(void) { }
void spurs_pm_build_context(spu_context* c, uint32_t a, uint32_t b, uint32_t d) { (void)c; (void)a; (void)b; (void)d; }
uint32_t g_spu_image_src_ea, g_spu_image_ls_start, g_spu_image_span;
void spu_thread_publish_ctx(uint32_t tid, void* ctx) { (void)tid; (void)ctx; }
