/* gcc -std=gnu17 -I include -I libs/video [-fsanitize=thread | -fsanitize=address,undefined]
 *   libs/video/tests/test_gcm_vblank_counted.c libs/video/tests/gcm_fixture_stubs.c
 *   runtime/platform/win32_compat.c -pthread -ffunction-sections -Wl,--gc-sections
 *   -o /tmp/test_gcm_vblank_counted && /tmp/test_gcm_vblank_counted
 *
 * Counted vblank delivery (PS3_VBLANK_COUNTED). On hardware every vblank
 * interrupt runs the guest's handler once. Here a tick is delivered when a
 * guest thread next pumps, so ticks pile up between pumps; each must still
 * reach the handler exactly once, oldest first, with its own vblank count.
 * A backlog beyond PS3_VBLANK_BACKLOG is dropped instead of replayed in a
 * burst. PS3_VBLANK_COUNTED=0 must still coalesce as the old pending bit did.
 *
 * With no arguments the fixture runs each mode in a child process, since the
 * switches are read once per process.
 */
#include "../cellGcmSys.c"
#include "../rsx_tiling.c"
#include <assert.h>
#include <pthread.h>
#include <sched.h>
#include <sys/wait.h>
#include <unistd.h>
uint8_t* vm_base;
uint32_t ppu_vm_size;
int g_resv_store_active;
uint32_t g_ww_lo, g_ww_hi;
void ppu_resv_break_store(uint64_t ea) { (void)ea; }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; }
int spu_coh_is_reserved(uint32_t a) { (void)a; return 0; }
void spu_coh_notify_write(uint32_t a) { (void)a; }
void spu_lockline_lock(void) {}
void spu_lockline_unlock(void) {}
ps3_guest_caller_fn g_ps3_guest_caller;

enum { VBLANK_OPD = 0x500, OTHER_OPD = 0x600, TICKS = 200000, PUMPERS = 2 };

/* Written only inside the pump guard, which serializes every callback. */
static u32 args[TICKS + 64];
static _Atomic(u32) calls, other_calls;
static void callback(uint32_t opd, uint64_t a, uint64_t b, uint64_t c,
                     uint64_t d, uint64_t e, uint64_t f, uint64_t h, uint64_t i)
{
    (void)b; (void)c; (void)d; (void)e; (void)f; (void)h; (void)i;
    if (opd == OTHER_OPD) { atomic_fetch_add(&other_calls, 1u); return; }
    if (opd != VBLANK_OPD) return;
    const u32 n = atomic_load_explicit(&calls, memory_order_relaxed);
    if (n < TICKS + 64) args[n] = (u32)a;
    atomic_store_explicit(&calls, n + 1u, memory_order_release);
}

static void reset_counts(void) { atomic_store(&calls, 0u); atomic_store(&other_calls, 0u); }

/* A cheap per-thread generator for the irregular pump spacing. */
static u32 next_rand(u32* s) { *s ^= *s << 13; *s ^= *s >> 17; *s ^= *s << 5; return *s; }

static _Atomic(int) ticking;
static int pace_to_bound;   /* counted mode: never let the backlog pass the bound */

static void* ticker(void* unused)
{
    (void)unused;
    u32 seed = 0x9E3779B9u;
    for (u32 i = 0; i < TICKS; ) {
        /* Bursts of 1..6 ticks back to back, so pumps see multi-tick backlogs. */
        u32 burst = 1 + next_rand(&seed) % 6;
        if (pace_to_bound) {
            while (i - atomic_load_explicit(&calls, memory_order_acquire) + burst > 8)
                sched_yield();
        }
        for (; burst && i < TICKS; burst--, i++) cellGcmTickVBlank();
        for (u32 spin = next_rand(&seed) % 2000; spin; spin--) __asm__ __volatile__("" ::: "memory");
    }
    atomic_store(&ticking, 0);
    return NULL;
}

static void* pumper(void* slot)
{
    u32 seed = 0x1234567u + (u32)(uintptr_t)slot * 7919u;
    while (atomic_load(&ticking)) {
        ppu_gcm_pump();
        const u32 r = next_rand(&seed) % 8;
        if (r == 0) sched_yield();
        else for (u32 spin = r * 400; spin; spin--) __asm__ __volatile__("" ::: "memory");
    }
    ppu_gcm_pump();
    return NULL;
}

static void run_threads(void)
{
    pthread_t t, p[PUMPERS];
    atomic_store(&ticking, 1);
    for (uintptr_t i = 0; i < PUMPERS; i++) pthread_create(&p[i], NULL, pumper, (void*)i);
    pthread_create(&t, NULL, ticker, NULL);
    pthread_join(t, NULL);
    for (int i = 0; i < PUMPERS; i++) pthread_join(p[i], NULL);
    ppu_gcm_pump();
}

static void print_stats(const char* mode)
{
    char line[512];
    cellGcm_vblank_stats_line(line, sizeof line);
    printf("  %s: %s\n", mode, line);
}

static int run_counted(void)
{
    /* 1. Threads: every tick delivered once, in order, with its count. */
    pace_to_bound = 1;
    run_threads();
    const u32 n = atomic_load(&calls);
    assert(n == TICKS);
    for (u32 i = 0; i < TICKS; i++) assert(args[i] == i + 1u);
    assert(GCM_A32_LOAD(s_vb_dropped) == 0 && GCM_A32_LOAD(s_vb_coalesced) == 0);
    const u32 widest = GCM_A32_LOAD(s_vb_max_backlog);
    assert(widest > 1 && widest <= 8);
    printf("[PASS] counted: %u ticks from one thread, pumped irregularly by %d threads: "
           "each delivered once, in order, with its count (largest backlog %u)\n",
           TICKS, PUMPERS, widest);
    print_stats("counted");

    /* 2. The bound: a stalled guest gets the newest 8 of 20, not a burst of 20. */
    u32 base = GCM_VBLANK_READ();
    reset_counts();
    for (int i = 0; i < 20; i++) cellGcmTickVBlank();
    ppu_gcm_pump();
    assert(atomic_load(&calls) == 8);
    for (u32 i = 0; i < 8; i++) assert(args[i] == base + 13u + i);
    assert(GCM_A32_LOAD(s_vb_dropped) == 12);
    for (int i = 0; i < 3; i++) cellGcmTickVBlank();
    ppu_gcm_pump();
    assert(atomic_load(&calls) == 11 && args[10] == base + 23u);
    printf("[PASS] counted: 20 ticks behind one pump replays the newest 8 and drops 12; "
           "the next 3 follow in order\n");

    /* 3. A guest query advances the count (cellGcmGetVBlankCount) but is not a
     * tick: no callback for it, and the next tick's count includes it. */
    reset_counts();
    base = GCM_VBLANK_READ();
    cellGcmTickVBlank();
    (void)cellGcmGetVBlankCount();
    (void)cellGcmGetVBlankCount();
    cellGcmTickVBlank();
    ppu_gcm_pump();
    assert(atomic_load(&calls) == 2 && args[0] == base + 1u && args[1] == base + 4u);
    printf("[PASS] counted: guest count queries are not ticks; each tick carries its own count\n");

    /* 4. A new handler does not inherit the old one's backlog, and a stretch
     * with no handler is not replayed when one is set. */
    reset_counts();
    for (int i = 0; i < 5; i++) cellGcmTickVBlank();
    cellGcmSetVBlankHandler((CellGcmVBlankHandler)(uintptr_t)OTHER_OPD);
    ppu_gcm_pump();
    assert(atomic_load(&other_calls) == 0);
    cellGcmTickVBlank();
    ppu_gcm_pump();
    assert(atomic_load(&other_calls) == 1);
    cellGcmSetVBlankHandler(NULL);
    for (int i = 0; i < 5; i++) cellGcmTickVBlank();
    ppu_gcm_pump();
    cellGcmSetVBlankHandler((CellGcmVBlankHandler)(uintptr_t)VBLANK_OPD);
    ppu_gcm_pump();
    assert(atomic_load(&calls) == 0);
    cellGcmTickVBlank();
    ppu_gcm_pump();
    assert(atomic_load(&calls) == 1);
    printf("[PASS] counted: a handler change or a stretch with none is not replayed\n");
    return 0;
}

static int run_bit(void)
{
    /* The old pending bit: three ticks, one pump, one callback with the count. */
    u32 base = GCM_VBLANK_READ();
    for (int i = 0; i < 3; i++) cellGcmTickVBlank();
    ppu_gcm_pump();
    assert(atomic_load(&calls) == 1 && args[0] == base + 3u);
    assert(GCM_A32_LOAD(s_vb_coalesced) == 2);
    /* Threads: under this load ticks really do coalesce. Callbacks plus
     * coalesced ticks cover every tick; the old bit can also run one extra
     * callback with no new tick (a tick lands between the pump taking the bit
     * and reading the count), so the sum may exceed it. */
    reset_counts();
    GCM_A32_STORE(s_vb_coalesced, 0u);
    pace_to_bound = 0;
    run_threads();
    const u32 n = atomic_load(&calls), lost = GCM_A32_LOAD(s_vb_coalesced);
    assert(n + lost >= TICKS && n < TICKS);
    assert(lost > 0);
    printf("[PASS] PS3_VBLANK_COUNTED=0: %u ticks gave %u callbacks, %u coalesced "
           "(the old behaviour)\n", TICKS, n, lost);
    print_stats("bit");
    return 0;
}

static int run_mode(const char* mode)
{
    vm_base = calloc(1, 0x10000);
    ppu_vm_size = 0x10000;
    g_ps3_guest_caller = callback;
    s_vblank_handler_opd = VBLANK_OPD;
    if (!strcmp(mode, "counted")) return run_counted();
    if (!strcmp(mode, "bit")) return run_bit();
    return 2;
}

int main(int argc, char** argv)
{
    if (argc > 1) return run_mode(argv[1]);
    const char* modes[] = { "counted", "bit" };
    for (int i = 0; i < 2; i++) {
        const pid_t pid = fork();
        if (pid == 0) {
            setenv("PS3_VBLANK_COUNTED", i == 0 ? "1" : "0", 1);
            execl("/proc/self/exe", argv[0], modes[i], (char*)NULL);
            execlp(argv[0], argv[0], modes[i], (char*)NULL);
            _exit(127);
        }
        int status = 0;
        waitpid(pid, &status, 0);
        if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
            fprintf(stderr, "[FAIL] mode %s: status 0x%x\n", modes[i], status);
            return 1;
        }
    }
    puts("GCM counted vblank checks passed");
    return 0;
}
