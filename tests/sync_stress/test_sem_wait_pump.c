/* PS3_GCM_WAIT_PUMP: a thread blocked in sys_semaphore_wait is woken by a GCM
 * tick (the kick in sys_semaphore_pump.h) to run the callback pump itself.
 *
 * PAIN's main thread waits on a semaphore that only its vblank handler posts,
 * and the handler runs only when some guest thread pumps. Here the waiter is
 * the only thread that ever pumps, through the real sys_semaphore_wait; the
 * "GCM side" (what it registers in sys_semaphore_pump.h) is this file's: a counted
 * tick queue whose handler posts the semaphore once two ticks have counted,
 * and the waiter resets the count when its wait returns, as PAIN does.
 *
 *   ticks      1500 ticks at 500 Hz: every one delivered, in order, by the
 *              parked waiter; frames one per two ticks.
 *   singles    one tick at a time, the next only after the waiter has its
 *              token, so a lost wake cannot be rescued by a later tick.
 *   destroy    a waiter parked (and pumping) still counts: destroy is EBUSY
 *              until it has its token; kicks around create/destroy churn of
 *              other semaphores never touch a torn-down one (run under TSan
 *              and ASan).
 *   off        PS3_GCM_WAIT_PUMP=0: a blocked waiter is never woken to pump.
 *
 * POSIX only (Windows has its own pump event). Each mode runs in a child
 * process, since the switch is read once.
 */
#include "runtime/platform/win32_compat.h"
#include "runtime/syscalls/sys_mutex.h"
#include "runtime/syscalls/sys_semaphore.h"
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef _WIN32
#include <pthread.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#endif

uint8_t* vm_base;
static int failures;
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x); failures++; } } while (0)

#ifdef _WIN32
int main(void) { puts("sem wait pump: POSIX only, skipped"); return 0; }
#else
typedef int64_t (*sc_fn)(ppu_context*);
static int32_t call(sc_fn f, uint32_t tid, uint64_t a, uint64_t b, uint64_t c, uint64_t d)
{
    ppu_context ctx;
    memset(&ctx, 0, sizeof ctx);
    ctx.thread_id = tid;
    ctx.gpr[3] = a; ctx.gpr[4] = b; ctx.gpr[5] = c; ctx.gpr[6] = d;
    return (int32_t)f(&ctx);
}
static uint32_t be_read(unsigned off)
{
    const uint8_t* p = vm_base + off;
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}
static void be_write(unsigned off, uint32_t v)
{
    vm_base[off] = (uint8_t)(v >> 24); vm_base[off + 1] = (uint8_t)(v >> 16);
    vm_base[off + 2] = (uint8_t)(v >> 8); vm_base[off + 3] = (uint8_t)v;
}
#define A_ID    0x200u
#define A_ATTR  0x300u
static uint32_t make_sem(int max)
{
    memset(vm_base + A_ATTR, 0, 0x30);
    be_write(A_ATTR + 0, SYS_SYNC_FIFO);
    if (call(sys_semaphore_create, 1, A_ID, A_ATTR, 0, (uint64_t)max) != CELL_OK) return 0;
    return be_read(A_ID);
}
static uint64_t mono_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec;
}
static void sleep_us(unsigned us)
{
    const struct timespec d = { us / 1000000u, (long)(us % 1000000u) * 1000L };
    nanosleep(&d, NULL);
}

/* ---- the GCM side: counted ticks, a pump guard, PAIN's handler ---------- */
#include "runtime/syscalls/sys_semaphore_pump.h"
static unsigned sys_semaphore_pump_kicks(void) { return atomic_load(&g_ps3_sem_pump_glue.kicks); }
static atomic_uint published, delivered_upto;
static atomic_flag pumping = ATOMIC_FLAG_INIT;
static uint32_t frame_sem, post_at = 2;
static uint32_t game_count, callbacks, out_of_order, last_tick;   /* pump guard only */
static atomic_uint pump_threads_seen;   /* bit 0: the waiter's thread pumped */

static int gcm_pending(void) { return atomic_load(&published) != atomic_load(&delivered_upto); }
static void gcm_pump(void)
{
    if (atomic_flag_test_and_set_explicit(&pumping, memory_order_acquire)) return;
    const uint32_t upto = atomic_load(&published);
    for (uint32_t t = atomic_load(&delivered_upto); t != upto; ) {
        t++;
        if (t != last_tick + 1u) out_of_order++;
        last_tick = t;
        callbacks++;
        if (++game_count >= post_at) (void)call(sys_semaphore_post, 9, frame_sem, 1, 0, 0);
        atomic_store(&delivered_upto, t);
    }
    atomic_fetch_or(&pump_threads_seen, 1u);
    atomic_flag_clear_explicit(&pumping, memory_order_release);
}
static void tick(void)
{
    atomic_fetch_add(&published, 1u);
    const ps3_sem_pump_fn kick = atomic_load(&g_ps3_sem_pump_glue.kick);
    if (kick) kick();       /* set by the first wait that parked */
}

/* ---- ticks ------------------------------------------------------------- */
enum { TICKS = 1500, TICK_US = 2000, SINGLES = 300 };
static atomic_int ticking;
static void* ticker(void* unused)
{
    (void)unused;
    uint64_t next = mono_ns();
    for (int i = 0; i < TICKS; i++) {
        next += TICK_US * 1000ull;
        for (uint64_t now = mono_ns(); now < next; now = mono_ns()) sleep_us((unsigned)((next - now) / 1000u) + 1u);
        tick();
    }
    atomic_store(&ticking, 0);
    return NULL;
}
static int run_ticks(void)
{
    frame_sem = make_sem(1);
    CHECK(frame_sem != 0);
    pthread_t t;
    atomic_store(&ticking, 1);
    pthread_create(&t, NULL, ticker, NULL);
    unsigned frames = 0, timeouts = 0;
    while (atomic_load(&ticking)) {
        const int32_t rc = call(sys_semaphore_wait, 1, frame_sem, 1000000, 0, 0);
        if (rc == (int32_t)CELL_ETIMEDOUT) {
            /* After the last tick an odd count never posts; only a timeout
             * while ticks still flow is a missed delivery. */
            if (atomic_load(&ticking)) timeouts++;
            continue;
        }
        CHECK(rc == CELL_OK);
        game_count = 0;
        frames++;
    }
    pthread_join(t, NULL);
    if (callbacks < TICKS) (void)call(sys_semaphore_wait, 1, frame_sem, 20000, 0, 0);
    const unsigned kicks = sys_semaphore_pump_kicks();
    printf("  ticks: timeouts=%u callbacks=%u out_of_order=%u frames=%u kicks=%u\n",
           timeouts, callbacks, out_of_order, frames, kicks);
    CHECK(timeouts == 0);
    CHECK(callbacks == TICKS && out_of_order == 0);
    /* Timing-dependent on a loaded host: most ticks are delivered by a kick
     * (one that lands while the waiter is not parked is found pending at its
     * next park), and a waiter woken after the next tick wastes one. */
    CHECK(frames <= TICKS / 2 && frames * 20u >= (TICKS / 2) * 19u);
    CHECK(kicks * 2u >= TICKS);

    return failures;
}

/* ---- singles: no later tick can rescue a lost wake ---------------------- */
static atomic_uint singles_done;
static void* single_ticker(void* unused)
{
    (void)unused;
    for (unsigned i = 0; i < SINGLES; i++) {
        sleep_us((i * 7919u) % 1000u);      /* lands anywhere in park/wait/unpark */
        tick();
        while (atomic_load(&singles_done) != i + 1u) sleep_us(50);
    }
    return NULL;
}
static int run_singles(void)
{
    frame_sem = make_sem(1);
    post_at = 1;
    pthread_t t;
    pthread_create(&t, NULL, single_ticker, NULL);
    unsigned ok = 0;
    for (unsigned i = 0; i < SINGLES; i++) {
        const int32_t rc = call(sys_semaphore_wait, 1, frame_sem, 2000000, 0, 0);
        CHECK(rc == CELL_OK);
        if (rc != CELL_OK) break;
        ok++;
        game_count = 0;
        atomic_store(&singles_done, i + 1u);
    }
    if (ok != SINGLES) atomic_store(&singles_done, SINGLES);   /* let the ticker finish */
    pthread_join(t, NULL);
    printf("  singles: %u of %u single ticks woke the parked waiter\n", ok, SINGLES);
    return failures;
}

/* ---- destroy: a parked or pumping waiter still counts -------------------- */
static atomic_int churning;
static void* churn(void* unused)
{
    (void)unused;
    /* Other semaphores created and destroyed while kicks fly: the kick must
     * never pick one that is gone. A short timed waiter parks on each. */
    while (atomic_load(&churning)) {
        const uint32_t id = make_sem(1);
        if (!id) continue;
        (void)call(sys_semaphore_wait, 3, id, 200, 0, 0);      /* parks, times out */
        (void)call(sys_semaphore_destroy, 3, id, 0, 0, 0);
    }
    return NULL;
}
static void* kicker(void* unused)
{
    (void)unused;
    while (atomic_load(&churning)) { tick(); sleep_us(100); }
    return NULL;
}
static void* waiter(void* rc)
{
    *(int32_t*)rc = call(sys_semaphore_wait, 2, frame_sem, 0, 0, 0);
    return NULL;
}
static int run_destroy(void)
{
    frame_sem = make_sem(1);
    post_at = 1000000;                      /* the handler never posts here */
    atomic_store(&churning, 1);
    pthread_t c, k, w;
    pthread_create(&c, NULL, churn, NULL);
    pthread_create(&k, NULL, kicker, NULL);
    int32_t wrc = -1;
    pthread_create(&w, NULL, waiter, &wrc);
    unsigned busy = 0;
    for (int i = 0; i < 400; i++) {         /* the waiter keeps pumping meanwhile */
        sleep_us(500);
        if (call(sys_semaphore_destroy, 1, frame_sem, 0, 0, 0) == (int32_t)CELL_EBUSY) busy++;
    }
    CHECK(busy == 400);
    CHECK(call(sys_semaphore_post, 1, frame_sem, 1, 0, 0) == CELL_OK);
    pthread_join(w, NULL);
    CHECK(wrc == CELL_OK);
    CHECK(call(sys_semaphore_destroy, 1, frame_sem, 0, 0, 0) == CELL_OK);
    atomic_store(&churning, 0);
    pthread_join(c, NULL);
    pthread_join(k, NULL);
    printf("  destroy: EBUSY %u of 400 times while the waiter was parked and pumping; "
           "%u callbacks pumped by it; then OK\n", busy, callbacks);
    CHECK(callbacks > 0);
    return failures;
}

/* ---- off ---------------------------------------------------------------- */
static int run_off(void)
{
    frame_sem = make_sem(1);
    post_at = 1;
    for (int i = 0; i < 5; i++) tick();
    const int32_t rc = call(sys_semaphore_wait, 1, frame_sem, 100000, 0, 0);
    printf("  off: wait -> 0x%08X, callbacks=%u, kicks=%u\n", (unsigned)rc, callbacks,
           sys_semaphore_pump_kicks());
    CHECK(rc == (int32_t)CELL_ETIMEDOUT && callbacks == 0 && sys_semaphore_pump_kicks() == 0);
    return failures;
}

/* sys_semaphore_wait reads several debug switches into lazily set statics on
 * its first call (an older pattern, racy only on that first call). Make that
 * call here, before any thread starts, so TSan reports only this fixture's
 * subject. */
static void warm_up(void)
{
    const uint32_t id = make_sem(1);
    (void)call(sys_semaphore_post, 1, id, 1, 0, 0);
    (void)call(sys_semaphore_wait, 1, id, 0, 0, 0);
    (void)call(sys_semaphore_destroy, 1, id, 0, 0, 0);
}

int main(int argc, char** argv)
{
    vm_base = calloc(1, 0x10000);
    if (argc > 1) {
        warm_up();
        atomic_store(&g_ps3_sem_pump_glue.pending, gcm_pending);
        atomic_store(&g_ps3_sem_pump_glue.pump, gcm_pump);
        if (!strcmp(argv[1], "ticks")) return run_ticks() != 0;
        if (!strcmp(argv[1], "singles")) return run_singles() != 0;
        if (!strcmp(argv[1], "destroy")) return run_destroy() != 0;
        if (!strcmp(argv[1], "off")) return run_off() != 0;
        return 2;
    }
    const char* modes[] = { "ticks", "singles", "destroy", "off" };
    int bad = 0;
    for (int i = 0; i < 4; i++) {
        const pid_t pid = fork();
        if (pid == 0) {
            setenv("PS3_GCM_WAIT_PUMP", i == 3 ? "0" : "1", 1);
            execl("/proc/self/exe", argv[0], modes[i], (char*)NULL);
            execlp(argv[0], argv[0], modes[i], (char*)NULL);
            _exit(127);
        }
        int status = 0;
        waitpid(pid, &status, 0);
        const int ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;
        printf("[%s] sem wait pump: %s\n", ok ? "PASS" : "FAIL", modes[i]);
        bad |= !ok;
    }
    if (!bad) puts("sem wait pump checks passed");
    return bad;
}
#endif
