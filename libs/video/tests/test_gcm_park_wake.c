/* The FIFO park wake (PS3_GCM_PARK_WAKE, cellGcmSys.c).
 *
 * A self-JUMP park is a hole another thread fills: PAIN's SPU mesh jobs patch
 * each into a JUMP to the commands they built, by DMA, which wakes nothing.
 * The walker stops its pass there and the frame clock waits for a kick. With
 * the wake on, the wait watches the park word and ends as soon as it changes;
 * with it off (=0), only a kick or the timeout ends it, as before.
 *
 * Block A (io 0) runs a probe method and parks at io 8; put is the end of
 * block B (io 0x8000). The park is patched into a JUMP to B by a second
 * thread -- through the PPU store path and by a raw write, as an SPU DMA
 * lands -- while the real cellGcm_fifo_kick_wait waits, PUT wake armed.
 * Checked: how long the wait took, that the next pass follows the patch, the
 * counters, a park nobody patches (the wait runs to its timeout), a PUT store
 * during a park wait (still wakes), and the poll window (past
 * PS3_GCM_PARK_POLL_MS on one park the wait is the plain one again).
 *
 * gcc -std=gnu17 -I include -I libs/video -pthread -ffunction-sections
 *   -fdata-sections -Wl,--gc-sections   (-Wl,-dead_strip on macOS)
 *   libs/video/tests/test_gcm_park_wake.c runtime/spu/spu_coherency.c
 *   runtime/platform/win32_compat.c -o /tmp/test_gcm_park_wake
 * Run it as is (wake on) and with PS3_GCM_PARK_WAKE=0; PS3_VBLANK_STATS=1
 * adds the counter checks.
 */
#include "../cellGcmSys.c"
#include "../rsx_tiling.c"
#include <pthread.h>
#if defined(__APPLE__)
#  include <pthread/qos.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
/* GCM_REPORT_SPU_HOLD_MS asks the SPU task layer whether a task round is
 * running; there are no SPU tasks here. */
int spu_taskset_round_busy(uint32_t* ts, uint32_t* tid) { (void)ts; (void)tid; return 0; }
uint8_t* vm_base;
uint32_t ppu_vm_size;
int g_resv_store_active;
uint32_t g_ww_lo, g_ww_hi;
void ppu_resv_break_store(uint64_t ea) { (void)ea; }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; }
ps3_guest_caller_fn g_ps3_guest_caller;
void spu_ch_wake(spu_context* ctx) { (void)ctx; }
/* The walker's other consumers: no graphics backend, no lifted program. */
PPU_THREAD_LOCAL ppu_context* g_active_ctx;
u32 g_rsx_last_reference;
void (*g_spu_put_hook)(uint32_t ea, uint32_t size);
void ppu_dump_bctrl_ring(uint32_t n, const char* why) { (void)n; (void)why; }
void ppu_dump_guest_stack(ppu_context* c, const char* why) { (void)c; (void)why; }
void rsx_state_init(rsx_state* state) { (void)state; }
/* Counts the probe methods the walker executed. */
#define PROBE_METHOD 0x1EE0u
static u32 seen_n;
int  rsx_process_method(rsx_state* state, u32 method, u32 data)
{
    (void)state; (void)data;
    if (method == PROBE_METHOD) seen_n++;
    return 0;
}
int  rsx_draw_engine_enabled(void) { return 0; }
/* The walker hands vertex-program runs here (#554) and asks for a vtable
 * backend (#466); this fixture has neither. */
int  rsx_process_vp_run(rsx_state* state, u32 method, const u8* src, u32 count)
{ (void)state; (void)method; (void)src; (void)count; return 0; }
rsx_backend* rsx_get_backend(void) { return NULL; }
void rsx_draw_engine_method(u32 method, u32 arg) { (void)method; (void)arg; }
void rsx_draw_engine_note_flip(u32 buffer_id) { (void)buffer_id; }
void rsx_draw_engine_guest_read(u32 l, u32 o, u32 p, u32 w, u32 h) { (void)l; (void)o; (void)p; (void)w; (void)h; }
void rsx_draw_engine_guest_wrote(u32 l, u32 o, u32 p, u32 w, u32 h) { (void)l; (void)o; (void)p; (void)w; (void)h; }
int  rsx_live_draw_enabled(void) { return 0; }
void rsx_live_draw_method(u32 method, u32 arg) { (void)method; (void)arg; }
int  rsx_live_draw_blit(u32 a, u32 b, u32 c, u32 d, u32 e, u32 f, u32 w, u32 h)
{ (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; (void)w; (void)h; return 0; }
void rsx_live_draw_note_resolve(u32 a, u32 b, u32 c, u32 d) { (void)a; (void)b; (void)c; (void)d; }
int  rsx_live_draw_resolve_blit(u32 a, u32 b, u32 c, u32 d, u32 e, u32 f, u32 g, u32 h,
                                u32 i, u32 j, u32 k, u32 l)
{ (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; (void)g; (void)h; (void)i; (void)j; (void)k; (void)l; return 0; }

void rsx_raise_user_cmd(uint32_t arg) { (void)arg; }

#define IO_ADDR   0x00100000u
#define IO_SIZE   0x00100000u
#define CMD_SIZE  0x00010000u
#define SEG_B     0x00008000u

static uint32_t g_heap = 0x00400000u;
static u32 host_alloc(u32 size, u32 align)
{
    if (!align) align = 4;
    g_heap = (g_heap + align - 1u) & ~(align - 1u);
    u32 r = g_heap;
    g_heap += size;
    return r;
}
static void gw32(uint32_t ea, uint32_t v)
{
    uint8_t* p = vm_base + ea;
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}
static void host_w32(u32 ea, u32 v) { gw32(ea, v); }

static int fails;
static void check(int ok, const char* what)
{
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    fails += !ok;
}

/* One method, count 1, subchannel 0. */
static u32 put_probe(u32 io, u32 arg)
{
    gw32(IO_ADDR + io, (1u << 18) | PROBE_METHOD);
    gw32(IO_ADDR + io + 4, arg);
    return io + 8;
}

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000u;
}

/* Under TSan the park is patched only DMA-style (one atomic word): the PPU
 * store path is a plain store by design, which TSan reports against the
 * poll's atomic load. */
#if defined(__SANITIZE_THREAD__)
#  define PPU_PATCH_OK 0
#elif defined(__has_feature)
#  if __has_feature(thread_sanitizer)
#    define PPU_PATCH_OK 0
#  endif
#endif
#ifndef PPU_PATCH_OK
#  define PPU_PATCH_OK 1
#endif
struct patch { uint32_t ea, value; unsigned delay_us; int raw; };
static uint64_t g_patched_us;   /* when the last patch landed (atomic) */
static void* patcher(void* arg)
{
    const struct patch* p = (const struct patch*)arg;
    usleep(p->delay_us);
    if (p->raw) {   /* as an SPU DMA lands: one word, no PPU store hook */
        __atomic_store_n((uint32_t*)(vm_base + p->ea), __builtin_bswap32(p->value), __ATOMIC_RELEASE);
    } else {
        vm_write32(p->ea, p->value);
    }
    __atomic_store_n(&g_patched_us, now_us(), __ATOMIC_RELEASE);
    return NULL;
}

/* Wait with `ms` while `p` lands from another thread; elapsed microseconds. */
static uint64_t g_wake_after_patch_us;   /* the last wait's end minus the patch time */
static uint64_t timed_wait(unsigned ms, struct patch* p)
{
    pthread_t t;
    if (p && pthread_create(&t, NULL, patcher, p) != 0) { puts("FAIL: thread"); exit(1); }
    const uint64_t t0 = now_us();
    cellGcm_fifo_kick_wait(ms);
    const uint64_t t1 = now_us(), dt = t1 - t0;
    if (p) {
        pthread_join(t, NULL);
        const uint64_t at = __atomic_load_n(&g_patched_us, __ATOMIC_ACQUIRE);
        g_wake_after_patch_us = t1 > at ? t1 - at : 0;
    }
    return dt;
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
#if defined(__APPLE__)
    /* As the game's frame clock runs (PS3_THREAD_QOS): at the default QoS
     * macOS coalesces short sleeps by milliseconds. */
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
#endif
    const char* e = getenv("PS3_GCM_PARK_WAKE");
    const int wake = !(e && *e == '0');
    const int stats = getenv("PS3_VBLANK_STATS") && *getenv("PS3_VBLANK_STATS") != '0';
    setenv("PS3_GCM_PARK_POLL_MS", "60", 1);
    setenv("GCM_PARK_GRACE_MS", "100000", 1);   /* no jump-park resync in this fixture */
    const uint32_t vm_size = ppu_hle_inject_base + 0x10000u;
    vm_base = (uint8_t*)calloc(1, vm_size);
    if (!vm_base) { puts("FAIL: guest VM alloc"); return 1; }
    ppu_vm_size = vm_size;

    u32 ctx_out = host_alloc(4, 4);
    (void)cellGcmSetupContext(ctx_out, CMD_SIZE, IO_SIZE, IO_ADDR, host_alloc, host_w32);
    const uint32_t ctrl = (uint32_t)(uintptr_t)cellGcmGetControlRegister();

    const u32 park = put_probe(0, 0);
    const u32 put = put_probe(SEG_B, 1);
    const u32 put2 = put_probe(put, 2);         /* a later PUT, for the PUT-wake case */
    const u32 park_word = 0x20000000u | park, jump_b = 0x20000000u | SEG_B;
    vm_write32(ctrl + 4, 0);
    vm_write32(ctrl + 0, put);
    cellGcm_fifo_kick_wait(0);   /* installs the PUT watch before any other thread runs */

    enum { ROUNDS = 40, DELAY_US = 8000, TIMEOUT_MS = 40 };
    uint64_t worst = 0, best = ~0ull, sum = 0, lat_sum = 0, lat_worst = 0;
    int followed = 0;
    for (int i = 0; i < ROUNDS; i++) {
        gw32(IO_ADDR + park, park_word);
        s_fifo_getoff = 0;
        seen_n = 0;
        cellGcm_rsx_process_fifo();
        if (s_fifo_getoff != park || !GCM_A32_LOAD(s_park_watch_on) || GCM_A32_LOAD(s_park_watch_io) != park) {
            check(0, "the walker parks and records the park"); break;
        }
        struct patch p = { IO_ADDR + park, jump_b, DELAY_US, PPU_PATCH_OK ? i & 1 : 1 };
        const uint64_t dt = timed_wait(TIMEOUT_MS, &p);
        sum += dt; if (dt > worst) worst = dt; if (dt < best) best = dt;
        lat_sum += g_wake_after_patch_us;
        if (g_wake_after_patch_us > lat_worst) lat_worst = g_wake_after_patch_us;
        cellGcm_rsx_process_fifo();
        followed += s_fifo_getoff == put && seen_n == 2 && !GCM_A32_LOAD(s_park_watch_on);
    }
    printf("park patched %u us into a %u ms wait: waited %llu..%llu us, mean %llu (wake=%d)\n",
           DELAY_US, TIMEOUT_MS, (unsigned long long)best, (unsigned long long)worst,
           (unsigned long long)(sum / ROUNDS), wake);
    printf("wait end after the patch landed: mean %llu us, worst %llu us\n",
           (unsigned long long)(lat_sum / ROUNDS), (unsigned long long)lat_worst);
    check(followed == ROUNDS, "every patched park is followed to put by the next pass");
    if (wake) {
        check(best >= DELAY_US - 500 && lat_sum / ROUNDS < 1500 && lat_worst < 6000,
              "wake on: the wait ends about one poll after the patch lands (PPU store and raw write)");
        uint64_t c[10]; cellGcm_pace_counters(c, 10);
        check(c[7] == ROUNDS, "wake on: every round counted as a patch wake");
    } else {
        check(best >= TIMEOUT_MS * 1000u - 1000u, "wake off: the patch wakes nothing, the wait runs to its timeout");
        if (stats) {
            uint64_t c[10]; cellGcm_pace_counters(c, 10);
            const double late_ms = c[6] / 1e6 / ROUNDS;
            printf("park_late per round %.2f ms (expected ~%u)\n", late_ms, TIMEOUT_MS - DELAY_US / 1000u);
            check(late_ms > (TIMEOUT_MS - DELAY_US / 1000u) - 3 && late_ms < (TIMEOUT_MS - DELAY_US / 1000u) + 3,
                  "wake off, stats on: the time spent at an already-patched park is measured");
            check(c[7] == 0, "wake off: no patch wakes");
        }
    }

    /* Nobody patches: the wait runs out. */
    gw32(IO_ADDR + park, park_word);
    s_fifo_getoff = 0;
    cellGcm_rsx_process_fifo();
    uint64_t dt = timed_wait(15, NULL);
    check(dt >= 14000 && dt < 40000, "an unpatched park waits out the timeout");

    /* A PUT store while parked still wakes the wait (the PUT wake is armed
     * around the park watch). */
    struct patch p = { ctrl + 0, put2, 3000, 0 };
    dt = timed_wait(TIMEOUT_MS, &p);
    check(dt < 20000, "a PUT store during a park wait still ends it");
    cellGcm_rsx_process_fifo();   /* reads the new PUT, parks again */

    /* Past the poll window on one park the wait is the plain one: a patch is
     * then only seen at the timeout, in both modes. */
    usleep(70000);
    cellGcm_rsx_process_fifo();
    check(s_fifo_getoff == park, "still parked after the poll window");
    struct patch q = { IO_ADDR + park, jump_b, 3000, 1 };
    dt = timed_wait(20, &q);
    check(dt >= 19000, "past PS3_GCM_PARK_POLL_MS the park is not polled");
    cellGcm_rsx_process_fifo();
    check(s_fifo_getoff == put2, "...and the next pass follows it");

    printf("[gcm_park_wake] %s (%d failed, wake=%d stats=%d)\n", fails ? "FAIL" : "PASS", fails, wake, stats);
    free(vm_base);
    return fails ? 1 : 0;
}
