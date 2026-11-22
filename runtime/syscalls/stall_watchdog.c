/*
 * ps3recomp - SPU/PPU stall watchdog (diagnostic only; changes no guest state)
 *
 * The failure it exists for: an SPU task spins on GETLLAR waiting for a word
 * nobody will change, while the PPU thread that should change it sits in
 * sys_event_queue_receive waiting for that SPU. Each side looks healthy on its
 * own -- the SPU is busy, the PPU is parked in a normal wait -- so nothing
 * else in the runtime reports it, and by the time anyone attaches a sampler
 * the event that went missing is long gone.
 *
 * When the main PPU thread (tid 1) has been in one sys_event_queue_receive for
 * more than two seconds AND an SPU's GETLLAR spin detector has fired in that
 * time, this prints, once:
 *   - every registered SPU reservation holder (image, pc, lr, reservation,
 *     events, mailbox depths) and the 128-byte line it holds,
 *   - the lines the spin detector named,
 *   - every PPU thread's reservation address and lr,
 *   - every live lv2 event queue (depth, capacity, waiters, push failures),
 *   - every PPU thread currently parked in a receive and for how long,
 *   - the last SPU user events (sys_spu_thread_send_event from SPURS tasks)
 *     with the result each one was answered with.
 *
 * Cost when nothing spins: a timestamp store per blocking receive and a
 * counter per SPU user event. The watchdog thread starts on the first spin
 * report. PS3_HAVOK_WATCHDOG=0 leaves every hook NULL.
 */

#include "../spu/spu_context.h"
#include "../platform/win32_compat.h"
#include "ps3emu/log_first.h"
#include "stall_watchdog.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dumpers that live next to the state they read. */
void spu_coh_dump_state(FILE* f, const uint8_t* base);
void spu_coh_trace_arm(uint8_t* const* vm_base_var);
extern uint8_t* vm_base;
void sys_event_dump_state(FILE* f);
/* ppu_loader.cpp is compiled per game; elsewhere there are no PPU
 * reservations to show. Same weak-default pattern as ppu_resv_break_line. */
#if defined(_MSC_VER)
void ppu_resv_dump_state(FILE* f);
void ppu_resv_dump_state_absent(FILE* f) { fprintf(f, "  (no PPU reservation registry linked)\n"); }
#pragma comment(linker, "/alternatename:ppu_resv_dump_state=ppu_resv_dump_state_absent")
#else
__attribute__((weak)) void ppu_resv_dump_state(FILE* f)
{
    fprintf(f, "  (no PPU reservation registry linked)\n");
}
#endif

static int stall_wd_enabled(void)
{
    static int v = -1;
    if (v < 0) { const char* e = getenv("PS3_HAVOK_WATCHDOG"); v = !(e && e[0] == '0'); }
    return v;
}

/* --- PPU threads parked in a blocking receive ---------------------------- */
#define STALL_TID_MAX 128
static volatile uint64_t s_recv_since_ms[STALL_TID_MAX];   /* 0 = not in a receive */
static volatile uint32_t s_recv_queue[STALL_TID_MAX];

static void ps3_stall_recv_begin(uint64_t tid, uint32_t queue)
{
    if (tid >= STALL_TID_MAX || !stall_wd_enabled()) return;
    s_recv_queue[tid] = queue;
    __atomic_store_n(&s_recv_since_ms[tid], (uint64_t)GetTickCount64() | 1u, __ATOMIC_RELEASE);
}

/* SPU user events as PPU threads took them off a queue, per port: the other
 * half of the send ring below. A port-10 send with result 0 and no matching
 * receive means the event sat in (or vanished from) the queue; a receive
 * means the helper had it and the loss is downstream of the event. */
#define STALL_SPU_EVT_SOURCE 0xFFFFFFFF53505501ull
#define STALL_PORTS 64
#define STALL_PORT_RING 8
typedef struct { uint64_t ms; uint32_t tid, spu_id, value, data; } stall_rcv;
static stall_rcv s_rcv_ring[STALL_PORTS][STALL_PORT_RING];
static volatile unsigned long s_rcv_n[STALL_PORTS];

static void ps3_stall_recv_end(uint64_t tid, int64_t rc, uint64_t source, uint64_t d1, uint64_t d2, uint64_t d3)
{
    if (tid < STALL_TID_MAX) __atomic_store_n(&s_recv_since_ms[tid], 0, __ATOMIC_RELEASE);
    if (rc != 0 || source != STALL_SPU_EVT_SOURCE || !stall_wd_enabled()) return;
    const unsigned port = (unsigned)(d2 >> 32) & (STALL_PORTS - 1);
    const unsigned long i = __atomic_fetch_add(&s_rcv_n[port], 1, __ATOMIC_RELAXED) % STALL_PORT_RING;
    stall_rcv* r = &s_rcv_ring[port][i];
    r->ms = GetTickCount64(); r->tid = (uint32_t)tid; r->spu_id = (uint32_t)d1;
    r->value = (uint32_t)d2 & 0xFFFFFFu; r->data = (uint32_t)d3;
}

/* --- SPU user events (sys_spu_thread_send_event from SPU code) ----------- */
typedef struct {
    uint64_t ms;
    int      image_id;
    uint32_t spu_ctx_lo;   /* low bits of the context pointer, to tell tasks apart */
    uint32_t pc;
    uint32_t port, value, data, queue, result;
} stall_evt;
/* Per port, so a chatty port (audio, SPURS printf) cannot push the one send
 * that matters out of the ring before the dump. */
static stall_evt s_evt_ring[STALL_PORTS][STALL_PORT_RING];
static volatile unsigned long s_evt_port_n[STALL_PORTS];
static volatile unsigned long s_evt_n;
static volatile unsigned long s_evt_fail_full, s_evt_fail_unbound, s_evt_fail_nodata;
/* Defined in spu_channels.c, written by spu_rchcnt(SPU_RdInMbox). */
extern volatile unsigned long g_spurs_inmbox_busy_polls;
extern volatile uint32_t g_spurs_inmbox_busy_where;

static void ps3_stall_user_event(const struct spu_context* spu, uint32_t port, uint32_t value,
                          uint32_t data, uint32_t queue, uint32_t result, int kind)
{
    if (!stall_wd_enabled()) return;
    /* kind: 0 delivered, 1 queue full, 2 unbound, 3 no data word */
    if (kind == 1) __atomic_fetch_add(&s_evt_fail_full, 1, __ATOMIC_RELAXED);
    else if (kind == 2) __atomic_fetch_add(&s_evt_fail_unbound, 1, __ATOMIC_RELAXED);
    else if (kind == 3) __atomic_fetch_add(&s_evt_fail_nodata, 1, __ATOMIC_RELAXED);
    __atomic_fetch_add(&s_evt_n, 1, __ATOMIC_RELAXED);
    const unsigned pi = port & (STALL_PORTS - 1);
    unsigned long i = __atomic_fetch_add(&s_evt_port_n[pi], 1, __ATOMIC_RELAXED) % STALL_PORT_RING;
    stall_evt* e = &s_evt_ring[pi][i];
    e->ms = GetTickCount64();
    e->image_id = spu ? spu->image_id : -1;
    e->spu_ctx_lo = (uint32_t)(uintptr_t)spu;
    e->pc = spu ? (uint32_t)(spu->pc & 0x3FFFFu) : 0;
    e->port = port; e->value = value; e->data = data; e->queue = queue; e->result = result;
}

/* --- GETLLAR spins ------------------------------------------------------- */
#define STALL_SPIN_MAX 8
static volatile uint64_t s_spin_last_ms;
static uint32_t s_spin_ea[STALL_SPIN_MAX];
static volatile unsigned s_spin_n;
static volatile LONG s_wd_started;
static volatile int  s_wd_fired;
static uint64_t      s_wd_fired_since;

static void stall_dump(uint64_t main_wait_ms)
{
    FILE* f = stderr;
    fprintf(f, "[stall-wd] ===== main PPU thread in sys_event_queue_receive(q=%u) for %llu ms while SPU tasks spin on GETLLAR =====\n",
            (unsigned)s_recv_queue[1], (unsigned long long)main_wait_ms);
    fprintf(f, "[stall-wd] SPU reservation holders:\n");
    spu_coh_dump_state(f, vm_base);
    fprintf(f, "[stall-wd] lines named by the GETLLAR spin detector:\n");
    { extern uint8_t* vm_base;
      unsigned n = s_spin_n < STALL_SPIN_MAX ? s_spin_n : STALL_SPIN_MAX;
      for (unsigned i = 0; i < n; i++) {
          const uint8_t* m = vm_base ? vm_base + (s_spin_ea[i] & ~127u) : NULL;
          fprintf(f, "  line 0x%08X:", s_spin_ea[i] & ~127u);
          for (int k = 0; m && k < 128; k += 4)
              fprintf(f, "%s%02X%02X%02X%02X", (k % 32) ? " " : "\n    ", m[k], m[k + 1], m[k + 2], m[k + 3]);
          fprintf(f, "\n");
      } }
    fprintf(f, "[stall-wd] PPU reservations:\n");
    ppu_resv_dump_state(f);
    fprintf(f, "[stall-wd] PPU threads in a blocking receive:\n");
    { const uint64_t now = GetTickCount64();
      for (unsigned t = 0; t < STALL_TID_MAX; t++) {
          uint64_t s = s_recv_since_ms[t];
          if (s) fprintf(f, "  tid %u queue %u for %llu ms\n", t, (unsigned)s_recv_queue[t],
                         (unsigned long long)(now - (s & ~1ull)));
      } }
    fprintf(f, "[stall-wd] lv2 event queues:\n");
    sys_event_dump_state(f);
    fprintf(f, "[stall-wd] SPU user events: %lu sent, %lu lost to a full queue, %lu to an unbound port, %lu without a data word\n",
            (unsigned long)s_evt_n, (unsigned long)s_evt_fail_full, (unsigned long)s_evt_fail_unbound,
            (unsigned long)s_evt_fail_nodata);
    { const uint64_t now = GetTickCount64();
      for (unsigned p = 0; p < STALL_PORTS; p++) {
          const unsigned long ns = s_evt_port_n[p], nr = s_rcv_n[p];
          if (!ns && !nr) continue;
          fprintf(f, "  port %u: %lu sent, %lu received by PPU threads\n", p, ns, nr);
          for (unsigned long i = ns > STALL_PORT_RING ? ns - STALL_PORT_RING : 0; i < ns; i++) {
              const stall_evt* e = &s_evt_ring[p][i % STALL_PORT_RING];
              fprintf(f, "    sent -%6llu ms img=%d ctx=..%05X pc=0x%05X value=0x%06X data=0x%08X queue=%u result=0x%08X\n",
                      (unsigned long long)(now - e->ms), e->image_id, e->spu_ctx_lo & 0xFFFFFu, e->pc,
                      e->value, e->data, e->queue, e->result);
          }
          for (unsigned long i = nr > STALL_PORT_RING ? nr - STALL_PORT_RING : 0; i < nr; i++) {
              const stall_rcv* r = &s_rcv_ring[p][i % STALL_PORT_RING];
              fprintf(f, "    rcvd -%6llu ms tid=%u spu=0x%X value=0x%06X data=0x%08X\n",
                      (unsigned long long)(now - r->ms), r->tid, r->spu_id, r->value, r->data);
          }
      } }
    fprintf(f, "[stall-wd] SPURS rchcnt(SPU_RdInMbox) found a word waiting %lu times; last img=%u pc=0x%05X\n",
            (unsigned long)g_spurs_inmbox_busy_polls, g_spurs_inmbox_busy_where >> 24,
            g_spurs_inmbox_busy_where & 0x3FFFFu);
    { extern void havok_rerequest_report(FILE*); havok_rerequest_report(f); }
    fprintf(f, "[stall-wd] ===== end =====\n");
    fflush(f);
}

static DWORD WINAPI stall_wd_thread(LPVOID p)
{
    (void)p;
    for (;;) {
        Sleep(250);
        const uint64_t now = GetTickCount64();
        const uint64_t since = __atomic_load_n(&s_recv_since_ms[1], __ATOMIC_ACQUIRE);
        const uint64_t spin = __atomic_load_n(&s_spin_last_ms, __ATOMIC_ACQUIRE);
        if (!since || !spin) continue;
        const uint64_t waited = now - (since & ~1ull);
        /* Once per receive episode of the main thread, at most four times: a
         * load screen can park it for seconds while policy modules poll, and
         * that one must not use up the report a real stall needs. */
        if (waited > 2000 && now - spin < 2000 && since != s_wd_fired_since && s_wd_fired < 4) {
            s_wd_fired_since = since;
            s_wd_fired++;
            stall_dump(waited);
        }
    }
    return 0;
}

static void ps3_stall_spin(uint32_t ea)
{
    if (!stall_wd_enabled()) return;
    unsigned n = s_spin_n;
    int seen = 0;
    for (unsigned i = 0; i < n && i < STALL_SPIN_MAX; i++)
        if ((s_spin_ea[i] & ~127u) == (ea & ~127u)) { seen = 1; break; }
    if (!seen && n < STALL_SPIN_MAX) { s_spin_ea[n] = ea; s_spin_n = n + 1; }
    __atomic_store_n(&s_spin_last_ms, (uint64_t)GetTickCount64(), __ATOMIC_RELEASE);
    if (!s_wd_started && !InterlockedExchange(&s_wd_started, 1)) {
        HANDLE th = CreateThread(NULL, 0, stall_wd_thread, NULL, 0, NULL);
        if (th) CloseHandle(th);
    }
}

void ps3_stall_watchdog_install(void)
{
    if (!stall_wd_enabled()) return;
    spu_coh_trace_arm(&vm_base);   /* PS3_STALL_TRACE_LINE */
    g_ps3_stall_user_event = ps3_stall_user_event;
    g_ps3_stall_recv_begin = ps3_stall_recv_begin;
    g_ps3_stall_recv_end   = ps3_stall_recv_end;
    g_ps3_stall_spin       = ps3_stall_spin;
}

#ifdef __cplusplus
}
#endif
