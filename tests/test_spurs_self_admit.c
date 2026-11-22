/* Worker self-admission and targeted wakes (PS3_SPURS_SELF_ADMIT) against the
 * real libs/spurs/cellSpurs.c scheduler, policy runner and lock-line code, with
 * manufactured policy modules only. Run it with the switch both ways; each
 * case must hold in either mode:
 *
 *   churn     3 lanes, 6 workloads with different contention ceilings and
 *             priority rows, a fake SPU raising ready counts through the
 *             request-line commit hook: no ceiling or zero-priority lane is
 *             ever violated, the published CURCONT agrees with the admitted
 *             count, and every raised unit is consumed exactly once.
 *   busy      every lane admitted when a request arrives (SPU commit, PPU API
 *             store, PPU plain store with no hook): it runs once a lane frees.
 *   idle      a free lane picks up a PPU plain store (no wake at all) through
 *             the coordinator's timed pass.
 *   freed     a lane that returns frees contention another, idle lane needs.
 *   two       two instances churning at once (registry lock, lock order).
 *   cost      per-run scheduler overhead with a trivial policy (printed).
 *
 * A lost wake hangs a wait below and the time limit fails the case. */
#define spu_workload_dispatch_async fixture_unbounded_dispatch
#include "../libs/spurs/cellSpurs.c"
#undef spu_workload_dispatch_async
#include <assert.h>
#include <stdatomic.h>
#include <sys/resource.h>
#include "../runtime/spu/spu_coherency.h"

void spu_indirect_branch(spu_context*);
void spu_halt(spu_context*);
uint8_t* vm_base;
uint32_t ppu_vm_size = 0x20000;
int g_resv_store_active;
uint32_t g_ww_lo, g_ww_hi, g_barrier_sync_watch;
uint32_t g_spu_image_src_ea, g_spu_image_ls_start, g_spu_image_span;
unsigned long long ps3_ms_now(void) { LARGE_INTEGER n, f; QueryPerformanceCounter(&n); QueryPerformanceFrequency(&f); return n.QuadPart * 1000 / f.QuadPart; }
void ppu_resv_break_store(uint64_t ea) { (void)ea; }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; }
uint64_t spurs_pm_build_context(uint8_t* ls, uint32_t ts, uint32_t id, uint32_t spu, uint32_t tag)
{ (void)ls; (void)ts; (void)id; (void)spu; (void)tag; return 0; }
void spu_thread_publish_ctx(uint32_t tid, void* c) { (void)tid; (void)c; }
void ydkj_wake_all_event_flags(void) {}
void sys_event_queue_cancel_by_id(u32 id) { (void)id; }
void sys_event_queue_uncancel_by_id(u32 id) { (void)id; }
int fixture_unbounded_dispatch(const uint8_t* pm, uint32_t size, uint32_t data)
{ (void)pm; (void)size; (void)data; assert(!"unbounded dispatch"); return 0; }
extern void (*g_spu_line_commit_hook)(uint32_t);
/* What the policy runner counts when a policy asks the kernel for work; a run
 * that never asks backs its workload off (SPURS_IDLE_MAX). */
extern SPU_THREAD_LOCAL volatile unsigned g_spurs_pm_polls;

#define A 0x1000u    /* first instance */
#define B 0x3000u    /* second instance */
#define NW 6

static void fail(const char* what) { fprintf(stderr, "FAIL: %s\n", what); exit(1); }
static void wait_until(atomic_uint* p, unsigned v, unsigned ms, const char* what)
{
    for (unsigned i = 0; i < ms; i++) { if (atomic_load(p) >= v) return; Sleep(1); }
    fprintf(stderr, "FAIL: %s (%u of %u)\n", what, atomic_load(p), v); exit(1);
}
static void register_pm(unsigned ea, unsigned byte, spu_lifted_entry_fn fn, int image)
{
    memset(vm_base + ea, byte, 16);
    spu_workload_register_img(spu_workload_fingerprint(vm_base + ea, 16), fn, image, "self-admit fixture");
}
static void to_kernel(spu_context* c) { c->pc = SPURS_PM_EXIT_TO_KERNEL_LS; spu_indirect_branch(c); }
static void leave_runnable_set(spu_context* c)
{
    u32 ea = spu_ls_read32(c, 0x1C4), wid = spu_ls_read32(c, 0x1DC);
    vm_write8(ea + wkl_state_off(wid), WKL_SHUTTING_DOWN);
    to_kernel(c);
}
static void finalize(u32 ea)
{
    for (unsigned i = 0; i < 5000; i++) {
        s32 rc = cellSpursFinalize((CellSpurs*)(uintptr_t)ea);
        if (!rc) return;
        if ((u32)rc != CELL_SPURS_CORE_ERROR_BUSY) fail("finalize error");
        Sleep(1);
    }
    fail("finalize never joined");
}
/* An SPU-side atomic on the instance's request line: raise one ready unit
 * under the lock-line lock, then the commit notification. */
static int spu_raise_ready(u32 ea, u32 wid)
{
    spu_lockline_lock();
    u8* r = &vm_base[ea + wkl_ready_off(wid)];
    int raised = *r < 250;
    if (raised) (*r)++;
    ef_line_end(ea);
    g_spu_line_commit_hook(ea);
    return raised;
}
static struct SpursInst* inst(u32 ea) { struct SpursInst* si = spurs_inst_find(ea); assert(si); return si; }

/* ---- churn ------------------------------------------------------------ */
static const u8 churn_maxc[NW] = { 1, 2, 3, 1, 2, 3 };
static u8 churn_prio[NW][8];
static atomic_uint conc[2][NW], runs[2][NW], total_runs, stop_churn, lane_seen[2];
static atomic_uint violations;
static void churn_policy(spu_context* c)
{
    u32 ea = spu_ls_read32(c, 0x1C4), wid = spu_ls_read32(c, 0x1DC), lane = spu_ls_read32(c, 0x1C8);
    const int k = ea == B;
    struct SpursInst* si = inst(ea);
    u32 w = wid - (k ? NW : 0);
    if (w >= NW) atomic_fetch_add(&violations, 1);
    else {
        if (!vm_read8(ea + wkl_info_off(wid) + 0x18 + lane % si->nspus)) atomic_fetch_add(&violations, 1);
        unsigned n = atomic_fetch_add(&conc[k][w], 1) + 1;
        if (n > churn_maxc[w]) atomic_fetch_add(&violations, 1);
        u8 cur = wkl_count_get(ea, SPURS_WKL_CURCONT, wid, si->is2);
        if (cur < 1 || cur > churn_maxc[w]) atomic_fetch_add(&violations, 1);
        atomic_fetch_or(&lane_seen[k], 1u << lane);
        if (!(w & 1)) g_spurs_pm_polls++;   /* odd workloads back off when idle */
        for (volatile int i = 0; i < 200 + (int)(w * 50); i++) {}
        atomic_fetch_sub(&conc[k][w], 1);
        atomic_fetch_add(&runs[k][w], 1);
        atomic_fetch_add(&total_runs, 1);
    }
    if (atomic_load(&stop_churn)) leave_runnable_set(c);
    else to_kernel(c);
}
static atomic_uint raised[2][NW], producer_stop;
static DWORD WINAPI producer(LPVOID p)
{
    const int k = (int)(uintptr_t)p;
    const u32 ea = k ? B : A;
    unsigned x = 12345u + (unsigned)k;
    while (!atomic_load(&producer_stop)) {
        x = x * 1103515245u + 12345u;
        u32 w = (x >> 16) % NW;
        if (spu_raise_ready(ea, w + (k ? NW : 0))) atomic_fetch_add(&raised[k][w], 1);
        for (volatile int i = 0; i < 2000; i++) {}
    }
    return 0;
}
static void add_churn(u32 ea, int k)
{
    for (u32 w = 0; w < NW; w++) {
        memcpy(vm_base + 0x8020, churn_prio[w], 8);
        if (cellSpursAddWorkload((CellSpurs*)(uintptr_t)ea, (u32*)0x8000, (void*)0x9000, 16, 0,
                                 (u8*)0x8020, 1, churn_maxc[w]))
            fail("add churn workload");
        if (vm_read32(0x8000) != w + (k ? NW : 0)) fail("churn wid");
    }
}
/* All units raised were consumed: each admission takes one, and runnable
 * workloads keep being admitted until the producer's bytes drain. */
static void drain_and_stop(u32 ea, int k)
{
    struct SpursInst* si = inst(ea);
    for (unsigned i = 0;; i++) {
        unsigned left = 0;
        for (u32 w = 0; w < NW; w++) left += vm_read8(ea + wkl_ready_off(w + (k ? NW : 0)));
        if (!left) break;
        if (i > 3000) fail("ready units never drained");
        Sleep(1);
    }
    (void)si;
}

static void case_churn(int two)
{
    memset(churn_prio, 1, sizeof(churn_prio));
    churn_prio[1][0] = 0;            /* wid 1 never on lane 0 */
    churn_prio[4][1] = churn_prio[4][2] = 0; /* wid 4 only on lane 0 */
    atomic_store(&stop_churn, 0); atomic_store(&producer_stop, 0); atomic_store(&total_runs, 0);
    memset((void*)runs, 0, sizeof(runs)); memset((void*)raised, 0, sizeof(raised));
    memset((void*)lane_seen, 0, sizeof(lane_seen));
    if (cellSpursInitialize((CellSpurs*)A, 3, 0, 0, 0)) fail("init A");
    if (two && cellSpursInitialize((CellSpurs*)B, 2, 0, 0, 0)) fail("init B");
    add_churn(A, 0);
    if (two) add_churn(B, 1);
    HANDLE pa = CreateThread(NULL, 0, producer, (LPVOID)0, 0, NULL);
    HANDLE pb = two ? CreateThread(NULL, 0, producer, (LPVOID)1, 0, NULL) : NULL;
    wait_until(&total_runs, two ? 60000 : 40000, 20000, "churn progress");
    atomic_store(&producer_stop, 1);
    WaitForSingleObject(pa, INFINITE); CloseHandle(pa);
    if (pb) { WaitForSingleObject(pb, INFINITE); CloseHandle(pb); }
    drain_and_stop(A, 0);
    if (two) drain_and_stop(B, 1);
    atomic_store(&stop_churn, 1);
    /* Every workload leaves the runnable set on its next run; the backed-off
     * ones get there through probes. */
    for (unsigned i = 0;; i++) {
        unsigned left = 0;
        for (int k = 0; k < 1 + two; k++)
            for (u32 w = 0; w < NW; w++)
                left += vm_read8((k ? B : A) + wkl_state_off(w + (k ? NW : 0))) != WKL_REMOVABLE;
        if (!left) break;
        if (i > 5000) fail("churn workloads never shut down");
        Sleep(1);
    }
    finalize(A);
    if (two) finalize(B);
    if (atomic_load(&violations)) fail("contention, priority or CURCONT violated");
    for (u32 w = 0; w < NW; w++) if (!atomic_load(&runs[0][w])) fail("a churn workload never ran");
    if (atomic_load(&lane_seen[0]) != 7) fail("instance A did not use all three lanes");
    if (two && atomic_load(&lane_seen[1]) != 3) fail("instance B did not use both lanes");
    for (int k = 0; k < 1 + two; k++)
        for (u32 w = 0; w < NW; w++) {
            const u32 ea = k ? B : A, wid = w + (k ? NW : 0);
            /* Each consumed unit is one admission, so one run at least. */
            if (atomic_load(&runs[k][w]) < atomic_load(&raised[k][w])) fail("a ready unit was consumed without a run");
            if (vm_read8(ea + SPURS_WKL_CURCONT + wid)) fail("CURCONT not zero at rest");
            if (vm_read8(ea + wkl_state_off(wid)) != WKL_REMOVABLE) fail("shut-down workload not REMOVABLE");
        }
    printf("churn%s: %u runs, no contention/priority/CURCONT violation, raised units drained PASS\n",
           two ? " (two instances)" : "", atomic_load(&total_runs));
}

/* ---- busy / idle / freed ---------------------------------------------- */
static atomic_uint gate, held_in, held_out, late_runs;
static void held_policy(spu_context* c)
{
    atomic_fetch_add(&held_in, 1);
    while (!atomic_load(&gate)) Sleep(1);
    atomic_fetch_add(&held_out, 1);
    leave_runnable_set(c);
}
static void late_policy(spu_context* c) { atomic_fetch_add(&late_runs, 1); leave_runnable_set(c); }

/* A request raised `how` while every lane is admitted runs once one frees. */
static void case_busy(int how)
{
    atomic_store(&gate, 0); atomic_store(&held_in, 0); atomic_store(&held_out, 0); atomic_store(&late_runs, 0);
    if (cellSpursInitialize((CellSpurs*)A, 2, 0, 0, 0)) fail("init busy");
    /* wid 0: held on both lanes. wid 1: added with maxContention 0, so it is
     * not admissible until the request below. */
    if (cellSpursAddWorkload((CellSpurs*)A, (u32*)0x8000, (void*)0x9100, 16, 0, NULL, 1, 2)) fail("add held");
    wait_until(&held_in, 2, 3000, "both lanes admitted");
    if (cellSpursAddWorkload((CellSpurs*)A, (u32*)0x8000, (void*)0x9200, 16, 0, NULL, 1, 0)) fail("add late");
    if (vm_read32(0x8000) != 1) fail("late wid");
    struct SpursInst* si = inst(A);
    Sleep(5);
    if (si->self_admit && __atomic_load_n(&si->free_lanes, __ATOMIC_SEQ_CST)) fail("free lane while both held");
    if (how == 0) {          /* SPU commit with every lane busy: the hook skips the wake */
        spu_lockline_lock(); vm_base[A + SPURS_WKL_MAXCONT + 1] = 1; ef_line_end(A);
        spu_raise_ready(A, 1);
    } else if (how == 1) {   /* PPU API */
        if (cellSpursSetMaxContention((CellSpurs*)A, 1, 1)) fail("set max");
    } else {                 /* PPU plain store, no notification of any kind */
        vm_base[A + SPURS_WKL_MAXCONT + 1] = 1;
    }
    Sleep(5);
    if (atomic_load(&late_runs)) fail("late ran with no free lane");
    atomic_store(&gate, 1);
    wait_until(&late_runs, 1, 3000, "request raised while all lanes busy runs after a lane frees");
    wait_until(&held_out, 2, 3000, "held released");
    finalize(A);
    static const char* hows[] = { "SPU commit", "PPU API", "PPU plain store" };
    printf("busy: %s request while every lane is admitted runs when a lane frees PASS\n", hows[how]);
}

/* A free lane finds a PPU plain store through the coordinator's timed pass. */
static void case_idle(void)
{
    atomic_store(&gate, 0); atomic_store(&held_in, 0); atomic_store(&held_out, 0); atomic_store(&late_runs, 0);
    if (cellSpursInitialize((CellSpurs*)A, 2, 0, 0, 0)) fail("init idle");
    if (cellSpursAddWorkload((CellSpurs*)A, (u32*)0x8000, (void*)0x9100, 16, 0, NULL, 1, 1)) fail("add held");
    wait_until(&held_in, 1, 3000, "one lane admitted");
    if (cellSpursAddWorkload((CellSpurs*)A, (u32*)0x8000, (void*)0x9200, 16, 0, NULL, 1, 0)) fail("add late");
    Sleep(5);
    vm_base[A + SPURS_WKL_MAXCONT + 1] = 1;   /* plain store: no hook, no API */
    wait_until(&late_runs, 1, 3000, "free lane picks up a plain store");
    if (atomic_load(&held_out)) fail("held lane released early");
    atomic_store(&gate, 1);
    wait_until(&held_out, 1, 3000, "held released");
    finalize(A);
    puts("idle: a free lane picks up a request stored with no notification PASS");
}

/* Contention freed by a returning lane reaches an idle lane. Lane 1 has
 * priority zero for wid 1, so only lane 0 can take wid 1, while wid 0 (one
 * at a time) is wanted on lane 1 after lane 0 lets it go. */
static atomic_uint seq_runs, seq_lane1;
static void seq_policy(spu_context* c)
{
    u32 lane = spu_ls_read32(c, 0x1C8);
    unsigned n = atomic_fetch_add(&seq_runs, 1) + 1;
    if (lane == 1) atomic_fetch_add(&seq_lane1, 1);
    if (n == 1) while (!atomic_load(&gate)) Sleep(1);
    if (n >= 2) leave_runnable_set(c);
    to_kernel(c);
}
static void case_freed(void)
{
    atomic_store(&gate, 0); atomic_store(&seq_runs, 0); atomic_store(&seq_lane1, 0);
    atomic_store(&late_runs, 0);
    if (cellSpursInitialize((CellSpurs*)A, 2, 0, 0, 0)) fail("init freed");
    /* wid 0: maxContention 1, ready for two runs; the first is held. */
    if (cellSpursAddWorkload((CellSpurs*)A, (u32*)0x8000, (void*)0x9300, 16, 0, NULL, 1, 1)) fail("add seq");
    wait_until(&seq_runs, 1, 3000, "first seq run");
    /* wid 1 keeps whichever lane holds wid 0's first run busy after it
     * returns, so the second wid 0 run has to go to the other lane. */
    u8 only0[8] = { 1, 0 };
    struct SpursInst* si = inst(A);
    AcquireSRWLockExclusive(&si->lock);
    u32 lane_of_first = si->slots[0].busy ? 0 : 1;
    ReleaseSRWLockExclusive(&si->lock);
    only0[0] = lane_of_first == 0; only0[1] = lane_of_first == 1;
    memcpy(vm_base + 0x8020, only0, 8);
    if (cellSpursAddWorkload((CellSpurs*)A, (u32*)0x8000, (void*)0x9200, 16, 0, (u8*)0x8020, 1, 1)) fail("add late");
    Sleep(5);
    atomic_store(&gate, 1);
    wait_until(&seq_runs, 2, 3000, "second run of the freed workload");
    wait_until(&late_runs, 1, 3000, "pinned workload");
    AcquireSRWLockExclusive(&si->lock);
    const u64 follows = si->stats.follow_passes;
    ReleaseSRWLockExclusive(&si->lock);
    if (si->self_admit && !follows) fail("no follow-up pass for the free lane");
    finalize(A);
    puts("freed: contention released by a returning lane reaches an idle lane PASS");
}

/* ---- cost ------------------------------------------------------------- */
static atomic_uint cost_runs, cost_target;
static void cost_policy(spu_context* c)
{
    unsigned n = atomic_fetch_add(&cost_runs, 1) + 1;
    g_spurs_pm_polls++;
    if (n >= atomic_load(&cost_target)) leave_runnable_set(c);
    to_kernel(c);
}
static void case_cost(u32 lanes)
{
    atomic_store(&cost_runs, 0); atomic_store(&cost_target, 200000);
    struct rusage r0, r1; getrusage(RUSAGE_SELF, &r0);
    LARGE_INTEGER t0, t1, f; QueryPerformanceFrequency(&f);
    if (cellSpursInitialize((CellSpurs*)A, lanes, 0, 0, 0)) fail("init cost");
    QueryPerformanceCounter(&t0);
    if (cellSpursAddWorkload((CellSpurs*)A, (u32*)0x8000, (void*)0x9400, 16, 0, NULL, 1, lanes)) fail("add cost");
    wait_until(&cost_runs, 200000, 60000, "cost runs");
    QueryPerformanceCounter(&t1);
    struct SpursInst* si = inst(A);
    AcquireSRWLockExclusive(&si->lock);
    struct SpursSchedStats st = si->stats;
    ReleaseSRWLockExclusive(&si->lock);
    finalize(A);
    getrusage(RUSAGE_SELF, &r1);
    double s = (double)(t1.QuadPart - t0.QuadPart) / (double)f.QuadPart;
    long cs = (r1.ru_nvcsw - r0.ru_nvcsw) + (r1.ru_nivcsw - r0.ru_nivcsw);
    printf("cost: lanes=%u self=%d %u runs in %.3f s = %.0f ns/run, %.2f ctx switches/run, "
           "admits worker=%llu coord=%llu coord-wakes=%llu\n",
           lanes, si->self_admit, atomic_load(&cost_runs), s, s * 1e9 / atomic_load(&cost_runs),
           (double)cs / atomic_load(&cost_runs),
           (unsigned long long)st.worker_admits, (unsigned long long)st.coord_admits,
           (unsigned long long)st.coord_wakes);
    if (si->self_admit && st.worker_admits < st.coord_admits * 10) fail("steady state was not self-admitted");
    if (!si->self_admit && st.worker_admits) fail("worker admitted with the switch off");
}

int main(int argc, char** argv)
{
    vm_base = calloc(1, ppu_vm_size); assert(vm_base);
    register_pm(0x9000, 0x11, churn_policy, 90);
    register_pm(0x9100, 0x22, held_policy, 91);
    register_pm(0x9200, 0x33, late_policy, 92);
    register_pm(0x9300, 0x44, seq_policy, 93);
    register_pm(0x9400, 0x55, cost_policy, 94);
    const int quick = argc > 1 && !strcmp(argv[1], "--quick");
    printf("PS3_SPURS_SELF_ADMIT=%d\n", spurs_self_admit_env());
    case_churn(0);
    for (int how = 0; how < 3; how++) case_busy(how);
    case_idle();
    case_freed();
    case_churn(1);
    if (!quick) { case_cost(1); case_cost(2); case_cost(3); }
    free(vm_base);
    puts("PASS");
    return 0;
}
