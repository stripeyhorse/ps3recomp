/* A lane that is admitted but never woken must not freeze cellSpursWaitForWorkloadShutdown.
 * Fault injection: the coordinator's wake of one admitted lane is dropped (the only
 * WakeConditionVariable call in cellSpurs.c is that wake). Before the bounded wait this
 * test hangs (exit 3 from the watchdog); with it, the shutdown completes within a slice.
 * Built like tests/test_spurs_scheduler.c. */
#include "runtime/platform/win32_compat.h"
#include <stdatomic.h>
static atomic_int drop_wakes;
static inline void fixture_wake_one(CONDITION_VARIABLE* c)
{
    int n = atomic_load(&drop_wakes);
    while (n > 0 && !atomic_compare_exchange_weak(&drop_wakes, &n, n - 1)) {}
    if (n > 0) return;                 /* dropped: the lane stays asleep with busy = 1 */
    WakeConditionVariable(c);
}
#define WakeConditionVariable fixture_wake_one
#define spu_workload_dispatch_async fixture_unbounded_dispatch
#include "libs/spurs/cellSpurs.c"
#undef spu_workload_dispatch_async
#undef WakeConditionVariable
#include <assert.h>
#include <pthread.h>
#include <unistd.h>
#include "runtime/spu/spu_coherency.h"

void spu_indirect_branch(spu_context*);
uint8_t* vm_base;
uint32_t ppu_vm_size=0x20000;
int g_resv_store_active;
uint32_t g_ww_lo,g_ww_hi,g_barrier_sync_watch;
uint32_t g_spu_image_src_ea,g_spu_image_ls_start,g_spu_image_span;
unsigned long long ps3_ms_now(void) { LARGE_INTEGER n,f; QueryPerformanceCounter(&n); QueryPerformanceFrequency(&f); return n.QuadPart*1000/f.QuadPart; }
void ppu_resv_break_store(uint64_t ea) {(void)ea;}
void ps3_ww_report_inline(uint32_t a,uint64_t v,int w){(void)a;(void)v;(void)w;}
uint64_t spurs_pm_build_context(uint8_t* ls,uint32_t ts,uint32_t id,uint32_t spu,uint32_t tag)
{(void)ls;(void)ts;(void)id;(void)spu;(void)tag;return 0;}
void spu_thread_publish_ctx(uint32_t tid,void* c){(void)tid;(void)c;}
void ydkj_wake_all_event_flags(void){}
void sys_event_queue_cancel_by_id(u32 id) {(void)id;}
void sys_event_queue_uncancel_by_id(u32 id) {(void)id;}
int fixture_unbounded_dispatch(const uint8_t* pm,uint32_t size,uint32_t data)
{ (void)pm;(void)size;(void)data;return 0; }

static atomic_uint runs, jobs_left;
static void jm_policy(spu_context* c)
{
    atomic_fetch_add(&runs,1);
    unsigned left=atomic_load(&jobs_left);
    while(left && !atomic_compare_exchange_weak(&jobs_left,&left,left-1)) {}
    c->pc=SPURS_PM_EXIT_TO_KERNEL_LS;spu_indirect_branch(c);
}
static atomic_int done;
static void* watchdog(void* a)
{
    (void)a;
    for(int i=0;i<30 && !atomic_load(&done);i++) usleep(100000);
    if(atomic_load(&done)) return NULL;
    struct SpursInst* si=spurs_inst_find(0x1000);
    fprintf(stderr,"FAIL: cellSpursWaitForWorkloadShutdown did not return in 3 s: state=%u active=%u busy=[%d %d %d]\n",
        vm_read8(0x1000+SPURS_WKL_STATE1),si->active[0],si->slots[0].busy,si->slots[1].busy,si->slots[2].busy);
    _exit(3);
}
int main(void)
{
    setenv("PS3_SPURS_LANES_OF_2","3",1);      /* PAIN's mesh_lanes = 3 */
    vm_base=calloc(1,ppu_vm_size);assert(vm_base);
    memset(vm_base+0x9000,0x11,16);
    spu_workload_register_img(spu_workload_fingerprint(vm_base+0x9000,16),jm_policy,77,"lost-wake fixture");
    assert(cellSpursInitialize((CellSpurs*)0x1000,2,0,0,0)==0);
    memset(vm_base+0x8020,1,8);
    /* Let the idle back-off settle so the next admissions are the kick's. */
    assert(cellSpursAddWorkload((CellSpurs*)0x1000,(u32*)0x8000,(void*)0x9000,16,0x18A8A80,(u8*)0x8020,1,6)==0);
    assert(vm_read32(0x8000)==0);
    Sleep(60);
    struct SpursInst* si=spurs_inst_find(0x1000);
    pthread_t wd; pthread_create(&wd,NULL,watchdog,NULL);
    /* Kick three lanes; the first lane the coordinator admits is never woken. */
    for(int tries=0;;tries++) {
        AcquireSRWLockExclusive(&si->lock);
        const int idle = !si->slots[0].busy && !si->slots[1].busy && !si->slots[2].busy;
        if (idle) atomic_store(&drop_wakes,1);
        ReleaseSRWLockExclusive(&si->lock);
        if (idle) break;
        assert(tries<2000); Sleep(1);
    }
    atomic_store(&jobs_left,3);
    assert(cellSpursReadyCountStore((CellSpurs*)0x1000,0,3)==0);
    while(atomic_load(&jobs_left)) Sleep(0);     /* the other lanes finish the work */
    Sleep(5);
    unsigned long long t0=ps3_ms_now();
    assert(cellSpursShutdownWorkload(0x1000,0)==0);
    const int stuck = vm_read8(0x1000+SPURS_WKL_STATE1)==WKL_SHUTTING_DOWN && atomic_load(&drop_wakes)==0;
    assert(cellSpursWaitForWorkloadShutdown(0x1000,0)==0);
    unsigned long long ms=ps3_ms_now()-t0;
    atomic_store(&done,1);
    assert(vm_read8(0x1000+SPURS_WKL_STATE1)==WKL_REMOVABLE);
    for(;;) { s32 rc=cellSpursRemoveWorkload((CellSpurs*)0x1000,0); if(!rc)break; assert((u32)rc==0x8041080A); Sleep(1); }
    printf("%s: shutdown returned after %llu ms (a lane wake was %s)\n", stuck?"PASS":"INCONCLUSIVE", ms, stuck?"dropped":"not dropped in time");
    return stuck?0:2;
}
