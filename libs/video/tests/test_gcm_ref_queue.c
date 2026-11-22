/* gcc -std=gnu17 -O2 -I include -I libs/video -fsanitize=thread -pthread
 *   -ffunction-sections -Wl,--gc-sections   (-Wl,-dead_strip on macOS)
 *   libs/video/tests/test_gcm_ref_queue.c libs/video/tests/gcm_fixture_stubs.c
 *   runtime/platform/win32_compat.c -o /tmp/test_gcm_ref_queue && /tmp/test_gcm_ref_queue
 * (and again with -fsanitize=address,undefined)
 *
 * The SET_REFERENCE queue between the FIFO walker, which pushes every fence it
 * drains, and the threads that publish them one at a time into the ref
 * register: the walker at the end of a pass and every guest thread polling
 * ref (cellGcm_ref_on_poll), serialised by s_ref_pub_lock. The ref register
 * must show each pushed fence, in order, and never a value nobody pushed.
 *
 * The queue's tail used to be a plain volatile stored before the slot (GCC
 * reorders a non-volatile store after a volatile one), so a publisher on
 * another core could read the slot before it was written: 0 for the first
 * 4096 fences, a fence 4096 old after that. Pain then waited for good on the
 * fence it lost (put == get, ref == 0). TSan reports that version as a race;
 * the value check below catches it when the reordering is hit.
 */
#include "../cellGcmSys.c"
#include "../rsx_tiling.c"
#include <assert.h>
#include <pthread.h>

static uint8_t g_guest[0x8000];
uint8_t* vm_base = g_guest;
uint32_t ppu_vm_size = sizeof g_guest;
int g_resv_store_active;
uint32_t g_ww_lo, g_ww_hi;
void ppu_resv_break_store(uint64_t ea) { (void)ea; }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; }
int spu_coh_is_reserved(uint32_t a) { (void)a; return 0; }
void spu_coh_notify_write(uint32_t a) { (void)a; }
void spu_lockline_lock(void) {}
void spu_lockline_unlock(void) {}
ps3_guest_caller_fn g_ps3_guest_caller;

enum { FENCES = 300000, PUBLISHERS = 2, LAG = 64 };
static _Atomic(u32) s_published;          /* how many fences the publishers took */
static _Atomic(int) s_bad;

/* Publishers, like the poll path: take the lock if free, publish one, read
 * the register back under the same lock (another publisher may move it the
 * moment the lock is released). Every value seen must be the next one. */
static void* publisher(void* unused)
{
    (void)unused;
    for (;;) {
        if (atomic_load_explicit(&s_published, memory_order_relaxed) >= FENCES) return NULL;
        if (!TryAcquireSRWLockExclusive(&s_ref_pub_lock)) continue;
        const u32 before = GCM_REFQ_HEAD_LOAD();
        gcm_ref_publish_one();
        if (GCM_REFQ_HEAD_LOAD() != before) {
            const u32 n = atomic_fetch_add_explicit(&s_published, 1, memory_order_relaxed) + 1;
            const u32 ref = vm_read32(GCM_CONTROL_GUEST_ADDR + 8);
            if (ref != n) {
                if (atomic_fetch_add(&s_bad, 1) < 8)
                    fprintf(stderr, "[FAIL] publication %u wrote ref 0x%08X\n", n, ref);
            }
        }
        ReleaseSRWLockExclusive(&s_ref_pub_lock);
    }
}

int main(void)
{
    ppu_hle_inject_base = 0;   /* labels at 0, control at 0x2000 of g_guest */
    pthread_t p[PUBLISHERS];
    for (uintptr_t i = 0; i < PUBLISHERS; i++) pthread_create(&p[i], NULL, publisher, NULL);
    /* The walker: fences 1..FENCES, never more than LAG ahead of the
     * publishers so the queue never overflows (overflow drops by design). */
    for (u32 v = 1; v <= FENCES; v++) {
        while (v - atomic_load_explicit(&s_published, memory_order_relaxed) > LAG) {}
        gcm_ref_push_at(v, v * 4u);
    }
    for (u32 i = 0; i < PUBLISHERS; i++) pthread_join(p[i], NULL);

    assert(atomic_load(&s_bad) == 0);
    assert(vm_read32(GCM_CONTROL_GUEST_ADDR + 8) == FENCES);
    assert(GCM_REFQ_HEAD_LOAD() == GCM_REFQ_TAIL_LOAD());

    /* Overflow: with nobody publishing, a full queue drops its oldest fence
     * and keeps the newest, and the publishers then see the newest 4096. */
    for (u32 v = 1; v <= GCM_REF_QLEN + 10; v++) gcm_ref_push_at(FENCES + v, 0);
    assert(GCM_REFQ_TAIL_LOAD() - GCM_REFQ_HEAD_LOAD() == GCM_REF_QLEN);
    gcm_ref_publish_one();
    assert(vm_read32(GCM_CONTROL_GUEST_ADDR + 8) == FENCES + 11);

    printf("[PASS] %u fences pushed by one thread and published by %d: each seen once, "
           "in order, none unwritten; overflow keeps the newest %u\n",
           FENCES, PUBLISHERS, GCM_REF_QLEN);
    return 0;
}
