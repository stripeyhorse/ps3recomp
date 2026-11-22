/* gcc -std=gnu17 -I include -I libs/video -fsanitize=address,undefined -pthread
 *   -ffunction-sections -Wl,--gc-sections   (-Wl,-dead_strip on macOS)
 *   libs/video/tests/test_gcm_put_wake.c libs/video/tests/gcm_fixture_stubs.c
 *   runtime/spu/spu_coherency.c
 *   runtime/platform/win32_compat.c -o /tmp/test_gcm_put_wake
 *
 * The FIFO PUT wake (PS3_GCM_PUT_WAKE) with the real PPU store path, the
 * coherency watch registry and auto-reset events. A PUT stored by another
 * thread while the present thread waits must set the event; one stored
 * before it waits, and not yet read by the walker, must skip the wait; GET,
 * a neighbouring word and a PUT rewritten unchanged must leave the event
 * alone, and so must a PUT the walker has already read.
 */
#include "../cellGcmSys.c"
#include "../rsx_tiling.c"
#include <assert.h>
#include <pthread.h>

static uint8_t g_guest[4096];
uint8_t* vm_base = g_guest;
uint32_t ppu_vm_size = sizeof g_guest;
int g_resv_store_active;
uint32_t g_ww_lo, g_ww_hi;
void ppu_resv_break_store(uint64_t ea) { (void)ea; }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; }
void spu_ch_wake(spu_context* ctx) { (void)ctx; }
ps3_guest_caller_fn g_ps3_guest_caller;

enum { PUT = 0x200, GET = PUT + 4, OTHER = 0x400 };

static void* publish(void* value)
{
    vm_write32(PUT, (uint32_t)(uintptr_t)value);
    return NULL;
}

int main(void)
{
    static GcmPutWake wake;
    HANDLE event = CreateEventA(NULL, FALSE, FALSE, NULL);
    assert(event);
    gcm_put_wake_init(&wake, PUT, event);
    assert(spu_coh_is_reserved(PUT));
    for (uintptr_t i = 1; i <= 250; ++i) {
        const uint32_t next = 8u * (uint32_t)i;
        /* Nothing new since the walker's last read: wait, and only PUT wakes. */
        assert(!gcm_put_wake_arm(&wake));
        vm_write32(GET, next);
        vm_write32(OTHER, next);
        vm_write32(PUT, next - 8);                      /* unchanged */
        assert(WaitForSingleObject(event, 0) == WAIT_TIMEOUT);

        pthread_t producer;
        assert(pthread_create(&producer, NULL, publish, (void*)(uintptr_t)next) == 0);
        assert(WaitForSingleObject(event, 2000) == WAIT_OBJECT_0);
        assert(pthread_join(producer, NULL) == 0);
        gcm_put_wake_disarm(&wake);
        assert(vm_read32(PUT) == next);

        /* The walker has not read it yet: the next wait is skipped, without
         * leaving the event set behind. */
        assert(gcm_put_wake_arm(&wake));
        gcm_put_wake_disarm(&wake);
        assert(WaitForSingleObject(event, 0) == WAIT_TIMEOUT);

        /* A publication while nobody waits sets nothing, and is still seen
         * by the next arm until the walker reads it. */
        gcm_put_wake_read(&wake, next);
        vm_write32(PUT, next + 4);
        assert(WaitForSingleObject(event, 0) == WAIT_TIMEOUT);
        assert(gcm_put_wake_arm(&wake));
        gcm_put_wake_disarm(&wake);
        gcm_put_wake_read(&wake, next + 4);
        assert(!gcm_put_wake_arm(&wake));
        gcm_put_wake_disarm(&wake);

        vm_write32(PUT, next);                          /* back for the next cycle */
        gcm_put_wake_read(&wake, next);
    }
    spu_lockline_lock();
    spu_coh_watch_remove(&wake.watch);
    spu_lockline_unlock();
    CloseHandle(event);
    printf("[PASS] 250 PUT wake cycles through real PPU stores, the coherency watch "
           "and auto-reset events\n");
    return 0;
}
