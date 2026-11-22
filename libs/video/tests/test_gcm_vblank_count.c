/* gcc -std=gnu17 -I include -I libs/video -fsanitize=thread
 *   libs/video/tests/test_gcm_vblank_count.c libs/video/tests/gcm_fixture_stubs.c
 *   runtime/platform/win32_compat.c -pthread -ffunction-sections -Wl,--gc-sections
 *   -o /tmp/test_gcm_vblank_count && /tmp/test_gcm_vblank_count
 *
 * The vblank count is advanced by a ticker thread of its own (the port's
 * PS3_VBLANK_TIMER) while guest threads read it through
 * cellGcmGetVBlankCount, which also advances it, and the callback pump
 * passes it to the handler. No beat may be lost and no two readers may see
 * the same count: each tick and each query is one increment. Run it under
 * TSan as well.
 */
#include "../cellGcmSys.c"
#include "../rsx_tiling.c"
#include <assert.h>
#include <pthread.h>
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

enum { TICKS = 200000, QUERIES = 100000, READERS = 2 };
static u32 seen[READERS][QUERIES];

static void* ticker(void* unused)
{
    (void)unused;
    for (u32 i = 0; i < TICKS; i++) cellGcmTickVBlank();
    return NULL;
}

static void* reader(void* slot)
{
    u32* out = seen[(uintptr_t)slot];
    for (u32 i = 0; i < QUERIES; i++) out[i] = cellGcmGetVBlankCount();
    return NULL;
}

static int ascending(const void* a, const void* b)
{
    const u32 x = *(const u32*)a, y = *(const u32*)b;
    return x < y ? -1 : x > y;
}

int main(void)
{
    pthread_t t, r[READERS];
    pthread_create(&t, NULL, ticker, NULL);
    for (uintptr_t i = 0; i < READERS; i++) pthread_create(&r[i], NULL, reader, (void*)i);
    pthread_join(t, NULL);
    for (u32 i = 0; i < READERS; i++) pthread_join(r[i], NULL);

    /* Each query returns the count before its own increment. */
    const u32 total = TICKS + READERS * QUERIES;
    assert(cellGcmGetVBlankCount() == total);
    static u32 all[READERS * QUERIES];
    memcpy(all, seen, sizeof all);
    qsort(all, READERS * QUERIES, sizeof all[0], ascending);
    for (u32 i = 1; i < READERS * QUERIES; i++) assert(all[i] != all[i - 1]);
    printf("[PASS] %u ticks and %u queries on three threads: no increment lost, "
           "no count returned twice\n", TICKS, READERS * QUERIES);
    return 0;
}
