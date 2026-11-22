/*
 * ps3recomp - the lifted PPU code's raw stores into SPU-reserved lines
 *
 * Its own file, apart from spu_coherency.c, because it writes guest memory
 * through vm_base and the coherency fixtures compile spu_coherency.c without
 * one.
 */
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "spu_coherency.h"
#include "spu_coh_bitmap.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t* vm_base;

/* A PPU store the lifted code makes without vm_write* -- the vector stores
 * (stvx, stvewx, stvlx/stvrx), the byte-reversed ones (stwbrx, sthbrx,
 * stdbrx) and dcbz -- into a line an SPU has reserved. The generated header
 * tests the bitmap inline and calls this only on a hit, the same contract the
 * inline vm_write*_mm stores keep with the out-of-line coherent path: the
 * bytes land under the lock-line lock, so they cannot fall between a PUTLLC's
 * compare and its copy and be written straight back over, and every holder of
 * a touched line gets SPU_EVENT_LR. `src == NULL` stores zeros (dcbz).
 *
 * Aligned 2/4/8/16-byte pieces stay single-copy atomic per 8 bytes, as the
 * raw inline stores are. PS3_RAW_STORE_COH=0 keeps the old behaviour: the
 * same bytes, written without the lock and without a notify. */
int g_ppu_raw_store_coh = -1;
void spu_coh_store_raw(uint32_t ea, const void* src, uint32_t n)
{
    if (g_ppu_raw_store_coh < 0) {
        const char* e = getenv("PS3_RAW_STORE_COH");
        g_ppu_raw_store_coh = !(e && e[0] == '0');
    }
    uint8_t* p = vm_base + ea;
    const int coh = g_ppu_raw_store_coh;
    if (coh) spu_lockline_lock();
    if (!src) {
        memset(p, 0, n);
    } else if (n == 16 && !((uintptr_t)p & 7)) {
        uint64_t lo, hi;
        memcpy(&lo, src, 8); memcpy(&hi, (const uint8_t*)src + 8, 8);
        __atomic_store_n((uint64_t*)p, lo, __ATOMIC_RELAXED);
        __atomic_store_n((uint64_t*)(p + 8), hi, __ATOMIC_RELAXED);
    } else if ((n == 8 || n == 4 || n == 2) && !((uintptr_t)p & (n - 1))) {
        if (n == 8) { uint64_t v; memcpy(&v, src, 8); __atomic_store_n((uint64_t*)p, v, __ATOMIC_RELAXED); }
        else if (n == 4) { uint32_t v; memcpy(&v, src, 4); __atomic_store_n((uint32_t*)p, v, __ATOMIC_RELAXED); }
        else { uint16_t v; memcpy(&v, src, 2); __atomic_store_n((uint16_t*)p, v, __ATOMIC_RELAXED); }
    } else {
        memcpy(p, src, n);
    }
    if (coh) {
        const uint32_t first = ea & ~127u, last = (ea + n - 1u) & ~127u;
        for (uint32_t line = first; ; line += 128u) {
            if (spu_coh_line_reserved(line)) spu_coh_notify_write(line);
            if (line == last) break;
        }
        spu_lockline_unlock();
    }
}

#ifdef __cplusplus
}
#endif
