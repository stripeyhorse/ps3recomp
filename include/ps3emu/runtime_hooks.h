#ifndef PS3EMU_RUNTIME_HOOKS_H
#define PS3EMU_RUNTIME_HOOKS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Optional title callbacks. Install before starting guest threads and leave
 * the pointers unchanged while the runtime is active. */
struct spu_context;

/* Invoked after a successful ReadyCountStore while SPURS holds its locks.
 * The callback must not call into SPURS. */
extern void (*g_spurs_ready_count_store_hook)(uint32_t spurs_ea, uint32_t wid,
                                             uint32_t value, uint64_t workload_data);

/* Invoked after the copy of a GET or a GETL element. The callback may replace
 * bytes in dst. The SPU context is owned by the calling lane. */
extern void (*g_spu_dma_get_hook)(const struct spu_context* spu, uint32_t ea,
                                 uint32_t size, uint8_t* dst);

#ifdef __cplusplus
}
#endif

#endif
