/*
 * ps3recomp - SPU/PPU stall watchdog hooks (see stall_watchdog.c).
 *
 * Hook pointers, defined NULL next to their call sites (spu_channels.c,
 * lv2_register.c, sys_event.c) and set by ps3_stall_watchdog_install, which
 * ppu_loader.cpp calls from a static initializer. A fixture that compiles one
 * of those files on its own links nothing extra and never takes a hook. (Weak
 * undefined references, the first version, do not link under ld64 or
 * link.exe.) A call site costs one load and a not-taken branch.
 */
#ifndef PS3_STALL_WATCHDOG_H
#define PS3_STALL_WATCHDOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct spu_context;
/* kind: 0 delivered, 1 refused (queue full), 2 unbound port, 3 no data word */
extern void (*g_ps3_stall_user_event)(const struct spu_context* spu, uint32_t port, uint32_t value,
                                      uint32_t data, uint32_t queue, uint32_t result, int kind);
extern void (*g_ps3_stall_recv_begin)(uint64_t tid, uint32_t queue);
/* rc and the event the receive returned (r4..r7). */
extern void (*g_ps3_stall_recv_end)(uint64_t tid, int64_t rc, uint64_t source, uint64_t d1, uint64_t d2,
                                    uint64_t d3);
extern void (*g_ps3_stall_spin)(uint32_t ea);

/* Sets the hooks unless PS3_HAVOK_WATCHDOG=0. */
void ps3_stall_watchdog_install(void);

#define PS3_STALL_CALL(hook, ...) do { if (hook) (hook)(__VA_ARGS__); } while (0)

#ifdef __cplusplus
}
#endif

#endif
