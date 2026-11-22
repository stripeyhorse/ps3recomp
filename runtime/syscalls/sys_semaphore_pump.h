/* PS3_GCM_WAIT_PUMP on POSIX: the link between GCM callback delivery
 * (libs/video/cellGcmSys.c) and blocked semaphore waits (sys_semaphore.c).
 *
 * A guest thread blocked in sys_semaphore_wait parks; each GCM tick kicks one
 * parked thread awake to run the callback pump. Neither file names the other:
 * each is linked alone somewhere (the GCM fixtures and tools/test_gcm_*.py,
 * the sync tests), and macOS will not link a weak undefined reference. The one
 * shared object is this struct, weak-DEFINED in both files
 * (PS3_SEM_PUMP_GLUE_DEFINE), which the linker merges into one; zero means
 * "not registered". cellGcmSys.c fills pump and pending on its first tick,
 * sys_semaphore.c fills kick on its first parked wait. Windows has its own
 * path (ppu_gcm_wait_pump, an event) and does not use this. */
#ifndef PS3_SYS_SEMAPHORE_PUMP_H
#define PS3_SYS_SEMAPHORE_PUMP_H
#ifndef _WIN32
#include <stdatomic.h>

typedef void (*ps3_sem_pump_fn)(void);
typedef int  (*ps3_sem_pending_fn)(void);

typedef struct ps3_sem_pump_glue {
    _Atomic(ps3_sem_pump_fn)    pump;     /* GCM: deliver what is pending */
    _Atomic(ps3_sem_pending_fn) pending;  /* GCM: is anything pending? */
    _Atomic(ps3_sem_pump_fn)    kick;     /* semaphores: wake one parked waiter */
    atomic_uint                 kicks;    /* waiters kicked, for the stats line */
} ps3_sem_pump_glue;

extern ps3_sem_pump_glue g_ps3_sem_pump_glue;
#define PS3_SEM_PUMP_GLUE_DEFINE __attribute__((weak)) ps3_sem_pump_glue g_ps3_sem_pump_glue

#endif
#endif
