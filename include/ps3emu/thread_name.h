/*
 * ps3_thread_name: name the calling host thread ("ppu main", "spu <group>:0",
 * "audio mix", ...), so a debugger, a profiler or a per-thread CPU table can
 * tell the guest's PPU and SPU threads and the runtime's own threads apart.
 *
 * Windows: SetThreadDescription (Windows 10 1607 and later) is looked up at
 * run time, so older systems just skip it. macOS: pthread_setname_np. Linux:
 * prctl(PR_SET_NAME), since pthread_setname_np needs _GNU_SOURCE on glibc
 * before the first pthread.h, which a header included this late cannot
 * promise. Elsewhere it does nothing.
 */
#ifndef PS3EMU_THREAD_NAME_H
#define PS3EMU_THREAD_NAME_H

#include <stdarg.h>
#include <stdio.h>

#ifdef _WIN32
#include <windows.h>

static inline void ps3_thread_name(const char* fmt, ...)
{
    typedef HRESULT (WINAPI *set_desc_fn)(HANDLE, PCWSTR);
    static set_desc_fn set_desc;
    static int looked_up;
    if (!looked_up) {
        set_desc = (set_desc_fn)(void*)GetProcAddress(GetModuleHandleA("kernel32.dll"),
                                                      "SetThreadDescription");
        looked_up = 1;
    }
    if (!set_desc) return;
    char s[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s, sizeof s, fmt, ap);
    va_end(ap);
    WCHAR w[64];
    int i = 0;
    for (; s[i] && i < 63; i++) w[i] = (WCHAR)(unsigned char)s[i];
    w[i] = 0;
    set_desc(GetCurrentThread(), w);
}
#else
#if defined(__APPLE__)
/* macOS: the name shows in `sample`, in crash reports and in the runtime's own
 * per-thread CPU line (PS3RECOMP_METAL_FPS). pthread names are 63 bytes. */
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
static inline void ps3_thread_name(const char* fmt, ...)
{
    char s[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s, sizeof s, fmt, ap);
    va_end(ap);
    pthread_setname_np(s);
}
#elif defined(__linux__)
/* Linux: prctl names the calling thread without _GNU_SOURCE. The kernel keeps
 * 15 bytes; /proc/<pid>/task/<tid>/comm and top -H show it. */
#include <sys/prctl.h>
#include <stdarg.h>
#include <stdio.h>
static inline void ps3_thread_name(const char* fmt, ...)
{
    char s[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s, sizeof s, fmt, ap);
    va_end(ap);
    prctl(PR_SET_NAME, s, 0, 0, 0);
}
#else
static inline void ps3_thread_name(const char* fmt, ...) { (void)fmt; }
#endif
#endif

#endif /* PS3EMU_THREAD_NAME_H */
