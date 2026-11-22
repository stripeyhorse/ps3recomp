/*
 * ps3recomp - transform program and constant packets applied whole
 *
 * rsx_dispatch_upload_run against rsx_dispatch_method word by word, over the
 * entire dispatcher state: random load pointers (some past the end of the
 * store), start lanes, lengths and prior contents. An accepted packet must
 * leave every register, program word, constant and write count as the word
 * path does; a declined one must leave the state untouched.
 *
 * Builds and runs on Linux as well as macOS; meant for ASan and UBSan.
 */
#include "rsx_dispatch.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static rsx_dispatch reference, candidate;
static u32 g_seed = 62412;
static u32 rnd(void)
{
    g_seed ^= g_seed << 13;
    g_seed ^= g_seed >> 17;
    g_seed ^= g_seed << 5;
    return g_seed;
}

int main(void)
{
    u32 accepted = 0, declined = 0, failures = 0;
    for (u32 run = 0; run < 4096; ++run) {
        const int program = (run & 1) == 0;
        const u32 base = program ? 0x0B80u : 0x1F00u;
        const u32 method = base + 4u * (rnd() & 31u);
        const u32 count = 1u + rnd() % 40u;
        const u32 pointer = program ? 0x1E9Cu : 0x1EFCu;
        const u32 load = run % 5 == 0 ? UINT32_MAX - (run % 12u) : rnd() % 544u;
        rsx_dispatch_init(&reference, NULL);
        rsx_dispatch_method(&reference, pointer, load);
        for (u32 v = 0; v < 10u; ++v)
            rsx_dispatch_method(&reference, base + (rnd() & 31u) * 4u, rnd());
        rsx_dispatch_method(&reference, pointer, load);   /* after the scribbles */
        memcpy(&candidate, &reference, sizeof candidate);

        u8 bytes[40 * 4 + 1];   /* the words start at an odd address */
        for (u32 i = 0; i < count * 4u; ++i) bytes[1 + i] = (u8)rnd();
        if (!rsx_dispatch_upload_run(&candidate, method, bytes + 1, count)) {
            ++declined;
            if (memcmp(&candidate, &reference, sizeof reference)) {
                printf("[FAIL] case %u changed the state before declining\n", run);
                ++failures;
            }
            continue;
        }
        ++accepted;
        for (u32 i = 0; i < count; ++i) {
            const u8* p = bytes + 1 + i * 4u;
            rsx_dispatch_method(&reference, method + i * 4u,
                                ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3]);
        }
        if (memcmp(&candidate, &reference, sizeof reference)) {
            printf("[FAIL] case %u (method 0x%X, %u words, load %u) differs from the word path\n",
                   run, method, count, load);
            ++failures;
        }
    }
    if (!accepted || !declined) ++failures;
    printf("[%s] %u packets applied whole match the word path exactly; %u declined "
           "without touching the state\n", failures ? "FAIL" : "PASS", accepted, declined);
    return failures != 0;
}
