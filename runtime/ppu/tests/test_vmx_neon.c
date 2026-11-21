/* The arm64 NEON vaddfp/vsubfp/vmaddfp/vnmsubfp against the element loops,
 * bit for bit, over special values and random bits, with NJ off and on.
 * Build: cc -O2 -I runtime/ppu runtime/ppu/tests/test_vmx_neon.c -o test_vmx_neon */
#include <stdio.h>
#include <stdlib.h>
#ifndef VMX_NJ
#define VMX_NJ  0x00010000u
#define VMX_SAT 0x00000001u
#endif
#include "ppu_vmx.h"

static const uint32_t special[] = {
    0x00000000u, 0x80000000u, 0x00000001u, 0x80000001u, 0x007FFFFFu, 0x807FFFFFu, 0x00800000u,
    0x80800000u, 0x3F800000u, 0xBF800000u, 0x7F7FFFFFu, 0xFF7FFFFFu, 0x7F800000u, 0xFF800000u,
    0x7F800001u, 0xFF800001u, 0x7FC00000u, 0xFFC00000u, 0x7FFFFFFFu, 0x33800000u, 0x4B800000u,
    0x3F7FFFFFu, 0x3F800001u, 0x00FFFFFFu, 0x01000000u };
enum { NS = sizeof special / sizeof special[0] };

static uint64_t rng = 0x9E3779B97F4A7C15ull;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (uint32_t)(rng >> 16); }
static uint32_t pick(void) { const uint32_t r = rnd(); return (r & 3) ? rnd() : special[(r >> 2) % NS]; }
static void fill(uint8_t* v) { for (int i = 0; i < 4; i++) vmx_sw(v, i, pick()); }

int main(void)
{
    unsigned long bad = 0, n = 0;
    for (unsigned it = 0; it < 4000000u; it++) {
        uint8_t a[16], b[16], c[16], r0[16], r1[16];
        fill(a); fill(b); fill(c);
        const uint32_t vscr = (it & 1) ? VMX_NJ : 0;
        for (int op = 0; op < 4; op++) {
            switch (op) {
            case 0: vmx_vaddfp_ref(r0, a, b, vscr);       vmx_vaddfp(r1, a, b, vscr);       break;
            case 1: vmx_vsubfp_ref(r0, a, b, vscr);       vmx_vsubfp(r1, a, b, vscr);       break;
            case 2: vmx_vmaddfp_ref(r0, a, b, c, vscr);   vmx_vmaddfp(r1, a, b, c, vscr);   break;
            default: vmx_vnmsubfp_ref(r0, a, b, c, vscr); vmx_vnmsubfp(r1, a, b, c, vscr);  break;
            }
            n++;
            if (memcmp(r0, r1, 16)) {
                if (bad++ < 8) {
                    printf("op %d nj %d:", op, vscr != 0);
                    for (int i = 0; i < 4; i++)
                        printf(" [%08X %08X %08X -> ref %08X neon %08X]", vmx_w(a, i), vmx_w(b, i),
                               vmx_w(c, i), vmx_w(r0, i), vmx_w(r1, i));
                    printf("\n");
                }
            }
        }
    }
    /* in place: the destination is also a source */
    { uint8_t a[16], b[16], c[16], r0[16]; fill(a); fill(b); fill(c);
      vmx_vmaddfp_ref(r0, a, b, c, 0); vmx_vmaddfp(a, a, b, c, 0);
      if (memcmp(r0, a, 16)) { bad++; printf("in-place vmaddfp differs\n"); } }
    printf("%lu comparisons, %lu mismatches\n", n, bad);
    return bad != 0;
}
