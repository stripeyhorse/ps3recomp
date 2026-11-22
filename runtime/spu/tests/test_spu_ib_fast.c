/* PS3_SPU_IB_FAST (spu_channels.c): the per-context indirect-branch target
 * cache must give the same guest state as the full resolver, in every mode.
 * Built and run by tools/test_spu_ib_fast.py, which runs each scenario with
 * PS3_SPU_IB_FAST=0, 1 and shadow and requires the same digest from all
 * three, no shadow mismatch, and cache hits where the scenario expects them.
 *
 * The "lifted" functions below are written the way the lifter emits an
 * indirect branch (ctx->pc = target; g_spu_trampoline_fn =
 * spu_indirect_branch; return) and a computed call (spu_indirect_branch
 * then spu_drain_call). Each one runs the next steps of a small script that
 * changes what the resolver depends on between branches:
 *
 *   residency   a verified streamed region is evicted by an unrelated GET and
 *               streamed back (pending -> verified) while its targets are hot
 *   reloc       relocatable code moves between two placements: reloc_delta
 *   task        taskset entry adoption, the TaskInfo ELF path, clearing
 *               below the task region, and a registration that changes the
 *               answer for a target already cached (the epoch)
 *   switch      image and overlay changes between branches
 *   special     policy select-workload service, the LS 0 job end and the
 *               job-return address are never answered from the cache
 *   stackreset  a stack-reset entry registered after its target was cached
 *   irq         interrupts injected between hops: the handler must run at
 *               the same pc after the same number of hops, and its iret
 *               must restore registers (the drain's own interrupt check)
 *   calls       computed calls (bisl shape) with their drains
 *
 *   bench       ns per indirect branch on the ICE-lane shape (policy mode,
 *               verified streamed module, interrupts enabled, four hot
 *               targets, a registry of ~24,000 functions) */
#include "../spu_helpers.h"
#include "../spu_overlay_residency.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

uint8_t* vm_base;
uint32_t ppu_vm_size;
extern void spu_begin_image(int);
extern void spu_register_function(uint32_t, void (*)(spu_context*));
extern void spu_register_stack_reset_entry(uint32_t, int);
extern void spu_taskset_register_task_entry(uint32_t, int);
extern void spu_taskset_register_task_elf(uint32_t, int, int);
extern void spu_overlay_register_region_reloc(uint32_t, uint32_t, uint32_t, int);
extern void spu_overlay_note_get(spu_context*, uint32_t, const uint8_t*, uint32_t);
extern int spu_run_with_halt(void (*)(spu_context*), spu_context*);
extern void spu_halt(spu_context*);
/* Weak so the bench also links against a tree without the cache (the
 * baseline comparison in tools/test_spu_ib_fast.py --baseline). */
extern int spu_ib_fast_stats(unsigned long long out[5]) __attribute__((weak));
static int ib_stats(unsigned long long out[5])
{
    if (spu_ib_fast_stats) return spu_ib_fast_stats(out);
    memset(out, 0, 5 * sizeof out[0]);
    return -1;
}

static spu_context ctx;

enum { OP_END, OP_GO, OP_CALL, OP_LINK, OP_NOTE, OP_REG, OP_IMG, OP_OVL, OP_RAISE,
       OP_WORD, OP_RESET, OP_LOOP };
typedef struct { int op; uint32_t a, b, c; } step;
static const step* s_script;
static unsigned s_ip;
static unsigned s_loop_left[8];
static unsigned s_hops;                 /* lifted bodies entered, in order */

#define FINISH_PC 0x240u
#define HANDLER_PC 0x300u
static void finish(spu_context* c)
{
    c->status = SPU_STATUS_STOPPED_BY_STOP;
    spu_halt(c);
}

static void (*s_fns[16])(spu_context*);
/* Run script steps until one transfers control. */
static void script(spu_context* c)
{
    for (;;) {
        const step* s = &s_script[s_ip++];
        switch (s->op) {
        case OP_END:
            c->pc = FINISH_PC; g_spu_trampoline_fn = spu_indirect_branch; return;
        case OP_GO:
            c->pc = s->a; g_spu_trampoline_fn = spu_indirect_branch; return;
        case OP_CALL: {         /* bisl $0, target ; continues at the next step */
            int32_t si = c->image_id;
            c->gpr[0] = spu_link(s->b);
            c->host_depth++;
            c->pc = s->a;
            spu_indirect_branch(c);
            spu_drain_call(c, s->b);
            c->host_depth--;
            spu_img_restore(c, si);
            break; }
        case OP_LINK: c->gpr[0] = spu_link(s->a); break;
        case OP_NOTE: spu_overlay_note_get(c, s->a, c->ls + s->b, s->c); break;
        case OP_REG:
            spu_begin_image((int)s->c);
            spu_register_function(s->a, s_fns[s->b]);
            spu_begin_image(0);
            break;
        case OP_IMG: c->image_id = (int)s->a; break;
        case OP_OVL: c->resident_ovl = (int)s->a; break;
        case OP_RAISE: SPU_EVENT_SET(c, s->a); break;
        case OP_WORD:
            c->ls[s->a] = (uint8_t)(s->b >> 24); c->ls[s->a + 1] = (uint8_t)(s->b >> 16);
            c->ls[s->a + 2] = (uint8_t)(s->b >> 8); c->ls[s->a + 3] = (uint8_t)s->b;
            break;
        case OP_RESET: spu_register_stack_reset_entry(s->a, (int)s->b); break;
        case OP_LOOP:           /* jump back to step a, b more times (slot c) */
            if (!s_loop_left[s->c]) s_loop_left[s->c] = s->b + 1;
            if (--s_loop_left[s->c]) s_ip = s->a;
            break;
        }
    }
}

/* f(N): a lifted function body that records its arrival and runs the script. */
#define DEF_F(N) static void f##N(spu_context* c) {                             \
        c->gpr[20 + N] = spu_ai(c->gpr[20 + N], 1);                         \
        c->gpr[40]._u32[0] += (uint32_t)c->reloc_delta;                     \
        c->gpr[41]._u32[0] = c->gpr[41]._u32[0] * 31u + (uint32_t)N + 1u;    \
        c->gpr[42]._u32[0] += (uint32_t)c->resident_task;                    \
        s_hops++;                                                           \
        script(c); }
DEF_F(0) DEF_F(1) DEF_F(2) DEF_F(3) DEF_F(4) DEF_F(5) DEF_F(6) DEF_F(7)
DEF_F(8) DEF_F(9) DEF_F(10) DEF_F(11) DEF_F(12) DEF_F(13) DEF_F(14) DEF_F(15)

/* Called as a leaf: returns through the link register. */
static void leaf(spu_context* c)
{
    c->gpr[60] = spu_ai(c->gpr[60], 1);
    s_hops++;
    SPU_RET(c);
}

/* Interrupt handler (LS 0 holds `bra 0x300`): records where and when it ran
 * in LS, clobbers a register, then irets. The iret's dispatch restores every
 * register (spu_irq_regs_maybe_restore), so only the LS record survives. */
static unsigned s_irqs;
static void handler(spu_context* c)
{
    uint8_t* r = c->ls + 0x3E000 + 16 * (s_irqs++ & 255);
    const uint32_t w[4] = { c->srr0, s_hops, s_ip, c->gpr[41]._u32[0] };
    for (int k = 0; k < 4; k++) {
        r[4*k] = (uint8_t)(w[k] >> 24); r[4*k+1] = (uint8_t)(w[k] >> 16);
        r[4*k+2] = (uint8_t)(w[k] >> 8); r[4*k+3] = (uint8_t)w[k];
    }
    c->gpr[41]._u32[0] ^= 0xDEADBEEFu;   /* must be undone by the restore */
    SPU_EVENT_CLEAR(c, 0x2u);
    c->int_enable = 1;                    /* irete */
    c->pc = c->srr0;
    g_spu_trampoline_fn = spu_indirect_branch;
}

static uint64_t digest(void)
{
    uint64_t h = 1469598103934665603ull;
#define MIX(v) do { h ^= (uint64_t)(v); h *= 1099511628211ull; } while (0)
    for (int r = 0; r < 128; r++) for (int k = 0; k < 4; k++) MIX(ctx.gpr[r]._u32[k]);
    for (uint32_t a = 0x3E000; a < 0x3F000; a++) MIX(ctx.ls[a]);
    MIX(ctx.pc); MIX(ctx.status); MIX(ctx.resident_task); MIX(ctx.resident_ovl);
    MIX(ctx.image_id); MIX((uint32_t)ctx.reloc_delta); MIX(ctx.int_enable); MIX(ctx.srr0);
    MIX(ctx.steps); MIX(s_hops); MIX(s_irqs); MIX(s_ip);
#undef MIX
    return h;
}

/* The script runs inside a call drain, as job code does. */
static void entry_go(spu_context* c)
{
    int32_t si = c->image_id;
    c->gpr[0] = spu_link(0x3F000);
    c->host_depth++;
    script(c);
    spu_drain_call(c, 0x3F000);
    c->host_depth--;
    spu_img_restore(c, si);
    finish(c);
}

/* ---------------------------------------------------------------- scenarios */
static const uint8_t k_code[64] = { 0x40, 0x80, 0x00, 0x03, 0x12, 0x34, 0x56, 0x78 };

static const step sc_residency[] = {
    { OP_GO, 0x4100 },                          /* 0: verified image 200 */
    { OP_GO, 0x4140 },
    { OP_GO, 0x4180 },
    { OP_LOOP, 0, 200, 0 },
    { OP_NOTE, 0x00999000, 0x4000, 0x100 },     /* 4: unrelated GET evicts the slot */
    { OP_GO, 0x4100 },                          /* now image 199's functions */
    { OP_GO, 0x4140 },
    { OP_LOOP, 5, 50, 1 },
    { OP_NOTE, 0x00B59500, 0x4000, 0x2000 },    /* 8: streamed back: pending -> verified */
    { OP_GO, 0x4100 },
    { OP_GO, 0x4180 },
    { OP_LOOP, 9, 50, 2 },
    { OP_NOTE, 0x00B59500 + 0x100, 0x4100, 0x80 },  /* 12: contiguous continuation: pending */
    { OP_GO, 0x4140 },
    { OP_LOOP, 13, 30, 3 },
    { OP_LOOP, 0, 3, 4 },
    /* 16: the code is overwritten and streamed again: verification fails, so
     * the span is interpreted. 0x4100 holds `bra 0x4140`, 0x4140 `bra 0x240`
     * (finish); a stale cached answer for 0x4140 would run f1 instead. */
    { OP_GO, 0x4140 },
    { OP_WORD, 0x4100, 0x30082800 },
    { OP_WORD, 0x4140, 0x30004800 },
    { OP_WORD, 0x4000, 0xDEADBEEF },
    { OP_NOTE, 0x00B59500, 0x4000, 0x2000 },
    { OP_GO, 0x4100 },
    { OP_END },
};
static const step sc_reloc[] = {
    { OP_NOTE, 0x00C00000, 0x8000, 0x400 },     /* 0: placement 0x8000 */
    { OP_GO, 0x8040 },
    { OP_GO, 0x8080 },
    { OP_LOOP, 1, 100, 0 },
    { OP_NOTE, 0x00C00000, 0x9000, 0x400 },     /* 4: moves to 0x9000 */
    { OP_GO, 0x9040 },
    { OP_GO, 0x9080 },
    { OP_LOOP, 5, 100, 1 },
    { OP_LOOP, 0, 4, 2 },
    { OP_END },
};
static const step sc_task[] = {
    { OP_GO, 0x5000 },                          /* 0: task entry: adopt 300 */
    { OP_GO, 0x5040 },                          /* 300's */
    { OP_GO, 0x5080 },                          /* 300 has none: primary 77's */
    { OP_GO, 0x1000 },                          /* below the region: task cleared */
    { OP_GO, 0x5040 },                          /* no task, not an entry: 77's */
    { OP_LOOP, 0, 100, 0 },
    { OP_REG, 0x5080, 9, 300 },                 /* 6: now 300 owns 0x5080 */
    { OP_GO, 0x5000 },
    { OP_GO, 0x5080 },
    { OP_GO, 0x1000 },
    { OP_LOOP, 7, 100, 1 },
    { OP_OVL, 199 },                            /* 11: TaskInfo ELF names image 301 */
    { OP_WORD, 0x2794, 0x00D00000 },
    { OP_GO, 0x5040 },                          /* 13: adopted from TaskInfo */
    { OP_GO, 0x5040 },                          /* already resident */
    { OP_GO, 0x5040 },
    { OP_GO, 0x1000 },
    { OP_LOOP, 13, 100, 2 },
    { OP_WORD, 0x2794, 0 },                     /* 18 */
    { OP_GO, 0x5040 },
    { OP_GO, 0x1000 },
    { OP_LOOP, 19, 50, 3 },
    { OP_END },
};
static const step sc_switch[] = {
    { OP_GO, 0x6000 },                          /* 0: image 77 */
    { OP_IMG, 78 },
    { OP_GO, 0x6000 },                          /* image 78's */
    { OP_OVL, 79 },
    { OP_GO, 0x6000 },                          /* overlay 79's */
    { OP_GO, 0x6040 },                          /* overlay has none: image 78's */
    { OP_OVL, 0 },
    { OP_IMG, 77 },
    { OP_LOOP, 0, 300, 0 },
    { OP_END },
};
static const step sc_special[] = {
    { OP_LINK, 0x1040 },                        /* 0 */
    { OP_GO, SPURS_PM_SELECT_WORKLOAD_LS },     /* service: r3 = 0, back to r0 */
    { OP_GO, 0x1000 },
    { OP_LINK, 0x1080 },
    { OP_GO, SPURS_PM_SELECT_WORKLOAD_LS },
    { OP_LOOP, 0, 200, 0 },
    { OP_GO, 0x0 },                             /* 6: policy: lifted at LS 0 here */
    { OP_GO, 0x1000 },
    { OP_LOOP, 6, 100, 1 },
    { OP_END },
};
/* 0x7040 is branched to (and cached) a hundred times, then registered as a
 * stack-reset entry and called from inside a frame: that call must unwind to
 * the driver (longjmp 2), which re-dispatches it at depth 0. */
static const step sc_stackreset[] = {
    { OP_CALL, 0x7000, 0x7004 },                /* 0: computed call, hot */
    { OP_GO, 0x7040 },
    { OP_LOOP, 0, 100, 0 },
    { OP_RESET, 0x7040, 77 },                   /* 3 */
    { OP_CALL, 0x7040, 0x7044 },                /* 4: the reset */
    { OP_GO, 0x7080 },                          /* 5: runs at depth 0 */
    { OP_END },
};
static const step sc_irq[] = {
    { OP_GO, 0x4100 },                          /* 0 */
    { OP_GO, 0x4140 },
    { OP_GO, 0x4180 },
    { OP_GO, 0x41C0 },
    { OP_LOOP, 0, 7, 0 },
    { OP_RAISE, 0x2 },                          /* 5: taken at the next hop */
    { OP_GO, 0x4140 },
    { OP_GO, 0x4180 },
    { OP_CALL, 0x4200, 0x4204 },                /* computed call while enabled */
    { OP_RAISE, 0x2 },
    { OP_CALL, 0x41C0, 0x4204 },                /* 10: the script goes on inside the call */
    { OP_GO, 0x4180 },                          /* taken at this hop, in the call's drain */
    { OP_GO, 0x4204 },                          /* return from the call */
    { OP_LOOP, 0, 60, 1 },
    { OP_END },
};
static const step sc_calls[] = {
    { OP_CALL, 0x7000, 0x7004 },                /* 0 */
    { OP_CALL, 0x7100, 0x7104 },
    { OP_GO, 0x7040 },
    { OP_LOOP, 0, 500, 0 },
    { OP_END },
};

static void setup_common(void)
{
    spu_context_init(&ctx, 0);
    ctx.status = SPU_STATUS_RUNNING;
    ctx.image_id = 77;
    spu_begin_image(77);
    spu_register_function(FINISH_PC, finish);
    spu_register_function(HANDLER_PC, handler);
    spu_begin_image(199);
    spu_register_function(FINISH_PC, finish);
    spu_register_function(HANDLER_PC, handler);
    spu_begin_image(0);
}
static void reg(int image, uint32_t pc, void (*fn)(spu_context*))
{
    spu_begin_image(image);
    spu_register_function(pc, fn);
    spu_begin_image(0);
}
static void make_verified_200(void)
{
    static const spu_overlay_code_segment seg = { 0x4000u, sizeof k_code, k_code };
    int ok = spu_overlay_register_verified_region(0x00B59500u, 0x4000u, 0x2000u, 200, &seg, 1);
    assert(ok);
    memcpy(ctx.ls + 0x4000, k_code, sizeof k_code);
    spu_overlay_note_get(&ctx, 0x00B59500u, ctx.ls + 0x4000, 0x2000);
}

static int setup(const char* mode)
{
    for (int i = 0; i < 16; i++) s_fns[i] = (void (*)(spu_context*))0;
    s_fns[0] = f0; s_fns[1] = f1; s_fns[2] = f2; s_fns[3] = f3; s_fns[4] = f4;
    s_fns[5] = f5; s_fns[6] = f6; s_fns[7] = f7; s_fns[8] = f8; s_fns[9] = f9;
    s_fns[10] = f10; s_fns[11] = f11; s_fns[12] = f12; s_fns[13] = f13;
    s_fns[14] = f14; s_fns[15] = f15;
    setup_common();
    if (!strcmp(mode, "residency") || !strcmp(mode, "irq")) {
        ctx.policy_mode = 1;
        ctx.image_id = 199;
        reg(200, 0x4100, f0); reg(200, 0x4140, f1); reg(200, 0x4180, f2); reg(200, 0x41C0, f3);
        reg(200, 0x4200, leaf);
        reg(199, 0x4100, f4); reg(199, 0x4140, f5); reg(199, 0x4180, f6);
        make_verified_200();
        s_script = !strcmp(mode, "irq") ? sc_irq : sc_residency;
        if (!strcmp(mode, "irq")) {
            ctx.ls[0] = 0x30; ctx.ls[1] = 0x00; ctx.ls[2] = 0x60; ctx.ls[3] = 0x00; /* bra 0x300 */
            ctx.int_enable = 1;
            ctx.event_mask = 0x2u;
        }
    } else if (!strcmp(mode, "reloc")) {
        ctx.policy_mode = 1;
        ctx.image_id = 199;
        spu_overlay_register_region_reloc(0x00C00000u, 0x400u, 0x20000u, 210);
        reg(210, 0x20040, f0); reg(210, 0x20080, f1);
        s_script = sc_reloc;
    } else if (!strcmp(mode, "task")) {
        ctx.policy_mode = 1;
        spu_taskset_register_task_entry(0x5000, 300);
        spu_taskset_register_task_elf(0x00D00000u, 301, 199);
        reg(300, 0x5000, f0); reg(300, 0x5040, f1);
        reg(301, 0x5040, f2);
        reg(77, 0x5040, f3); reg(77, 0x5080, f4); reg(77, 0x1000, f5);
        s_script = sc_task;
    } else if (!strcmp(mode, "switch")) {
        reg(77, 0x6000, f0); reg(78, 0x6000, f1); reg(79, 0x6000, f2);
        reg(78, 0x6040, f3); reg(77, 0x6040, f4);
        s_script = sc_switch;
    } else if (!strcmp(mode, "special")) {
        ctx.policy_mode = 1;
        reg(77, 0x1000, f0); reg(77, 0x1040, f1); reg(77, 0x1080, f2); reg(77, 0x0, f3);
        s_script = sc_special;
    } else if (!strcmp(mode, "ls0")) {
        reg(77, 0x1000, f0);
        static const step sc[] = { { OP_GO, 0x1000 }, { OP_LOOP, 0, 50, 0 }, { OP_GO, 0x0 }, { OP_END } };
        s_script = sc;
    } else if (!strcmp(mode, "stackreset")) {
        reg(77, 0x7000, leaf); reg(77, 0x7040, f1); reg(77, 0x7080, f2);
        s_script = sc_stackreset;
    } else if (!strcmp(mode, "calls")) {
        reg(77, 0x7000, leaf); reg(77, 0x7100, leaf); reg(77, 0x7040, f0);
        s_script = sc_calls;
    } else {
        return 0;
    }
    return 1;
}

/* stackreset: f2 at 0x7080 is reached at top level after the reset; it then
 * calls 0x7040 from inside a frame, which must unwind (longjmp 2) to the
 * driver and continue there. */

/* ------------------------------------------------------------------- bench */
static uint32_t s_bench_left;
static int s_bench_known, s_bench_tail;
static void (*s_bench_fn[4])(spu_context*);
static const uint32_t k_hot[4] = { 0x11458, 0x11420, 0xBDD8, 0xBDF8 };
#define BENCH_FN(N, NEXT) static void b##N(spu_context* c) {                 \
        c->gpr[20] = spu_ai(c->gpr[20], 1);                                 \
        if (__builtin_expect(--s_bench_left == 0, 0)) {                     \
            c->pc = 0x10004; g_spu_trampoline_fn = spu_indirect_branch; return; } \
        if (c->int_enable && (c->event_status & c->event_mask)) {           \
            c->pc = k_hot[N]; g_spu_trampoline_fn = b##N; return; }         \
        c->pc = c->gpr[21 + NEXT]._u32[0];                                  \
        BENCH_TAIL(c);                                                      \
        g_spu_trampoline_fn = s_bench_known ? s_bench_fn[NEXT] : spu_indirect_branch; return; }
/* bench-tail (clang only): the lifter option this package describes but does
 * not ship. The site tail-calls the dispatcher under SPU_DIRECT's own
 * conditions (spu_context.h) instead of returning to the drain, so a cached
 * target costs no drain step. */
#ifdef SPU_DIRECT_TAILCALLS
#define BENCH_TAIL(c) do {                                                  \
        const uint32_t _p = (c)->pc & SPU_LS_MASK & ~3u;                     \
        if (s_bench_tail && g_spu_direct > 0 && _p && (c)->steps &&           \
            !g_spu_stack_reset_hint[(_p >> 2) & 1023u] &&                    \
            !((c)->int_enable && ((c)->event_status & (c)->event_mask)))      \
            __attribute__((musttail)) return spu_indirect_branch(c);         \
    } while (0)
#else
#define BENCH_TAIL(c) do { } while (0)
#endif
/* bench-known: the same loop with each target known (the trampoline names the
 * function, as for a direct branch the lifter could not tail-call): the cost
 * of the drain step alone, without any indirect-branch resolution. */
BENCH_FN(0, 1) BENCH_FN(1, 2) BENCH_FN(2, 3) BENCH_FN(3, 0)
static void bench_filler(spu_context* c) { (void)c; abort(); }
static void bench_entry(spu_context* c)
{
    int32_t si = c->image_id;
    c->gpr[0] = spu_link(0x10004);
    c->host_depth++;
    c->pc = k_hot[0];
    g_spu_trampoline_fn = spu_indirect_branch;
    spu_drain_call(c, 0x10004);
    c->host_depth--;
    spu_img_restore(c, si);
    finish(c);
}
static double now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec * 1e9 + (double)t.tv_nsec;
}
static int bench(uint32_t n)
{
    setup_common();
    /* ~24,000 registered functions; eleven streamed modules share the code
     * buffer, so the hot addresses' hash chains are eleven deep. */
    for (int img = 201; img <= 210; img++)
        for (uint32_t a = 0x4000; a < 0x4000 + 4 * 2000; a += 4) reg(img, a, bench_filler);
    for (int img = 201; img <= 210; img++)
        for (int k = 0; k < 4; k++) reg(img, k_hot[k], bench_filler);
    reg(200, k_hot[0], b0); reg(200, k_hot[1], b1); reg(200, k_hot[2], b2); reg(200, k_hot[3], b3);
    s_bench_fn[0] = b0; s_bench_fn[1] = b1; s_bench_fn[2] = b2; s_bench_fn[3] = b3;
    static uint8_t code[0x100];
    for (unsigned i = 0; i < sizeof code; i++) code[i] = (uint8_t)(i * 7 + 1);
    static const spu_overlay_code_segment seg = { 0x4000u, sizeof code, code };
    int ok = spu_overlay_register_verified_region(0x00B59500u, 0x4000u, 0x11C24u, 200, &seg, 1);
    assert(ok);
    double best = 1e30;
    for (int rep = 0; rep < 7; rep++) {
        spu_context_init(&ctx, 0);
        ctx.status = SPU_STATUS_RUNNING;
        ctx.image_id = 199;
        ctx.policy_mode = 1;
        ctx.int_enable = 1;
        ctx.event_mask = 0x2u;     /* list-stall interrupts armed, none pending */
        memcpy(ctx.ls + 0x4000, code, sizeof code);
        spu_overlay_note_get(&ctx, 0x00B59500u, ctx.ls + 0x4000, 0x11C24u);
        for (int k = 0; k < 4; k++) ctx.gpr[21 + k]._u32[0] = k_hot[k];
        s_bench_left = n;
        const double t0 = now_ns();
        const int halted = spu_run_with_halt(bench_entry, &ctx);
        const double t1 = now_ns();
        assert(halted == 1 && ctx.gpr[20]._u32[0] == n);
        const double ns = (t1 - t0) / n;
        if (ns < best) best = ns;
    }
    unsigned long long st[5];
    const int mode = ib_stats(st);
    printf("bench%s: mode=%d branches=%u best_ns_per_branch=%.2f\n",
           s_bench_known ? "-known" : s_bench_tail ? "-tail" : "", mode, n, best);
    return 0;
}

int main(int argc, char** argv)
{
    assert(argc >= 2);
    const char* mode = argv[1];
    if (!strcmp(mode, "bench-known")) { s_bench_known = 1; mode = "bench"; }
    if (!strcmp(mode, "bench-tail")) {
#ifndef SPU_DIRECT_TAILCALLS
        puts("bench-tail: needs clang musttail"); return 0;
#endif
        s_bench_tail = 1; mode = "bench";
    }
    if (!strcmp(mode, "bench")) return bench(argc > 2 ? (uint32_t)strtoul(argv[2], 0, 0) : 20000000u);
    if (!setup(mode)) { fprintf(stderr, "unknown mode %s\n", mode); return 2; }
    int halted, runs = 0;
    halted = spu_run_with_halt(entry_go, &ctx);
    runs++;
    if (!strcmp(mode, "stackreset")) {
        /* After the reset the driver re-dispatched 0x7040 at depth 0 and the
         * script went on to its end there. */
        assert(halted == 1);
    }
    unsigned long long st[5];
    const int fast = ib_stats(st);
    printf("%s: fast=%d halted=%d runs=%d hops=%u irqs=%u steps=%u status=%X digest=%016llx "
           "hits=%llu misses=%llu inserts=%llu mismatches=%llu escapes=%llu\n",
           mode, fast, halted, runs, s_hops, s_irqs, ctx.steps, ctx.status,
           (unsigned long long)digest(), st[0], st[1], st[2], st[3], st[4]);
    if (!strcmp(mode, "irq")) {
        printf("irq records:");
        for (unsigned i = 0; i < s_irqs && i < 6; i++) {
            const uint8_t* r = ctx.ls + 0x3E000 + 16 * i;
            printf(" [srr0=%02X%02X%02X%02X hop=%u]", r[0], r[1], r[2], r[3],
                   ((unsigned)r[4] << 24) | ((unsigned)r[5] << 16) | ((unsigned)r[6] << 8) | r[7]);
        }
        printf("\n");
        assert(s_irqs == 122);
    }
    assert(halted == 1);
    assert(st[3] == 0 && st[4] == 0);
    puts("PASS");
    return 0;
}

/* Link-only boundaries outside this CPU-only fixture. Fail if any is reached. */
void spu_thread_publish_ctx(uint32_t tid, void* c) { (void)tid; (void)c; abort(); }
void ppu_resv_break_store(uint64_t ea) { (void)ea; abort(); }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; abort(); }
unsigned long long ps3_ms_now(void) { abort(); }
uint32_t g_ww_lo = 0xFFFFFFFFu, g_ww_hi;
int g_resv_store_active;
uint32_t g_barrier_sync_watch;
uint32_t g_ydkj_real_spurs_ea, g_ydkj_real_taskset_ea, g_ydkj_real_taskid;
void spurs_ef_set_from_spu(uint32_t ea, uint16_t bits) { (void)ea; (void)bits; abort(); }
void ydkj_wake_all_event_flags(void) { abort(); }
void spurs_pm_build_context(spu_context* c, uint32_t a, uint32_t b, uint32_t d) {
    (void)c; (void)a; (void)b; (void)d; abort();
}
uint32_t g_spu_image_src_ea, g_spu_image_ls_start, g_spu_image_span;
