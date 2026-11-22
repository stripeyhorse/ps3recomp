/*
 * PS3_HAVOK_REREQUEST (runtime/spu/spu_channels.c, havok_rerequest): a Havok
 * pool request whose refill another task took is sent again while the
 * requesting task keeps spinning on the empty pool line.
 *
 * Deterministic, single-threaded, through the real channel paths: the task's
 * local store holds the Havok request block at LS 0xAA18, it GETLLARs the pool
 * line, sends port 10 exactly as 0x24318 does (rchcnt InMbox, WrOutMbox data,
 * WrOutIntrMbox, rdch InMbox), and then GETLLARs the unchanged line. The hook
 * stands in for lv2 delivery and counts.
 *
 *   default (2048):          the request is delivered again after exactly
 *                            2048 GETLLARs, with the same port/value/data; the
 *                            next repeat 4096 later (back-off); a repeat the
 *                            queue refuses (EBUSY) is retried 2048 later; the
 *                            reply words are consumed, so the task's next own
 *                            send is not skipped;
 *   PS3_HAVOK_REREQUEST=0:   delivered once only -- the freeze -- and the
 *                            test FAILS (exit 1), which is the reproduction;
 *   a successful PUTLLC between send and spin, a different call site, or a
 *   spin on another line: never repeated.
 *
 * Build (from ps3recomp/):
 *   gcc -std=gnu17 -O1 -I include -I runtime/spu -I runtime/platform \
 *       -o /tmp/test_spu_havok_rerequest runtime/spu/tests/test_spu_havok_rerequest.c \
 *       runtime/spu/spu_drain.c runtime/spu/spu_lockstep.c runtime/spu/spu_coherency.c \
 *       runtime/spu/spu_workload.c runtime/spu/spurs_policy.c runtime/spu/spurs_job.c \
 *       runtime/spu/spu_tsp_weak.c runtime/spu/spu_vm_pagemap.c \
 *       runtime/spu/spurs_policy_blob_weak.c runtime/spu/spu_interp.c \
 *       runtime/platform/win32_compat.c -lpthread -lm
 *   /tmp/test_spu_havok_rerequest && PS3_HAVOK_REREQUEST=0 /tmp/test_spu_havok_rerequest
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t g_guest_mem[1u << 20];
uint8_t* vm_base = g_guest_mem;
uint32_t ppu_vm_size = 0;

#include "../spu_channels.c"

/* The runtime beyond the SPU channel layer, stubbed (as in test_spu_lockline_peer.c). */
void ppu_resv_break_store(uint64_t ea)                 { (void)ea; }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; }
unsigned long long ps3_ms_now(void)                    { return 0; }
uint32_t g_ww_lo = 0xFFFFFFFFu, g_ww_hi = 0;
int      g_resv_store_active = 0;
uint32_t g_barrier_sync_watch = 0;
void (*g_spurs_kernel_hook)(uint32_t) = 0;
uint32_t g_ydkj_spurs_ctx_ea = 0;
uint32_t g_ydkj_real_spurs_ea = 0;
uint32_t g_ydkj_real_taskset_ea = 0;
uint32_t g_ydkj_real_taskid = 0;
void spurs_ef_set_from_spu(uint32_t ea, uint16_t bits) { (void)ea; (void)bits; }
void ydkj_wake_all_event_flags(void)                   { }
void spurs_pm_build_context(spu_context* c, uint32_t a, uint32_t b, uint32_t d) { (void)c; (void)a; (void)b; (void)d; }
uint32_t g_spu_image_src_ea, g_spu_image_ls_start, g_spu_image_span;
void spu_thread_publish_ctx(uint32_t tid, void* ctx) { (void)tid; (void)ctx; }

#define POOL     0x00040000u
#define OTHER    0x00040400u
#define LSA      0x00025800u
#define SITE     0x0000AA18u
#define REQ      ((10u << 24) | 0x00000Cu)   /* port 10, pool 0xC */
#define REQ_DATA 0x00000200u

static spu_context g_task;
static int g_delivered;
static uint32_t g_last_value, g_last_data;

/* lv2's spu_deliver_user_event, reduced: consume the data word, reply CELL_OK
 * -- or CELL_EBUSY, as for a full queue, when g_refuse is set. */
static int g_refuse, g_refused;
static int hook(spu_context* spu, uint32_t value)
{
    g_last_data = spu->ch_out_mbox.count ? spu_channel_read(&spu->ch_out_mbox) : 0xDEADu;
    g_last_value = value;
    if (g_refuse) { g_refused++; spu_channel_write(&spu->ch_in_mbox, 0x8001000Au); return 1; }
    g_delivered++;
    spu_channel_write(&spu->ch_in_mbox, 0);
    return 1;
}

static int g_pass, g_fail;
#define CHECK(c) do { if (c) g_pass++; else { g_fail++; fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

static void wrch(uint32_t ch, uint32_t v) { spu_wrch(&g_task, ch, spu_make_preferred_u32(v)); }
static void atomic_op(uint32_t ea, uint32_t cmd)
{
    wrch(MFC_LSA, LSA); wrch(MFC_EAH, 0); wrch(MFC_EAL, ea); wrch(MFC_Cmd, cmd);
}
static void spin(uint32_t ea, unsigned long n) { for (unsigned long i = 0; i < n; i++) atomic_op(ea, MFC_GETLLAR_CMD); }

/* 0x24318, as the guest runs it. */
static void send_request(void)
{
    if (spu_rchcnt(&g_task, SPU_RdInMbox)) return;   /* EBUSY, unsent */
    wrch(SPU_WrOutMbox, REQ_DATA);
    wrch(SPU_WrOutIntrMbox, REQ);
    (void)spu_rdch(&g_task, SPU_RdInMbox);
}

static void reset(const uint8_t* site_bytes)
{
    spu_context_init(&g_task, 0);
    g_task.image_id = 2;
    memset(g_guest_mem + POOL, 0, 0x800);
    memcpy(&g_task.ls[SITE], site_bytes, 32);
    g_delivered = 0;
}

int main(void)
{
    const char* env = getenv("PS3_HAVOK_REREQUEST");
    g_spu_user_event_hook = hook;

    /* 1. The freeze: request, then spin on the still-empty pool. */
    reset(k_havok_req_site);
    atomic_op(POOL, MFC_GETLLAR_CMD);   /* finds the pool empty */
    g_task.pc = SITE;                   /* lifted code is in the request block */
    send_request();
    CHECK(g_delivered == 1);
    CHECK(g_last_value == REQ && g_last_data == REQ_DATA);
    spin(POOL, 2047);
    CHECK(g_delivered == 1);            /* not before the threshold */
    spin(POOL, 1);
    {   /* Fails with PS3_HAVOK_REREQUEST=0: the task is left with its one
         * request spent, which is the freeze. */
        CHECK(g_delivered == 2);
        CHECK(g_last_value == REQ && g_last_data == REQ_DATA);
        CHECK(g_task.ch_in_mbox.count == 0);    /* repeat's reply consumed */
        CHECK(g_task.ch_out_mbox.count == 0);
        spin(POOL, 4095);                       /* back-off: the next one is 4096 later */
        CHECK(g_delivered == 2);
        spin(POOL, 1);
        CHECK(g_delivered == 3);
        /* Queue full: the repeat is refused, and retried after the base
         * interval rather than dropped or pushed out by the back-off. */
        g_refuse = 1;
        spin(POOL, 8192);
        CHECK(g_refused == 1 && g_delivered == 3);
        CHECK(g_task.ch_in_mbox.count == 0);    /* EBUSY reply consumed too */
        g_refuse = 0;
        spin(POOL, 2048);
        CHECK(g_delivered == 4);
        /* The task's own next send still goes out (nothing left in InMbox). */
        g_task.pc = SITE;
        send_request();
        CHECK(g_delivered == 5);
    }

    /* 2. The task got its slot (a successful PUTLLC) before spinning: no repeat. */
    reset(k_havok_req_site);
    atomic_op(POOL, MFC_GETLLAR_CMD);
    g_task.pc = SITE;
    send_request();
    atomic_op(POOL, MFC_GETLLAR_CMD);
    g_task.ls[LSA] = 0x80;                      /* claim: set the busy bit */
    atomic_op(POOL, MFC_PUTLLC_CMD);
    CHECK(g_task.atomic_stat == 0);
    spin(POOL, 100000);
    CHECK(g_delivered == 1);

    /* 3. Spinning on a different line: no repeat. */
    reset(k_havok_req_site);
    atomic_op(POOL, MFC_GETLLAR_CMD);
    g_task.pc = SITE;
    send_request();
    spin(OTHER, 100000);
    CHECK(g_delivered == 1);

    /* 4. Any other call site: never recorded. */
    { uint8_t other[32]; memcpy(other, k_havok_req_site, 32); other[31] ^= 1;
      reset(other); }
    atomic_op(POOL, MFC_GETLLAR_CMD);
    g_task.pc = SITE;
    send_request();
    spin(POOL, 100000);
    CHECK(g_delivered == 1);

    havok_rerequest_report(stdout);
    printf("havok re-request (PS3_HAVOK_REREQUEST=%s): %d passed, %d failed\n",
           env ? env : "(unset)", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
