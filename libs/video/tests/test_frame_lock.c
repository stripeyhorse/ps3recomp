/* rsx_frame_lock.h: hold, timer lock and shedding, simulated against screens.
 *   cc -std=c11 -O1 -I libs/video libs/video/tests/test_frame_lock.c -lm -o /tmp/t && /tmp/t */
#include "rsx_frame_lock.h"
#include <assert.h>
#include <stdio.h>

static int near(double a, double b, double eps) { return fabs(a - b) <= eps; }
/* What a frame needs to make its refresh, as the Metal backend's timer mode has it. */
#define LEAD(refresh) ((refresh) + 0.0025)
#define LATCH 0.003

static void test_refreshes(void)
{
    assert(rsx_frame_lock_refreshes(1.0 / 30, 1.0 / 120) == 4);
    assert(rsx_frame_lock_refreshes(1.0 / 60, 1.0 / 120) == 2);
    assert(rsx_frame_lock_refreshes(1.0 / 60, 1.0 / 60) == 1);
    assert(rsx_frame_lock_refreshes(1.0 / 30, 1.0 / 119.88) == 4);    /* a hair off nominal */
    assert(rsx_frame_lock_refreshes(1.0 / 30, 1.0 / 59.94) == 2);
    assert(rsx_frame_lock_refreshes(1.0 / 30, 1.0 / 144) == 0);       /* 4.8 refreshes */
    assert(rsx_frame_lock_refreshes(1.0 / 30, 1.0 / 100) == 0);       /* 3.33 */
    assert(rsx_frame_lock_refreshes(1.0 / 30, 1.0 / 75) == 0);        /* 2.5 */
    assert(rsx_frame_lock_refreshes(1.0 / 60, 1.0 / 30) == 0);        /* screen slower than the frames */
    assert(rsx_frame_lock_refreshes(1.0 / 30, 1.0 / 240) == 8);
    assert(rsx_frame_lock_refreshes(0, 1.0 / 60) == 0);
    assert(rsx_frame_lock_refreshes(1.0 / 60, 0) == 0);
    assert(rsx_frame_lock_refreshes(NAN, 1.0 / 60) == 0);
}

static void test_hold(void)
{
    const double r = 1.0 / 120;
    assert(near(rsx_frame_lock_hold(1.0 / 30, r, 0), 3.5 * r, 1e-9));   /* 29.17 ms */
    assert(near(rsx_frame_lock_hold(1.0 / 30, r, 1), 2.5 * r, 1e-9));
    assert(near(rsx_frame_lock_hold(1.0 / 30, r, 2), 1.5 * r, 1e-9));
    assert(rsx_frame_lock_hold(1.0 / 30, r, 3) == 0);                    /* nothing left to hold: plain */
    assert(rsx_frame_lock_hold(1.0 / 30, r, 9) == 0);
    assert(near(rsx_frame_lock_hold(1.0 / 60, r, 0), 1.5 * r, 1e-9));   /* 12.5 ms */
    assert(rsx_frame_lock_hold(1.0 / 60, r, 1) == 0);
    assert(rsx_frame_lock_hold(1.0 / 60, 1.0 / 60, 0) == 0);             /* one refresh a frame: plain is the same */
    assert(rsx_frame_lock_hold(1.0 / 30, 1.0 / 144, 0) == 0);            /* no whole relation: never hold */
    assert(rsx_frame_lock_hold(1.0 / 60, 1.0 / 30, 0) == 0);
}

static void test_grid(void)
{
    assert(near(rsx_frame_lock_grid(1.0 / 60, 1.0 / 120), 1.0 / 120, 1e-12));   /* a tick is two refreshes */
    assert(near(rsx_frame_lock_grid(1.0 / 120, 1.0 / 120), 1.0 / 120, 1e-12));
    assert(near(rsx_frame_lock_grid(1.0 / 120, 1.0 / 60), 1.0 / 120, 1e-12));   /* two ticks in a refresh */
    assert(near(rsx_frame_lock_grid(1.0 / 60, 1.0 / 59.94), 1.0 / 59.94, 1e-12));
    assert(rsx_frame_lock_grid(1.0 / 60, 1.0 / 144) == 0);
    assert(rsx_frame_lock_grid(1.0 / 60, 1.0 / 100) == 0);
    assert(rsx_frame_lock_grid(1.0 / 120, 1.0 / 50) == 0);
    assert(rsx_frame_lock_grid(1.0 / 60, 0) == 0);
}

/* A timer of `period` against a screen of `refresh`, starting `phase` off the
 * grid. Every beat sees the latest refresh at or before the beat it has just
 * published, as the backend reports it. Returns the largest distance from the
 * grid over the last second, and the longest and shortest beat of the run. */
typedef struct { double settled_error, longest, shortest; int locked; } sim_result;
static sim_result simulate(double period, double refresh, double phase, double seconds)
{
    sim_result r = { 0, 0, 1e9, 0 };
    double deadline = phase;
    const int beats = (int)(seconds / period);
    for (int i = 0; i < beats; i++) {
        const double shown_at = floor(deadline / refresh) * refresh;
        int locked = 0;
        const double next = rsx_frame_lock_next(deadline, period, shown_at, refresh, &locked, NULL);
        const double beat = next - deadline;
        if (beat > r.longest) r.longest = beat;
        if (beat < r.shortest) r.shortest = beat;
        r.locked = locked;
        deadline = next;
        if (i >= beats - (int)(1.0 / period)) {
            const double grid = rsx_frame_lock_grid(period, refresh);
            const double off = grid > 0 ? fabs(remainder(deadline, grid)) : 0;
            if (off > r.settled_error) r.settled_error = off;
        }
    }
    return r;
}

static void test_lock(void)
{
    /* Exact 120 Hz, 60 Hz ticks, worst starting phase (half a refresh off). */
    sim_result r = simulate(1.0 / 60, 1.0 / 120, 0.5 / 120, 10);
    assert(r.locked && r.settled_error < 5e-6);
    assert(r.longest <= (1.0 / 60) * 1.0201 && r.shortest >= (1.0 / 60) * 0.9799);   /* never more than the slew */

    /* A 119.88 Hz panel: the timer follows it instead of sliding across it. */
    r = simulate(1.0 / 60, 1.0 / 119.88, 0.002, 60);
    assert(r.locked && r.settled_error < 100e-6);
    /* The same panel left alone slides a whole refresh every 8.3 s. */
    assert(fabs((1.0 / 60 - 2.0 / 119.88) * 60 * 60) > 1.0 / 119.88);

    /* 120 Hz ticks (the 60 FPS setting) on 120 Hz and on 60 Hz screens. */
    r = simulate(1.0 / 120, 1.0 / 120.05, 0.003, 30);
    assert(r.locked && r.settled_error < 100e-6);
    r = simulate(1.0 / 120, 1.0 / 59.95, 0.003, 30);
    assert(r.locked && r.settled_error < 100e-6);

    /* An integer-microsecond period, as a port's clock has it (16667 us). */
    r = simulate(16667e-6, 1.0 / 120, 0.001, 30);
    assert(r.locked && r.settled_error < 20e-6);

    /* No whole relation: the timer is not touched at all. */
    r = simulate(1.0 / 60, 1.0 / 144, 0.001, 5);
    assert(!r.locked && near(r.longest, 1.0 / 60, 1e-12) && near(r.shortest, 1.0 / 60, 1e-12));
}

static void test_no_refresh(void)
{
    /* Nothing shown lately: one plain period, whatever the stale numbers say. */
    int locked = 1; double error = 1;
    const double next = rsx_frame_lock_next(10.0, 1.0 / 60, 3.0, 0, &locked, &error);
    assert(near(next, 10.0 + 1.0 / 60, 1e-12) && !locked && error == 0);
    assert(near(rsx_frame_lock_next(10.0, 1.0 / 60, 3.0, -1, NULL, NULL), 10.0 + 1.0 / 60, 1e-12));

    /* Hidden for a while, then shown again half a refresh out of step: comes
     * back into step by slewing, with no beat more than 2% off. */
    double deadline = 0.5 / 120;
    double longest = 0, shortest = 1e9;
    for (int i = 0; i < 300; i++) {
        const int hidden = i >= 60 && i < 180;
        const double period = 1.0 / 60;
        /* While hidden the screen drifts away by a third of a refresh. */
        const double screen_offset = i < 180 ? 0 : (1.0 / 120) / 3;
        const double shown_at = floor((deadline - screen_offset) / (1.0 / 120)) * (1.0 / 120) + screen_offset;
        const double next = rsx_frame_lock_next(deadline, period, shown_at, hidden ? 0 : 1.0 / 120, NULL, NULL);
        const double beat = next - deadline;
        if (beat > longest) longest = beat;
        if (beat < shortest) shortest = beat;
        deadline = next;
    }
    assert(longest <= (1.0 / 60) * 1.0201 && shortest >= (1.0 / 60) * 0.9799);
    assert(fabs(remainder(deadline - (1.0 / 120) / 3, 1.0 / 120)) < 5e-6);
}

/* The whole loop: a guest whose timer is (or is not) kept in step with the
 * screen, that hands each frame over a varying time after its timer beat; a
 * GPU that needs a varying time; a screen that behaves as measured. Counts frames whose on-screen interval is not the nominal one
 * over the last 20 s, and the delay from hand-over to screen. */
typedef struct { unsigned uneven, frames, changes; double worst_delay, mean_delay; } loop_result;
typedef struct { double fps, refresh_hz; int lock, plan; double seconds;
                 double work_lo, work_hi, gpu_lo, gpu_hi; double burst_at; } loop_setup;
static unsigned rnd(unsigned* seed) { *seed = *seed * 1103515245u + 12345u; return (*seed >> 16) & 0x7FFF; }
static loop_result loop(loop_setup u)
{
    const double interval = 1.0 / u.fps, refresh = 1.0 / u.refresh_hz, period = interval / 2;   /* two beats a frame */
    loop_result r = { 0, 0, 0, 0, 0 };
    rsx_frame_plan plan = { .interval_s = interval, .refresh_s = refresh, .lead_s = LEAD(refresh), .latch_s = LATCH };
    double deadline = 0.0031, shown_prev = -1, anchor = 0, delay_sum = 0;
    unsigned seed = 12345;
    const int frames = (int)(u.seconds / interval);
    for (int f = 0; f < frames; f++) {
        for (int b = 0; b < 2; b++)
            deadline = rsx_frame_lock_next(deadline, period, anchor, u.lock ? refresh : 0, NULL, NULL);
        double work = interval * (u.work_lo + (u.work_hi - u.work_lo) * rnd(&seed) / 32768.0);
        /* A hitch: one frame takes three intervals, then the guest catches up. */
        if (u.burst_at > 0 && fabs(f * interval - u.burst_at) < interval / 2) { deadline += 3 * interval; }
        const double handed = deadline + work;
        const double gpu = u.gpu_lo + (u.gpu_hi - u.gpu_lo) * rnd(&seed) / 32768.0;
        unsigned seq = 0;
        double slot = u.plan ? rsx_frame_plan_next(&plan, handed, anchor, u.gpu_hi, &seq) : 0;
        /* The screen, as measured: a plain present is shown at the first
         * refresh a whole refresh after it, with the GPU done 2.7 ms before.
         * A planned frame is presented 1.5 ms after the refresh two before
         * its own, or at once when that has passed. */
        double presented = handed + 0.0005;
        if (slot > 0 && slot - 2 * refresh + 0.0015 > presented) presented = slot - 2 * refresh + 0.0015;
        double at = presented + refresh + 0.0002;
        if (handed + gpu + 0.0027 > at) at = handed + gpu + 0.0027;
        double shown = ceil(at / refresh) * refresh;
        if (shown <= shown_prev) shown = shown_prev + refresh;
        if (u.plan) {
            double slack = slot - handed - LEAD(refresh);
            if (slot - (handed + gpu) - LATCH < slack) slack = slot - (handed + gpu) - LATCH;
            rsx_frame_plan_shown(&plan, seq, slot, shown, slack);
        }
        if (shown_prev >= 0 && u.seconds - f * interval < 20) {
            r.frames++;
            r.uneven += fabs((shown - shown_prev) - interval) > 0.002;
            if (shown - handed > r.worst_delay) r.worst_delay = shown - handed;
            delay_sum += shown - handed;
        }
        shown_prev = shown;
        anchor = shown;
    }
    r.mean_delay = r.frames ? delay_sum / r.frames : 0;
    r.changes = plan.changes;
    return r;
}

static void test_loop(void)
{
    /* 30 FPS on a 119.88 Hz panel: work 40-70% of a frame, GPU 5-10 ms. */
    loop_setup u = { 30, 119.88, 0, 0, 120, 0.40, 0.70, 0.005, 0.010, 0 };
    loop_result off = loop(u);
    assert(off.uneven > off.frames / 10);            /* neither: uneven all the time */
    u.lock = 1; u.plan = 1;
    loop_result on = loop(u);
    assert(on.uneven == 0);
    /* The delay is what a frame needs (here the GPU's 10 ms and the latch),
     * the slack the plan keeps, at most a refresh and a half more before it
     * gives one up, and the spread of the hand-over itself. */
    const double keep = 0.010 + LATCH + RSX_FRAME_PLAN_TARGET_SLACK + 1.5 / 119.88;
    assert(on.worst_delay < keep + 0.30 / 30 + 0.001);
    printf("30 FPS @119.88: off uneven %u/%u; in step and planned uneven %u/%u, delay mean %.1f ms worst %.1f ms, %u changes\n",
           off.uneven, off.frames, on.uneven, on.frames, on.mean_delay * 1e3, on.worst_delay * 1e3, on.changes);

    /* The 60 FPS setting: work 55-85% of a frame, GPU 9-11 ms (more than half a frame). */
    loop_setup v = { 60, 119.88, 1, 1, 120, 0.55, 0.85, 0.009, 0.011, 0 };
    on = loop(v);
    assert(on.uneven == 0);
    printf("60 FPS @119.88: in step and planned uneven %u/%u, delay mean %.1f ms worst %.1f ms, %u changes\n",
           on.uneven, on.frames, on.mean_delay * 1e3, on.worst_delay * 1e3, on.changes);

    /* A hitch: the frames queued behind it are not left a frame late for ever. */
    loop_setup w = u; w.burst_at = 60;
    loop_result hitch = loop(w);
    assert(hitch.uneven == 0);                         /* settled again well before the last 20 s */
    assert(hitch.mean_delay < loop(u).mean_delay + 1.5 / 119.88);

    /* Planned but not in step (a host that does not lock its timer): the
     * guest slides against the screen; the plan keeps the delay bounded and
     * the uneven frames to the unavoidable one per slide. */
    loop_setup x = u; x.lock = 0;
    loop_result free_run = loop(x);
    assert(free_run.uneven < free_run.frames / 20);
    assert(free_run.worst_delay < keep + 1.0 / 119.88 + 0.30 / 30);

    /* A screen with no whole relation: the plan stands aside. */
    rsx_frame_plan none = { .interval_s = 1.0 / 30, .refresh_s = 1.0 / 144, .lead_s = LEAD(1.0 / 144), .latch_s = LATCH };
    unsigned seq = 9;
    assert(rsx_frame_plan_next(&none, 1.0, 0.99, 0.005, &seq) == 0 && seq == 0);
}

static void test_plan(void)
{
    const double r = 1.0 / 120, interval = 1.0 / 30;
    rsx_frame_plan p = { .interval_s = interval, .refresh_s = r, .lead_s = LEAD(r), .latch_s = LATCH };
    unsigned seq;
    /* What a frame needs: the lead when the GPU is quick, the GPU and the latch when it is not. */
    assert(near(rsx_frame_plan_need(&p, 0.001), LEAD(r), 1e-12));
    assert(near(rsx_frame_plan_need(&p, 0.010), 0.013, 1e-12));
    /* First frame: the first refresh it can make (1.0005 + 10.8 ms -> 1.0113 -> 1.0167). */
    double slot = rsx_frame_plan_next(&p, 1.0005, 1.0, 0.005, &seq);
    assert(seq == 0 && near(slot, 1.0 + 2 * r, 1e-9));
    /* On time: one interval on, whatever moment within the interval it is handed over. */
    double next = rsx_frame_plan_next(&p, 1.030, 1.0 + 2 * r, 0.005, &seq);
    assert(seq == 1 && near(next, slot + interval, 1e-9));
    next = rsx_frame_plan_next(&p, 1.0501, 1.0 + 6 * r, 0.005, &seq);   /* just after the previous one appeared */
    assert(near(next, slot + 2 * interval, 1e-9));
    /* Late: cannot make its refresh, takes the first it can, and the grid moves with it. */
    double late = rsx_frame_plan_next(&p, slot + 3 * interval - 0.004, slot + 2 * interval, 0.005, &seq);
    assert(near(late, slot + 3 * interval + r, 1e-9));
    next = rsx_frame_plan_next(&p, late + 0.010, late, 0.005, &seq);
    assert(near(next, late + interval, 1e-9));
    /* A pause (nothing planned for seconds): starts again from now. */
    next = rsx_frame_plan_next(&p, 9.0001, 9.0, 0.005, &seq);
    assert(near(next, 9.0 + 2 * r, 1e-9));

    /* Slack well over a refresh more than is kept, settled, no miss lately: one frame early, once. */
    rsx_frame_plan q = { .interval_s = interval, .refresh_s = r, .lead_s = LEAD(r), .latch_s = LATCH };
    double s2 = rsx_frame_plan_next(&q, 2.0, 2.0, 0.005, &seq);
    for (int i = 0; i < RSX_FRAME_PLAN_SETTLED + 2; i++) {
        rsx_frame_plan_shown(&q, seq, s2, s2, 0.017);                  /* 17 ms slack: 1.7 refreshes more than is kept */
        const double before = s2;
        s2 = rsx_frame_plan_next(&q, before + 0.005, before, 0.005, &seq);   /* handed over 28 ms before its refresh */
        if (i < RSX_FRAME_PLAN_SETTLED - 1) assert(near(s2 - before, interval, 1e-9));
    }
    assert(q.changes == 1);
    /* A refresh and a bit over is not enough to give one up. */
    rsx_frame_plan k = { .interval_s = interval, .refresh_s = r, .lead_s = LEAD(r), .latch_s = LATCH };
    double sk = rsx_frame_plan_next(&k, 2.0, 2.0, 0.005, &seq);
    for (int i = 0; i < 2 * RSX_FRAME_PLAN_SETTLED; i++) {
        rsx_frame_plan_shown(&k, seq, sk, sk, 0.013);
        const double before = sk;
        sk = rsx_frame_plan_next(&k, before + 0.005, before, 0.005, &seq);
        assert(near(sk - before, interval, 1e-9));
    }
    assert(k.changes == 0);

    /* Missing a refresh with slack in hand is the compositor's doing: not counted. */
    rsx_frame_plan m = { .interval_s = interval, .refresh_s = r, .lead_s = LEAD(r), .latch_s = LATCH };
    double sm = rsx_frame_plan_next(&m, 5.0, 5.0, 0.005, &seq);
    rsx_frame_plan_shown(&m, seq, sm, sm + r, 0.015);
    rsx_frame_plan_shown(&m, seq, sm, sm + r, 0.015);
    assert(!m.later && m.last_slip == 0);
    /* Without slack, twice within two seconds: one refresh later, then left alone. */
    rsx_frame_plan_shown(&m, seq, sm, sm + r, 0.001);
    double sn = rsx_frame_plan_next(&m, 5.03, sm + r, 0.005, &seq);
    assert(near(sn - sm, interval, 1e-9) && !m.later);                 /* one miss: nothing */
    rsx_frame_plan_shown(&m, seq, sn, sn + r, -0.002);
    assert(m.later);
    double so = rsx_frame_plan_next(&m, 5.06, sn + r, 0.005, &seq);
    assert(near(so - sn, interval + r, 1e-9) && !m.later && m.changes == 1);
    /* A queue (frames with 100 ms of slack) is given back in one step,
     * hold-off or not, and never to less than one refresh on. */
    {
        rsx_frame_plan d = m;
        unsigned dseq;
        double ds = rsx_frame_plan_next(&d, 5.07, sn + r, 0.005, &dseq);
        for (int i = 0; i < RSX_FRAME_PLAN_MIN_SAMPLES; i++) rsx_frame_plan_shown(&d, dseq, ds, ds, 0.100);
        const double before = ds;
        ds = rsx_frame_plan_next(&d, before - 0.095, before, 0.005, &dseq);   /* handed over 95 ms ahead */
        assert(near(ds - before, r, 1e-9));
    }
    /* ...and with slack to give up it still stays later for the hold-off. */
    for (int i = 0; i < 2 * RSX_FRAME_PLAN_SETTLED; i++) {
        rsx_frame_plan_shown(&m, seq, so, so, 0.017);
        const double before = so;
        so = rsx_frame_plan_next(&m, before + 0.005, before, 0.005, &seq);
        assert(near(so - before, interval, 1e-9));
    }
}

int main(void)
{
    test_refreshes();
    test_hold();
    test_grid();
    test_lock();
    test_no_refresh();
    test_plan();
    test_loop();
    puts("test_frame_lock: ok");
    return 0;
}
