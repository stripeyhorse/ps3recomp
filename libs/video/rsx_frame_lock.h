/* Frame pacing against a screen's refresh: the arithmetic only. No clock, no
 * window system, no state of its own, so it is tested on any host
 * (tests/test_frame_lock.c).
 *
 * A guest frame clock that is a CPU timer and a screen that refreshes on its
 * own clock drift against each other, and a frame handed over near a refresh
 * falls on either side of it. Three pieces put the two in step:
 *
 *   1. rsx_frame_plan: the refresh every frame is to appear at, one interval
 *      after the frame before, so frames that become ready at uneven moments
 *      are shown at even ones, with the delay between a finished frame and the
 *      screen kept as short as the GPU and the compositor allow;
 *   2. rsx_frame_lock_next: the timer's next deadline, nudged toward the grid
 *      of the screen's refreshes, so the phase between the two stops moving
 *      (without it the plan has to give or take a refresh every few seconds);
 *   3. rsx_frame_lock_hold: the older way to 1, a minimum time on screen that
 *      the compositor counts. Kept for comparison.
 *
 * All of it applies only when one interval is a whole number of refreshes (or
 * a whole fraction of one). On any other screen every function answers "leave
 * it alone". Times are seconds. */
#ifndef PS3RECOMP_RSX_FRAME_LOCK_H
#define PS3RECOMP_RSX_FRAME_LOCK_H
#include <math.h>

/* How far a rate may be from a whole multiple and still count as one: 1%.
 * A 119.88 Hz panel is 0.1% from 120; a 144 Hz one is 20% from any multiple
 * of a 33.3 ms frame. */
#define RSX_FRAME_LOCK_TOLERANCE 0.01

/* Whole refreshes in one interval: 4 for 33.3 ms frames on a 120 Hz screen.
 * 0 when the interval is not a whole number of refreshes, or shorter than one. */
static inline unsigned rsx_frame_lock_refreshes(double interval_s, double refresh_s)
{
    if (!(interval_s > 0) || !(refresh_s > 0)) return 0;
    const double ratio = interval_s / refresh_s, whole = floor(ratio + 0.5);
    if (whole < 1 || whole > 64 || fabs(ratio - whole) > RSX_FRAME_LOCK_TOLERANCE * whole) return 0;
    return (unsigned)whole;
}

/* Minimum time on screen for the frame before this one: the interval less half
 * a refresh, so a panel a hair off its nominal rate still counts the right
 * number of refreshes, and one refresh less again for each frame the caller
 * asks to shed. 0: present plainly. (The plan below replaces this; a backend
 * keeps it as a fallback.) */
static inline double rsx_frame_lock_hold(double interval_s, double refresh_s, unsigned shed)
{
    const unsigned refreshes = rsx_frame_lock_refreshes(interval_s, refresh_s);
    if (!refreshes || shed + 1 >= refreshes) return 0;
    return ((double)(refreshes - shed) - 0.5) * refresh_s;
}

/* ---- a show time for every frame -------------------------------------------
 *
 * The hold above leaves the timing to the compositor, which counts the hold
 * from when it believes the previous frame appeared. A frame handed over within
 * a couple of milliseconds after the previous one appeared is counted from one
 * refresh later (measured on macOS 27: handed over 2.2 ms after, GPU work done
 * 22 ms early, shown a refresh late), and that moment is exactly where a guest
 * in step with the screen keeps handing over. The plan below does not depend on
 * it: every frame is given the refresh it is to appear at, one interval after
 * the frame before, and the backend presents it for that refresh in whatever
 * way its compositor keeps to best (rsx_metal_backend.m times a plain present
 * itself: 1 frame in 720 a refresh late, against 39 in 806 when the compositor
 * was given the time).
 *
 * What a frame needs to make a refresh is the backend's to say, as two times:
 *   lead   from the hand-over to the refresh, whatever the frame costs;
 *   latch  from the GPU finishing the frame to the refresh.
 * "Slack" is how much later the frame could have been handed over and still
 * have made its refresh: what is left of the delay once both are met.
 *
 * The delay between a finished frame and the screen is then the plan's to
 * choose, one refresh at a time:
 *   - a frame that cannot make its refresh takes the first one it can, and the
 *     frames after it follow from there (the guest ran late: one long frame);
 *   - when every frame for a while had well over a refresh of slack, one frame
 *     is shown a refresh early (one short frame, and every frame after it one
 *     refresh sooner);
 *   - when frames keep missing their refresh for want of slack, the plan moves
 *     one refresh later and stays there for a while. */
#define RSX_FRAME_PLAN_TARGET_SLACK 0.003  /* slack to keep, for the spread of the frames' cost */
#define RSX_FRAME_PLAN_MIN_SAMPLES  4      /* frames of an epoch seen before judging its slack */
#define RSX_FRAME_PLAN_SETTLED      90     /* ... before giving up a single refresh of it */
#define RSX_FRAME_PLAN_QUIET_S      10.0   /* no missed refresh for this long before showing a frame early */
#define RSX_FRAME_PLAN_SLIPS_S      5.0    /* two missed refreshes within this: move later */
#define RSX_FRAME_PLAN_HOLD_OFF_S   30.0   /* after moving later, stay there this long */
typedef struct {
    /* Set by the caller before use; the rest starts at zero. */
    double interval_s, refresh_s;
    double lead_s, latch_s;
    /* State. */
    double last_slot;          /* when the frame planned last is to appear; 0: none */
    unsigned seq;              /* frames planned */
    unsigned epoch;            /* the first frame planned since the delay last changed */
    double slack[RSX_FRAME_PLAN_SETTLED];   /* slack of this epoch's frames, the latest of them */
    unsigned slack_n;          /* how many of this epoch's frames have been shown */
    double last_slip, prev_slip;   /* when the last two frames missed their refresh; 0: never */
    double hold_off_until;     /* no earlier frame before this (after moving later) */
    int later;                 /* move one refresh later at the next frame */
    unsigned changes;          /* frames shown early or late on purpose, for the log */
} rsx_frame_plan;

/* The delay changes with the frame being planned: judge the slack afresh,
 * from that frame on. */
static inline void rsx_frame_plan_new_epoch(rsx_frame_plan* p)
{
    p->epoch = p->seq;
    p->slack_n = 0;
}

/* The least slack among the latest frames of this epoch (at most the last
 * RSX_FRAME_PLAN_SETTLED of them). */
static inline double rsx_frame_plan_slack(const rsx_frame_plan* p)
{
    const unsigned n = p->slack_n < RSX_FRAME_PLAN_SETTLED ? p->slack_n : RSX_FRAME_PLAN_SETTLED;
    double least = 0;
    for (unsigned i = 0; i < n; i++)
        if (!i || p->slack[i] < least) least = p->slack[i];
    return least;
}

static inline void rsx_frame_plan_reset(rsx_frame_plan* p)
{
    p->last_slot = 0;
    p->later = 0;
    rsx_frame_plan_new_epoch(p);
}

/* How long before its refresh a frame handed over now, that the GPU will need
 * `gpu_s` for, has to be handed over. */
static inline double rsx_frame_plan_need(const rsx_frame_plan* p, double gpu_s)
{
    const double by_gpu = (gpu_s > 0 ? gpu_s : 0) + p->latch_s;
    return by_gpu > p->lead_s ? by_gpu : p->lead_s;
}

/* The refresh the next frame is to appear at, or 0 to present it plainly
 * (pacing does not apply on this screen).
 *   now      the hand-over;
 *   anchor   a recent moment the screen refreshed (the last frame shown);
 *   gpu_s    how long the GPU is expected to need for the frame from now.
 * `*seq` receives the frame's number for rsx_frame_plan_shown. */
static inline double rsx_frame_plan_next(rsx_frame_plan* p, double now, double anchor, double gpu_s, unsigned* seq)
{
    const double r = p->refresh_s;
    const unsigned refreshes = rsx_frame_lock_refreshes(p->interval_s, r);
    if (seq) *seq = p->seq;
    if (!refreshes) { rsx_frame_plan_reset(p); p->seq++; return 0; }
    /* The first refresh the frame can be on screen at. */
    const double ready = now + rsx_frame_plan_need(p, gpu_s);
    const double earliest = anchor + ceil((ready - anchor) / r - 1e-3) * r;
    double slot = earliest;
    if (p->last_slot > 0) {
        int step = (int)refreshes;
        if (p->later) {
            step += 1;
        } else if (p->slack_n >= RSX_FRAME_PLAN_MIN_SAMPLES) {
            const double over = rsx_frame_plan_slack(p) - RSX_FRAME_PLAN_TARGET_SLACK;
            const int quiet = now >= p->hold_off_until &&
                              (!(p->last_slip > 0) || now - p->last_slip >= RSX_FRAME_PLAN_QUIET_S);
            /* Two refreshes over is a queue (the guest caught up after a
             * stall and its frames are waiting in line): give it all back at
             * once, down to one refresh over. A single refresh is given up
             * only when half a refresh would still be left over after it, and
             * once that has plainly been so for a while: the worst frame in a
             * hundred is later than the worst of the last few dozen, and a
             * plan that gives up its last refresh gets it back by missing. */
            if (over >= 2 * r) step -= (int)floor(over / r) - 1;
            else if (over >= 1.5 * r && quiet && p->slack_n >= RSX_FRAME_PLAN_SETTLED) step -= 1;
        }
        if (step < 1) step = 1;
        /* On the grid as the screen has it now, not as it was a frame ago. */
        const double wanted = p->last_slot + step * r;
        const double snapped = anchor + floor((wanted - anchor) / r + 0.5) * r;
        if (snapped >= earliest) {
            slot = snapped;
            if (step != (int)refreshes) { p->changes++; rsx_frame_plan_new_epoch(p); }
        } else {
            /* Late, or back after a pause: start again from what is possible. */
            rsx_frame_plan_new_epoch(p);
        }
        if (p->later) { p->later = 0; p->hold_off_until = now + RSX_FRAME_PLAN_HOLD_OFF_S; }
    } else {
        rsx_frame_plan_new_epoch(p);
    }
    p->last_slot = slot;
    p->seq++;
    return slot;
}

/* A planned frame reached the screen: `slot` as planned, `shown_at` as it
 * happened, `slack_s` how much later it could have been handed over and
 * still have made `slot` (negative: it could not have made it). */
static inline void rsx_frame_plan_shown(rsx_frame_plan* p, unsigned seq, double slot, double shown_at, double slack_s)
{
    if (!(slot > 0) || !(shown_at > 0)) return;
    if (shown_at - slot > 0.5 * p->refresh_s) {
        /* Missed its refresh. With slack in hand that was the compositor's
         * doing and a longer delay would not have helped. Without, and twice
         * in a short while, the delay is too tight: move a refresh later. */
        if (slack_s < RSX_FRAME_PLAN_TARGET_SLACK) {
            p->prev_slip = p->last_slip;
            p->last_slip = shown_at;
            if (p->prev_slip > 0 && p->last_slip - p->prev_slip <= RSX_FRAME_PLAN_SLIPS_S &&
                shown_at >= p->hold_off_until)
                p->later = 1;
        }
        return;                       /* its slack says nothing about the plan */
    }
    if ((int)(seq - p->epoch) < 0) return;
    p->slack[p->slack_n++ % RSX_FRAME_PLAN_SETTLED] = slack_s;
}

/* The spacing of the grid the timer may lock to: one refresh when a tick is a
 * whole number of refreshes (60 Hz ticks on a 120 Hz screen), a whole fraction
 * of a refresh when two to four ticks fit in one (120 Hz ticks on a 60 Hz
 * screen). 0: no whole relation, the timer runs free. */
static inline double rsx_frame_lock_grid(double period_s, double refresh_s)
{
    if (rsx_frame_lock_refreshes(period_s, refresh_s)) return refresh_s;
    const unsigned ticks = rsx_frame_lock_refreshes(refresh_s, period_s);
    return ticks >= 2 && ticks <= 4 ? refresh_s / (double)ticks : 0;
}

/* The deadline after `deadline` for a timer of period `period_s`.
 *
 * `shown_at` is a moment the screen refreshed (on the caller's clock) and
 * `refresh_s` its refresh period; refresh_s <= 0 means the caller has no recent
 * refresh to go by (window hidden, screen locked, nothing presented): the
 * timer then simply runs on from where it is, with no jump either way.
 *
 * With a refresh to go by, the next deadline is one period on, moved a quarter
 * of the way toward the nearest point of the refresh grid, and never by more
 * than 2% of a period, so the guest sees neither a short nor a long beat while
 * the two come into step (about twenty beats from the worst starting phase).
 * In step, the pull each beat is the drift between the two clocks, a few
 * microseconds. `*error_s` (optional) receives the distance to the grid before
 * the nudge; `*locked` (optional) whether a grid was used. */
#define RSX_FRAME_LOCK_GAIN 0.25
#define RSX_FRAME_LOCK_SLEW 0.02
static inline double rsx_frame_lock_next(double deadline, double period_s, double shown_at, double refresh_s,
                                         int* locked, double* error_s)
{
    const double next = deadline + period_s;
    const double grid = refresh_s > 0 ? rsx_frame_lock_grid(period_s, refresh_s) : 0;
    if (locked) *locked = grid > 0;
    if (error_s) *error_s = 0;
    if (!(grid > 0)) return next;
    const double error = remainder(shown_at - next, grid);   /* next + error is on the grid */
    if (error_s) *error_s = error;
    const double slew = RSX_FRAME_LOCK_SLEW * period_s;
    const double nudge = RSX_FRAME_LOCK_GAIN * error;
    return next + (nudge > slew ? slew : nudge < -slew ? -slew : nudge);
}

#endif
