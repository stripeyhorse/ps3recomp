/* Bounded diagnostics, using monotonic seconds. Caller supplies synchronization.
 * Display events are actual MTLDrawable presentations; guest events are explicit
 * host submissions. Keeping them separate exposes repeated host-only redraws. */
#ifndef PS3RECOMP_METAL_FRAME_RATE_H
#define PS3RECOMP_METAL_FRAME_RATE_H
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define PS3RECOMP_METAL_FRAME_RATE 1
#define METAL_FRAME_RATE_INTERVALS 256
typedef struct {
    double epoch, last_presented, interval_sum, worst_ms, wait_sum, wait_worst_ms;
    double intervals[METAL_FRAME_RATE_INTERVALS];
    unsigned long long displayed, guest, interval_count, wait_count;
} MetalFrameRate;
typedef struct {
    double display_fps, guest_fps, average_ms, p95_ms, worst_ms, age_seconds;
    /* Intervals more than METAL_FRAME_RATE_UNEVEN_MS from the sample's median:
     * uneven frame spacing at a steady average rate (41.7 + 25 ms pairs). */
    unsigned uneven;
    /* From the start of a frame's present to the moment the screen showed it:
     * the average and the longest in the sample. */
    double wait_ms, wait_worst_ms;
} MetalFrameRateSample;
#define METAL_FRAME_RATE_UNEVEN_MS 2.0
static inline void metal_frame_rate_reset(MetalFrameRate *s, double now) {
    memset(s, 0, sizeof(*s)); s->epoch=now;
}
static inline void metal_frame_rate_guest(MetalFrameRate *s) { ++s->guest; }
static inline void metal_frame_rate_presented(MetalFrameRate *s, double now) {
    if (!isfinite(now) || now<=0 || now<=s->last_presented) return;
    if (s->last_presented>0) {
        double ms=(now-s->last_presented)*1000;
        s->intervals[s->interval_count%METAL_FRAME_RATE_INTERVALS]=ms;
        s->interval_sum+=ms; ++s->interval_count;
        if (ms>s->worst_ms) s->worst_ms=ms;
    }
    s->last_presented=now; ++s->displayed;
}
static inline void metal_frame_rate_waited(MetalFrameRate *s, double began, double shown) {
    if (!isfinite(began) || !isfinite(shown) || began<=0 || shown<began) return;
    double ms=(shown-began)*1000;
    s->wait_sum+=ms; ++s->wait_count;
    if (ms>s->wait_worst_ms) s->wait_worst_ms=ms;
}
static inline int metal_frame_rate_compare(const void *a,const void *b) {
    double x=*(const double*)a,y=*(const double*)b;return (x>y)-(x<y);
}
static inline MetalFrameRateSample metal_frame_rate_sample(MetalFrameRate *s, double now) {
    MetalFrameRateSample r={0}; double elapsed=now-s->epoch;
    if (elapsed<=0) return r;
    r.display_fps=s->displayed/elapsed; r.guest_fps=s->guest/elapsed;
    if (s->interval_count) {
        r.average_ms=s->interval_sum/s->interval_count;
        r.worst_ms=s->worst_ms;
        unsigned n=s->interval_count<METAL_FRAME_RATE_INTERVALS?(unsigned)s->interval_count:METAL_FRAME_RATE_INTERVALS;
        double sorted[METAL_FRAME_RATE_INTERVALS];memcpy(sorted,s->intervals,n*sizeof(double));
        qsort(sorted,n,sizeof(double),metal_frame_rate_compare);
        r.p95_ms=sorted[(95*n+99)/100-1];
        const double median=sorted[n/2];
        for (unsigned i=0;i<n;i++) r.uneven+=fabs(sorted[i]-median)>METAL_FRAME_RATE_UNEVEN_MS;
    }
    r.age_seconds=s->last_presented>0?fmax(0,now-s->last_presented):0;
    if (s->wait_count) { r.wait_ms=s->wait_sum/s->wait_count; r.wait_worst_ms=s->wait_worst_ms; }
    s->epoch=now; s->displayed=s->guest=s->interval_count=s->wait_count=0;
    s->interval_sum=s->worst_ms=s->wait_sum=s->wait_worst_ms=0;
    return r;
}
#endif
