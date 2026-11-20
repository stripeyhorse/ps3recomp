/* Frame-dump queue + writer thread for the Vulkan backend (PS3RECOMP_VK_DUMP).
 *
 * The present thread also ticks vblank/flip and drains the GCM FIFO, so it must
 * never wait on disk. vk_dump_enqueue() only copies the RGBA frame into a
 * preallocated slot (a memcpy) and returns; a dedicated writer thread packs
 * RGB and writes `<path>.tmp`, then rename()s it over `<path>`, so a reader
 * never sees a partial PPM. A full queue drops the capture and counts it.
 *
 * Header-only (static functions) so the backend and tests/test_vk_dump.c use
 * the same code. The includer may define VK_DUMP_LOG(...) before including. */
#ifndef RSX_VK_DUMP_H
#define RSX_VK_DUMP_H

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef VK_DUMP_LOG
#define VK_DUMP_LOG(...) fprintf(stderr, __VA_ARGS__)
#endif

#define VK_DUMP_SLOTS     4
#define VK_DUMP_DROP_LOG  16   /* log the drop count once per this many drops */

typedef struct vk_dump_slot {
    unsigned char* buf;        /* RGBA, width*height*4 */
    size_t         cap;
    uint32_t       width, height;
    int            busy;       /* filling, queued or being written */
} vk_dump_slot;

typedef struct vk_dump_q {
    pthread_mutex_t mu;
    pthread_cond_t  cv;
    pthread_t       thread;
    int             started, stop, paused;   /* paused: test barrier for the writer */
    vk_dump_slot    slot[VK_DUMP_SLOTS];
    int             order[VK_DUMP_SLOTS];    /* FIFO of queued slot indices */
    int             head, count;
    char*           path;
    uint32_t        accepted, written, dropped;
} vk_dump_q;

static void vk_dump_write_file(const char* path, const vk_dump_slot* s)
{
    size_t n = strlen(path) + 5;
    char* tmp = (char*)malloc(n);
    if (!tmp) return;
    snprintf(tmp, n, "%s.tmp", path);
    FILE* f = fopen(tmp, "wb");
    if (!f) { VK_DUMP_LOG("cannot open %s for the frame dump\n", tmp); free(tmp); return; }
    fprintf(f, "P6\n%u %u\n255\n", s->width, s->height);
    for (size_t i = 0; i < (size_t)s->width * s->height; i++)
        fwrite(s->buf + i * 4, 1, 3, f);
    int bad = ferror(f);
    if (fclose(f) != 0) bad = 1;
    if (bad || rename(tmp, path) != 0) { VK_DUMP_LOG("frame dump to %s failed\n", path); remove(tmp); }
    free(tmp);
}

static void* vk_dump_writer(void* arg)
{
    vk_dump_q* q = (vk_dump_q*)arg;
    pthread_mutex_lock(&q->mu);
    for (;;) {
        while (!q->stop && (q->paused || q->count == 0)) pthread_cond_wait(&q->cv, &q->mu);
        if (q->count == 0) break;                      /* stop and drained */
        int i = q->order[q->head];
        q->head = (q->head + 1) % VK_DUMP_SLOTS;
        q->count--;
        pthread_mutex_unlock(&q->mu);
        vk_dump_write_file(q->path, &q->slot[i]);
        pthread_mutex_lock(&q->mu);
        q->slot[i].busy = 0;
        q->written++;
        pthread_cond_broadcast(&q->cv);
    }
    pthread_mutex_unlock(&q->mu);
    return NULL;
}

/* Never blocks on I/O. Returns 1 if the frame was accepted, 0 if dropped.
 * Single producer (the present thread). */
static int vk_dump_enqueue(vk_dump_q* q, const char* path, uint32_t w, uint32_t h,
                           const unsigned char* rgba)
{
    size_t need = (size_t)w * h * 4;
    if (!q->started) {
        pthread_mutex_init(&q->mu, NULL);
        pthread_cond_init(&q->cv, NULL);
        q->path = strdup(path);
        if (!q->path) return 0;
        if (pthread_create(&q->thread, NULL, vk_dump_writer, q) != 0) {
            VK_DUMP_LOG("cannot start the frame dump writer\n");
            free(q->path); q->path = NULL; return 0;
        }
        q->started = 1;
    }
    int si = -1;
    pthread_mutex_lock(&q->mu);
    for (int i = 0; i < VK_DUMP_SLOTS; i++)
        if (!q->slot[i].busy) { si = i; q->slot[i].busy = 1; break; }
    if (si < 0) {
        uint32_t d = ++q->dropped;
        pthread_mutex_unlock(&q->mu);
        if (d % VK_DUMP_DROP_LOG == 1)
            VK_DUMP_LOG("frame dump queue full: %u capture(s) dropped so far\n", d);
        return 0;
    }
    pthread_mutex_unlock(&q->mu);

    vk_dump_slot* s = &q->slot[si];            /* busy: the writer will not touch it */
    if (s->cap < need) {
        unsigned char* nb = (unsigned char*)realloc(s->buf, need);
        if (!nb) { pthread_mutex_lock(&q->mu); s->busy = 0; q->dropped++; pthread_mutex_unlock(&q->mu); return 0; }
        s->buf = nb; s->cap = need;
    }
    memcpy(s->buf, rgba, need);
    s->width = w; s->height = h;

    pthread_mutex_lock(&q->mu);
    q->order[(q->head + q->count) % VK_DUMP_SLOTS] = si;
    q->count++;
    q->accepted++;
    pthread_cond_signal(&q->cv);
    pthread_mutex_unlock(&q->mu);
    return 1;
}

/* Drain accepted frames, join the writer, free everything. Safe if never started. */
static void vk_dump_shutdown(vk_dump_q* q)
{
    if (!q->started) return;
    pthread_mutex_lock(&q->mu);
    q->stop = 1; q->paused = 0;
    pthread_cond_broadcast(&q->cv);
    pthread_mutex_unlock(&q->mu);
    pthread_join(q->thread, NULL);
    if (q->dropped)
        VK_DUMP_LOG("frame dump: %u written, %u dropped (queue full)\n", q->written, q->dropped);
    for (int i = 0; i < VK_DUMP_SLOTS; i++) free(q->slot[i].buf);
    free(q->path);
    pthread_cond_destroy(&q->cv);
    pthread_mutex_destroy(&q->mu);
    memset(q, 0, sizeof *q);
}

#endif /* RSX_VK_DUMP_H */
