/* Frame-dump queue: enqueue never blocks, overflow drops, output matches the old synchronous writer. */
#include <assert.h>
#include <dirent.h>
#include <time.h>
#include <unistd.h>
#include "../libs/video/rsx_vk_dump.h"

static double now_ms(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e3 + t.tv_nsec / 1e6; }

/* The pre-change vk_dump_ppm formatting, verbatim, writing straight to `path`. */
static void reference_ppm(const char* path, uint32_t w, uint32_t h, const unsigned char* rgba)
{
    FILE* f = fopen(path, "wb"); assert(f);
    fprintf(f, "P6\n%u %u\n255\n", w, h);
    for (size_t i = 0; i < (size_t)w * h; i++) fwrite(rgba + i * 4, 1, 3, f);
    fclose(f);
}
static unsigned char* slurp(const char* p, size_t* n)
{
    FILE* f = fopen(p, "rb"); assert(f);
    fseek(f, 0, SEEK_END); *n = (size_t)ftell(f); fseek(f, 0, SEEK_SET);
    unsigned char* b = malloc(*n); assert(fread(b, 1, *n, f) == *n); fclose(f); return b;
}

int main(void)
{
    char dir[] = "/tmp/vkdumpXXXXXX"; assert(mkdtemp(dir));
    char out[300], ref[300];
    snprintf(out, sizeof out, "%s/frame.ppm", dir);
    snprintf(ref, sizeof ref, "%s/ref.ppm", dir);

    enum { W = 640, H = 360 };
    unsigned char* px = malloc((size_t)W * H * 4);
    vk_dump_q q; memset(&q, 0, sizeof q);

    /* Park the writer: start it, then pause before any frame is queued. */
    int total = VK_DUMP_SLOTS + 6;
    for (size_t i = 0; i < (size_t)W * H * 4; i++) px[i] = (unsigned char)(i * 7 + 3);
    assert(vk_dump_enqueue(&q, out, W, H, px) == 1);   /* starts the thread */
    /* let frame 0 finish, then park the writer so later frames stay queued */
    for (int i = 0; i < 5000; i++) { pthread_mutex_lock(&q.mu); int w = q.written; pthread_mutex_unlock(&q.mu); if (w) break; usleep(1000); }
    pthread_mutex_lock(&q.mu); assert(q.written == 1); q.paused = 1; pthread_mutex_unlock(&q.mu);

    double worst = 0; int accepted = 0;
    for (int n = 0; n < total; n++) {
        for (size_t i = 0; i < (size_t)W * H * 4; i++) px[i] = (unsigned char)(i * 7 + n + 11);
        double t0 = now_ms();
        accepted += vk_dump_enqueue(&q, out, W, H, px);
        double dt = now_ms() - t0; if (dt > worst) worst = dt;
        assert(dt < 5.0);
    }
    assert(accepted == VK_DUMP_SLOTS);
    assert(q.dropped == (uint32_t)(total - VK_DUMP_SLOTS));
    printf("enqueue worst %.3f ms, accepted %d, dropped %u\n", worst, accepted, q.dropped);

    /* Release; the last accepted frame (index VK_DUMP_SLOTS-1) is the final file. */
    vk_dump_shutdown(&q);
    assert(q.started == 0);
    for (size_t i = 0; i < (size_t)W * H * 4; i++) px[i] = (unsigned char)(i * 7 + (VK_DUMP_SLOTS - 1) + 11);
    reference_ppm(ref, W, H, px);
    size_t n1, n2; unsigned char* a = slurp(out, &n1); unsigned char* b = slurp(ref, &n2);
    assert(n1 == n2 && memcmp(a, b, n1) == 0);

    /* No .tmp left. */
    DIR* d = opendir(dir); struct dirent* e; int files = 0;
    while ((e = readdir(d))) { if (e->d_name[0] == '.') continue; files++; assert(!strstr(e->d_name, ".tmp")); }
    closedir(d);
    assert(files == 2);

    /* Dimensions grow: reallocation path. */
    vk_dump_q q2; memset(&q2, 0, sizeof q2);
    assert(vk_dump_enqueue(&q2, out, 8, 8, px) == 1);
    assert(vk_dump_enqueue(&q2, out, W, H, px) == 1);
    vk_dump_shutdown(&q2);
    reference_ppm(ref, W, H, px);
    free(a); free(b); a = slurp(out, &n1); b = slurp(ref, &n2);
    assert(n1 == n2 && memcmp(a, b, n1) == 0);
    vk_dump_shutdown(&q2);   /* idempotent when never started */

    remove(out); remove(ref); rmdir(dir);
    puts("vk_dump: OK");
    return 0;
}
