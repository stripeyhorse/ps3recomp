/*
 * ps3recomp - packed indexed triangle lists (rsx_vertex_compact.h)
 *
 * The GPU vertex decode is only as right as the packet it reads, so every
 * piece is checked against the CPU fetch it replaces, with no GPU:
 *  - the 16-bit index scan (NEON on AArch64) against a plain per-index walk,
 *    at every source alignment, across the 20-bit wrap, with sentinels;
 *  - a prepared packet, decoded after its guest arrays and indices have been
 *    overwritten, against rsx_vertex_fetch_many over the same references;
 *  - the shader's block layout, read back the way rsx_vertex_pull_hlsl reads
 *    it (aligned 32-bit loads), against the CPU decode for every format;
 *  - the shapes the builder must decline, and the shader rewrite.
 * The HLSL itself runs in test_metal_vertex_pull.m on a Mac.
 *
 * Builds and runs on Linux as well as macOS; meant for ASan and UBSan.
 */
#include "rsx_vertex_compact.h"
#include "rsx_vertex_formats.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static int g_failures;
#define CHECK(cond, ...)                                                      \
    do {                                                                      \
        if (cond) { printf("[PASS] " __VA_ARGS__); printf("\n"); }            \
        else { printf("[FAIL] " __VA_ARGS__); printf("   (%s:%d)\n",         \
                      __FILE__, __LINE__); g_failures++; }                    \
    } while (0)

static u32 g_seed = 0x8cc79af3u;
static u32 rnd(void)
{
    g_seed ^= g_seed << 13;
    g_seed ^= g_seed >> 17;
    g_seed ^= g_seed << 5;
    return g_seed;
}

/* ---- index scan ---------------------------------------------------------- */

static int reference_span(const u8* src, u16* owned, u32 count, u32 base, u32 restart,
                          u32* first, u32* last)
{
    *first = UINT32_MAX;
    *last = 0;
    for (u32 i = 0; i < count; i++) {
        const u32 index = (u32)((src[2 * i] << 8) | src[2 * i + 1]);
        if (index == restart) return 0;
        owned[i] = (u16)index;
        const u32 effective = (index + base) & 0xFFFFFu;
        if (effective < *first) *first = effective;
        if (effective > *last) *last = effective;
    }
    return 1;
}

static void test_index_span(void)
{
    enum { CASES = 20000, MAX = 300 };
    static u8 buffer[2 * MAX + 16];
    static u16 want[MAX], got[MAX];
    u32 mismatches = 0, rejected = 0, wrapped = 0;
    for (u32 c = 0; c < CASES; c++) {
        const u32 align = c & 15u, count = 1u + rnd() % MAX;
        u8* src = buffer + align;
        const u32 spread = (rnd() & 1) ? 0xFFFFu : 64u + rnd() % 512u;
        const u32 low = rnd() & 0xFFFFu;
        for (u32 i = 0; i < count; i++) {
            const u32 v = (low + rnd() % spread) & 0xFFFFu;
            src[2 * i] = (u8)(v >> 8);
            src[2 * i + 1] = (u8)v;
        }
        u32 base;
        switch (rnd() % 4) {
        case 0:  base = 0; break;
        case 1:  base = 0xFFFFFu - (rnd() % 0x10000u); break;   /* near the wrap */
        case 2:  base = rnd(); break;                           /* high bits ignored */
        default: base = rnd() & 0xFFFFFu; break;
        }
        u32 restart;
        switch (rnd() % 4) {
        case 0:  restart = UINT32_MAX; break;
        case 1:  restart = (u32)((src[2 * (count / 2)] << 8) | src[2 * (count / 2) + 1]); break;
        case 2:  restart = 0x10000u | (rnd() & 0xFFFFu); break; /* wider than an index */
        default: restart = rnd() & 0xFFFFu; break;
        }
        u32 wf, wl, gf, gl;
        const int wok = reference_span(src, want, count, base, restart, &wf, &wl);
        const int gok = rsx_vertex_index_span_u16(src, got, count, base, restart, &gf, &gl);
        if (wok != gok || (wok && (wf != gf || wl != gl ||
                                   memcmp(want, got, count * sizeof(u16)))))
            mismatches++;
        rejected += !wok;
        wrapped += wok && ((base & 0xFFFFFu) + low + spread > 0xFFFFFu);
    }
    CHECK(!mismatches, "%u index runs match the per-index walk at alignments 0-15 "
          "(%u rejected by a sentinel, ~%u across the wrap)", CASES, rejected, wrapped);
}

/* ---- packets against the CPU fetch --------------------------------------- */

enum { N = 192, RAW_STRIDE = 32, SLOT_BYTES = 16, FIRST = 192, BASE = 0xFFFF2u,
       SIDE = 64 * 1024 };
static u8 g_raw[N * RAW_STRIDE], g_side[SIDE], g_indices[N * 2];
static int g_unmapped;

/* Main memory: the interleaved array at FIRST's element, a separate array
 * at 0x100000. */
static const u8* resolve(void* user, u32 location, u32 offset, u32 bytes)
{
    (void)user; (void)location;
    if (g_unmapped) return NULL;
    const u32 first = FIRST * RAW_STRIDE;
    if (offset >= first && (u64)offset + bytes <= first + sizeof g_raw)
        return g_raw + (offset - first);
    if (offset >= 0x100000u && (u64)offset + bytes <= 0x100000u + sizeof g_side)
        return g_side + (offset - 0x100000u);
    return NULL;
}

static void set_attr(rsx_vertex_fetch_plan* plan, u32 attr, u32 offset, u32 type,
                     u32 size, u32 stride)
{
    rsx_vertex_fetch_attr* a = &plan->attr[attr];
    a->desc = (rsx_dsp_vertex_attr){ .offset = offset, .type = type, .size = size,
                                     .stride = stride, .frequency = 1 };
    a->elem_size = rsx_vertex_attrib_size(type, size);
    a->stride = stride;
}

static rsx_packed_triangles g_packet;
static _Alignas(16) u8 g_expected[N * 5 * SLOT_BYTES], g_actual[N * 5 * SLOT_BYTES];
static _Alignas(16) u8 g_decoded[RSX_PACKED_VERTEX_LIMIT * 5 * SLOT_BYTES];
static u32 g_decoded_indices[RSX_PACKED_VERTEX_LIMIT];

static void test_packets(void)
{
    rsx_vertex_fetch_plan plan;
    memset(&plan, 0, sizeof plan);
    plan.guest_ptr = resolve;
    rsx_vertex_layout_plan_init(&plan.layout, 0x10F);              /* 0..3 and 8 */
    /* Three interleaved, overlapping attributes at odd offsets, a default,
     * and one array of its own. */
    set_attr(&plan, 0, 1, RSX_VTX_TYPE_FLOAT, 3, RAW_STRIDE);
    set_attr(&plan, 1, 15, RSX_VTX_TYPE_HALF, 2, RAW_STRIDE);
    set_attr(&plan, 2, 21, RSX_VTX_TYPE_UINT8, 4, RAW_STRIDE);
    plan.attr[3].default_value[0] = 1.0f;
    plan.attr[3].default_value[3] = -2.0f;
    /* The separate array's element e sits at 0x100000 + 6 * (e - FIRST). */
    set_attr(&plan, 8, 0x100000u - FIRST * 6u, RSX_VTX_TYPE_CMP32, 1, 6);
    const u32 stride = plan.layout.stride;

    u32 corners = 0;
    for (u32 round = 0; round < 128; ++round) {
        rsx_vertex_ref refs[N];
        for (u32 j = 0; j < sizeof g_raw; ++j) g_raw[j] = (u8)(j * (3 + round) + (j >> 3));
        for (u32 j = 0; j < N * 6u; ++j) g_side[j] = (u8)rnd();
        for (u32 i = 0; i < N; ++i) {
            /* Duplicate corners in a new order every round, through a base
             * index that wraps. */
            const u32 vertex = (FIRST + ((53u * (i % 176u) + round) % N) - BASE) & 0xFFFFFu;
            refs[i] = (rsx_vertex_ref){ vertex, BASE };
            g_indices[2 * i] = (u8)(vertex >> 8);
            g_indices[2 * i + 1] = (u8)vertex;
        }
        rsx_vertex_fetch_plan stock = plan;
        rsx_vertex_fetch_plan_prepare(&stock, refs, N);
        if (!rsx_vertex_fetch_many(&stock, refs, N, g_expected)) { g_failures++; return; }
        if (!rsx_vertex_packed_triangles_prepare(&g_packet, &plan, g_indices, N, BASE,
                                                 UINT32_MAX)) {
            CHECK(0, "round %u prepared", round);
            return;
        }
        /* The guest is free to rewrite everything now. */
        memset(g_raw, 0xFF, sizeof g_raw);
        memset(g_side, 0xEE, sizeof g_side);
        memset(g_indices, 0x55, sizeof g_indices);
        if (!rsx_vertex_packed_triangles_decode(&g_packet, stride, g_decoded,
                                                g_decoded_indices)) {
            g_failures++;
            return;
        }
        for (u32 i = 0; i < N; ++i)
            memcpy(g_actual + i * stride, g_decoded + g_decoded_indices[i] * stride, stride);
        if (memcmp(g_actual, g_expected, (size_t)N * stride)) {
            CHECK(0, "round %u matches the CPU fetch", round);
            return;
        }
        corners += N;
    }
    CHECK(g_packet.slots == 5 && g_packet.count == N && g_packet.bytes < 2u * N * RAW_STRIDE,
          "the packet owns one span for the interleaved array and one for the other "
          "(%u bytes)", g_packet.bytes);
    CHECK(corners == 128u * N, "%u corners decode to the CPU fetch after their inputs "
          "were overwritten", corners);

    /* What the builder must leave to the CPU fetch. */
    for (u32 i = 0; i < N; ++i) {
        const u32 vertex = (FIRST + i - BASE) & 0xFFFFFu;
        g_indices[2 * i] = (u8)(vertex >> 8);
        g_indices[2 * i + 1] = (u8)vertex;
    }
    const u32 sentinel = (u32)((g_indices[20] << 8) | g_indices[21]);
    CHECK(rsx_vertex_packed_triangles_prepare(&g_packet, &plan, g_indices, N, BASE, UINT32_MAX),
          "an ordinary list is admitted");
    CHECK(!rsx_vertex_packed_triangles_prepare(&g_packet, &plan, g_indices, N, BASE, sentinel),
          "a list holding the restart sentinel is not");
    CHECK(!rsx_vertex_packed_triangles_prepare(&g_packet, &plan, g_indices, N - 1, BASE, UINT32_MAX),
          "nor one that is not whole triangles");
    plan.attr[1].desc.frequency = 2;
    CHECK(!rsx_vertex_packed_triangles_prepare(&g_packet, &plan, g_indices, N, BASE, UINT32_MAX),
          "nor one with a divided attribute");
    plan.attr[1].desc.frequency = 1;
    plan.divider_mask = 1u << 1;
    CHECK(!rsx_vertex_packed_triangles_prepare(&g_packet, &plan, g_indices, N, BASE, UINT32_MAX),
          "nor one with a modulo attribute");
    plan.divider_mask = 0;
    g_unmapped = 1;
    CHECK(!rsx_vertex_packed_triangles_prepare(&g_packet, &plan, g_indices, N, BASE, UINT32_MAX),
          "nor one whose arrays are not mapped");
    g_unmapped = 0;
    plan.attr[0].desc.type = 9;
    CHECK(!rsx_vertex_packed_triangles_prepare(&g_packet, &plan, g_indices, N, BASE, UINT32_MAX),
          "nor one with a format the decoder lacks");
    plan.attr[0].desc.type = RSX_VTX_TYPE_FLOAT;
    /* Indices 0 and 4096 apart: a range the packet cannot hold. */
    g_indices[0] = 0; g_indices[1] = 0;
    g_indices[2] = 0x10; g_indices[3] = 0x00;
    CHECK(!rsx_vertex_packed_triangles_prepare(&g_packet, &plan, g_indices, N, 0, UINT32_MAX),
          "nor one spanning %u vertices", RSX_PACKED_VERTEX_LIMIT + 1u);
    rsx_vertex_fetch_plan inline_plan = plan;
    inline_plan.inline_data = g_raw;
    CHECK(!rsx_vertex_packed_triangles_prepare(&g_packet, &inline_plan, g_indices, N, BASE,
                                               UINT32_MAX),
          "nor an inline stream");
}

/* ---- the block as the shader reads it ------------------------------------ */

/* rsx_vertex_pull_hlsl, statement for statement, over a ByteAddressBuffer
 * that only has aligned 32-bit little-endian loads. */
static const u8* g_block;
static u32 load(u32 offset)
{
    u32 w;
    memcpy(&w, g_block + (offset & ~3u), 4);
    return w;
}
static u32 pull_byte(u32 offset) { return (load(offset) >> ((offset & 3u) * 8u)) & 255u; }
static u32 pull_be16(u32 offset) { return (pull_byte(offset) << 8) | pull_byte(offset + 1u); }
static u32 pull_be32(u32 offset) { return (pull_be16(offset) << 16) | pull_be16(offset + 2u); }
static u32 pull_half(u32 word)
{
    const u32 sign = (word >> 15) << 31, exp = (word >> 10) & 31u, fraction = (word & 1023u) << 13;
    if (exp == 0u) return sign;
    if (exp == 31u) return sign | 0x7f800000u | fraction;
    return sign | ((exp + 112u) << 23) | fraction;
}
static float as_float(u32 w) { float f; memcpy(&f, &w, 4); return f; }
static u32 as_uint(float f) { u32 w; memcpy(&w, &f, 4); return w; }
static s32 sext16(u32 v) { return (s32)(v << 16) >> 16; }

static void pull_attribute(u32 index, u32 slot, u32 out[4])
{
    const u32 desc = 16u + 32u * slot;
    const u32 offset = load(desc), stride = load(desc + 4), type = load(desc + 8),
              size = load(desc + 12);
    if (type == 0u) { for (u32 c = 0; c < 4; c++) out[c] = load(desc + 16u + 4u * c); return; }
    const u32 element = ((index + load(4)) & 0xFFFFFu) - load(0);
    const u32 source = load(8) + offset + stride * element;
    if (type == 6u) {
        const u32 word = pull_be32(source);
        const s32 x = (s32)(word << 21) >> 21, y = (s32)(word << 10) >> 21, z = (s32)word >> 22;
        out[0] = as_uint((float)x / 1023.0f);
        out[1] = as_uint((float)y / 1023.0f);
        out[2] = as_uint((float)z / 511.0f);
        out[3] = 0x3f800000u;
        return;
    }
    out[0] = out[1] = out[2] = 0u;
    out[3] = 0x3f800000u;
    for (u32 c = 0; c < size; c++) {
        switch (type) {
        case 1u: out[c] = as_uint((float)sext16(pull_be16(source + 2u * c)) / 32767.0f); break;
        case 2u: out[c] = pull_be32(source + 4u * c); break;
        case 3u: out[c] = pull_half(pull_be16(source + 2u * c)); break;
        case 4u: out[c] = as_uint((float)pull_byte(source + c) / 255.0f); break;
        case 5u: out[c] = as_uint((float)sext16(pull_be16(source + 2u * c))); break;
        case 7u: out[c] = as_uint((float)pull_byte(source + c)); break;
        }
    }
}

static void test_block_layout(void)
{
    static u8 block[sizeof(rsx_packed_triangles) + 64];
    u32 values = 0, wrong = 0;
    for (u32 trial = 0; trial < 256; trial++) {
        rsx_packed_triangles* p = &g_packet;
        memset(p, 0, sizeof *p);
        p->slots = 1u + rnd() % 16u;
        p->base_index = (trial & 1) ? 0xFFF40u : rnd() & 0xFFFFu;
        p->first = (0x2000u + p->base_index) & 0xFFFFFu;
        p->count = 3u * (1u + rnd() % 40u);
        const u32 elements = 1u + rnd() % 64u;
        p->last = p->first + elements - 1u;
        /* Unaligned offsets and strides, and a data length of any size: the
         * padding must keep the last byte inside an aligned load. */
        p->bytes = 0;
        for (u32 s = 0; s < p->slots; s++) {
            rsx_packed_attribute* a = &p->attributes[s];
            a->type = rnd() % 8u;
            a->size = a->type == 6u ? 1u : 1u + rnd() % 4u;
            const u32 bytes = a->type ? rsx_vertex_attrib_size(a->type, a->size) : 0u;
            a->stride = bytes + rnd() % 8u;
            a->offset = rnd() % 64u;
            for (u32 c = 0; c < 4; c++) a->defaults[c] = rnd();
            const u32 end = a->offset + (elements - 1u) * a->stride + bytes;
            if (a->type && end > p->bytes) p->bytes = end;
        }
        p->bytes += rnd() % 4u;
        for (u32 i = 0; i < p->bytes; i++) p->data[i] = (u8)rnd();
        for (u32 i = 0; i < p->count; i++)
            p->indices[i] = (u16)((0x2000u + rnd() % elements) & 0xFFFFu);
        u32 index_offset;
        const u32 size = rsx_vertex_packed_triangles_size(p, &index_offset);
        memset(block, 0xCD, sizeof block);
        rsx_vertex_packed_triangles_write(p, block);
        g_block = block;
        if (index_offset & 3u || block[size] != 0xCD ||
            memcmp(block + index_offset, p->indices, p->count * 2u))
            wrong++;
        if (!rsx_vertex_packed_triangles_decode(p, 16u * p->slots, g_decoded, g_decoded_indices)) {
            wrong++;
            continue;
        }
        for (u32 i = 0; i < p->count; i++) {
            for (u32 s = 0; s < p->slots; s++) {
                u32 gpu[4];
                pull_attribute(p->indices[i], s, gpu);
                const u8* cpu = g_decoded + g_decoded_indices[i] * 16u * p->slots + 16u * s;
                wrong += memcmp(gpu, cpu, 16) != 0;
                values += 4;
            }
        }
    }
    CHECK(!wrong, "%u component words read from the shader block match the CPU decode, "
          "every format and size at any byte offset", values);
    (void)as_float;
}

/* ---- the shader rewrite --------------------------------------------------- */

static void test_shader_rewrite(void)
{
    static const char vs[] =
        "struct VSInput {\n    float4 a0:ATTR0;\n    float4 a3:ATTR3;\n};\n"
        "struct VSOutput { float4 pos:SV_Position; };\n"
        "VSOutput main(VSInput input) {\n"
        "    VSOutput o; o.pos = input.a0 + input.a3; return o;\n}\n";
    static char out[16384];
    rsx_vertex_layout_plan layout;
    rsx_vertex_layout_plan_init(&layout, (1u << 0) | (1u << 3));
    const int ok = rsx_vertex_pull_shader(vs, &layout, out, sizeof out);
    CHECK(ok && strstr(out, "ByteAddressBuffer rsx_pull_packet : register(t28)") &&
          strstr(out, "VSOutput main(uint vertex_id:SV_VertexID) {\n    VSInput input;\n"
                      "    input.a0 = rsx_pull_attribute(vertex_id, 0u);\n"
                      "    input.a3 = rsx_pull_attribute(vertex_id, 1u);\n"
                      "    VSOutput o; o.pos = input.a0 + input.a3; return o;\n}\n") &&
          !strstr(out, "main(VSInput input)") &&
          strstr(out, "struct VSInput {"),
          "the entry point reads the packet into the VSInput the program uses");
    CHECK(!rsx_vertex_pull_shader("float4 main() : SV_Position { return 0; }", &layout,
                                  out, sizeof out),
          "a program without the decompiler's entry point is not rewritten");
    CHECK(!rsx_vertex_pull_shader("struct VSInput { uint vertex_id:SV_VertexID; };\n"
                                  "VSOutput main(VSInput input) {\n}\n", &layout, out, sizeof out),
          "nor one that reads SV_VertexID itself");
    CHECK(!rsx_vertex_pull_shader(vs, &layout, out, 600),
          "nor one that does not fit");
}

/* ---- timing (information only) ------------------------------------------- */

static double now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec * 1e9 + (double)t.tv_nsec;
}

static void time_index_span(void)
{
    static u8 src[2 * 3072];
    static u16 owned[3072];
    for (u32 i = 0; i < 3072; i++) { src[2 * i] = (u8)(i >> 4); src[2 * i + 1] = (u8)(i * 7u); }
    static const u32 sizes[] = { 48, 192, 768, 3072 };
    for (u32 k = 0; k < 4; k++) {
        const u32 reps = 400000u / sizes[k] * 16u;
        u32 f, l, sink = 0;
        double t0 = now_ns();
        for (u32 r = 0; r < reps; r++) {
            sink += (u32)reference_span(src, owned, sizes[k], r & 0xFFu, UINT32_MAX, &f, &l);
            sink += owned[r % sizes[k]];
        }
        const double walk = (now_ns() - t0) / reps;
        t0 = now_ns();
        for (u32 r = 0; r < reps; r++) {
            sink += (u32)rsx_vertex_index_span_u16(src, owned, sizes[k], r & 0xFFu, UINT32_MAX, &f, &l);
            sink += owned[r % sizes[k]];
        }
        const double scan = (now_ns() - t0) / reps;
        printf("[time] %4u indices: per-index walk %8.1f ns, rsx_vertex_index_span_u16 %8.1f ns (%u)\n",
               sizes[k], walk, scan, sink & 1u);
    }
}

int main(int argc, char** argv)
{
    test_index_span();
    test_packets();
    test_block_layout();
    test_shader_rewrite();
    if (argc > 1 && !strcmp(argv[1], "--time")) time_index_span();
    printf(g_failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", g_failures);
    return g_failures ? 1 : 0;
}
