#include "rsx_vertex_compact.h"
#include "rsx_commands.h"
#include "rsx_vertex_formats.h"

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__aarch64__) && defined(__ARM_NEON) && \
    (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
#include <arm_neon.h>
#define RSX_PACKED_INDEX_NEON 1
#endif

static float rsx_compact_be_f32(const u8* p)
{
    const u32 w = ((u32)p[0] << 24) | ((u32)p[1] << 16) |
                  ((u32)p[2] << 8) | (u32)p[3];
    float f;
    memcpy(&f, &w, sizeof(f));
    return f;
}

static float rsx_compact_be_f16(const u8* p)
{
    const u16 h = (u16)((p[0] << 8) | p[1]);
    const u32 sign = (u32)(h >> 15) << 31;
    const u32 exp = (h >> 10) & 0x1F;
    const u32 man = h & 0x3FF;
    u32 out;
    if (exp == 0)
        out = sign;
    else if (exp == 31)
        out = sign | 0x7F800000u | (man << 13);
    else
        out = sign | ((exp + 112) << 23) | (man << 13);
    float f;
    memcpy(&f, &out, sizeof(f));
    return f;
}

int rsx_vertex_decode_element(
    u32 type, u32 size, const u8* source, float out[4])
{
    if (!source || !out)
        return 0;
    out[0] = out[1] = out[2] = 0.0f;
    out[3] = 1.0f;
    for (u32 component = 0; component < size && component < 4;
         component++) {
        switch (type) {
        case RSX_VTX_TYPE_FLOAT:
            out[component] = rsx_compact_be_f32(source + component * 4);
            break;
        case RSX_VTX_TYPE_HALF:
            out[component] = rsx_compact_be_f16(source + component * 2);
            break;
        case RSX_VTX_TYPE_UNORM8:
            out[component] = source[component] / 255.0f;
            break;
        case RSX_VTX_TYPE_UINT8:
            out[component] = (float)source[component];
            break;
        case RSX_VTX_TYPE_SNORM16: {
            const s16 value = (s16)(
                (source[component * 2] << 8) | source[component * 2 + 1]);
            out[component] = value / 32767.0f;
            break;
        }
        case RSX_VTX_TYPE_SINT16: {
            const s16 value = (s16)(
                (source[component * 2] << 8) | source[component * 2 + 1]);
            out[component] = (float)value;
            break;
        }
        case RSX_VTX_TYPE_CMP32: {
            const u32 word =
                ((u32)source[0] << 24) | ((u32)source[1] << 16) |
                ((u32)source[2] << 8) | (u32)source[3];
            s32 x = (s32)(word & 0x7FF);
            s32 y = (s32)((word >> 11) & 0x7FF);
            s32 z = (s32)((word >> 22) & 0x3FF);
            if (x & 0x400) x -= 0x800;
            if (y & 0x400) y -= 0x800;
            if (z & 0x200) z -= 0x400;
            out[0] = (float)x / 1023.0f;
            out[1] = (float)y / 1023.0f;
            out[2] = (float)z / 511.0f;
            out[3] = 1.0f;
            return 1;
        }
        default:
            return 0;
        }
    }
    return 1;
}

void rsx_vertex_layout_plan_init(
    rsx_vertex_layout_plan* plan, u32 input_mask)
{
    if (!plan)
        return;
    memset(plan, 0, sizeof(*plan));
    plan->mask = input_mask & 0xFFFFu;
    for (u32 attr = 0; attr < RSX_DSP_NUM_VERTEX_ATTR; attr++)
        plan->slot_by_attr[attr] = -1;
    for (u32 attr = 0; attr < RSX_DSP_NUM_VERTEX_ATTR; attr++) {
        if (!(plan->mask & (1u << attr)))
            continue;
        const u32 slot = plan->count++;
        plan->attrs[slot] = (u8)attr;
        plan->slot_by_attr[attr] = (s8)slot;
    }
    plan->stride = (u32)plan->count * 16u;
}

void rsx_vertex_fetch_default(const rsx_dispatch* rsx, u32 attr, float out[4])
{
    rsx_dsp_vertex_default(rsx, attr, out);
    if (attr != 3)
        return;
    const u32 base = (0x1C00u + attr * 0x10u) >> 2;
    const u32 written = rsx->seen[base] | rsx->seen[base + 1] |
        rsx->seen[base + 2] | rsx->seen[base + 3];
    /* Preserve the legacy white color fallback only for an unwritten
     * attribute. An explicit (0,0,0,1) is shader input, even when it equals
     * the hardware default; a shader may swizzle x into output alpha. */
    if (!written && out[0] == 0.0f && out[1] == 0.0f &&
        out[2] == 0.0f && out[3] == 1.0f)
        out[0] = out[1] = out[2] = 1.0f;
}

void rsx_vertex_fetch_plan_init(
    rsx_vertex_fetch_plan* plan, const rsx_dispatch* rsx,
    const rsx_vertex_layout_plan* layout,
    rsx_vertex_guest_ptr_fn guest_ptr, void* guest_user)
{
    if (!plan || !rsx || !layout)
        return;
    memset(plan, 0, sizeof(*plan));
    plan->layout = *layout;
    plan->base_offset = rsx_dsp_vertex_data_base_offset(rsx);
    plan->divider_mask = rsx_dsp_reg(rsx, 0x1FC0);
    plan->guest_ptr = guest_ptr;
    plan->guest_user = guest_user;
    for (u32 attr = 0; attr < RSX_DSP_NUM_VERTEX_ATTR; attr++) {
        rsx_vertex_fetch_attr* fetch = &plan->attr[attr];
        rsx_dsp_get_vertex_attr(rsx, attr, &fetch->desc);
        fetch->elem_size =
            rsx_vertex_attrib_size(fetch->desc.type, fetch->desc.size);
        fetch->stride =
            fetch->desc.stride ? fetch->desc.stride : fetch->elem_size;
        rsx_vertex_fetch_default(rsx, attr, fetch->default_value);
    }
}

u32 rsx_vertex_inline_layout(
    const rsx_dispatch* rsx, u32 offsets[RSX_DSP_NUM_VERTEX_ATTR])
{
    u32 packed = 0, declared = 0;
    for (u32 attr = 0; attr < RSX_DSP_NUM_VERTEX_ATTR; attr++) {
        rsx_dsp_vertex_attr desc;
        rsx_dsp_get_vertex_attr(rsx, attr, &desc);
        if (offsets)
            offsets[attr] = packed;
        if (!desc.type || !desc.size)
            continue;
        packed += rsx_vertex_attrib_size(desc.type, desc.size);
        if (desc.stride > declared)
            declared = desc.stride;
    }
    return declared ? declared : packed;
}

void rsx_vertex_fetch_plan_set_inline(
    rsx_vertex_fetch_plan* plan, const rsx_dispatch* rsx,
    const u8* data, u32 bytes)
{
    if (!plan)
        return;
    plan->inline_data = NULL;
    plan->inline_bytes = 0;
    if (!data || !bytes || !rsx)
        return;
    plan->inline_stride = rsx_vertex_inline_layout(rsx, plan->inline_off);
    plan->inline_data = data;
    plan->inline_bytes = bytes;
}

void rsx_vertex_fetch_plan_prepare(
    rsx_vertex_fetch_plan* plan, const rsx_vertex_ref* refs, u32 count)
{
    if (!plan)
        return;
    /* Ordinary per-vertex attributes (rsx_vertex_freq_plain) all span the same
     * elements: find them once for the draw instead of once per attribute.
     * The per-attribute walk was 2.2 ms of the RSX thread a frame in
     * inFAMOUS 2's New Marais. */
    int plain_known = 0;
    u32 plain_first = UINT_MAX, plain_last = 0;
    for (u32 slot = 0; slot < plan->layout.count; slot++) {
        const u32 attr = plan->layout.attrs[slot];
        rsx_vertex_fetch_attr* fetch = &plan->attr[attr];
        fetch->stable_base = NULL;
        fetch->stable_first = 0;
        fetch->stable_last = 0;
        if (!refs || !count ||
            !fetch->desc.type || !fetch->desc.size || !fetch->elem_size)
            continue;

        /* INLINE_ARRAY: the stream is already one contiguous, correctly
         * strided array per attribute, which is exactly what stable_base
         * describes -- so it needs no resolver and no bounds fallback, only
         * the right base. */
        if (plan->inline_data) {
            const u32 stride =
                plan->inline_stride ? plan->inline_stride : fetch->stride;
            const u32 off = plan->inline_off[attr];
            if (!stride || off + fetch->elem_size > stride)
                continue;
            const u32 verts = plan->inline_bytes / stride;
            if (!verts)
                continue;
            fetch->stride = stride;
            fetch->stable_base = plan->inline_data + off;
            fetch->stable_first = 0;
            fetch->stable_last = verts - 1;
            continue;
        }

        if (!plan->guest_ptr)
            continue;

        u32 first = UINT_MAX;
        u32 last = 0;
        if (rsx_vertex_freq_plain(fetch->desc.frequency,
                                  (plan->divider_mask >> attr) & 1u)) {
            if (!plain_known) {
                plain_known = 1;
                for (u32 i = 0; i < count; i++) {
                    const u32 element = rsx_vertex_element_index(
                        refs[i].vertex_id, refs[i].base_index, 0, 0);
                    if (element < plain_first) plain_first = element;
                    if (element > plain_last) plain_last = element;
                }
            }
            first = plain_first;
            last = plain_last;
        } else for (u32 i = 0; i < count; i++) {
            const u32 element = rsx_vertex_element_index(
                refs[i].vertex_id, refs[i].base_index,
                fetch->desc.frequency,
                (plan->divider_mask >> attr) & 1u);
            if (element < first) first = element;
            if (element > last) last = element;
        }

        const u64 prefix = rsx_dsp_resolve_vertex_offset(
            plan->base_offset, fetch->desc.offset);
        const u64 start = prefix + (u64)first * fetch->stride;
        const u64 end =
            prefix + (u64)last * fetch->stride + fetch->elem_size;
        if (start > UINT_MAX || end > 0x100000000ull || end <= start ||
            end - start > UINT_MAX)
            continue;

        const u8* base = plan->guest_ptr(
            plan->guest_user, fetch->desc.location, (u32)start,
            (u32)(end - start));
        if (!base)
            continue;
        fetch->stable_base = base;
        fetch->stable_first = first;
        fetch->stable_last = last;
    }
}

/* One attribute of one vertex, exactly as rsx_vertex_fetch_one does it. */
static int rsx_vertex_fetch_attr_value(
    const rsx_vertex_fetch_plan* plan, u32 attr,
    const rsx_vertex_ref* ref, float* value)
{
    const rsx_vertex_fetch_attr* fetch = &plan->attr[attr];
    if (!fetch->desc.type || !fetch->desc.size) {
        memcpy(value, fetch->default_value, 16);
        return 1;
    }

    const u32 element = rsx_vertex_element_index(
        ref->vertex_id, ref->base_index, fetch->desc.frequency,
        (plan->divider_mask >> attr) & 1u);
    const u8* source = NULL;
    if (fetch->stable_base &&
        element >= fetch->stable_first && element <= fetch->stable_last) {
        source = fetch->stable_base +
            (u64)(element - fetch->stable_first) * fetch->stride;
    } else if (plan->inline_data) {
        /* An inline draw has no vertex array to fall back to: the array
         * offsets still hold whatever the last ordinary draw bound, and
         * reading them would feed this draw another mesh's vertices.
         * An attribute the stream does not supply gets its default,
         * which is what the hardware feeds. */
        source = NULL;
    } else if (plan->guest_ptr) {
        const u32 offset =
            rsx_dsp_resolve_vertex_offset(plan->base_offset,
                                          fetch->desc.offset) +
            element * fetch->stride;
        source = plan->guest_ptr(
            plan->guest_user, fetch->desc.location, offset,
            fetch->elem_size);
    }

    if (source && rsx_vertex_decode_element(
            fetch->desc.type, fetch->desc.size, source, value))
        return 1;
    if (attr == 0)
        return 0;
    memcpy(value, fetch->default_value, 16);
    return 1;
}

int rsx_vertex_fetch_one(
    const rsx_vertex_fetch_plan* plan, const rsx_vertex_ref* ref, u8* out)
{
    if (!plan || !ref || (!out && plan->layout.stride))
        return 0;
    for (u32 slot = 0; slot < plan->layout.count; slot++)
        if (!rsx_vertex_fetch_attr_value(plan, plan->layout.attrs[slot], ref,
                                   (float*)(out + slot * 16u)))
            return 0;
    return 1;
}

/* The stable-range loop for one attribute and one vertex type: `decode`
 * reads `src` into `v` (pre-set to 0,0,0,1) for components c < n. Elements
 * outside the stable range take the one-vertex path. */
#define RSX_FETCH_LOOP(decode)                                                  \
    for (u32 i = 0; i < count; i++) {                                           \
        float* v = (float*)(out + (size_t)i * stride + slot * 16u);             \
        const u32 element = plain                                               \
            ? ((refs[i].vertex_id + refs[i].base_index) & 0x000FFFFFu)          \
            : rsx_vertex_element_index(refs[i].vertex_id, refs[i].base_index,   \
                                       fetch->desc.frequency, modulo);          \
        if (element - first > span) {                                          \
            if (!rsx_vertex_fetch_attr_value(plan, attr, &refs[i], v)) return 0;      \
            continue;                                                           \
        }                                                                       \
        const u8* src = base + (u64)(element - first) * fstride;                \
        v[0] = v[1] = v[2] = 0.0f; v[3] = 1.0f;                                 \
        for (u32 c = 0; c < n; c++) { decode; }                                 \
    }

int rsx_vertex_fetch_many(
    const rsx_vertex_fetch_plan* plan, const rsx_vertex_ref* refs, u32 count,
    u8* out)
{
    if (!plan || (!refs && count) || (!out && plan->layout.stride && count))
        return 0;
    const u32 stride = plan->layout.stride;
    for (u32 slot = 0; slot < plan->layout.count; slot++) {
        const u32 attr = plan->layout.attrs[slot];
        const rsx_vertex_fetch_attr* fetch = &plan->attr[attr];
        const u8* base = fetch->stable_base;
        const u32 type = fetch->desc.type;
        if (!type || !fetch->desc.size || !base ||
            type < RSX_VTX_TYPE_SNORM16 || type > RSX_VTX_TYPE_UINT8) {
            for (u32 i = 0; i < count; i++)
                if (!rsx_vertex_fetch_attr_value(plan, attr, &refs[i],
                        (float*)(out + (size_t)i * stride + slot * 16u)))
                    return 0;
            continue;
        }
        const u32 first = fetch->stable_first;
        const u32 span = fetch->stable_last - first;
        const u32 fstride = fetch->stride;
        const u32 modulo = (plan->divider_mask >> attr) & 1u;
        const int plain = rsx_vertex_freq_plain(fetch->desc.frequency, modulo);
        const u32 n = fetch->desc.size < 4 ? fetch->desc.size : 4;
        switch (type) {
        case RSX_VTX_TYPE_FLOAT:
            RSX_FETCH_LOOP(v[c] = rsx_compact_be_f32(src + c * 4))
            break;
        case RSX_VTX_TYPE_HALF:
            RSX_FETCH_LOOP(v[c] = rsx_compact_be_f16(src + c * 2))
            break;
        case RSX_VTX_TYPE_UNORM8:
            RSX_FETCH_LOOP(v[c] = src[c] / 255.0f)
            break;
        case RSX_VTX_TYPE_UINT8:
            RSX_FETCH_LOOP(v[c] = (float)src[c])
            break;
        case RSX_VTX_TYPE_SNORM16:
            RSX_FETCH_LOOP(v[c] = (s16)((src[c * 2] << 8) | src[c * 2 + 1]) / 32767.0f)
            break;
        case RSX_VTX_TYPE_SINT16:
            RSX_FETCH_LOOP(v[c] = (float)(s16)((src[c * 2] << 8) | src[c * 2 + 1]))
            break;
        default:   /* RSX_VTX_TYPE_CMP32: one packed word, whatever the size */
            for (u32 i = 0; i < count; i++)
                if (!rsx_vertex_fetch_attr_value(plan, attr, &refs[i],
                        (float*)(out + (size_t)i * stride + slot * 16u)))
                    return 0;
            break;
        }
    }
    return 1;
}
#undef RSX_FETCH_LOOP

static u64 rsx_vertex_remap_hash(u64 value)
{
    value ^= value >> 33;
    value *= 0xff51afd7ed558ccdull;
    value ^= value >> 33;
    value *= 0xc4ceb9fe1a85ec53ull;
    value ^= value >> 33;
    return value;
}

int rsx_vertex_remap_build(
    rsx_vertex_remap* remap, rsx_vertex_ref* refs, u32 count,
    u32* unique_count)
{
    if (!remap || (!refs && count) || !unique_count)
        return 0;
    *unique_count = count;
    if (!count)
        return 1;

    if (count > remap->occurrence_capacity) {
        u32* occurrences = (u32*)realloc(
            remap->occurrence_to_unique, (size_t)count * sizeof(u32));
        if (!occurrences)
            return 0;
        remap->occurrence_to_unique = occurrences;
        remap->occurrence_capacity = count;
    }

    /* Dense fast path. A mesh's indices nearly always cover a compact range
     * with one base index, and then a table indexed by (vertex - lowest) does
     * what the hash below does, in the same first-occurrence order, without a
     * hash and a probe per index. This function was a tenth of the RSX thread
     * in a scene of a few hundred indexed meshes. RSX_REMAP_DENSE=0 turns it
     * off. */
    {
        static int dense = -1;
        if (dense < 0) { const char* e = getenv("RSX_REMAP_DENSE"); dense = !(e && *e == '0'); }
        u32 lo = refs[0].vertex_id, hi = lo;
        const u32 base_index = refs[0].base_index;
        int one_base = 1;
        for (u32 i = 1; i < count; i++) {
            const u32 v = refs[i].vertex_id;
            lo = v < lo ? v : lo;
            hi = v > hi ? v : hi;
            one_base &= refs[i].base_index == base_index;
        }
        const u64 range = (u64)hi - lo + 1u;
        if (dense && one_base && range <= (u64)count * 4u + 1024u) {
            if (range > remap->direct_capacity) {
                u32* table = (u32*)realloc(remap->direct, (size_t)range * sizeof(u32));
                if (!table)
                    return 0;
                remap->direct = table;
                remap->direct_capacity = (u32)range;
            }
            memset(remap->direct, 0xFF, (size_t)range * sizeof(u32));
            u32 n = 0;
            for (u32 occurrence = 0; occurrence < count; occurrence++) {
                const rsx_vertex_ref ref = refs[occurrence];
                u32* at = &remap->direct[ref.vertex_id - lo];
                if (*at == 0xFFFFFFFFu) {
                    *at = n;
                    refs[n] = ref;      /* n <= occurrence: already read */
                    n++;
                }
                remap->occurrence_to_unique[occurrence] = *at;
            }
            *unique_count = n;
            return 1;
        }
    }

    if (count > UINT_MAX / 2u)
        return 0;
    u32 slot_capacity = 16u;
    const u32 required_slots = count * 2u;
    while (slot_capacity < required_slots) {
        if (slot_capacity > UINT_MAX / 2u)
            return 0;
        slot_capacity *= 2u;
    }
    if (slot_capacity > remap->slot_capacity) {
        rsx_vertex_remap_slot* slots = (rsx_vertex_remap_slot*)realloc(
            remap->slots, (size_t)slot_capacity * sizeof(*slots));
        if (!slots)
            return 0;
        remap->slots = slots;
        remap->slot_capacity = slot_capacity;
        memset(remap->slots, 0,
               (size_t)remap->slot_capacity * sizeof(*remap->slots));
        remap->generation = 1u;
    } else {
        remap->generation++;
        if (!remap->generation) {
            memset(remap->slots, 0,
                   (size_t)remap->slot_capacity * sizeof(*remap->slots));
            remap->generation = 1u;
        }
    }

    u32 unique = 0;
    const u32 mask = remap->slot_capacity - 1u;
    for (u32 occurrence = 0; occurrence < count; occurrence++) {
        const rsx_vertex_ref ref = refs[occurrence];
        const u64 key = ((u64)ref.base_index << 32) | ref.vertex_id;
        u32 slot_index = (u32)rsx_vertex_remap_hash(key) & mask;
        for (;;) {
            rsx_vertex_remap_slot* slot = &remap->slots[slot_index];
            if (slot->generation != remap->generation) {
                slot->key = key;
                slot->value = unique;
                slot->generation = remap->generation;
                refs[unique] = ref;
                remap->occurrence_to_unique[occurrence] = unique;
                unique++;
                break;
            }
            if (slot->key == key) {
                remap->occurrence_to_unique[occurrence] = slot->value;
                break;
            }
            slot_index = (slot_index + 1u) & mask;
        }
    }
    *unique_count = unique;
    return 1;
}

u32 rsx_vertex_remap_index(
    const rsx_vertex_remap* remap, u32 occurrence)
{
    return remap && occurrence < remap->occurrence_capacity
        ? remap->occurrence_to_unique[occurrence] : occurrence;
}

void rsx_vertex_remap_destroy(rsx_vertex_remap* remap)
{
    if (remap) { free(remap->direct); remap->direct = NULL; remap->direct_capacity = 0; }
    if (!remap)
        return;
    free(remap->occurrence_to_unique);
    free(remap->slots);
    memset(remap, 0, sizeof(*remap));
}

int rsx_vertex_topology_plan(
    u32 primitive, int refs_remapped, int* indexed)
{
    if (!indexed)
        return 0;
    switch (primitive) {
    case RSX_PRIMITIVE_TRIANGLES:
        *indexed = refs_remapped != 0;
        return 1;
    case RSX_PRIMITIVE_TRIANGLE_STRIP:
    case RSX_PRIMITIVE_TRIANGLE_FAN:
    case RSX_PRIMITIVE_QUADS:
        *indexed = 1;
        return 1;
    default:
        *indexed = 0;
        return 0;
    }
}

/* ---- packed indexed triangles ------------------------------------------ */

_Static_assert(sizeof(rsx_packed_attribute) == 32,
               "the shader reads eight words a slot");

int rsx_vertex_index_span_u16(
    const u8* source, u16* owned, u32 count, u32 base, u32 restart,
    u32* first, u32* last)
{
    u32 low = UINT16_MAX, high = 0, i = 0;
#if RSX_PACKED_INDEX_NEON
    /* Eight indices a step: byte-swap, store, and keep the running extrema
     * and a sticky sentinel match. The sentinel compare is meaningful only
     * for a restart value a 16-bit index can hold. */
    uint16x8_t minimum = vdupq_n_u16(UINT16_MAX);
    uint16x8_t maximum = vdupq_n_u16(0);
    uint16x8_t markers = vdupq_n_u16(0);
    const uint16x8_t sentinel = vdupq_n_u16((uint16_t)restart);
    for (; i + 8 <= count; i += 8) {
        const uint16x8_t values =
            vreinterpretq_u16_u8(vrev16q_u8(vld1q_u8(source + 2u * i)));
        vst1q_u16(owned + i, values);
        minimum = vminq_u16(minimum, values);
        maximum = vmaxq_u16(maximum, values);
        markers = vorrq_u16(markers, vceqq_u16(sentinel, values));
    }
    if (restart <= UINT16_MAX && vmaxvq_u16(markers)) return 0;
    low = vminvq_u16(minimum);
    high = vmaxvq_u16(maximum);
#endif
    for (; i < count; ++i) {
        const u16 index = (u16)((source[2u * i] << 8) | source[2u * i + 1u]);
        if (index == restart) return 0;
        owned[i] = index;
        if (index < low) low = index;
        if (index > high) high = index;
    }
    const u32 mask = 0xFFFFFu;
    base &= mask;
    *first = (base + low) & mask;
    *last = (base + high) & mask;
    if (*first > *last) {
        /* The based range crosses the 20-bit wrap, so the raw extrema no
         * longer bound it: take the exact ones from the owned copy. */
        *first = UINT32_MAX;
        *last = 0;
        for (i = 0; i < count; ++i) {
            const u32 effective = (base + owned[i]) & mask;
            if (effective < *first) *first = effective;
            if (effective > *last) *last = effective;
        }
    }
    return 1;
}

typedef struct { uintptr_t start, end; u32 slot; } rsx_packed_span;

int rsx_vertex_packed_triangles_prepare(
    rsx_packed_triangles* out, const rsx_vertex_fetch_plan* plan,
    const u8* index_run, u32 count, u32 base_index, u32 restart_index)
{
    if (!out || !plan || !index_run || count < 3 || count % 3 ||
        count > RSX_PACKED_VERTEX_LIMIT || plan->inline_data ||
        !plan->guest_ptr || !plan->layout.count)
        return 0;
    for (u32 slot = 0; slot < plan->layout.count; ++slot) {
        const u32 attr = plan->layout.attrs[slot];
        const rsx_dsp_vertex_attr* desc = &plan->attr[attr].desc;
        if (!desc->type || !desc->size) continue;
        if (desc->type > RSX_VTX_TYPE_UINT8 || desc->size > 4 ||
            !rsx_vertex_freq_plain(desc->frequency,
                                   (plan->divider_mask >> attr) & 1u))
            return 0;
    }

    u32 first, last;
    if (!rsx_vertex_index_span_u16(index_run, out->indices, count, base_index,
                                   restart_index, &first, &last) ||
        last - first >= RSX_PACKED_VERTEX_LIMIT)
        return 0;

    /* Map each attribute over the draw's whole vertex range, exactly as the
     * CPU fetch would for these references. */
    const rsx_vertex_ref bounds[2] = {{first, 0}, {last, 0}};
    rsx_vertex_fetch_plan stable = *plan;
    rsx_vertex_fetch_plan_prepare(&stable, bounds, 2);

    /* Own the mapped spans, merging the ones that overlap (interleaved
     * attributes share one array), in ascending address order. */
    rsx_packed_span spans[RSX_DSP_NUM_VERTEX_ATTR];
    u32 used = 0;
    for (u32 slot = 0; slot < stable.layout.count; ++slot) {
        rsx_packed_attribute* a = &out->attributes[slot];
        const rsx_vertex_fetch_attr* fetch = &stable.attr[stable.layout.attrs[slot]];
        memset(a, 0, sizeof(*a));
        memcpy(a->defaults, fetch->default_value, sizeof(a->defaults));
        if (!fetch->desc.type || !fetch->desc.size) continue;
        if (!fetch->stable_base || !fetch->elem_size ||
            fetch->stable_first != first || fetch->stable_last != last)
            return 0;
        const uintptr_t start = (uintptr_t)fetch->stable_base;
        const u64 size = (u64)(last - first) * fetch->stride + fetch->elem_size;
        if (size > RSX_PACKED_BYTES_LIMIT || size > UINTPTR_MAX - start)
            return 0;
        a->stride = fetch->stride;
        a->type = fetch->desc.type;
        a->size = fetch->desc.size;
        u32 pos = used++;
        for (; pos > 0 && spans[pos - 1].start > start; --pos)
            spans[pos] = spans[pos - 1];
        spans[pos] = (rsx_packed_span){start, start + (uintptr_t)size, slot};
    }
    u32 bytes = 0;
    for (u32 head = 0; head < used;) {
        const uintptr_t start = spans[head].start;
        uintptr_t end = spans[head].end;
        u32 next = head + 1;
        for (; next < used && spans[next].start < end; ++next)
            if (spans[next].end > end) end = spans[next].end;
        if (end - start > sizeof(out->data) - bytes) return 0;
        memcpy(out->data + bytes, (const void*)start, (size_t)(end - start));
        for (; head < next; ++head)
            out->attributes[spans[head].slot].offset =
                bytes + (u32)(spans[head].start - start);
        bytes += (u32)(end - start);
    }

    out->first = first;
    out->last = last;
    out->base_index = base_index & 0xFFFFFu;
    out->count = count;
    out->slots = stable.layout.count;
    out->bytes = bytes;
    return 1;
}

int rsx_vertex_packed_triangles_decode(
    const rsx_packed_triangles* packet, u32 stride, u8* vertices,
    u32* indices)
{
    const u32 elements = packet->last - packet->first + 1u;
    for (u32 element = 0; element < elements; ++element) {
        u8* vertex = vertices + (size_t)element * stride;
        for (u32 slot = 0; slot < packet->slots; ++slot) {
            const rsx_packed_attribute* a = &packet->attributes[slot];
            float value[4];
            if (!a->type)
                memcpy(value, a->defaults, sizeof(value));
            else if (!rsx_vertex_decode_element(
                         a->type, a->size,
                         packet->data + a->offset + (size_t)element * a->stride,
                         value))
                return 0;
            memcpy(vertex + 16u * slot, value, sizeof(value));
        }
    }
    for (u32 i = 0; i < packet->count; ++i)
        indices[i] = ((packet->indices[i] + packet->base_index) & 0xFFFFFu) -
                     packet->first;
    return 1;
}

u32 rsx_vertex_packed_triangles_size(
    const rsx_packed_triangles* packet, u32* index_offset)
{
    const u32 data = 16u + packet->slots * (u32)sizeof(rsx_packed_attribute);
    *index_offset = data + ((packet->bytes + 3u) & ~3u);
    return *index_offset + packet->count * (u32)sizeof(u16);
}

void rsx_vertex_packed_triangles_write(
    const rsx_packed_triangles* packet, u8* dst)
{
    u32 index_offset;
    rsx_vertex_packed_triangles_size(packet, &index_offset);
    const u32 data = 16u + packet->slots * (u32)sizeof(rsx_packed_attribute);
    const u32 header[4] = {packet->first, packet->base_index, data, packet->slots};
    memcpy(dst, header, sizeof(header));
    memcpy(dst + sizeof(header), packet->attributes,
           packet->slots * sizeof(rsx_packed_attribute));
    memcpy(dst + data, packet->data, packet->bytes);
    /* The padding keeps the last data byte inside an aligned shader load. */
    memset(dst + data + packet->bytes, 0, index_offset - data - packet->bytes);
    memcpy(dst + index_offset, packet->indices, packet->count * sizeof(u16));
}

/* Reads the block rsx_vertex_packed_triangles_write lays out. Each component
 * follows rsx_vertex_decode_element bit for bit (the guest programs compile
 * with safe math): big-endian bytes, half denormals to signed zero, NaN
 * payloads kept, and the same divisions. */
const char rsx_vertex_pull_hlsl[] =
    "ByteAddressBuffer rsx_pull_packet : register(t28);\n"
    "\n"
    "uint rsx_pull_byte(uint offset)\n"
    "{\n"
    "    return (rsx_pull_packet.Load(offset & ~3u) >> ((offset & 3u) * 8u)) & 255u;\n"
    "}\n"
    "\n"
    "uint rsx_pull_be16(uint offset)\n"
    "{\n"
    "    return (rsx_pull_byte(offset) << 8) | rsx_pull_byte(offset + 1u);\n"
    "}\n"
    "\n"
    "uint rsx_pull_be32(uint offset)\n"
    "{\n"
    "    return (rsx_pull_be16(offset) << 16) | rsx_pull_be16(offset + 2u);\n"
    "}\n"
    "\n"
    "uint rsx_pull_half(uint word)\n"
    "{\n"
    "    uint sign = (word >> 15) << 31;\n"
    "    uint exp = (word >> 10) & 31u;\n"
    "    uint fraction = (word & 1023u) << 13;\n"
    "    if (exp == 0u) return sign;\n"
    "    if (exp == 31u) return sign | 0x7f800000u | fraction;\n"
    "    return sign | ((exp + 112u) << 23) | fraction;\n"
    "}\n"
    "\n"
    "float4 rsx_pull_attribute(uint index, uint slot)\n"
    "{\n"
    "    uint desc = 16u + 32u * slot;\n"
    "    uint4 format = rsx_pull_packet.Load4(desc);\n"
    "    uint type = format.z;\n"
    "    if (type == 0u) return asfloat(rsx_pull_packet.Load4(desc + 16u));\n"
    "    uint4 header = rsx_pull_packet.Load4(0u);\n"
    "    uint element = ((index + header.y) & 0xfffffu) - header.x;\n"
    "    uint source = header.z + format.x + format.y * element;\n"
    "    if (type == 6u) {\n"
    "        uint word = rsx_pull_be32(source);\n"
    "        int x = asint(word << 21) >> 21;\n"
    "        int y = asint(word << 10) >> 21;\n"
    "        int z = asint(word) >> 22;\n"
    "        return float4(float(x) / 1023.0, float(y) / 1023.0, float(z) / 511.0, 1.0);\n"
    "    }\n"
    "    uint4 result = uint4(0u, 0u, 0u, 0x3f800000u);\n"
    "    for (uint c = 0u; c < format.w; c++) {\n"
    "        switch (type) {\n"
    "        case 1u: result[c] = asuint(float(asint(rsx_pull_be16(source + 2u * c) << 16) >> 16) / 32767.0); break;\n"
    "        case 2u: result[c] = rsx_pull_be32(source + 4u * c); break;\n"
    "        case 3u: result[c] = rsx_pull_half(rsx_pull_be16(source + 2u * c)); break;\n"
    "        case 4u: result[c] = asuint(float(rsx_pull_byte(source + c)) / 255.0); break;\n"
    "        case 5u: result[c] = asuint(float(asint(rsx_pull_be16(source + 2u * c) << 16) >> 16)); break;\n"
    "        case 7u: result[c] = asuint(float(rsx_pull_byte(source + c))); break;\n"
    "        }\n"
    "    }\n"
    "    return asfloat(result);\n"
    "}\n";

int rsx_vertex_pull_shader(
    const char* vs_hlsl, const rsx_vertex_layout_plan* layout,
    char* out, size_t capacity)
{
    static const char entry[] = "VSOutput main(VSInput input) {\n";
    const char* start = vs_hlsl ? strstr(vs_hlsl, entry) : NULL;
    /* A program with no attribute inputs reads SV_VertexID through VSInput,
     * which the rewritten entry point would leave unset. */
    if (!start || !layout || !layout->count ||
        layout->count > RSX_DSP_NUM_VERTEX_ATTR || strstr(vs_hlsl, "SV_VertexID"))
        return 0;
    int length = snprintf(out, capacity,
                          "%s\n%.*sVSOutput main(uint vertex_id:SV_VertexID) {\n"
                          "    VSInput input;\n",
                          rsx_vertex_pull_hlsl, (int)(start - vs_hlsl), vs_hlsl);
    if (length < 0 || (size_t)length >= capacity) return 0;
    size_t used = (size_t)length;
    for (u32 slot = 0; slot < layout->count; ++slot) {
        length = snprintf(out + used, capacity - used,
                          "    input.a%u = rsx_pull_attribute(vertex_id, %uu);\n",
                          layout->attrs[slot], slot);
        if (length < 0 || (size_t)length >= capacity - used) return 0;
        used += (size_t)length;
    }
    const char* body = start + sizeof(entry) - 1;
    const size_t bytes = strlen(body) + 1;
    if (bytes > capacity - used) return 0;
    memcpy(out + used, body, bytes);
    return 1;
}
