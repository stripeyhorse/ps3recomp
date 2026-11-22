/*
 * ps3recomp - compact CPU vertex-fetch planning for the live RSX renderer
 *
 * The legacy renderer expands every guest attribute into a 16 x float4
 * vertex.  This helper preserves the same conversion rules while allowing a
 * caller to fetch only the attributes statically read by the active vertex
 * program.  It is D3D12-independent so format/bounds behavior can be tested
 * without creating a device.
 */
#ifndef PS3RECOMP_RSX_VERTEX_COMPACT_H
#define PS3RECOMP_RSX_VERTEX_COMPACT_H

#include "rsx_dispatch.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef const u8* (*rsx_vertex_guest_ptr_fn)(
    void* user, u32 location, u32 offset, u32 min_bytes);

typedef struct rsx_vertex_layout_plan {
    u32 mask;
    u32 stride;
    u8 count;
    u8 attrs[RSX_DSP_NUM_VERTEX_ATTR];
    s8 slot_by_attr[RSX_DSP_NUM_VERTEX_ATTR];
} rsx_vertex_layout_plan;

typedef struct rsx_vertex_ref {
    u32 vertex_id;
    u32 base_index;
} rsx_vertex_ref;

typedef struct rsx_vertex_remap_slot {
    u64 key;
    u32 value;
    u32 generation;
} rsx_vertex_remap_slot;

typedef struct rsx_vertex_remap {
    u32* occurrence_to_unique;
    u32 occurrence_capacity;
    rsx_vertex_remap_slot* slots;
    u32 slot_capacity;
    u32 generation;
    u32* direct;            /* dense-range table (rsx_vertex_remap_build) */
    u32 direct_capacity;
} rsx_vertex_remap;

typedef struct rsx_vertex_fetch_attr {
    rsx_dsp_vertex_attr desc;
    u32 elem_size;
    u32 stride;
    float default_value[4];
    const u8* stable_base;
    u32 stable_first;
    u32 stable_last;
} rsx_vertex_fetch_attr;

typedef struct rsx_vertex_fetch_plan {
    rsx_vertex_layout_plan layout;
    rsx_vertex_fetch_attr attr[RSX_DSP_NUM_VERTEX_ATTR];
    u32 base_offset;
    u32 divider_mask;
    rsx_vertex_guest_ptr_fn guest_ptr;
    void* guest_user;
    /* INLINE_ARRAY source: when set, the vertices came down the FIFO and the
     * array offsets/locations mean nothing -- every attribute is read out of
     * this stream instead. See rsx_vertex_inline_layout. */
    const u8* inline_data;
    u32 inline_bytes;
    u32 inline_off[RSX_DSP_NUM_VERTEX_ATTR]; /* byte offset within a vertex */
    u32 inline_stride;                       /* decoded once, at _init      */
} rsx_vertex_fetch_plan;

/* Deterministic ascending-register layout; each selected register remains a
 * float4 and retains its original ATTR semantic index. */
void rsx_vertex_layout_plan_init(
    rsx_vertex_layout_plan* plan, u32 input_mask);

/* Byte offset of each attribute within one INLINE_ARRAY vertex, and the
 * vertex size as the return value.
 *
 * An inline stream carries no per-array offset registers -- the only thing
 * that says where an attribute sits is the VTXFMT declaration, and the
 * hardware rule is that the ENABLED attributes pack in ascending register
 * order. The declared stride wins over the packed size when the two differ
 * (a format may pad); the packed size is the fallback when no enabled array
 * declares a stride. Shared so the hoisted, legacy and reference fetch paths
 * cannot drift on it. */
u32 rsx_vertex_inline_layout(
    const rsx_dispatch* rsx, u32 offsets[RSX_DSP_NUM_VERTEX_ATTR]);

/* Point a plan at an INLINE_ARRAY stream instead of guest memory. Call
 * between _init and _prepare; a NULL `data` leaves the plan reading guest
 * memory, so an ordinary draw can call it unconditionally and pay nothing. */
void rsx_vertex_fetch_plan_set_inline(
    rsx_vertex_fetch_plan* plan, const rsx_dispatch* rsx,
    const u8* data, u32 bytes);

/* Renderer default for a disabled attribute. The legacy white ATTR3 fallback
 * applies only before a current color has been explicitly written. Shared by
 * the compact and legacy fetch paths. */
void rsx_vertex_fetch_default(const rsx_dispatch* rsx, u32 attr, float out[4]);

/* Decode all 16 descriptors/defaults once for a draw. */
void rsx_vertex_fetch_plan_init(
    rsx_vertex_fetch_plan* plan, const rsx_dispatch* rsx,
    const rsx_vertex_layout_plan* layout,
    rsx_vertex_guest_ptr_fn guest_ptr, void* guest_user);

/* Attempt one bounds-checked contiguous mapping per used attribute.  Any
 * attribute whose complete span is not contiguous safely falls back to the
 * legacy per-element resolver in rsx_vertex_fetch_one. */
void rsx_vertex_fetch_plan_prepare(
    rsx_vertex_fetch_plan* plan, const rsx_vertex_ref* refs, u32 count);

/* Fetch one compact vertex in layout order.  Disabled attributes use their
 * RSX current/default value.  Enabled non-position attributes retain the
 * legacy fallback-to-default behavior on an unsupported/invalid fetch; an
 * invalid enabled ATTR0 remains fatal. */
int rsx_vertex_fetch_one(
    const rsx_vertex_fetch_plan* plan, const rsx_vertex_ref* ref, u8* out);

/* rsx_vertex_fetch_one for refs[0..count) into consecutive compact vertices
 * at out (layout.stride apart), with the same values and the same failure:
 * 0 when some vertex's enabled ATTR0 cannot be fetched. Walks attribute by
 * attribute so each vertex type decodes in a loop of its own. */
int rsx_vertex_fetch_many(
    const rsx_vertex_fetch_plan* plan, const rsx_vertex_ref* refs, u32 count,
    u8* out);

/* Collapse repeated (vertex_id, base_index) references in first-use order.
 * refs is rewritten in place with the unique prefix.  The occurrence map
 * remains valid until the next call using the same remap object. */
int rsx_vertex_remap_build(
    rsx_vertex_remap* remap, rsx_vertex_ref* refs, u32 count,
    u32* unique_count);

u32 rsx_vertex_remap_index(
    const rsx_vertex_remap* remap, u32 occurrence);

void rsx_vertex_remap_destroy(rsx_vertex_remap* remap);

/* Select the live renderer's triangle topology path.  A remapped triangle
 * list must use an index buffer to reconstruct duplicate occurrences, just
 * like converted strips, fans, and quads.  Returns zero for unsupported
 * primitive classes. */
int rsx_vertex_topology_plan(
    u32 primitive, int refs_remapped, int* indexed);

/* Public for focused format/endian tests. */
int rsx_vertex_decode_element(
    u32 type, u32 size, const u8* source, float out[4]);

/* ---- packed indexed triangles ------------------------------------------ */
/*
 * An ordinary indexed triangle list (16-bit indices, per-vertex attributes,
 * no restart sentinel in the run) can reach the GPU undecoded: the guest's
 * big-endian attribute bytes and the indices in host order, decoded in the
 * vertex shader by the same rules as rsx_vertex_decode_element. The packet
 * owns copies of everything it needs, so it stays valid after the guest
 * rewrites its arrays. Anything else keeps the CPU fetch path.
 */
#define RSX_PACKED_VERTEX_LIMIT 4096u            /* indices, and vertex range */
#define RSX_PACKED_BYTES_LIMIT  (128u * 1024u)   /* owned attribute bytes     */

/* One layout slot. type 0 means the slot reads `defaults` (raw float bits).
 * The layout is the packet's wire format: eight 32-bit words a slot. */
typedef struct rsx_packed_attribute {
    u32 offset;         /* into data, of the element at `first`            */
    u32 stride;
    u32 type;           /* RSX_VTX_TYPE_*, 0 = default value              */
    u32 size;           /* components                                      */
    u32 defaults[4];
} rsx_packed_attribute;

typedef struct rsx_packed_triangles {
    u32 first, last;    /* effective (base-applied, 20-bit) vertex range   */
    u32 base_index;
    u32 count;          /* indices, a nonzero multiple of three            */
    u32 slots;          /* layout slots, in rsx_vertex_layout_plan order   */
    u32 bytes;          /* used bytes of data                              */
    rsx_packed_attribute attributes[RSX_DSP_NUM_VERTEX_ATTR];
    u16 indices[RSX_PACKED_VERTEX_LIMIT];   /* host order, base not applied */
    u8 data[RSX_PACKED_BYTES_LIMIT];
} rsx_packed_triangles;

/* Convert `count` big-endian 16-bit indices to host order in `owned` and
 * return the effective vertex range ((index + base) & 0xFFFFF) through
 * first/last. Returns 0, with `owned` partly written, when an index equals
 * `restart` (pass UINT32_MAX when restart is off). Vectorised on AArch64. */
int rsx_vertex_index_span_u16(
    const u8* source, u16* owned, u32 count, u32 base, u32 restart,
    u32* first, u32* last);

/* Fill `out` for an indexed triangle draw of `count` indices read from
 * `index_run`, against a plan from rsx_vertex_fetch_plan_init (not
 * prepared; this prepares its own copy). Returns 0 -- leaving the CPU path
 * to draw it -- for a restart sentinel, a count that is not whole
 * triangles or exceeds the limits, an inline stream, a non-plain
 * frequency, a format the shader decoder does not have, or an attribute
 * span that is not contiguously mapped. */
int rsx_vertex_packed_triangles_prepare(
    rsx_packed_triangles* out, const rsx_vertex_fetch_plan* plan,
    const u8* index_run, u32 count, u32 base_index, u32 restart_index);

/* CPU decode of a prepared packet, for a backend that declines it: the
 * vertex range first..last as compact vertices `stride` apart (16 bytes a
 * slot, rsx_vertex_layout_plan's layout), and `count` 32-bit indices into
 * them. Same values as rsx_vertex_fetch_many over the same references. */
int rsx_vertex_packed_triangles_decode(
    const rsx_packed_triangles* packet, u32 stride, u8* vertices,
    u32* indices);

/* The packet as the shader reads it, one contiguous block: a 16-byte header
 * {first, base_index, data offset, slots}, the slots' attributes, the data
 * padded to four bytes, then the 16-bit indices at *index_offset. Returns
 * the block's size; _write fills exactly that many bytes. */
u32 rsx_vertex_packed_triangles_size(
    const rsx_packed_triangles* packet, u32* index_offset);
void rsx_vertex_packed_triangles_write(
    const rsx_packed_triangles* packet, u8* dst);

/* The HLSL decoder for that block (a ByteAddressBuffer at t28), and the
 * guest vertex program rewritten to call it: `main` takes SV_VertexID,
 * the original index, and builds the VSInput the translated program reads
 * from the packet. Returns 0 if `vs_hlsl` lacks the expected entry point or
 * the result does not fit. */
extern const char rsx_vertex_pull_hlsl[];
int rsx_vertex_pull_shader(
    const char* vs_hlsl, const rsx_vertex_layout_plan* layout,
    char* out, size_t capacity);

#ifdef __cplusplus
}
#endif

#endif /* PS3RECOMP_RSX_VERTEX_COMPACT_H */
