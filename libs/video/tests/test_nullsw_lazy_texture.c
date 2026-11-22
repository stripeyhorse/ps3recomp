/* The headless software backend's texture binding (PS3_NULLSW_LAZY_TEX).
 *
 * With the default (lazy) a bind only records the texture, and the first draw
 * that samples it decodes it, at the texels the guest holds then. Binds that
 * no draw samples cost nothing: a Pain level binds hundreds of large textures
 * a frame and this backend draws nothing for indexed draws, so decoding at the
 * bind cost seconds a frame on the FIFO walker. With PS3_NULLSW_LAZY_TEX=0
 * every bind decodes, as before. Both modes must draw the same pixels when the
 * texture does not change between the bind and the draw.
 *
 * ctest runs it twice (nullsw_lazy_texture, nullsw_lazy_texture_off). By hand:
 *   gcc -std=gnu17 -I include -I libs/video -fsanitize=address,undefined
 *     libs/video/tests/test_nullsw_lazy_texture.c libs/video/rsx_vertex_fetch.c
 *     libs/video/rsx_texture_layout.c -lm -o /tmp/t && /tmp/t
 */
#include "../rsx_null_backend.c"
#include "../rsx_texture_formats.h"
#include "../rsx_vertex_formats.h"
#include <assert.h>

static u8 g_guest[1u << 20];
u8* vm_base = g_guest;
/* LOCAL and MAIN both resolve to the same offset in g_guest here. */
u32 cellGcmResolveLocated(int location, u32 offset) { (void)location; return offset; }
u32 cellGcmResolveOffset(u32 offset) { return offset; }
static rsx_backend* s_be;
void rsx_set_backend(rsx_backend* backend) { s_be = backend; }

enum { POS = 0x1000, UV = 0x2000, TEX_A = 0x10000, TEX_B = 0x20000 };
enum { C1 = 0x0020C040u, C2 = 0x00A01060u };

static void w32(u32 a, u32 v)
{
    g_guest[a] = (u8)(v >> 24); g_guest[a + 1] = (u8)(v >> 16);
    g_guest[a + 2] = (u8)(v >> 8); g_guest[a + 3] = (u8)v;
}
static void wf(u32 a, float f) { u32 v; memcpy(&v, &f, 4); w32(a, v); }

/* A 16x16 A8R8G8B8 texture, every texel one colour (layout-independent). */
static void fill_tex(u32 at, u32 rgb)
{
    for (u32 i = 0; i < 16u * 16u; i++) w32(at + i * 4u, 0xFF000000u | rgb);
}

static rsx_texture_state tex_at(u32 at)
{
    rsx_texture_state t;
    memset(&t, 0, sizeof t);
    t.control0 = 0x80000000u;                  /* unit enabled */
    t.offset = at;
    t.image_rect = (16u << 16) | 16u;
    t.format = ((u32)(RSX_TEXTURE_A8R8G8B8 | 0x20u) << 8) | 1u;   /* linear, local */
    return t;
}

static u32 centre(void)
{
    s_be->present(s_be->userdata, 0);
    return rsx_null_backend_readback_center() & 0x00FFFFFFu;
}

int main(void)
{
    const int lazy = nullsw_lazy_tex();
    assert(rsx_null_backend_init(32, 32, "lazy-tex") == 0 && s_be);

    /* One triangle over the whole target, texcoord (0.5, 0.5) everywhere. */
    const float pos[3][2] = {{-1.f, -1.f}, {3.f, -1.f}, {-1.f, 3.f}};
    for (u32 v = 0; v < 3; v++) {
        wf(POS + v * 16u + 0, pos[v][0]); wf(POS + v * 16u + 4, pos[v][1]);
        wf(POS + v * 16u + 8, 0.5f);      wf(POS + v * 16u + 12, 1.0f);
        wf(UV + v * 8u, 0.5f);            wf(UV + v * 8u + 4, 0.5f);
    }
    static rsx_state st;
    memset(&st, 0, sizeof st);
    st.vertex_attribs[0] = (rsx_vertex_attrib){ .offset = POS, .stride = 16, .size = 4,
                                                 .type = RSX_VERTEX_TYPE_F, .enabled = 1 };
    st.vertex_attribs[8] = (rsx_vertex_attrib){ .offset = UV, .stride = 8, .size = 2,
                                                 .type = RSX_VERTEX_TYPE_F, .enabled = 1 };
    st.vertex_data4f[3][0] = 1.0f;             /* untextured draws are red */
    st.vertex_data4f[3][3] = 1.0f;
    s_be->set_vertex_attribs(s_be->userdata, &st);
    fill_tex(TEX_A, C1);
    fill_tex(TEX_B, C2);
    const rsx_texture_state a = tex_at(TEX_A), b = tex_at(TEX_B);

    /* 1. Binds no draw samples: lazily, nothing is decoded. */
    for (int i = 0; i < 200; i++) {
        s_be->bind_texture(s_be->userdata, 0, (i & 1) ? &a : &b);
        s_be->bind_texture(s_be->userdata, 1, &a);   /* other units are not kept */
    }
    assert(rsx_null_backend_texture_decodes() == (lazy ? 0u : 200u));

    /* 2. A draw after the last bind (a) samples it, and decodes it once. */
    s_be->draw_arrays(s_be->userdata, RSX_PRIMITIVE_TRIANGLES, 0, 3);
    assert(centre() == C1);
    const u32 d2 = rsx_null_backend_texture_decodes();
    assert(d2 == (lazy ? 1u : 200u));

    /* 3. Another draw with the binding unchanged decodes nothing more. */
    s_be->draw_arrays(s_be->userdata, RSX_PRIMITIVE_TRIANGLES, 0, 3);
    assert(centre() == C1 && rsx_null_backend_texture_decodes() == d2);

    /* 4. A rebind decodes again at the next draw (or at the bind). */
    s_be->bind_texture(s_be->userdata, 0, &b);
    s_be->draw_arrays(s_be->userdata, RSX_PRIMITIVE_TRIANGLES, 0, 3);
    assert(centre() == C2 && rsx_null_backend_texture_decodes() == d2 + 1u);

    /* 5. Texels written between the bind and the draw: the lazy decode reads
     *    them at the draw, as the RSX does; the bind-time decode kept the old. */
    s_be->bind_texture(s_be->userdata, 0, &a);
    fill_tex(TEX_A, C2);
    s_be->draw_arrays(s_be->userdata, RSX_PRIMITIVE_TRIANGLES, 0, 3);
    assert(centre() == (lazy ? C2 : C1));
    fill_tex(TEX_A, C1);

    /* 6. A disabled unit unbinds, pending or not: the draw is untextured. */
    s_be->bind_texture(s_be->userdata, 0, &a);
    rsx_texture_state off = a; off.control0 = 0;
    s_be->bind_texture(s_be->userdata, 0, &off);
    const u32 d6 = rsx_null_backend_texture_decodes();
    s_be->draw_arrays(s_be->userdata, RSX_PRIMITIVE_TRIANGLES, 0, 3);
    assert(centre() == 0xFF0000u && rsx_null_backend_texture_decodes() == d6);

    /* 7. Indexed draws sample nothing here, so a pending bind stays pending. */
    s_be->bind_texture(s_be->userdata, 0, &b);
    const u32 d7 = rsx_null_backend_texture_decodes();
    s_be->draw_indexed(s_be->userdata, RSX_PRIMITIVE_TRIANGLES, 0, 3);
    assert(rsx_null_backend_texture_decodes() == d7);

    rsx_null_backend_shutdown();
    printf("[PASS] software backend texture binding, %s: %u decodes for 205 unit-0 binds\n",
           lazy ? "decoded at the first sampling draw (PS3_NULLSW_LAZY_TEX=1)"
                : "decoded at every bind (PS3_NULLSW_LAZY_TEX=0)",
           rsx_null_backend_texture_decodes());
    return 0;
}
