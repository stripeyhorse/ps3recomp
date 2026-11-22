/* Link stubs for the fixtures that #include cellGcmSys.c on its own
 * (test_gcm_ref_queue.c, test_gcm_put_wake.c, test_gcm_vblank_count.c).
 *
 * cellGcmSys.c calls into the draw engine, the live-draw path, the RSX method
 * layer and the PPU runtime from code those fixtures never run. Clang's
 * dead-strip drops those references; GNU ld with --gc-sections keeps some of
 * them, so the fixtures did not link on Linux. Compiled on its own, without
 * the headers, so each stub only has to supply the symbol. Reaching one
 * means a fixture ran code it was not written for: abort.
 */
#include <stdint.h>
#include <stdlib.h>

#define GCM_STUB(name) void name(void); void name(void) { abort(); }
GCM_STUB(rsx_draw_engine_enabled)
GCM_STUB(rsx_draw_engine_guest_read)
GCM_STUB(rsx_draw_engine_guest_wrote)
GCM_STUB(rsx_draw_engine_method)
GCM_STUB(rsx_draw_engine_note_flip)
GCM_STUB(rsx_draw_engine_upload_run)
GCM_STUB(rsx_get_backend)
GCM_STUB(rsx_live_draw_blit)
GCM_STUB(rsx_live_draw_enabled)
GCM_STUB(rsx_live_draw_method)
GCM_STUB(rsx_live_draw_note_resolve)
GCM_STUB(rsx_live_draw_resolve_blit)
GCM_STUB(rsx_process_method)
GCM_STUB(rsx_process_vp_run)
GCM_STUB(rsx_raise_user_cmd)
GCM_STUB(rsx_state_init)
GCM_STUB(spu_taskset_round_busy)
GCM_STUB(ppu_dump_bctrl_ring)
GCM_STUB(ppu_dump_guest_stack)

uint32_t g_rsx_last_reference;
__thread void* g_active_ctx;   /* thread-local in the PPU runtime */
