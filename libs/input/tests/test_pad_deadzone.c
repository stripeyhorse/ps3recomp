/* cellPad's stick dead zone on the SDL path (PS3_PAD_DEADZONE): radial over
 * both axes, rescaled so the output ramps from centre at its edge to full
 * deflection with no jump, the exact centre is 128 on both axes, and 0 is the
 * plain linear map. Then the same through the pad poll with a fake controller.
 *
 *   clang -std=gnu17 -I include $(sdl2-config --cflags) -I "$(sdl2-config --prefix)/include" \
 *         libs/input/tests/test_pad_deadzone.c $(sdl2-config --libs) -lm -o /tmp/test_pad_deadzone && /tmp/test_pad_deadzone
 *
 * No controller is opened: the test defines the SDL calls the poll makes
 * (they take precedence over libSDL2's).
 */
#include "../cellPad.c"
#include <assert.h>
uint8_t* vm_base;
uint32_t ppu_vm_size;
int g_resv_store_active;
uint32_t g_ww_lo, g_ww_hi;
void ppu_resv_break_store(uint64_t ea) { (void)ea; }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; }
int spu_coh_is_reserved(uint32_t a) { (void)a; return 0; }
u32 cellGcm_flip_request_count(void) { return 0; }
void ps3_host_quit(const char* why) { (void)why; }
void spu_coh_notify_write(uint32_t a) { (void)a; }
void spu_lockline_lock(void) {}
void spu_lockline_unlock(void) {}

static int s_fake_gc;
static Sint16 s_axis[SDL_CONTROLLER_AXIS_MAX];
int SDL_GameControllerRumble(SDL_GameController* gc, Uint16 lo, Uint16 hi, Uint32 ms) { (void)gc; (void)lo; (void)hi; (void)ms; return 0; }
void SDL_GameControllerUpdate(void) {}
SDL_bool SDL_IsGameController(int i) { (void)i; return SDL_FALSE; }
SDL_bool SDL_GameControllerGetAttached(SDL_GameController* gc) { return gc == (SDL_GameController*)&s_fake_gc; }
Uint8 SDL_GameControllerGetButton(SDL_GameController* gc, SDL_GameControllerButton b) { (void)gc; (void)b; return 0; }
Sint16 SDL_GameControllerGetAxis(SDL_GameController* gc, SDL_GameControllerAxis a) { (void)gc; return s_axis[a]; }
void SDL_GameControllerClose(SDL_GameController* gc) { (void)gc; }

static void stick(int x, int y, int dz, int want_x, int want_y)
{
    u8 ox, oy;
    pad_sdl_stick(x, y, dz, &ox, &oy);
    if (ox != want_x || oy != want_y) {
        printf("FAIL stick(%d, %d, dz %d) = (%u, %u), want (%d, %d)\n", x, y, dz, ox, oy, want_x, want_y);
        assert(0);
    }
}
static u8 stick_x(int x, int y, int dz)
{
    u8 ox, oy;
    pad_sdl_stick(x, y, dz, &ox, &oy);
    return ox;
}

static void test_table(void)
{
    /* The default, 10 %: the edge is at 3277 of 32768. */
    stick(0, 0, 10, 128, 128);                 /* centre: 128 on both axes (it was 127) */
    stick(3000, 0, 10, 128, 128);              /* just inside */
    stick(-3000, 0, 10, 128, 128);
    stick(0, 3200, 10, 128, 128);
    stick(2000, 2000, 10, 128, 128);           /* diagonal inside: 0.086 of full */
    stick(3400, 0, 10, 128, 128);              /* just outside: starts at centre, no jump */
    stick(-3400, 0, 10, 127, 128);             /* (one step below centre is 127) */
    stick(6554, 0, 10, 142, 128);              /* 20 % in -> 11 % out */
    /* Radial, not per axis: each axis is inside 10 %, the stick is not. */
    stick(3000, 3000, 10, 130, 130);
    stick(-3000, -3000, 10, 125, 125);
    /* ... and a small sideways part of a big push is kept, not snapped to the axis. */
    assert(stick_x(1638, -29491, 10) > 128);   /* 5 % right while 90 % up */
    stick(32767, 0, 10, 255, 128);             /* full */
    stick(-32768, 0, 10, 0, 128);
    stick(0, -32768, 10, 128, 0);
    stick(0, 32767, 10, 128, 255);
    stick(32767, 32767, 10, 255, 255);         /* full diagonal, square gate: both ends */
    stick(-32768, -32768, 10, 0, 0);
    stick(23170, 23170, 10, 218, 218);         /* full diagonal, round gate (0.707 each) */
    stick(0, 0, 50, 128, 128);                 /* the largest setting */
    stick(16000, 0, 50, 128, 128);
    stick(32767, 0, 50, 255, 128);
}

/* No jump anywhere along an axis: each step of the input moves the output by
 * at most one, and it is monotonic. */
static void test_ramp(int dz)
{
    int prev = stick_x(-32768, 0, dz);
    assert(prev == 0);
    for (int x = -32767; x <= 32767; x++) {
        const int v = stick_x(x, 0, dz);
        if (v < prev || v - prev > 1) {
            printf("FAIL ramp dz %d: x %d -> %d after %d\n", dz, x, v, prev);
            assert(0);
        }
        prev = v;
    }
    assert(prev == 255);
}

/* 0 is the plain linear map, (raw + 32768) / 256, on each axis alone. */
static void test_zero(void)
{
    for (int x = -32768; x <= 32767; x++) {
        u8 ox, oy;
        pad_sdl_stick(x, x, 0, &ox, &oy);
        assert(ox == (x + 32768) >> 8 && oy == ox);
        pad_sdl_stick(x, -x - 1, 0, &ox, &oy);
        assert(ox == (x + 32768) >> 8 && oy == ((-x - 1) + 32768) >> 8);
    }
    stick(0, 0, 0, 128, 128);
    stick(-1, 1, 0, 127, 128);
    stick(1000, 0, 0, 131, 128);
}

static void test_env(void)
{
    static const struct { const char* v; int want; } t[] = {
        { NULL, 10 }, { "0", 0 }, { "25", 25 }, { "50", 50 },
        { "51", 10 }, { "-1", 10 }, { "abc", 10 }, { "10%", 10 }, { "", 10 }, { "7.5", 10 },
    };
    for (unsigned k = 0; k < sizeof t / sizeof t[0]; k++) {
        if (t[k].v) setenv("PS3_PAD_DEADZONE", t[k].v, 1); else unsetenv("PS3_PAD_DEADZONE");
        s_cfg_deadzone = -1;
        if (pad_deadzone_pct() != t[k].want) {
            printf("FAIL PS3_PAD_DEADZONE=%s -> %d, want %d\n", t[k].v ? t[k].v : "<unset>", pad_deadzone_pct(), t[k].want);
            assert(0);
        }
    }
    unsetenv("PS3_PAD_DEADZONE");
}

/* Through the poll: both sticks of a fake controller. */
static void test_poll(void)
{
    s_sdl_controllers[0] = (SDL_GameController*)&s_fake_gc;
    setenv("PS3_PAD_DEADZONE", "20", 1);
    s_cfg_deadzone = -1;
    s_axis[SDL_CONTROLLER_AXIS_LEFTX] = 4000;     /* drift, inside 20 % */
    s_axis[SDL_CONTROLLER_AXIS_LEFTY] = -3000;
    s_axis[SDL_CONTROLLER_AXIS_RIGHTX] = 32767;   /* full right */
    s_axis[SDL_CONTROLLER_AXIS_RIGHTY] = 0;
    pad_poll_sdl2();
    const PadHostState* hs = &s_host_state[0];
    assert(hs->connected);
    assert(hs->analog_lx == 128 && hs->analog_ly == 128);
    assert(hs->analog_rx == 255 && hs->analog_ry == 128);
    /* At 0 the same drift comes through. */
    s_cfg_deadzone = 0;
    pad_poll_sdl2();
    assert(hs->analog_lx == (4000 + 32768) >> 8 && hs->analog_ly == (-3000 + 32768) >> 8);
    /* Centred stick at the default: exact centre. */
    s_cfg_deadzone = -1;
    unsetenv("PS3_PAD_DEADZONE");
    memset(s_axis, 0, sizeof s_axis);
    pad_poll_sdl2();
    assert(hs->analog_lx == 128 && hs->analog_ly == 128 && hs->analog_rx == 128 && hs->analog_ry == 128);
}

int main(void)
{
    test_table();
    test_ramp(0);
    test_ramp(10);
    test_ramp(50);
    test_zero();
    test_env();
    test_poll();
    printf("test_pad_deadzone: ok\n");
    return 0;
}
