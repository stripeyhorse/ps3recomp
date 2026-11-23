/* cellPad's vibration strength (PS3_PAD_RUMBLE, 0-100 %, default 100) scales
 * both motors, and 0 never starts one; and the SDL hints set before the
 * game-controller subsystem starts: rumble for DualShock 4 / DualSense over
 * Bluetooth and positional face buttons, each still overridden by the
 * same-named environment variable.
 *
 *   clang -std=gnu17 -I include $(sdl2-config --cflags) -I "$(sdl2-config --prefix)/include" \
 *         libs/input/tests/test_pad_rumble_strength.c $(sdl2-config --libs) -lm -o /tmp/test_pad_rumble_strength \
 *         && /tmp/test_pad_rumble_strength
 *
 * No controller is opened and SDL is never initialised: the test defines the
 * SDL calls the backend makes (they take precedence over libSDL2's), except
 * SDL_SetHint/SDL_GetHint, which are the real library's, so the precedence of
 * an environment variable over a hint is SDL's own.
 */
#include "../cellPad.c"
#include <assert.h>
#include <sys/mman.h>
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

/* The fake controller: every rumble call is recorded. */
static int s_fake_gc;
static int s_calls, s_started;   /* s_started: calls that asked a motor to run */
static Uint16 s_lo, s_hi;
static Uint32 s_ms;
int SDL_GameControllerRumble(SDL_GameController* gc, Uint16 lo, Uint16 hi, Uint32 ms)
{
    assert(gc == (SDL_GameController*)&s_fake_gc);
    s_calls++; s_lo = lo; s_hi = hi; s_ms = ms;
    if (lo || hi) s_started++;
    return 0;
}
void SDL_GameControllerUpdate(void) {}
SDL_bool SDL_IsGameController(int i) { (void)i; return SDL_FALSE; }
SDL_bool SDL_GameControllerGetAttached(SDL_GameController* gc) { return gc == (SDL_GameController*)&s_fake_gc; }
Uint8 SDL_GameControllerGetButton(SDL_GameController* gc, SDL_GameControllerButton b) { (void)gc; (void)b; return 0; }
Sint16 SDL_GameControllerGetAxis(SDL_GameController* gc, SDL_GameControllerAxis a) { (void)gc; (void)a; return 0; }
void SDL_GameControllerClose(SDL_GameController* gc) { (void)gc; }
int SDL_NumJoysticks(void) { return 0; }
int SDL_PeepEvents(SDL_Event* ev, int n, SDL_eventaction action, Uint32 lo, Uint32 hi)
{ (void)ev; (void)n; (void)action; (void)lo; (void)hi; return 0; }
/* The subsystem is not running yet; starting it records the hints SDL holds
 * at that moment, which is what the HIDAPI drivers read when they open. */
static int s_inits;
static char s_at_init[3][8];
Uint32 SDL_WasInit(Uint32 flags) { (void)flags; return 0; }
int SDL_InitSubSystem(Uint32 flags)
{
    static const char* const names[3] = {
        SDL_HINT_JOYSTICK_HIDAPI_PS4_RUMBLE, SDL_HINT_JOYSTICK_HIDAPI_PS5_RUMBLE,
        SDL_HINT_GAMECONTROLLER_USE_BUTTON_LABELS };
    assert(flags == SDL_INIT_GAMECONTROLLER);
    for (int k = 0; k < 3; k++) {
        const char* v = SDL_GetHint(names[k]);
        snprintf(s_at_init[k], sizeof s_at_init[k], "%s", v ? v : "-");
    }
    s_inits++;
    return 0;
}

static void set_act(u8 small, u8 large)
{
    CellPadActParam* p = (CellPadActParam*)(vm_base + 0x100);
    p->motor[CELL_PAD_ACTUATOR_PARAM_SMALL] = small;
    p->motor[CELL_PAD_ACTUATOR_PARAM_LARGE] = large;
    assert(cellPadSetActDirect(0, (CellPadActParam*)(uintptr_t)0x100) == CELL_OK);
}

static void strength(int pct)
{
    s_cfg_rumble = pct;
    memset(&s_rumble[0], 0, sizeof s_rumble[0]);
    s_calls = s_started = 0;
}

static void test_strength(void)
{
    s_sdl_controllers[0] = (SDL_GameController*)&s_fake_gc;

    /* 100: full, as before. */
    strength(100);
    set_act(1, 255);
    assert(s_calls == 1 && s_lo == 65535 && s_hi == 65535 && s_ms == PAD_RUMBLE_HOLD_MS);
    set_act(0, 128);
    assert(s_lo == 128 * 257 && s_hi == 0);

    /* 50: both motors at half. */
    strength(50);
    set_act(1, 255);
    assert(s_lo == 32767 && s_hi == 32767 && s_ms == PAD_RUMBLE_HOLD_MS);
    assert(s_rumble[0].lo == 32767 && s_rumble[0].hi == 32767);
    /* The renewal from the poll sends the scaled level, not the title's. */
    s_rumble[0].sent_ms -= PAD_RUMBLE_RENEW_MS;
    pad_poll_sdl2();
    assert(s_calls == 2 && s_lo == 32767 && s_hi == 32767);
    set_act(0, 100);
    assert(s_lo == 100 * 257 / 2 && s_hi == 0);

    /* 1: the smallest non-zero. */
    strength(1);
    set_act(1, 255);
    assert(s_lo == 655 && s_hi == 655);

    /* 0: whatever the title asks, no call starts a motor, and nothing is renewed. */
    strength(0);
    set_act(1, 255);
    set_act(0, 200);
    set_act(255, 1);
    assert(s_calls == 3 && s_started == 0 && s_ms == 0);
    assert(s_rumble[0].lo == 0 && s_rumble[0].hi == 0 && s_rumble[0].sent_ms == 0);
    pad_poll_sdl2();
    assert(s_calls == 3 && s_started == 0);
    s_sdl_controllers[0] = NULL;
}

static void test_env(void)
{
    static const struct { const char* v; int want; } t[] = {
        { NULL, 100 }, { "0", 0 }, { "40", 40 }, { "100", 100 },
        { "101", 100 }, { "-5", 100 }, { "off", 100 }, { "", 100 },
    };
    for (unsigned k = 0; k < sizeof t / sizeof t[0]; k++) {
        if (t[k].v) setenv("PS3_PAD_RUMBLE", t[k].v, 1); else unsetenv("PS3_PAD_RUMBLE");
        s_cfg_rumble = -1;
        if (pad_rumble_pct() != t[k].want) {
            printf("FAIL PS3_PAD_RUMBLE=%s -> %d, want %d\n", t[k].v ? t[k].v : "<unset>", pad_rumble_pct(), t[k].want);
            assert(0);
        }
    }
    unsetenv("PS3_PAD_RUMBLE");
}

static void test_hints(void)
{
    /* The player set one of them in the environment: that one keeps its value. */
    setenv("SDL_JOYSTICK_HIDAPI_PS5_RUMBLE", "0", 1);
    unsetenv("SDL_JOYSTICK_HIDAPI_PS4_RUMBLE");
    unsetenv("SDL_GAMECONTROLLER_USE_BUTTON_LABELS");
    pad_init_backend();
    assert(s_inits == 1);
    printf("hints at init: PS4_RUMBLE=%s PS5_RUMBLE=%s USE_BUTTON_LABELS=%s\n",
           s_at_init[0], s_at_init[1], s_at_init[2]);
    assert(!strcmp(s_at_init[0], "1"));    /* set by cellPad */
    assert(!strcmp(s_at_init[1], "0"));    /* the environment won */
    assert(!strcmp(s_at_init[2], "0"));    /* positional face buttons */
    assert(!strcmp(SDL_GetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS), "1"));
}

int main(void)
{
    vm_base = mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    assert(vm_base != MAP_FAILED);
    s_pad_initialized = 1;
    test_strength();
    test_env();
    test_hints();
    printf("test_pad_rumble_strength: ok\n");
    return 0;
}
