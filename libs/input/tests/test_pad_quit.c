/* cellPad's hold to quit (PS3_PAD_QUIT, default on): the PS (Guide) button
 * held 1 s, or START+SELECT held 2 s, on any controller asks for the host
 * quit once (ps3_host_quit, as the window's close button does); START and
 * SELECT of the chord are hidden from the guest from the moment both are down
 * until both are up; the PS button never reaches the guest; off changes
 * nothing; and PAD_SCRIPT / PAD_FILE pressing START+SELECT never quit.
 *
 *   clang -std=gnu17 -I include $(sdl2-config --cflags) -I "$(sdl2-config --prefix)/include" \
 *         libs/input/tests/test_pad_quit.c $(sdl2-config --libs) -lm -o /tmp/test_pad_quit && /tmp/test_pad_quit
 *
 * Host time is a fake clock (as in test_pad_file_reports.c); no controller is
 * opened: the test defines the SDL calls the backend makes over two fake pads.
 */
#include <time.h>
#include <stdint.h>
static uint64_t fake_ms = 1000;
static int fixture_clock_gettime(clockid_t id, struct timespec* ts)
{ (void)id; ts->tv_sec = (time_t)(fake_ms / 1000); ts->tv_nsec = (long)(fake_ms % 1000) * 1000000; return 0; }
#define clock_gettime fixture_clock_gettime
#include "../cellPad.c"
#undef clock_gettime
#include <assert.h>
#include <unistd.h>
#include <sys/mman.h>
uint8_t* vm_base;
uint32_t ppu_vm_size;
int g_resv_store_active;
uint32_t g_ww_lo, g_ww_hi;
void ppu_resv_break_store(uint64_t ea) { (void)ea; }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; }
int spu_coh_is_reserved(uint32_t a) { (void)a; return 0; }
u32 cellGcm_flip_request_count(void) { return 0; }
void spu_coh_notify_write(uint32_t a) { (void)a; }
void spu_lockline_lock(void) {}
void spu_lockline_unlock(void) {}

/* The host quit: what cellPad asked for. */
static int s_quits;
static char s_why[64];
void ps3_host_quit(const char* why) { s_quits++; snprintf(s_why, sizeof s_why, "%s", why ? why : ""); }

/* Two fake controllers, buttons as SDL_GameControllerButton bits. */
static uint32_t s_pad[2];
#define BIT(b) (1u << (b))
#define GUIDE  BIT(SDL_CONTROLLER_BUTTON_GUIDE)
#define START  BIT(SDL_CONTROLLER_BUTTON_START)
#define SELECT BIT(SDL_CONTROLLER_BUTTON_BACK)
#define CROSS  BIT(SDL_CONTROLLER_BUTTON_A)
int SDL_GameControllerRumble(SDL_GameController* gc, Uint16 lo, Uint16 hi, Uint32 ms) { (void)gc; (void)lo; (void)hi; (void)ms; return 0; }
void SDL_GameControllerUpdate(void) {}
SDL_bool SDL_IsGameController(int i) { (void)i; return SDL_FALSE; }
SDL_bool SDL_GameControllerGetAttached(SDL_GameController* gc) { (void)gc; return SDL_TRUE; }
Uint8 SDL_GameControllerGetButton(SDL_GameController* gc, SDL_GameControllerButton b)
{ return (Uint8)((*(uint32_t*)gc >> b) & 1u); }
Sint16 SDL_GameControllerGetAxis(SDL_GameController* gc, SDL_GameControllerAxis a) { (void)gc; (void)a; return 0; }
void SDL_GameControllerClose(SDL_GameController* gc) { (void)gc; }

static CellPadData* const guest = (CellPadData*)(uintptr_t)0x100;
/* One host report later, port `port`'s digital buttons as the guest reads
 * them ((DIGITAL2 << 8) | DIGITAL1), and the PS-button word must be empty. */
static u16 guest_buttons(unsigned port, unsigned ms)
{
    fake_ms += ms;
    assert(cellPadGetData(port, guest) == CELL_OK);
    assert((s32)vm_read32(0x100) > 0);
    for (unsigned w = 0; w < CELL_PAD_MAX_CODES; w++)
        if (w != CELL_PAD_BTN_OFFSET_DIGITAL1 && w != CELL_PAD_BTN_OFFSET_DIGITAL2 && w != 1 &&
            (w < CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X || w > CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y))
            assert(vm_read16(0x104 + w * 2) == 0);   /* nothing else, the PS button included */
    return (u16)(vm_read16(0x104 + CELL_PAD_BTN_OFFSET_DIGITAL1 * 2) |
                 vm_read16(0x104 + CELL_PAD_BTN_OFFSET_DIGITAL2 * 2) << 8);
}
/* Hold for `ms`, polling every 10 ms. */
static void hold(unsigned ms)
{
    for (unsigned t = 0; t < ms; t += 10) { fake_ms += 10; cellPad_poll(); }
}
static void reset(int quit_on)
{
    s_pad[0] = s_pad[1] = 0;
    fake_ms += 100; cellPad_poll();
    memset(s_quit_hold, 0, sizeof s_quit_hold);
    s_quit_sent = 0; s_quits = 0; s_why[0] = 0;
    s_cfg_quit = quit_on;
}

static const u16 CHORD = CELL_PAD_CTRL_START | CELL_PAD_CTRL_SELECT;

/* PAD_SCRIPT and PAD_FILE press START+SELECT for longer than 2 s: the guest
 * sees it, no quit. (Run first: both are read on the first cellPadGetData.) */
static void test_scripted(const char* pad_file)
{
    reset(1);
    FILE* f = fopen(pad_file, "w"); assert(f); fputs("0x0009 400\n", f); fclose(f);
    int seen = 0;
    for (int n = 0; n < 400; n++) {
        if (guest_buttons(0, 10) == CHORD) seen++;
    }
    assert(seen >= 300);           /* the injected chord did reach the guest for 4 s */
    assert(s_quits == 0);
    /* Also with a real controller in the port that presses nothing. */
    s_sdl_controllers[0] = (SDL_GameController*)&s_pad[0];
    f = fopen(pad_file, "w"); assert(f); fputs("0x0009 300\n", f); fclose(f);
    for (int n = 0; n < 300; n++) (void)guest_buttons(0, 10);
    assert(s_quits == 0);
    unlink(pad_file);
}

static void test_guide(void)
{
    reset(1);
    s_pad[0] = GUIDE;
    assert(guest_buttons(0, 10) == 0);   /* never reported; the hold starts here */
    hold(980);                           /* 980 ms */
    assert(s_quits == 0);
    hold(20);                            /* 1000 ms */
    assert(s_quits == 1 && !strcmp(s_why, "controller"));
    hold(3000);
    assert(s_quits == 1);                /* once */

    /* Let go before the second: the next hold starts again from zero. */
    reset(1);
    s_pad[0] = GUIDE; hold(900);
    s_pad[0] = 0;     hold(20);
    s_pad[0] = GUIDE; hold(900);
    assert(s_quits == 0);
    hold(110);
    assert(s_quits == 1);

    /* Port 1's controller counts too. */
    reset(1);
    s_pad[1] = GUIDE; hold(1010);
    assert(s_quits == 1);
}

static void test_chord(void)
{
    reset(1);
    s_pad[0] = START | SELECT;
    hold(1980);
    assert(s_quits == 0);
    hold(30);
    assert(s_quits == 1 && !strcmp(s_why, "controller"));

    /* START+SELECT plus other buttons still counts; the others still reach the guest. */
    reset(1);
    s_pad[0] = START | SELECT | CROSS;
    assert(guest_buttons(0, 10) == CELL_PAD_CTRL_CROSS);
    hold(2000);
    assert(s_quits == 1);

    /* Two pads each holding half of the chord is not a chord. */
    reset(1);
    s_pad[0] = START; s_pad[1] = SELECT;
    hold(3000);
    assert(s_quits == 0);
}

static void test_masking(void)
{
    /* START first: the game sees START (its pause), then nothing more of
     * either until both are up; no fresh START press when SELECT goes first. */
    reset(1);
    s_pad[0] = START;
    assert(guest_buttons(0, 10) == CELL_PAD_CTRL_START);
    assert(guest_buttons(0, 10) == CELL_PAD_CTRL_START);
    s_pad[0] = START | SELECT;
    assert(guest_buttons(0, 10) == 0);       /* masked from the first report with both */
    assert(guest_buttons(0, 500) == 0);
    s_pad[0] = START;                         /* gave up: SELECT up, START still down */
    assert(guest_buttons(0, 10) == 0);       /* still masked: no new START edge */
    assert(guest_buttons(0, 500) == 0);
    s_pad[0] = 0;
    assert(guest_buttons(0, 10) == 0);
    s_pad[0] = START;                         /* a new press is a press */
    assert(guest_buttons(0, 10) == CELL_PAD_CTRL_START);
    assert(s_quits == 0);

    /* SELECT first: the guest sees SELECT, then never START. */
    reset(1);
    s_pad[0] = SELECT;
    assert(guest_buttons(0, 10) == CELL_PAD_CTRL_SELECT);
    s_pad[0] = SELECT | START;
    assert(guest_buttons(0, 10) == 0);
    s_pad[0] = START;
    assert(guest_buttons(0, 10) == 0);
    s_pad[0] = 0;
    assert(guest_buttons(0, 10) == 0);
    s_pad[0] = SELECT;
    assert(guest_buttons(0, 10) == CELL_PAD_CTRL_SELECT);

    /* Both in the same report: the guest never sees either. Masked until
     * both are up, through the quit. */
    reset(1);
    s_pad[0] = START | SELECT;
    for (int n = 0; n < 250; n++) assert(guest_buttons(0, 10) == 0);
    assert(s_quits == 1);

    /* The PS button masks nothing: START held with it reaches the guest. */
    reset(1);
    s_pad[0] = GUIDE | START;
    assert(guest_buttons(0, 10) == CELL_PAD_CTRL_START);
}

static void test_off(void)
{
    reset(0);
    s_pad[0] = GUIDE;
    assert(guest_buttons(0, 10) == 0);       /* the PS button is still not reported */
    hold(5000);
    s_pad[0] = START;
    assert(guest_buttons(0, 10) == CELL_PAD_CTRL_START);
    s_pad[0] = START | SELECT;
    for (int n = 0; n < 50; n++) assert(guest_buttons(0, 100) == CHORD);   /* 5 s, unmasked */
    s_pad[0] = START;
    assert(guest_buttons(0, 10) == CELL_PAD_CTRL_START);
    assert(s_quits == 0);

    /* The switch itself. */
    static const struct { const char* v; int on; } t[] = {
        { NULL, 1 }, { "1", 1 }, { "on", 1 }, { "0", 0 }, { "off", 0 }, { "no", 0 }, { "false", 0 }, { "", 0 },
    };
    for (unsigned k = 0; k < sizeof t / sizeof t[0]; k++) {
        if (t[k].v) setenv("PS3_PAD_QUIT", t[k].v, 1); else unsetenv("PS3_PAD_QUIT");
        s_cfg_quit = -1;
        assert(pad_quit_enabled() == t[k].on);
    }
    unsetenv("PS3_PAD_QUIT");
}

int main(void)
{
    vm_base = mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    assert(vm_base != MAP_FAILED);
    ppu_vm_size = 0x10000;
    setenv("PAD_NO_KEYBOARD", "1", 1);
    char pad_file[] = "/tmp/test_pad_quit.XXXXXX";
    const int fd = mkstemp(pad_file); assert(fd >= 0); close(fd);
    setenv("PAD_FILE", pad_file, 1);
    setenv("PAD_SCRIPT", "0:0x0009,0.4:0x0009,0.8:0x0009,1.2:0x0009,1.6:0x0009,2.0:0x0009,2.4:0x0009", 1);
    s_pad_initialized = 1;
    s_max_connect = 4;
    test_scripted(pad_file);
    unsetenv("PAD_FILE"); unsetenv("PAD_SCRIPT");
    s_sdl_controllers[0] = (SDL_GameController*)&s_pad[0];
    s_sdl_controllers[1] = (SDL_GameController*)&s_pad[1];
    test_guide();
    test_chord();
    test_masking();
    test_off();
    printf("test_pad_quit: ok\n");
    return 0;
}
