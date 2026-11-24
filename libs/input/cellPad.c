/*
 * ps3recomp - cellPad HLE implementation
 *
 * Reads real gamepad input from the host and translates to PS3 pad format.
 *
 * Backend selection:
 *   - Windows default: XInput, loaded at run time from System32 (XInput1_4.dll,
 *     XInput1_3.dll, XInput9_1_0.dll); no XInput DLL means no pads
 *   - Everywhere else / if PS3RECOMP_PAD_USE_SDL2 is defined: SDL2 GameController
 *
 * Define PS3RECOMP_PAD_USE_SDL2 to force SDL2 backend on Windows.
 */

#include <time.h>
#include "cellPad.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../runtime/platform/win32_compat.h"   /* GetTickCount64 on POSIX */
#include "../../runtime/ppu/ppu_memory.h"   /* GUEST_PTR, vm_write*: guest EA -> host */
#include "ps3emu/log_first.h"

/* PAD_CLOCK=flips: PAD_SCRIPT and PAD_STICK times count guest flips instead of
 * wall-clock seconds. A loaded machine runs a title at a third of its usual
 * frame rate, and a wall-clock schedule then presses START on the logos; a
 * flip schedule replays the same screens however slow the run is. */
extern u32 cellGcm_flip_request_count(void);
static double pad_clock(unsigned long long t0_ms)
{
    static int flips = -1;
    if (flips < 0) { const char* e = getenv("PAD_CLOCK"); flips = e && !strcmp(e, "flips"); }
    return flips ? (double)cellGcm_flip_request_count()
                 : (double)(GetTickCount64() - t0_ms) / 1000.0;
}

/* ---------------------------------------------------------------------------
 * Backend selection
 * -----------------------------------------------------------------------*/

#if defined(PS3RECOMP_PAD_USE_SDL2)
  #define PAD_BACKEND_SDL2  1
  #define PAD_BACKEND_XINPUT 0
#elif defined(_WIN32)
  #define PAD_BACKEND_SDL2  0
  #define PAD_BACKEND_XINPUT 1
#else
  #define PAD_BACKEND_SDL2  1
  #define PAD_BACKEND_XINPUT 0
#endif

#if PAD_BACKEND_XINPUT
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>
  /* Types only. The exe must not import an XInput DLL: MinGW's libxinput is
   * xinput1_3.dll, which only the DirectX June 2010 redistributable installs,
   * so on a stock Windows 10/11 the exe did not start (0xC0000135). The DLL is
   * loaded at run time instead, see pad_xinput_load(). */
  #include <xinput.h>
#endif

#if PAD_BACKEND_SDL2
  #include <SDL2/SDL.h>
#endif

/* ---------------------------------------------------------------------------
 * Internal state
 * -----------------------------------------------------------------------*/

#define PAD_MAX_HOST_PORTS  4  /* XInput supports max 4; SDL may support more */

typedef struct {
    int  connected;
    u16  buttons;           /* CELL_PAD_CTRL_* bitmask */
    u8   analog_lx;         /* 0-255, center=128 */
    u8   analog_ly;
    u8   analog_rx;
    u8   analog_ry;
    u8   trigger_l2;        /* 0-255 */
    u8   trigger_r2;        /* 0-255 */
    /* Pressure-sensitive face buttons (0-255) */
    u8   press_right;
    u8   press_left;
    u8   press_up;
    u8   press_down;
    u8   press_triangle;
    u8   press_circle;
    u8   press_cross;
    u8   press_square;
    u8   press_l1;
    u8   press_r1;
    /* Motion in DualShock 3 counts (accel X, Y, Z, gyro); only used while
     * motion_live, otherwise the guest gets the at-rest values. */
    u16  sensor[4];
    u8   motion_live;
} PadHostState;
/* A DualShock 3 lying still: 512 is 0 g, so Y = 399 is gravity (113 counts
 * per g); the gyro is still. */
static const u16 s_sensor_rest[4] = { 512, 399, 512, 512 };

static int           s_pad_initialized = 0;
static u32           s_max_connect = 0;
static u32           s_port_setting[CELL_PAD_MAX_PORT_NUM];
static PadHostState  s_host_state[PAD_MAX_HOST_PORTS];
/* Per-port "a report has arrived since your last read" flag; see cellPadGetData. */
static int s_data_fresh[PAD_MAX_HOST_PORTS];

#if PAD_BACKEND_SDL2
/* PS3 port i's controller, by SDL instance id (-1 = none); the port keeps the
 * GUID of the last one it had so that pad gets the same port back. */
static SDL_GameController* s_sdl_controllers[PAD_MAX_HOST_PORTS];
static SDL_JoystickID      s_sdl_ids[PAD_MAX_HOST_PORTS] = { -1, -1, -1, -1 };
static SDL_JoystickGUID    s_sdl_last_guid[PAD_MAX_HOST_PORTS];
static int                 s_sdl_had[PAD_MAX_HOST_PORTS];
static unsigned long long  s_sdl_scan_ms;   /* last device-list scan (pad_now_ms) */
static int s_sdl_inited = 0;
#endif

/* A port with no device behind it: nothing pressed and both sticks CENTRED.
 * Zero is not neutral for a stick -- 0 is full deflection up / left. Port 0 is
 * served even with no device (see cellPadGetData), and with the SDL2 backend
 * and no controller plugged in its state was the zeroed array: every title
 * read both sticks pushed to the top-left corner. Twisted Metal's car drove
 * itself in circles and its chase camera pitched up at the sky, so the
 * player's own vehicle left the screen within seconds of a match starting.
 * A pad unplugged mid-game kept its last report the same way. */
static void pad_host_release(PadHostState* hs)
{
    memset(hs, 0, sizeof(*hs));
    hs->analog_lx = hs->analog_ly = 128;
    hs->analog_rx = hs->analog_ry = 128;
}

/* One poll at a time: the guest polls from several threads, and a poll that
 * opens or closes a controller must not race another one using it.
 * cellPadSetActDirect takes it too (it talks to the controller). */
#ifdef _WIN32
  #define pad_yield() Sleep(0)
#else
  #include <sched.h>
  #define pad_yield() sched_yield()
#endif
static volatile char s_poll_lock;
static int  pad_lock_try(void) { return !__atomic_test_and_set(&s_poll_lock, __ATOMIC_ACQUIRE); }
static void pad_lock(void)     { while (!pad_lock_try()) pad_yield(); }
static void pad_unlock(void)   { __atomic_clear(&s_poll_lock, __ATOMIC_RELEASE); }

/* CELL_PAD_STATUS_ASSIGN_CHANGES: a port whose reported connection changed
 * since the title last asked (cellPadGetInfo/GetInfo2/PeriphGetInfo), as
 * libpad reports a controller plugged in or taken away. */
static int s_port_reported[PAD_MAX_HOST_PORTS];
static int s_port_changed[PAD_MAX_HOST_PORTS];
static void pad_track_assign(void)
{
    for (int i = 0; i < PAD_MAX_HOST_PORTS; i++) {
        const int on = s_host_state[i].connected || i == 0;   /* port 0: the virtual pad */
        if (on != s_port_reported[i]) { s_port_reported[i] = on; s_port_changed[i] = 1; }
    }
}
static u32 pad_status_take(u32 port)
{
    if (port >= PAD_MAX_HOST_PORTS) return 0;
    const int ch = __atomic_exchange_n(&s_port_changed[port], 0, __ATOMIC_RELAXED);
    return ch ? CELL_PAD_STATUS_ASSIGN_CHANGES : 0;
}

/* ---------------------------------------------------------------------------
 * Rumble (cellPadSetActDirect)
 *
 * A DualShock 3 has two motors: the small one (motor[0]) is on or off, the
 * large one (motor[1]) takes 0-255. A title sets them and they stay that way
 * until it sets them again; some titles only call when the value changes.
 * SDL's rumble runs for a duration instead, so the last request per port is
 * kept and renewed from the pad poll while it is non-zero. If the title stops
 * polling (a hang, a closed window), the motors stop within
 * PAD_RUMBLE_HOLD_MS. Large -> the host's low-frequency motor, small -> the
 * high-frequency one, which is how DualShock 4, DualSense and Xbox pads are
 * laid out too.
 * -------------------------------------------------------------------------*/

#define PAD_RUMBLE_HOLD_MS     1000   /* one SDL request lasts this long ... */
#define PAD_RUMBLE_RENEW_MS     400   /* ... and is renewed this often */

typedef struct {
    u16 lo, hi;                 /* what the host motors were last asked for */
    unsigned long long sent_ms; /* when (pad_now_ms), 0 = nothing to renew */
} PadRumble;
static PadRumble s_rumble[PAD_MAX_HOST_PORTS];

static unsigned long long pad_now_ms(void);

/* The two host motor levels (0-65535) for a PS3 actuator request. */
static void pad_rumble_levels(u8 small_on, u8 large, u16* lo, u16* hi)
{
    *lo = (u16)(large * 257u);
    *hi = small_on ? 0xFFFFu : 0u;
}

/* Player settings, read once from the environment (the port's settings layer
 * publishes them): PS3_PAD_DEADZONE, stick dead zone in percent, 0-50,
 * default 10 (SDL backend); PS3_PAD_RUMBLE, motor strength in percent, 0-100,
 * default 100; PS3_PAD_QUIT, hold-to-quit gesture, default on, "0", "off",
 * "no", "false" or empty turn it off (SDL backend). A value that is not a
 * whole number in range is ignored with one log line and the default used. */
static int pad_env_percent(const char* name, int lo, int hi, int def)
{
    const char* e = getenv(name);
    if (!e) return def;
    char* end = NULL;
    const long v = strtol(e, &end, 10);
    if (end == e || *end || v < lo || v > hi) {
        fprintf(stderr, "[cellPad] %s=%s: not a whole number %d-%d, using %d\n", name, e, lo, hi, def);
        return def;
    }
    return (int)v;
}
static int s_cfg_rumble = -1;
static int pad_rumble_pct(void)
{
    if (s_cfg_rumble < 0) s_cfg_rumble = pad_env_percent("PS3_PAD_RUMBLE", 0, 100, 100);
    return s_cfg_rumble;
}
#if PAD_BACKEND_SDL2
static int s_cfg_deadzone = -1, s_cfg_quit = -1, s_cfg_motion = -1, s_cfg_shake = -2;
static int pad_deadzone_pct(void)
{
    if (s_cfg_deadzone < 0) s_cfg_deadzone = pad_env_percent("PS3_PAD_DEADZONE", 0, 50, 10);
    return s_cfg_deadzone;
}
static int pad_quit_enabled(void)
{
    if (s_cfg_quit < 0) {
        const char* e = getenv("PS3_PAD_QUIT");
        s_cfg_quit = !(e && (!*e || !strcmp(e, "0") || !strcmp(e, "off") ||
                             !strcmp(e, "no") || !strcmp(e, "false")));
    }
    return s_cfg_quit;
}
/* PS3_PAD_MOTION: the controller's own accelerometer and gyroscope, default
 * on; the same words as PS3_PAD_QUIT turn it off. */
static int pad_motion_enabled(void)
{
    if (s_cfg_motion < 0) {
        const char* e = getenv("PS3_PAD_MOTION");
        s_cfg_motion = !(e && (!*e || !strcmp(e, "0") || !strcmp(e, "off") ||
                               !strcmp(e, "no") || !strcmp(e, "false")));
    }
    return s_cfg_motion;
}
/* PS3_PAD_SHAKE_BUTTON: off (default), l3, r3 or touchpad. The SDL button,
 * or -1 for none. */
static int pad_shake_button(void)
{
    if (s_cfg_shake < -1) {
        const char* e = getenv("PS3_PAD_SHAKE_BUTTON");
        s_cfg_shake = -1;
        if (!e || !strcmp(e, "off")) {}
        else if (!strcmp(e, "l3"))       s_cfg_shake = SDL_CONTROLLER_BUTTON_LEFTSTICK;
        else if (!strcmp(e, "r3"))       s_cfg_shake = SDL_CONTROLLER_BUTTON_RIGHTSTICK;
        else if (!strcmp(e, "touchpad")) s_cfg_shake = SDL_CONTROLLER_BUTTON_TOUCHPAD;
        else fprintf(stderr, "[cellPad] PS3_PAD_SHAKE_BUTTON=%s: not off, l3, r3 or touchpad, using off\n", e);
    }
    return s_cfg_shake;
}
#endif
/* A motor level scaled by PS3_PAD_RUMBLE: 0 never starts a motor. */
static u16 pad_rumble_scale(u16 level)
{
    return (u16)((u32)level * (u32)pad_rumble_pct() / 100u);
}

/* Keyboard fallback for port 0.
 *
 * On a machine with no controller plugged in a title gets a pad that is reported
 * present and never presses anything -- it can be watched but not played. That is
 * what The Simpsons Arcade Game hit: it reached its attract loop and no input
 * existed to start a game.
 *
 * Only fills in for port 0, only when the backend found no controller there, so a
 * real controller always wins and nothing changes for a port that has one. Keys
 * are read only while a window of THIS process is in the foreground, so typing in
 * another application does not drive the game. Set PAD_NO_KEYBOARD=1 to
 * disable it entirely.
 *
 * Arrows = d-pad, Z/X/A/S = cross/circle/square/triangle, Q/W = L1/R1,
 * 1/2 = L2/R2, Enter = START, Tab = SELECT. The left stick mirrors the d-pad
 * so a title that reads the stick instead is playable too.
 *
 * Where the keys come from: GetAsyncKeyState with the XInput backend; on macOS
 * CGEventSourceKeyState, which needs no event tap and no window; elsewhere SDL's
 * keyboard state, which is kept current by whoever pumps SDL's events (in
 * practice the window of the video backend). PS3RECOMP_PAD_KEYBOARD_SDL selects
 * the SDL source on macOS too, which is how the keyboard test is built. */
#if PAD_BACKEND_XINPUT
static int pad_host_window_focused(void)
{
    HWND fg = GetForegroundWindow();
    if (!fg) return 0;
    DWORD pid = 0;
    GetWindowThreadProcessId(fg, &pid);
    return pid == GetCurrentProcessId();
}
static int pad_key_down(int key) { return (GetAsyncKeyState(key) & 0x8000) != 0; }
#define PADKEY_UP VK_UP
#define PADKEY_DOWN VK_DOWN
#define PADKEY_LEFT VK_LEFT
#define PADKEY_RIGHT VK_RIGHT
#define PADKEY_Z 'Z'
#define PADKEY_X 'X'
#define PADKEY_A 'A'
#define PADKEY_S 'S'
#define PADKEY_Q 'Q'
#define PADKEY_W 'W'
#define PADKEY_1 '1'
#define PADKEY_2 '2'
#define PADKEY_RETURN VK_RETURN
#define PADKEY_TAB VK_TAB
#elif defined(__APPLE__) && !defined(PS3RECOMP_PAD_KEYBOARD_SDL)
/* macOS: virtual key codes (Carbon HIToolbox/Events.h kVK_*), polled with CGEventSourceKeyState,
 * which needs no event tap and no window. "Focused" is NSApplication.isActive of this process,
 * queried through the Objective-C runtime so this file stays plain C. */
/* The system headers are not included on purpose: ApplicationServices / objc.h clash with this
 * project's own `bool` and GetCurrentThread/GetCurrentProcess names, so the three entry points
 * are declared by hand (CGKeyCode is uint16_t, kCGEventSourceStateCombinedSessionState is 0). */
extern unsigned char CGEventSourceKeyState(int stateID, unsigned short key);
extern void* objc_getClass(const char* name);
extern void* sel_registerName(const char* name);
extern void objc_msgSend(void);
static int pad_host_window_focused(void)
{
    void* cls = objc_getClass("NSApplication");
    if (!cls) return 0;
    void* app = ((void* (*)(void*, void*))objc_msgSend)(cls, sel_registerName("sharedApplication"));
    return app && ((signed char (*)(void*, void*))objc_msgSend)(app, sel_registerName("isActive"));
}
static int pad_key_down(int key)
{
    return CGEventSourceKeyState(0, (unsigned short)key) != 0;
}
#define PADKEY_UP 126
#define PADKEY_DOWN 125
#define PADKEY_LEFT 123
#define PADKEY_RIGHT 124
#define PADKEY_Z 6
#define PADKEY_X 7
#define PADKEY_A 0
#define PADKEY_S 1
#define PADKEY_Q 12
#define PADKEY_W 13
#define PADKEY_1 18
#define PADKEY_2 19
#define PADKEY_RETURN 36
#define PADKEY_TAB 48
#elif PAD_BACKEND_SDL2
/* Everywhere else the keyboard is SDL's: scancodes are physical keys, like the macOS codes above. The state is kept
 * current by whoever pumps SDL's events (the window of the RSX backend); without a video subsystem nothing is ever
 * pressed. "Focused" is that some window of this process has the keyboard focus. */
static int pad_host_window_focused(void)
{
    return SDL_GetKeyboardFocus() != NULL;
}
static int pad_key_down(int key)
{
    return SDL_GetKeyboardState(NULL)[key] != 0;
}
#define PADKEY_UP SDL_SCANCODE_UP
#define PADKEY_DOWN SDL_SCANCODE_DOWN
#define PADKEY_LEFT SDL_SCANCODE_LEFT
#define PADKEY_RIGHT SDL_SCANCODE_RIGHT
#define PADKEY_Z SDL_SCANCODE_Z
#define PADKEY_X SDL_SCANCODE_X
#define PADKEY_A SDL_SCANCODE_A
#define PADKEY_S SDL_SCANCODE_S
#define PADKEY_Q SDL_SCANCODE_Q
#define PADKEY_W SDL_SCANCODE_W
#define PADKEY_1 SDL_SCANCODE_1
#define PADKEY_2 SDL_SCANCODE_2
#define PADKEY_RETURN SDL_SCANCODE_RETURN
#define PADKEY_TAB SDL_SCANCODE_TAB
#endif

static void pad_poll_keyboard(void)
{
    static int off = -1;
    if (off < 0) off = getenv("PAD_NO_KEYBOARD") ? 1 : 0;
    if (off) return;

    PadHostState* hs = &s_host_state[0];
    if (!pad_host_window_focused()) {
        /* Release everything on focus loss, or a key held while alt-tabbing
         * would stay down forever. */
        hs->buttons = 0;
        hs->analog_lx = hs->analog_ly = 128;
        hs->analog_rx = hs->analog_ry = 128;
        hs->connected = 1;
        return;
    }

    static const struct { int vk; u16 btn; } map[] = {
        { PADKEY_UP,     CELL_PAD_CTRL_UP },      { PADKEY_DOWN,  CELL_PAD_CTRL_DOWN },
        { PADKEY_LEFT,   CELL_PAD_CTRL_LEFT },    { PADKEY_RIGHT, CELL_PAD_CTRL_RIGHT },
        { PADKEY_Z,      CELL_PAD_CTRL_CROSS },   { PADKEY_X,     CELL_PAD_CTRL_CIRCLE },
        { PADKEY_A,      CELL_PAD_CTRL_SQUARE },  { PADKEY_S,     CELL_PAD_CTRL_TRIANGLE },
        { PADKEY_Q,      CELL_PAD_CTRL_L1 },      { PADKEY_W,     CELL_PAD_CTRL_R1 },
        { PADKEY_1,      CELL_PAD_CTRL_L2 },      { PADKEY_2,     CELL_PAD_CTRL_R2 },
        { PADKEY_RETURN, CELL_PAD_CTRL_START },   { PADKEY_TAB,   CELL_PAD_CTRL_SELECT },
    };

    u16 btns = 0;
    for (unsigned i = 0; i < sizeof map / sizeof map[0]; i++)
        if (pad_key_down(map[i].vk)) btns |= map[i].btn;

    hs->buttons   = btns;
    hs->connected = 1;
    hs->analog_lx = (u8)((btns & CELL_PAD_CTRL_LEFT) ? 0 :
                         (btns & CELL_PAD_CTRL_RIGHT) ? 255 : 128);
    hs->analog_ly = (u8)((btns & CELL_PAD_CTRL_UP) ? 0 :
                         (btns & CELL_PAD_CTRL_DOWN) ? 255 : 128);
    hs->analog_rx = hs->analog_ry = 128;
    hs->trigger_l2 = (u8)((btns & CELL_PAD_CTRL_L2) ? 255 : 0);
    hs->trigger_r2 = (u8)((btns & CELL_PAD_CTRL_R2) ? 255 : 0);

    { static int said = 0;
      if (!said && btns) { said = 1;
          printf("[cellPad] keyboard fallback active on port 0 (no controller)\n");
          fflush(stdout); } }
}

/* ---------------------------------------------------------------------------
 * XInput backend
 * -----------------------------------------------------------------------*/

#if PAD_BACKEND_XINPUT

/* Deadzone for analog sticks (same as XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE) */
#define PAD_STICK_DEADZONE  7849
#define PAD_TRIGGER_THRESHOLD 30

/* XInput is resolved at run time. Only XInputGetState and XInputSetState are
 * called, and XInput9_1_0.dll exports both, so any of the three DLLs serves.
 * Loaded once, from System32 only (not the exe folder, the working directory
 * or PATH), and kept loaded until exit. No DLL, or one without the two entry
 * points, means every slot reads as unplugged. */
typedef DWORD (WINAPI *pad_xi_get_state_fn)(DWORD, XINPUT_STATE*);
typedef DWORD (WINAPI *pad_xi_set_state_fn)(DWORD, XINPUT_VIBRATION*);
static pad_xi_get_state_fn p_xi_get_state;
static pad_xi_set_state_fn p_xi_set_state;
static int s_xi_tried;

static int pad_xinput_load(void)
{
    static const wchar_t* const names[] = { L"XInput1_4.dll", L"XInput1_3.dll", L"XInput9_1_0.dll" };
    if (s_xi_tried) return p_xi_get_state != NULL;
    s_xi_tried = 1;
    for (size_t k = 0; k < sizeof(names) / sizeof(names[0]); k++) {
        HMODULE m = LoadLibraryExW(names[k], NULL, LOAD_LIBRARY_SEARCH_SYSTEM32);
        pad_xi_get_state_fn g;
        pad_xi_set_state_fn st;
        if (!m) continue;
        g  = (pad_xi_get_state_fn)(void (*)(void))GetProcAddress(m, "XInputGetState");
        st = (pad_xi_set_state_fn)(void (*)(void))GetProcAddress(m, "XInputSetState");
        if (!g || !st) { FreeLibrary(m); continue; }
        p_xi_get_state = g;
        p_xi_set_state = st;
        printf("[cellPad] XInput from %ls\n", names[k]);
        return 1;
    }
    printf("[cellPad] no XInput DLL (XInput1_4, XInput1_3, XInput9_1_0): no XInput pads\n");
    return 0;
}

static u8 pad_xinput_stick_to_u8(short raw, short deadzone)
{
    float normalized;
    if (raw > deadzone)
        normalized = (float)(raw - deadzone) / (float)(32767 - deadzone);
    else if (raw < -deadzone)
        normalized = (float)(raw + deadzone) / (float)(32767 - deadzone);
    else
        normalized = 0.0f;

    /* Map -1.0..1.0 to 0..255 with center at 128 */
    int val = (int)(normalized * 127.0f) + 128;
    if (val < 0) val = 0;
    if (val > 255) val = 255;
    return (u8)val;
}

/* XInputGetState on a slot with nothing plugged in costs about 140 us (2 us on
 * a connected one), and the backend is polled at least once a frame: three
 * empty slots beside one pad were 0.6 ms of every frame on God of War HD's
 * main thread. An empty slot is looked at again once a second, which is how
 * soon a pad plugged in there is seen; the slots are staggered so one poll
 * never pays for all of them. */
#define PAD_EMPTY_RECHECK_MS 1000
static unsigned long long s_pad_empty_until[PAD_MAX_HOST_PORTS];

static void pad_poll_xinput(void)
{
    const unsigned long long now = pad_now_ms();
    for (int i = 0; i < PAD_MAX_HOST_PORTS; i++) {
        if (s_pad_empty_until[i] && now < s_pad_empty_until[i]) {
            s_host_state[i].connected = 0;
            continue;
        }
        XINPUT_STATE state;
        memset(&state, 0, sizeof(state));

        DWORD result = p_xi_get_state ? p_xi_get_state((DWORD)i, &state)
                                      : ERROR_DEVICE_NOT_CONNECTED;
        if (result != ERROR_SUCCESS) {
            pad_host_release(&s_host_state[i]);
            s_pad_empty_until[i] = now + PAD_EMPTY_RECHECK_MS
                                 + (unsigned long long)i * (PAD_EMPTY_RECHECK_MS / PAD_MAX_HOST_PORTS);
            continue;
        }
        s_pad_empty_until[i] = 0;

        s_host_state[i].connected = 1;
        XINPUT_GAMEPAD* gp = &state.Gamepad;

        /* Map XInput buttons to PS3 CELL_PAD_CTRL_* */
        u16 btns = 0;
        if (gp->wButtons & XINPUT_GAMEPAD_BACK)           btns |= CELL_PAD_CTRL_SELECT;
        if (gp->wButtons & XINPUT_GAMEPAD_LEFT_THUMB)     btns |= CELL_PAD_CTRL_L3;
        if (gp->wButtons & XINPUT_GAMEPAD_RIGHT_THUMB)    btns |= CELL_PAD_CTRL_R3;
        if (gp->wButtons & XINPUT_GAMEPAD_START)          btns |= CELL_PAD_CTRL_START;
        if (gp->wButtons & XINPUT_GAMEPAD_DPAD_UP)        btns |= CELL_PAD_CTRL_UP;
        if (gp->wButtons & XINPUT_GAMEPAD_DPAD_RIGHT)     btns |= CELL_PAD_CTRL_RIGHT;
        if (gp->wButtons & XINPUT_GAMEPAD_DPAD_DOWN)      btns |= CELL_PAD_CTRL_DOWN;
        if (gp->wButtons & XINPUT_GAMEPAD_DPAD_LEFT)      btns |= CELL_PAD_CTRL_LEFT;
        if (gp->bLeftTrigger > PAD_TRIGGER_THRESHOLD)     btns |= CELL_PAD_CTRL_L2;
        if (gp->bRightTrigger > PAD_TRIGGER_THRESHOLD)    btns |= CELL_PAD_CTRL_R2;
        if (gp->wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER)  btns |= CELL_PAD_CTRL_L1;
        if (gp->wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) btns |= CELL_PAD_CTRL_R1;
        if (gp->wButtons & XINPUT_GAMEPAD_Y)              btns |= CELL_PAD_CTRL_TRIANGLE;
        if (gp->wButtons & XINPUT_GAMEPAD_B)              btns |= CELL_PAD_CTRL_CIRCLE;
        if (gp->wButtons & XINPUT_GAMEPAD_A)              btns |= CELL_PAD_CTRL_CROSS;
        if (gp->wButtons & XINPUT_GAMEPAD_X)              btns |= CELL_PAD_CTRL_SQUARE;

        s_host_state[i].buttons = btns;

        /* Analog sticks. PS3 Y axis is inverted vs XInput (up = 0). Reflect about
         * 128 (256 - x), NOT 255 - x: the latter turns a centered stick into 127,
         * whose bits (0x7F) alias SELECT+START and can make the guest self-exit.
         * (via sagemono, PR #42) */
        s_host_state[i].analog_lx = pad_xinput_stick_to_u8(gp->sThumbLX, PAD_STICK_DEADZONE);
        s_host_state[i].analog_ly = (u8)(256 - pad_xinput_stick_to_u8(gp->sThumbLY, PAD_STICK_DEADZONE));
        s_host_state[i].analog_rx = pad_xinput_stick_to_u8(gp->sThumbRX, PAD_STICK_DEADZONE);
        s_host_state[i].analog_ry = (u8)(256 - pad_xinput_stick_to_u8(gp->sThumbRY, PAD_STICK_DEADZONE));

        /* Triggers */
        s_host_state[i].trigger_l2 = gp->bLeftTrigger;
        s_host_state[i].trigger_r2 = gp->bRightTrigger;

        /* Pressure-sensitive buttons: XInput has digital only, so 0 or 255 */
        s_host_state[i].press_up       = (btns & CELL_PAD_CTRL_UP)       ? 255 : 0;
        s_host_state[i].press_down     = (btns & CELL_PAD_CTRL_DOWN)     ? 255 : 0;
        s_host_state[i].press_left     = (btns & CELL_PAD_CTRL_LEFT)     ? 255 : 0;
        s_host_state[i].press_right    = (btns & CELL_PAD_CTRL_RIGHT)    ? 255 : 0;
        s_host_state[i].press_triangle = (btns & CELL_PAD_CTRL_TRIANGLE) ? 255 : 0;
        s_host_state[i].press_circle   = (btns & CELL_PAD_CTRL_CIRCLE)   ? 255 : 0;
        s_host_state[i].press_cross    = (btns & CELL_PAD_CTRL_CROSS)    ? 255 : 0;
        s_host_state[i].press_square   = (btns & CELL_PAD_CTRL_SQUARE)   ? 255 : 0;
        s_host_state[i].press_l1       = (btns & CELL_PAD_CTRL_L1)       ? 255 : 0;
        s_host_state[i].press_r1       = (btns & CELL_PAD_CTRL_R1)       ? 255 : 0;
    }
}

static void pad_init_backend(void)
{
    /* XInput needs no explicit init, only the run-time load of the DLL */
    pad_xinput_load();
}

static void pad_shutdown_backend(void)
{
    /* XInput needs no explicit shutdown */
}

#endif /* PAD_BACKEND_XINPUT */

/* ---------------------------------------------------------------------------
 * SDL2 backend
 * -----------------------------------------------------------------------*/

#if PAD_BACKEND_SDL2

static u8 pad_sdl_axis_to_u8(int raw)
{
    /* SDL axis: -32768..32767 -> 0..255, linear, 0 -> 128: the DualShock 3's
     * centre, and what the keyboard and the virtual pad report at rest. (The
     * old *255/65535 put the centre at 127.) */
    int val = (raw + 32768) >> 8;
    if (val < 0) val = 0;
    if (val > 255) val = 255;
    return (u8)val;
}

/* One stick through the dead zone (dz_pct, PS3_PAD_DEADZONE). Radial: the
 * stick's distance from centre over both axes, so a diagonal is not cut off
 * at the axes. Inside it the stick is centred; outside, the distance is
 * rescaled so it ramps from centre at the edge of the dead zone to full at
 * full deflection, with no jump. Each axis is clamped afterwards, so a
 * square-gated corner still reaches both ends. 0 is the plain linear map. */
static void pad_sdl_stick(int x, int y, int dz_pct, u8* ox, u8* oy)
{
    if (dz_pct > 0) {
        const double fx = x / 32768.0, fy = y / 32768.0, dz = dz_pct / 100.0;
        const double mag = sqrt(fx * fx + fy * fy);
        if (mag <= dz) {
            x = y = 0;
        } else {
            const double k = (mag - dz) / (1.0 - dz) / mag * 32768.0;
            const double vx = fx * k, vy = fy * k;
            x = (int)(vx < 0 ? vx - 0.5 : vx + 0.5);
            y = (int)(vy < 0 ? vy - 0.5 : vy + 0.5);
            if (x > 32767) x = 32767;
            if (x < -32768) x = -32768;
            if (y > 32767) y = 32767;
            if (y < -32768) y = -32768;
        }
    }
    *ox = pad_sdl_axis_to_u8(x);
    *oy = pad_sdl_axis_to_u8(y);
}

/* ---- hold to quit (PS3_PAD_QUIT) -------------------------------------------
 * Holding the PS (Guide) button for PAD_QUIT_GUIDE_MS, or START and SELECT
 * together for PAD_QUIT_CHORD_MS, on any controller quits the game the way
 * the window's close button does (cellSysutil.c ps3_host_quit). Only the
 * controller's own buttons count: PAD_SCRIPT, PAD_FILE and the other injected
 * inputs are added later, in cellPadGetData, and the keyboard is not a
 * controller. The PS button is never reported to the guest.
 *
 * Masking: from the poll in which START and SELECT are first down together
 * until both are up again, neither is reported to the guest. Whichever of the
 * two was pressed first reached the game as an ordinary press (it cannot be
 * known yet that a chord is coming); the second one never does, and letting go
 * of one while the other is still held does not produce a fresh press of it,
 * so the game sees no more than the first button's single press. */
#define PAD_QUIT_GUIDE_MS 1000
#define PAD_QUIT_CHORD_MS 2000
extern void ps3_host_quit(const char* why);   /* libs/system/cellSysutil.c */
typedef struct {
    unsigned long long guide_ms, chord_ms;   /* when the hold began */
    int guide, chord, masked;
} PadQuitHold;
static PadQuitHold s_quit_hold[PAD_MAX_HOST_PORTS];
static int s_quit_sent;

/* Port p's buttons as the guest should see them; quits when a hold is long enough. */
static u16 pad_quit_watch(int p, u16 btns, int guide, unsigned long long now)
{
    const u16 chord = CELL_PAD_CTRL_START | CELL_PAD_CTRL_SELECT;
    PadQuitHold* q = &s_quit_hold[p];
    if (guide && !q->guide) q->guide_ms = now;
    q->guide = guide;
    const int both = (btns & chord) == chord;
    if (both && !q->chord) q->chord_ms = now;
    q->chord = both;
    if (both) q->masked = 1;
    else if (!(btns & chord)) q->masked = 0;
    if (q->masked) btns &= (u16)~chord;
    const char* how = q->guide && now - q->guide_ms >= PAD_QUIT_GUIDE_MS ? "PS button held 1 s"
                    : q->chord && now - q->chord_ms >= PAD_QUIT_CHORD_MS ? "START+SELECT held 2 s" : NULL;
    if (how && !s_quit_sent) {
        s_quit_sent = 1;
        fprintf(stderr, "[cellPad] port %d: %s\n", p, how);
        ps3_host_quit("controller");
    }
    return btns;
}

/* ---- motion (PS3_PAD_MOTION, PS3_PAD_SHAKE_BUTTON) -------------------------
 * The guest's sensor block (CELL_PAD_BTN_OFFSET_SENSOR_X..G, sent only in
 * sensor mode) is a DualShock 3's: 10-bit values centred on 512, 113 counts
 * per g on the accelerometer, and one gyro axis, yaw, at 123 counts per
 * 90 degrees/s. A DS3 lying flat reads X 512, Y 399, Z 512, G 512.
 * SDL's sensor axes (SDL_sensor.h) are +X right, +Y up out of the face,
 * +Z towards the player, in m/s^2 with gravity included (at rest
 * Y = +SDL_STANDARD_GRAVITY), and rad/s counter-clockwise; each accelerometer
 * axis is negated into the DS3's, as RPCS3's SDL handler does, which gives
 * exactly the at-rest values above.
 *
 * SDL's sensors are enabled for a port only while the guest has sensor mode
 * on there and PS3_PAD_MOTION is on, and disabled again when either goes.
 *
 * Shake button: PAIN counts a shake when the mean frame-to-frame change of
 * sensor Y, after its own low-pass filter, is at least 20 counts over its
 * last four samples (func_0021D8EC, threshold at 0xC6F164). While the button
 * is held, Y swings 3 g either side of rest at 4 Hz in a triangle wave (an
 * even change per frame), from the moment of the press, which keeps that
 * mean above 20 from 15 to 60 game frames per second; release returns to rest
 * at once. The button's own press still reaches the guest. */
#define PAD_DS3_ONE_G     113
#define PAD_SHAKE_G         3
#define PAD_SHAKE_PERIOD_MS 250
#define PAD_MOTION_ACCEL    1
#define PAD_MOTION_GYRO     2
static int s_motion_want[PAD_MAX_HOST_PORTS];      /* sensors asked for on this port */
static int s_motion_on[PAD_MAX_HOST_PORTS];        /* PAD_MOTION_* SDL enabled */
static unsigned long long s_shake_ms[PAD_MAX_HOST_PORTS];   /* press time, 0 = not held */

static u16 pad_ds3_count(double v)
{
    v = v < 0 ? v - 0.5 : v + 0.5;
    return (u16)(v < 0 ? 0 : v > 1023 ? 1023 : (int)v);
}

/* SDL accelerometer (m/s^2) and yaw rate (rad/s) as DS3 sensor counts. */
static void pad_ds3_sensors(const float accel[3], float yaw, u16 out[4])
{
    for (int k = 0; k < 3; k++)
        out[k] = pad_ds3_count(512.0 - accel[k] / SDL_STANDARD_GRAVITY * PAD_DS3_ONE_G);
    out[3] = pad_ds3_count(512.0 + yaw * (180.0 / 3.14159265358979323846) * (123.0 / 90.0));
}

/* Turn port p's SDL sensors on or off to match `want`. */
static void pad_sdl_motion_switch(int p, SDL_GameController* gc, int want)
{
    if (want == s_motion_want[p]) return;
    s_motion_want[p] = want;
    static const SDL_SensorType type[2] = { SDL_SENSOR_ACCEL, SDL_SENSOR_GYRO };
    for (int k = 0; k < 2; k++) {
        const int bit = k ? PAD_MOTION_GYRO : PAD_MOTION_ACCEL;
        if (want && SDL_GameControllerHasSensor(gc, type[k]) &&
            SDL_GameControllerSetSensorEnabled(gc, type[k], SDL_TRUE) == 0)
            s_motion_on[p] |= bit;
        else if (!want && (s_motion_on[p] & bit)) {
            SDL_GameControllerSetSensorEnabled(gc, type[k], SDL_FALSE);
            s_motion_on[p] &= ~bit;
        }
    }
    if (want)
        fprintf(stderr, "[cellPad] port %d: motion sensors %s\n", p,
                s_motion_on[p] == (PAD_MOTION_ACCEL | PAD_MOTION_GYRO) ? "accelerometer and gyroscope"
                : s_motion_on[p] == PAD_MOTION_ACCEL ? "accelerometer only"
                : s_motion_on[p] == PAD_MOTION_GYRO ? "gyroscope only" : "none");
}

/* Port p's sensor block for this poll: the shake while its button is held,
 * else the controller's own sensors, else nothing (the at-rest values). */
static void pad_sdl_motion(int p, SDL_GameController* gc, unsigned long long now)
{
    PadHostState* hs = &s_host_state[p];
    pad_sdl_motion_switch(p, gc, pad_motion_enabled() && (s_port_setting[p] & CELL_PAD_SETTING_SENSOR_ON));
    hs->motion_live = 0;
    const int sb = pad_shake_button();
    if (sb >= 0 && SDL_GameControllerGetButton(gc, (SDL_GameControllerButton)sb)) {
        if (!s_shake_ms[p]) s_shake_ms[p] = now;
        /* A triangle wave, -1..1, starting at 0 on its way up. */
        const double q = (double)((now - s_shake_ms[p]) % PAD_SHAKE_PERIOD_MS) / PAD_SHAKE_PERIOD_MS;
        const double wave = q < 0.25 ? 4 * q : q < 0.75 ? 2 - 4 * q : 4 * q - 4;
        memcpy(hs->sensor, s_sensor_rest, sizeof hs->sensor);
        hs->sensor[1] = pad_ds3_count(s_sensor_rest[1] + PAD_SHAKE_G * PAD_DS3_ONE_G * wave);
        hs->motion_live = 1;
        return;
    }
    s_shake_ms[p] = 0;
    if (!s_motion_on[p]) return;
    float accel[3] = { 0, 0, 0 }, gyro[3] = { 0, 0, 0 };
    const int got_accel = (s_motion_on[p] & PAD_MOTION_ACCEL) &&
        SDL_GameControllerGetSensorData(gc, SDL_SENSOR_ACCEL, accel, 3) == 0;
    const int got_gyro = (s_motion_on[p] & PAD_MOTION_GYRO) &&
        SDL_GameControllerGetSensorData(gc, SDL_SENSOR_GYRO, gyro, 3) == 0;
    /* All zero is a sensor that has not reported yet, not free fall: a jump
     * from rest to 512 and back would read as a shake. */
    if (!got_accel || (accel[0] == 0 && accel[1] == 0 && accel[2] == 0)) {
        accel[0] = accel[2] = 0;
        accel[1] = SDL_STANDARD_GRAVITY;
    }
    if (!got_gyro) gyro[1] = 0;
    pad_ds3_sensors(accel, gyro[1], hs->sensor);
    hs->motion_live = 1;
}

static void pad_sdl_motion_reset(int p)
{
    s_motion_want[p] = s_motion_on[p] = 0;
    s_shake_ms[p] = 0;
}

static u8 pad_sdl_trigger_to_u8(int raw)
{
    /* SDL trigger: 0..32767 -> 0..255 */
    int val = (raw * 255) / 32767;
    if (val < 0) val = 0;
    if (val > 255) val = 255;
    return (u8)val;
}

/* Send port i's rumble state to its controller, with a fresh duration. */
static void pad_sdl_rumble_send(int i)
{
    SDL_GameController* gc = s_sdl_controllers[i];
    if (!gc) return;
    PadRumble* r = &s_rumble[i];
    const int on = r->lo || r->hi;
    SDL_GameControllerRumble(gc, r->lo, r->hi, on ? PAD_RUMBLE_HOLD_MS : 0);
    r->sent_ms = on ? pad_now_ms() : 0;
}

/* ---- hot-plug ------------------------------------------------------------
 * A DualSense over Bluetooth drops off when it is switched off or idles, and
 * comes back as a NEW SDL device (a new instance id, maybe another index).
 * This used to open controllers by device index == PS3 port and ask
 * SDL_IsGameController(port) again on every poll. With Homebrew's SDL2, which
 * is sdl2-compat over SDL3, a device index is resolved through a list that is
 * refreshed only by SDL_NumJoysticks() or by the SDL2 event filter (installed
 * with an SDL window, which the Metal build never makes): the list kept the
 * switched-off pad's instance id, SDL_IsGameController(0) stayed false, and
 * the pad that came back was never opened until the game was restarted.
 *
 * Now each poll drains SDL's joystick/controller events (nothing else here
 * reads them, and without a window nothing else does either, so they piled up
 * in the queue), closes a controller SDL says is gone, by its own handle, and
 * on a device event -- or every PAD_SDL_RESCAN_MS while a port is free, in
 * case a window's event loop ate the event -- walks the device list from
 * SDL_NumJoysticks() and opens every game controller that is not open yet:
 * into the port its GUID had last (the same pad reconnecting gets its port
 * back), else the lowest free one. A new controller starts with its motors
 * off and no rumble left over. The port reads connected again on the next
 * poll, and cellPadGetInfo/GetInfo2/PeriphGetInfo flag the change once with
 * CELL_PAD_STATUS_ASSIGN_CHANGES (port 0 is the virtual pad and never reads
 * disconnected). */
#define PAD_SDL_RESCAN_MS 1000

static int pad_sdl_port_of(SDL_JoystickID id)
{
    for (int p = 0; p < PAD_MAX_HOST_PORTS; p++)
        if (s_sdl_controllers[p] && s_sdl_ids[p] == id) return p;
    return -1;
}

/* Close port p's controller (gone, or shutting down). */
static void pad_sdl_close(int p, int gone)
{
    SDL_GameController* gc = s_sdl_controllers[p];
    if (!gc) return;
    if (gone) fprintf(stderr, "[cellPad] SDL controller on port %d disconnected (instance %d)\n", p, (int)s_sdl_ids[p]);
    else      SDL_GameControllerRumble(gc, 0, 0, 0);
    SDL_GameControllerClose(gc);
    s_sdl_controllers[p] = NULL;
    s_sdl_ids[p] = -1;
    memset(&s_rumble[p], 0, sizeof s_rumble[p]);
    memset(&s_quit_hold[p], 0, sizeof s_quit_hold[p]);
    pad_sdl_motion_reset(p);
    pad_host_release(&s_host_state[p]);
}

/* Open every game controller SDL lists that no port has yet. */
static void pad_sdl_scan(void)
{
    const int n = SDL_NumJoysticks();   /* also refreshes sdl2-compat's index list */
    for (int idx = 0; idx < n; idx++) {
        if (!SDL_IsGameController(idx)) continue;
        const SDL_JoystickID id = SDL_JoystickGetDeviceInstanceID(idx);
        if (id < 0 || pad_sdl_port_of(id) >= 0) continue;
        const SDL_JoystickGUID guid = SDL_JoystickGetDeviceGUID(idx);
        int port = -1, back = 0;
        for (int p = 0; p < PAD_MAX_HOST_PORTS && port < 0; p++)
            if (!s_sdl_controllers[p] && s_sdl_had[p] && !memcmp(&s_sdl_last_guid[p], &guid, sizeof guid)) { port = p; back = 1; }
        for (int p = 0; p < PAD_MAX_HOST_PORTS && port < 0; p++)
            if (!s_sdl_controllers[p]) port = p;
        if (port < 0) return;                 /* every port has a controller */
        SDL_GameController* gc = SDL_GameControllerOpen(idx);
        if (!gc) {
            fprintf(stderr, "[cellPad] SDL controller %d: open failed: %s\n", idx, SDL_GetError());
            continue;
        }
        s_sdl_controllers[port] = gc;
        s_sdl_ids[port] = id;
        s_sdl_last_guid[port] = guid;
        s_sdl_had[port] = 1;
        memset(&s_rumble[port], 0, sizeof s_rumble[port]);   /* nothing left over from before */
        pad_sdl_motion_reset(port);
        SDL_GameControllerRumble(gc, 0, 0, 0);
        const char* name = SDL_GameControllerName(gc);
        fprintf(stderr, "[cellPad] SDL controller %s on port %d: %s (instance %d)%s\n",
                back ? "reconnected" : "connected", port, name ? name : "?", (int)id,
                SDL_GameControllerHasRumble(gc) ? "" : " (no rumble)");
    }
}

/* Device changes since the last poll; force = scan now (init). */
static void pad_sdl_hotplug(int force)
{
    int changed = force;
    SDL_Event ev[32];
    int got;
    /* SDL_JOYAXISMOTION (0x600) up to the touch events (0x700): every
     * joystick and game controller event, which only this file uses. */
    while ((got = SDL_PeepEvents(ev, 32, SDL_GETEVENT, SDL_JOYAXISMOTION, SDL_JOYAXISMOTION + 0xFF)) > 0)
        for (int i = 0; i < got; i++)
            if (ev[i].type == SDL_JOYDEVICEADDED || ev[i].type == SDL_JOYDEVICEREMOVED ||
                ev[i].type == SDL_CONTROLLERDEVICEADDED || ev[i].type == SDL_CONTROLLERDEVICEREMOVED)
                changed = 1;
    for (int p = 0; p < PAD_MAX_HOST_PORTS; p++)
        if (s_sdl_controllers[p] && !SDL_GameControllerGetAttached(s_sdl_controllers[p])) {
            pad_sdl_close(p, 1);
            changed = 1;
        }
    int free_port = 0;
    for (int p = 0; p < PAD_MAX_HOST_PORTS; p++) if (!s_sdl_controllers[p]) free_port = 1;
    if (!free_port) return;
    const unsigned long long now = pad_now_ms();
    if (!changed && s_sdl_scan_ms && now - s_sdl_scan_ms < PAD_SDL_RESCAN_MS) return;
    s_sdl_scan_ms = now;
    pad_sdl_scan();
}

static void pad_poll_sdl2(void)
{
    SDL_GameControllerUpdate();
    pad_sdl_hotplug(0);

    for (int i = 0; i < PAD_MAX_HOST_PORTS; i++) {
        SDL_GameController* gc = s_sdl_controllers[i];
        if (!gc) {
            pad_host_release(&s_host_state[i]);
            memset(&s_rumble[i], 0, sizeof s_rumble[i]);
            memset(&s_quit_hold[i], 0, sizeof s_quit_hold[i]);
            pad_sdl_motion_reset(i);
            continue;
        }

        s_host_state[i].connected = 1;

        /* Keep a rumble the title set going (see PAD_RUMBLE_HOLD_MS). */
        if (s_rumble[i].sent_ms && pad_now_ms() - s_rumble[i].sent_ms >= PAD_RUMBLE_RENEW_MS)
            pad_sdl_rumble_send(i);

        u16 btns = 0;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_BACK))          btns |= CELL_PAD_CTRL_SELECT;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_LEFTSTICK))     btns |= CELL_PAD_CTRL_L3;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_RIGHTSTICK))    btns |= CELL_PAD_CTRL_R3;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_START))         btns |= CELL_PAD_CTRL_START;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_DPAD_UP))       btns |= CELL_PAD_CTRL_UP;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_DPAD_RIGHT))    btns |= CELL_PAD_CTRL_RIGHT;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_DPAD_DOWN))     btns |= CELL_PAD_CTRL_DOWN;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_DPAD_LEFT))     btns |= CELL_PAD_CTRL_LEFT;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_LEFTSHOULDER))  btns |= CELL_PAD_CTRL_L1;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER)) btns |= CELL_PAD_CTRL_R1;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_Y))             btns |= CELL_PAD_CTRL_TRIANGLE;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_B))             btns |= CELL_PAD_CTRL_CIRCLE;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_A))             btns |= CELL_PAD_CTRL_CROSS;
        if (SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_X))             btns |= CELL_PAD_CTRL_SQUARE;

        /* Triggers via axis */
        int lt = SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
        int rt = SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
        if (lt > 3000) btns |= CELL_PAD_CTRL_L2;
        if (rt > 3000) btns |= CELL_PAD_CTRL_R2;

        if (pad_quit_enabled())
            btns = pad_quit_watch(i, btns, SDL_GameControllerGetButton(gc, SDL_CONTROLLER_BUTTON_GUIDE) != 0, pad_now_ms());

        s_host_state[i].buttons = btns;

        /* Analog sticks, through the dead zone (PS3_PAD_DEADZONE) */
        const int dz = pad_deadzone_pct();
        pad_sdl_stick(SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_LEFTX),
                      SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_LEFTY), dz,
                      &s_host_state[i].analog_lx, &s_host_state[i].analog_ly);
        pad_sdl_stick(SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_RIGHTX),
                      SDL_GameControllerGetAxis(gc, SDL_CONTROLLER_AXIS_RIGHTY), dz,
                      &s_host_state[i].analog_rx, &s_host_state[i].analog_ry);

        /* Triggers */
        s_host_state[i].trigger_l2 = pad_sdl_trigger_to_u8(lt);
        s_host_state[i].trigger_r2 = pad_sdl_trigger_to_u8(rt);

        /* Pressure: SDL has digital buttons, so 0 or 255 */
        s_host_state[i].press_up       = (btns & CELL_PAD_CTRL_UP)       ? 255 : 0;
        s_host_state[i].press_down     = (btns & CELL_PAD_CTRL_DOWN)     ? 255 : 0;
        s_host_state[i].press_left     = (btns & CELL_PAD_CTRL_LEFT)     ? 255 : 0;
        s_host_state[i].press_right    = (btns & CELL_PAD_CTRL_RIGHT)    ? 255 : 0;
        s_host_state[i].press_triangle = (btns & CELL_PAD_CTRL_TRIANGLE) ? 255 : 0;
        s_host_state[i].press_circle   = (btns & CELL_PAD_CTRL_CIRCLE)   ? 255 : 0;
        s_host_state[i].press_cross    = (btns & CELL_PAD_CTRL_CROSS)    ? 255 : 0;
        s_host_state[i].press_square   = (btns & CELL_PAD_CTRL_SQUARE)   ? 255 : 0;
        s_host_state[i].press_l1       = (btns & CELL_PAD_CTRL_L1)       ? 255 : 0;
        s_host_state[i].press_r1       = (btns & CELL_PAD_CTRL_R1)       ? 255 : 0;

        pad_sdl_motion(i, gc, pad_now_ms());
    }
}

static void pad_init_backend(void)
{
    if (!s_sdl_inited) {
        if (SDL_WasInit(SDL_INIT_GAMECONTROLLER) == 0) {
            /* The game is not always the focused app: on macOS its window
             * can belong to another process (the launcher in
             * tools/host_window), and then this process is never in the
             * foreground. Controller input must not depend on focus. The
             * hint stops SDL dropping joystick events while it has no
             * focused window and lets Apple's GameController framework
             * deliver them to a background process
             * (GCController.shouldMonitorBackgroundEvents). The HIDAPI
             * drivers (DualShock 4, DualSense) read IOKit directly and never
             * cared. SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=0 in the
             * environment still wins over this. */
            SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
            /* Rumble on a DualShock 4 or DualSense over Bluetooth needs the
             * HIDAPI driver's extended reports, off by default in SDL 2
             * (they stay on until the controller is power-cycled). And face
             * buttons by position, not label: a Nintendo pad's bottom button
             * is Cross, as on a DualShock. The same-named environment
             * variables still win, as above (SDL_SetHint is normal priority). */
            SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS4_RUMBLE, "1");
            SDL_SetHint(SDL_HINT_JOYSTICK_HIDAPI_PS5_RUMBLE, "1");
            SDL_SetHint(SDL_HINT_GAMECONTROLLER_USE_BUTTON_LABELS, "0");
            SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");   /* SIGTERM/SIGINT: ps3_host_quit_on_signals */
            SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER);
        }
        s_sdl_inited = 1;
    }
    memset(s_sdl_controllers, 0, sizeof(s_sdl_controllers));
    for (int p = 0; p < PAD_MAX_HOST_PORTS; p++) s_sdl_ids[p] = -1;

    /* Open any controllers already connected */
    pad_sdl_hotplug(1);
}

static void pad_shutdown_backend(void)
{
    for (int i = 0; i < PAD_MAX_HOST_PORTS; i++) pad_sdl_close(i, 0);
    memset(s_rumble, 0, sizeof s_rumble);
}

#endif /* PAD_BACKEND_SDL2 */

/* ---------------------------------------------------------------------------
 * Poll dispatcher
 * -----------------------------------------------------------------------*/

/* Re-poll the host at most every PAD_POLL_INTERVAL_MS; serve cached state in
 * between.
 *
 * XInputGetState is a driver call, and on a slot with nothing plugged in it is
 * an expensive one: measured at 97.6 us for a sweep of the 7 host ports on a
 * machine with no controller attached. cellPadGetData ran that sweep on EVERY
 * call, and a guest that polls the pad from a spin loop -- The Simpsons Arcade
 * Game does, in its input stage -- therefore spends its whole main thread
 * inside the input driver.
 *
 * 4 ms is 250 Hz, well above the 60 Hz a title can actually observe (and above
 * a real DualShock 3's own report rate), so no game can tell the difference,
 * including one sampling for button-press edges. */
#define PAD_POLL_INTERVAL_MS 4

static unsigned long long pad_now_ms(void)
{
#ifdef _WIN32
    return (unsigned long long)GetTickCount64();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long long)ts.tv_sec * 1000ull +
           (unsigned long long)ts.tv_nsec / 1000000ull;
#endif
}

static void pad_poll_backend(void)
{
    static unsigned long long s_last_ms = 0;
    unsigned long long now = pad_now_ms();
    if (s_last_ms && now - s_last_ms < PAD_POLL_INTERVAL_MS) return;
    if (!pad_lock_try()) return;   /* another thread is polling: its result serves this read */
    s_last_ms = now;
    /* A real pad streams reports at roughly this rate whether or not anything
     * changed, and libpad buffers them; a fresh sweep is a fresh report for
     * every port. cellPadGetData hands each one out exactly once. */
    for (int _i = 0; _i < PAD_MAX_HOST_PORTS; _i++) s_data_fresh[_i] = 1;

#if PAD_BACKEND_XINPUT
    pad_poll_xinput();
#elif PAD_BACKEND_SDL2
    pad_poll_sdl2();
#endif
    if (!s_host_state[0].connected) pad_poll_keyboard();
    pad_track_assign();
    pad_unlock();
}

/* ---------------------------------------------------------------------------
 * API implementations
 * -----------------------------------------------------------------------*/

s32 cellPadInit(u32 max_connect)
{
    printf("[cellPad] Init(max_connect=%u)\n", max_connect);

    if (s_pad_initialized)
        return CELL_PAD_ERROR_ALREADY_OPENED;

    /* The bound is the 127-slot pad table, not the 7 ports: larger requests
     * are accepted and clamped. Resistance: Fall of Man asks for 8; rejected,
     * the library stayed closed, every GetInfo failed and the title never
     * polled the pad -- stuck on "Press the START button". */
    if (max_connect == 0 || max_connect > 127)
        return CELL_PAD_ERROR_INVALID_PARAMETER;

    s_pad_initialized = 1;
    s_max_connect = max_connect > CELL_PAD_MAX_PORT_NUM ? CELL_PAD_MAX_PORT_NUM : max_connect;
    memset(s_port_setting, 0, sizeof(s_port_setting));
    for (int i = 0; i < PAD_MAX_HOST_PORTS; i++)
        pad_host_release(&s_host_state[i]);

    pad_lock();
    pad_init_backend();
    pad_unlock();

    /* Do an initial poll to detect connected controllers */
    pad_poll_backend();
    /* What is there at start is the baseline; only later changes are
     * flagged CELL_PAD_STATUS_ASSIGN_CHANGES. */
    for (int i = 0; i < PAD_MAX_HOST_PORTS; i++) {
        s_port_reported[i] = s_host_state[i].connected || i == 0;
        s_port_changed[i] = 0;
    }

    return CELL_OK;
}

s32 cellPadEnd(void)
{
    printf("[cellPad] End()\n");

    if (!s_pad_initialized)
        return CELL_PAD_ERROR_NOT_OPENED;

    pad_lock();
    pad_shutdown_backend();
    pad_unlock();

    s_pad_initialized = 0;
    s_max_connect = 0;
    return CELL_OK;
}

void cellPad_poll(void)
{
    if (s_pad_initialized) {
        pad_poll_backend();
    }
}

/* HLE args are GUEST effective addresses; guest structs are BIG-ENDIAN. The
 * vm_write helpers from ppu_memory.h translate + byte-swap. (cellPad was
 * written for host pointers and was never actually invoked until its NIDs
 * were fixed.) These used to be redeclared here with a 64-bit address
 * parameter, which conflicts with the header now that it is included. */

s32 cellPadGetData(u32 port_no, CellPadData* data_guest)
{
    { static int _once = 0;
      if (!_once++) printf("[cellPad] GetData polling begins (port %u)\n", port_no); }
    if (!s_pad_initialized)
        return CELL_PAD_ERROR_NOT_OPENED;

    if (port_no >= s_max_connect || !data_guest)
        return CELL_PAD_ERROR_INVALID_PARAMETER;

    /* Build in a local host struct, then copy to guest memory big-endian. */
    CellPadData _d; CellPadData* data = &_d;
    memset(data, 0, sizeof(CellPadData));

    /* Poll fresh state */
    pad_poll_backend();

    /* YDKJ_INJECT_PAD: the game's wait loop polls the pad (cellPadSetActDirect /
     * GetData) -- if it's parked on a "press button" prompt after loading, no
     * host input means it waits forever. Inject a real button PULSE (CROSS+START+
     * CIRCLE) so it advances. Legit input simulation, not forged pixels. */
    /* NOTE: cache getenv() -- this runs on every poll from multiple threads, and the
     * uncached version crashed deterministically (varying fault address = race). Every
     * other probe in this tree uses the cached `static int x=-1` pattern; match it. */
    static int  s_inj   = -1;
    static u16  s_btn   = 0x0008u;   /* default START */
    static long s_delay = 3000;
    if (s_inj < 0) {
        s_inj = getenv("YDKJ_INJECT_PAD") ? 1 : 0;
        const char* be = getenv("YDKJ_PAD_BTN");   if (be) s_btn   = (u16)strtoul(be,0,0);
        const char* de = getenv("YDKJ_PAD_DELAY"); if (de) s_delay = strtol(de,0,0);
    }
    /* Gate on WALL-CLOCK, not poll count: the game polls the pad hundreds of thousands
     * of times inside an early wait loop, so any count-based delay fires during init
     * (crash: guest ctr=0x0005F8A0 r3=0 -- button handler on an unconstructed object).
     * YDKJ_PAD_DELAY is now SECONDS to wait before the first injected press. */
    if (s_inj && port_no < PAD_MAX_HOST_PORTS) {
        static long _pc = 0; _pc++;
        /* Cheap counter gate FIRST: clock() is a slow call and this poll loop is hammered
         * by the guest from multiple threads; sampling it every poll perturbs timing. Only
         * consult the clock once every 256 polls, and never after arming. */
        static int     s_armed = 0;
        static clock_t s_t0    = 0;
        if (!s_armed) {
            if ((_pc & 0xFF) == 0) {
                if (!s_t0) s_t0 = clock();
                else if ((double)(clock() - s_t0) / (double)CLOCKS_PER_SEC >= (double)s_delay) s_armed = 1;
            }
            goto skip_inject;
        }
        u16 btn = s_btn; long delay = 0;
        /* Delay past init, then a single clean pulse every ~180 calls (press 15, release 165). */
        if (_pc > delay) {
            long ph = (_pc - delay) % 180;
            if (ph < 15) s_host_state[port_no].buttons |= btn;
            else         s_host_state[port_no].buttons &= ~btn;
            s_host_state[port_no].connected = 1;
        }
    }
skip_inject: ;

    if (port_no >= PAD_MAX_HOST_PORTS || (!s_host_state[port_no].connected && port_no != 0)) {
        goto emit;   /* data stays zeroed -> len=0 */
    }
    /* port 0 is always a valid (virtual) pad; s_host_state[0] is neutral if no
     * physical device, which yields no-buttons-pressed data below. */

    {
    PadHostState* hs = &s_host_state[port_no];
    u32 setting = s_port_setting[port_no];

    /* Determine data length based on port settings */
    s32 len = CELL_PAD_LEN_CHANGE_DEFAULT;
    if (setting & CELL_PAD_SETTING_SENSOR_ON)
        len = CELL_PAD_LEN_CHANGE_SENSOR_ON;
    else if (setting & CELL_PAD_SETTING_PRESS_ON)
        len = CELL_PAD_LEN_CHANGE_PRESS_ON;

    data->len = len;
    /* Finalize this call's packet freshness before any injected hold budget
     * advances. len=0 drain reads still build the ordinary payload, but do
     * not consume a PAD_FILE report. See the emission contract below. */
    {
        static int always = -1;
        if (always < 0) always = getenv("PAD_ALWAYS_CHANGE") ? 1 : 0;
        if (!always && port_no < PAD_MAX_HOST_PORTS && data->len) {
            if (!s_data_fresh[port_no]) data->len = 0;   /* nothing new: end the drain */
            else                        s_data_fresh[port_no] = 0;
        }
    }
    /* button[0] is reserved and 0 on hardware; button[1] carries the pad mode
     * in its high nibble (0x7) and len/2 in its low one. GH3 drops any packet
     * whose button[0] is non-zero -- with len (8) there, every press was
     * received and thrown away, and "Press any button" never advanced. */
    data->button[0] = 0;
    data->button[1] = (u16)(0x70 | ((len / 2) & 0xF));

    /* Digital buttons. hs->buttons packs both halves into one 16-bit mask
     * (SELECT=bit0..LEFT=bit7 = DIGITAL1; L2=bit8..SQUARE=bit15 = DIGITAL2).
     * Split correctly — writing the whole value into DIGITAL1 with DIGITAL2=0
     * left every face button (cross/circle/triangle/square, L1/L2/R1/R2) dead.
     * (via sagemono, PR #42) */
    data->button[CELL_PAD_BTN_OFFSET_DIGITAL1] = (u16)(hs->buttons & 0xFF);
    data->button[CELL_PAD_BTN_OFFSET_DIGITAL2] = (u16)((hs->buttons >> 8) & 0xFF);

    /* PAD_AUTOPRESS: headless bring-up input -- pulse CROSS then START every few
     * seconds of polling so boot screens that wait for input advance without a
     * human at the pad. Env-gated test scaffolding, off by default. */
    { static int s_ap = -1; static unsigned s_apn = 0;
      if (s_ap < 0) s_ap = getenv("PAD_AUTOPRESS") ? 1 : 0;
      if (s_ap && port_no == 0) {
          unsigned ph = s_apn++ % 240;
          if (ph < 12)
              data->button[CELL_PAD_BTN_OFFSET_DIGITAL2] |= 0x40;   /* CROSS */
          else if (ph >= 120 && ph < 132)
              data->button[CELL_PAD_BTN_OFFSET_DIGITAL1] |= 0x08;   /* START */
      } }

    /* PAD_SCRIPT="<sec>:<mask>,<sec>:<mask>,..." -- press a NAMED button at a
     * given wall-clock second (or flip, under PAD_CLOCK=flips), each held ~250 ms. PAD_AUTOPRESS only pulses
     * CROSS and START, which is enough to clear a "press start" screen but not
     * to navigate a menu: You Don't Know Jack needs DOWN to move off its name
     * field before CROSS means anything. Masks are CELL_PAD_CTRL_* packed as
     * (DIGITAL2 << 8) | DIGITAL1: START 0x0008, UP 0x0010, DOWN 0x0040,
     * LEFT 0x0080, RIGHT 0x0020, CROSS 0x4000, CIRCLE 0x2000, TRIANGLE 0x1000.
     * Legit input simulation on the same footing as PAD_AUTOPRESS/PAD_STICK. */
    { static int s_sc = -1;
      static struct { double t; unsigned mask; } ev[256]; static int n_ev = 0;
      static unsigned long long t0 = 0;   /* not ULONGLONG: this builds on POSIX too */
      if (s_sc < 0) {
          s_sc = 0;
          const char* e = getenv("PAD_SCRIPT");
          if (e && *e) {
              s_sc = 1; t0 = GetTickCount64();
              const char* p = e;
              while (*p && n_ev < 256) {
                  double t = strtod(p, (char**)&p);
                  if (*p == ':') p++;
                  unsigned m = (unsigned)strtoul(p, (char**)&p, 0);
                  ev[n_ev].t = t; ev[n_ev].mask = m; n_ev++;
                  while (*p == ',' || *p == ' ') p++;
              }
              printf("[cellPad] PAD_SCRIPT: %d event(s)%s\n", n_ev,
                     *p ? " -- TRUNCATED at 256, the rest are ignored" : "");
          }
      }
      if (s_sc && port_no == 0) {
          double now = pad_clock(t0);
          /* Latch, do not window. A fixed time window is missed whenever the
           * title stops polling the pad for longer than the window (it does
           * exactly that on a loading screen), so the press silently never
           * happens. Fire on the first poll at or after the event time and
           * hold it for a fixed number of polls, which is what a real press
           * looks like regardless of poll rate. */
          static int fired[256], held[256];
          for (int i = 0; i < n_ev; i++) {
              if (!fired[i] && now >= ev[i].t) { fired[i] = 1; held[i] = 40; }
              if (held[i] > 0) {
                  held[i]--;
                  data->button[CELL_PAD_BTN_OFFSET_DIGITAL1] |= (u16)(ev[i].mask & 0xFF);
                  data->button[CELL_PAD_BTN_OFFSET_DIGITAL2] |= (u16)((ev[i].mask >> 8) & 0xFF);
                  static int said[256];
                  if (!said[i]) { said[i] = 1;
                      printf("[cellPad] PAD_SCRIPT t=%.1fs press 0x%04X\n", ev[i].t, ev[i].mask); }
              }
          }
      } }

    /* PAD_FILE=<path> -- closed-loop input. Each poll, if the file holds a
     * button mask (same encoding as PAD_SCRIPT), press it for a fixed number
     * of positive guest reports and empty the file on admission. A driver
     * outside the process can look at the log or a frame dump, decide which
     * screen the title is on, and
     * write the next button -- which a fixed time schedule cannot do, because
     * a menu system with several screens wanders when driven blind.
     *
     *   echo 0x0008 > pad.txt    # START
     *   echo 0x4000 > pad.txt    # CROSS
     *   echo 0x0040 4 > pad.txt  # DOWN for 4 reports (menus auto-repeat a
     *                            # long hold; the default is 40)
     */
    { static int s_pf = -1; static char pf_path[512];
      static unsigned held_mask = 0; static int held_n = 0;
      if (s_pf < 0) { const char* e = getenv("PAD_FILE");
                      s_pf = (e && *e) ? 1 : 0;
                      if (s_pf) { strncpy(pf_path, e, sizeof pf_path - 1); pf_path[sizeof pf_path - 1] = 0; } }
      if (s_pf && port_no == 0) {
          if (held_n <= 0) {
              FILE* f = fopen(pf_path, "rb");
              if (f) {
                  char buf[64]; buf[0] = 0;
                  if (fgets(buf, sizeof buf, f)) {
                      char* rest;
                      unsigned m = (unsigned)strtoul(buf, &rest, 0);
                      int n = (int)strtol(rest, 0, 0);
                      if (m) { held_mask = m; held_n = n > 0 ? n : 40;
                               printf("[cellPad] PAD_FILE press 0x%04X\n", m); fflush(stdout); }
                  }
                  fclose(f);
                  if (held_n > 0) { FILE* w = fopen(pf_path, "wb"); if (w) fclose(w); }
              }
          }
          if (held_n > 0) {
              if (data->len > 0) held_n--;
              data->button[CELL_PAD_BTN_OFFSET_DIGITAL1] |= (u16)(held_mask & 0xFF);
              data->button[CELL_PAD_BTN_OFFSET_DIGITAL2] |= (u16)((held_mask >> 8) & 0xFF); }
      } }

    /* Analog sticks */
    /* PAD_STICK="lx,ly,rx,ry" (0-255, 128 = centred): hold the analog sticks at
     * fixed positions. Rubber Ducky drives its camera from the sticks and never
     * moves on its own, so with no host pad the view is frozen wherever it
     * started -- which can leave the subject of the demo off-screen or too far
     * away to make out. Legit input simulation, same footing as YDKJ_INJECT_PAD. */
    { static int s_st = -1;
      static unsigned char s_v[4] = {128,128,128,128};
      /* Optional 5th/6th fields: the seconds the hold starts and ends, so the menus
       * before gameplay are not driven by it (PAD_STICK="128,0,128,128,300,320"). */
      static double s_from = 0, s_until = 1e30; static unsigned long long s_t0 = 0;
      if (!s_t0) s_t0 = GetTickCount64();
      if (s_st < 0) { const char* e = getenv("PAD_STICK");
        s_st = e ? 1 : 0;
        if (e) { int a=128,b=128,c=128,d=128;
                 sscanf(e, "%d,%d,%d,%d,%lf,%lf", &a,&b,&c,&d,&s_from,&s_until);
                 s_v[0]=(unsigned char)a; s_v[1]=(unsigned char)b;
                 s_v[2]=(unsigned char)c; s_v[3]=(unsigned char)d; } }
      if (s_st && pad_clock(s_t0) >= s_from && pad_clock(s_t0) < s_until) { hs->analog_lx = s_v[0]; hs->analog_ly = s_v[1];
                  hs->analog_rx = s_v[2]; hs->analog_ry = s_v[3];
                  hs->connected = 1; }
      /* PAD_SWEEP=<seconds per step>: walk the sticks through a fixed set of
       * deflections instead of holding one. Guessing a single stick position and
       * re-running costs ~5 minutes a try; a sweep answers "can input move the
       * camera anywhere useful" in one run, and the frame dumps say which step
       * paid off. Steps: centre, each left-stick direction, then each right. */
      { static int sw = -1; static clock_t t0 = 0;
        if (sw < 0) { const char* e = getenv("PAD_SWEEP");
                      sw = e ? atoi(e) : 0; if (sw < 0) sw = 0; }
        if (sw) {
            static const unsigned char steps[9][4] = {
                {128,128,128,128},
                {128,  0,128,128}, {128,255,128,128},
                {  0,128,128,128}, {255,128,128,128},
                {128,128,128,  0}, {128,128,128,255},
                {128,128,  0,128}, {128,128,255,128},
            };
            if (!t0) t0 = clock();
            long el = (long)((double)(clock() - t0) / (double)CLOCKS_PER_SEC);
            int k = (int)((el / (sw > 0 ? sw : 1)) % 9);
            hs->analog_lx = steps[k][0]; hs->analog_ly = steps[k][1];
            hs->analog_rx = steps[k][2]; hs->analog_ry = steps[k][3];
            hs->connected = 1;
            { static int lastk = -1;
              if (k != lastk) { lastk = k;
                fprintf(stderr, "[PADSWEEP] step %d: l=(%u,%u) r=(%u,%u)%c", k,
                        steps[k][0], steps[k][1], steps[k][2], steps[k][3], 10); } }
        } } }
    data->button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_X] = hs->analog_rx;
    data->button[CELL_PAD_BTN_OFFSET_ANALOG_RIGHT_Y] = hs->analog_ry;
    data->button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_X]  = hs->analog_lx;
    data->button[CELL_PAD_BTN_OFFSET_ANALOG_LEFT_Y]  = hs->analog_ly;

    /* Pressure-sensitive buttons (only meaningful if PRESS_ON) */
    if (setting & CELL_PAD_SETTING_PRESS_ON) {
        data->button[CELL_PAD_BTN_OFFSET_PRESS_RIGHT]    = hs->press_right;
        data->button[CELL_PAD_BTN_OFFSET_PRESS_LEFT]     = hs->press_left;
        data->button[CELL_PAD_BTN_OFFSET_PRESS_UP]       = hs->press_up;
        data->button[CELL_PAD_BTN_OFFSET_PRESS_DOWN]     = hs->press_down;
        data->button[CELL_PAD_BTN_OFFSET_PRESS_TRIANGLE] = hs->press_triangle;
        data->button[CELL_PAD_BTN_OFFSET_PRESS_CIRCLE]   = hs->press_circle;
        data->button[CELL_PAD_BTN_OFFSET_PRESS_CROSS]    = hs->press_cross;
        data->button[CELL_PAD_BTN_OFFSET_PRESS_SQUARE]   = hs->press_square;
        data->button[CELL_PAD_BTN_OFFSET_PRESS_L1]       = hs->press_l1;
        data->button[CELL_PAD_BTN_OFFSET_PRESS_R1]       = hs->press_r1;
        data->button[CELL_PAD_BTN_OFFSET_PRESS_L2]       = hs->trigger_l2;
        data->button[CELL_PAD_BTN_OFFSET_PRESS_R2]       = hs->trigger_r2;
        /* A button whose digital bit is set reads full pressure. Injected
         * input (PAD_FILE/PAD_SCRIPT/PAD_AUTOPRESS, keyboard) only sets the
         * digital bits, and the host values above zeroed any pressure it had
         * written -- so a title keying its menus off the pressure words (GH3,
         * Tornado Outbreak's title screen) never saw the press. */
        static const struct { u8 word; u8 bit; u8 off; } pm[] = {
            { CELL_PAD_BTN_OFFSET_DIGITAL1, 0x20, CELL_PAD_BTN_OFFSET_PRESS_RIGHT },
            { CELL_PAD_BTN_OFFSET_DIGITAL1, 0x80, CELL_PAD_BTN_OFFSET_PRESS_LEFT },
            { CELL_PAD_BTN_OFFSET_DIGITAL1, 0x10, CELL_PAD_BTN_OFFSET_PRESS_UP },
            { CELL_PAD_BTN_OFFSET_DIGITAL1, 0x40, CELL_PAD_BTN_OFFSET_PRESS_DOWN },
            { CELL_PAD_BTN_OFFSET_DIGITAL2, 0x10, CELL_PAD_BTN_OFFSET_PRESS_TRIANGLE },
            { CELL_PAD_BTN_OFFSET_DIGITAL2, 0x20, CELL_PAD_BTN_OFFSET_PRESS_CIRCLE },
            { CELL_PAD_BTN_OFFSET_DIGITAL2, 0x40, CELL_PAD_BTN_OFFSET_PRESS_CROSS },
            { CELL_PAD_BTN_OFFSET_DIGITAL2, 0x80, CELL_PAD_BTN_OFFSET_PRESS_SQUARE },
            { CELL_PAD_BTN_OFFSET_DIGITAL2, 0x04, CELL_PAD_BTN_OFFSET_PRESS_L1 },
            { CELL_PAD_BTN_OFFSET_DIGITAL2, 0x08, CELL_PAD_BTN_OFFSET_PRESS_R1 },
            { CELL_PAD_BTN_OFFSET_DIGITAL2, 0x01, CELL_PAD_BTN_OFFSET_PRESS_L2 },
            { CELL_PAD_BTN_OFFSET_DIGITAL2, 0x02, CELL_PAD_BTN_OFFSET_PRESS_R2 } };
        for (unsigned k = 0; k < sizeof pm / sizeof pm[0]; k++)
            if ((data->button[pm[k].word] & pm[k].bit) && data->button[pm[k].off] < 255)
                data->button[pm[k].off] = 255;
    }

    /* Sensor data (only meaningful if SENSOR_ON): the controller's motion
     * (SDL backend, see pad_sdl_motion), else the at-rest values. */
    if (setting & CELL_PAD_SETTING_SENSOR_ON) {
        const u16* s = hs->motion_live ? hs->sensor : s_sensor_rest;
        data->button[CELL_PAD_BTN_OFFSET_SENSOR_X] = s[0];
        data->button[CELL_PAD_BTN_OFFSET_SENSOR_Y] = s[1];
        data->button[CELL_PAD_BTN_OFFSET_SENSOR_Z] = s[2];
        data->button[CELL_PAD_BTN_OFFSET_SENSOR_G] = s[3];
    }
    }  /* end connected block */

emit:
    /* A non-zero len means "here is a CHANGE packet" -- the constant is literally
     * named CELL_PAD_LEN_CHANGE_DEFAULT. Hardware reports len = 0 once the port
     * has nothing new, and titles DRAIN on that: The Simpsons Arcade Game's
     * input stage is
     *
     *     while (cellPadGetData(port, &d) == CELL_OK && d.len > 0) { ... }
     *
     * (func_0013CFA8, loop 0x13D648 -> 0x13D040). Reporting a change on every
     * call made that loop infinite: the main thread pegged one core inside
     * libpad and the title never got past its first rendered screen.
     *
     * Freshness was finalized before input injection above. Hand out one
     * packet per host report and report len = 0 for any read
     * after that until the next one arrives (pad_poll_backend sets the flag when
     * it actually re-polls, every PAD_POLL_INTERVAL_MS). That is what the
     * hardware does: a pad streams reports at a fixed rate whether or not
     * anything changed, and libpad buffers them.
     *
     * Reporting only on CHANGE instead is wrong and was tried: a HELD button
     * produces exactly one packet, so a title that reads button[] only when
     * len > 0 sees the press for a single frame and then sees nothing. In this
     * game that came out as jump being the only control that seemed to work and
     * attack firing once.
     *
     * PAD_ALWAYS_CHANGE=1 restores the old always-a-packet behaviour. */
    {
        unsigned int ea = (unsigned int)(uintptr_t)data_guest;
        vm_write32((unsigned long long)ea + 0, (unsigned int)data->len);   /* len (s32) */
        for (int _i = 0; _i < CELL_PAD_MAX_CODES; _i++)                    /* button[] u16 */
            vm_write16((unsigned long long)ea + 4 + (unsigned int)_i * 2, data->button[_i]);
    }
    return CELL_OK;
}

/* cellPadGetInfo (v1 API, NID 0x3AAAD464) — used by PSL1GHT's ioPadGetInfo.
 * Layout (RPCS3 cellPad.h CellPadInfo, all big-endian):
 *   +0x00 u32 max_connect        +0x04 u32 now_connect   +0x08 u32 system_info
 *   +0x0C u16 vendor_id[127]     +0x10A u16 product_id[127] +0x208 u8 status[127]
 *   (CELL_MAX_PADS / PSL1GHT MAX_PADS = 127 -- NOT 7; see below)
 * Leaving this unimplemented let the guest read stack garbage as pad state and
 * run off into the weeds before ever reaching gcm init. */
s32 cellPadGetInfo(CellPadInfo2* info)
{
    uint32_t gaddr = (uint32_t)(uintptr_t)info;
    if (!s_pad_initialized)
        return CELL_PAD_ERROR_NOT_OPENED;
    if (!gaddr)
        return CELL_PAD_ERROR_INVALID_PARAMETER;

    pad_poll_backend();

    /* Present a virtual pad on port 0 when nothing physical is attached, the
     * same as cellPadGetInfo2 does.
     *
     * The two entry points disagreed: GetInfo2 carried the fallback and GetInfo
     * did not, so which one a title imports decided whether it believed a
     * controller existed. The Orange Box's chooser imports GetInfo (v1) and
     * never GetInfo2, so it saw zero pads, opened "SIXAXIS wireless controller
     * not detected", took the auto-OK, and opened it again -- forever, before
     * it ever finished booting. Both entry points describe the same hardware
     * and have to answer the same way. */
    u32 connected = 0;
    for (u32 i = 0; i < s_max_connect && i < PAD_MAX_HOST_PORTS; i++)
        if (s_host_state[i].connected || i == 0) connected++;

    vm_write32(gaddr + 0x00, s_max_connect);
    vm_write32(gaddr + 0x04, connected);
    vm_write32(gaddr + 0x08, 0);                       /* system_info */
    /* CellPadInfo (v1) sizes its arrays by CELL_MAX_PADS = 127, not by the 7
     * ports: vendor_id[127] @0x0C, product_id[127] @0x10A, status[127] @0x208.
     * Packed for 7 ports, status landed at 0x28 and GH3 -- which reads
     * info+0x208+port -- saw port 0 disconnected and never polled the pad. */
    for (u32 i = 0; i < CELL_PAD_MAX_PORT_NUM; i++) {
        int on = (i < s_max_connect && i < PAD_MAX_HOST_PORTS &&
                  (s_host_state[i].connected || i == 0));
        vm_write16(gaddr + 0x00C + i * 2, on ? 0x054C : 0);   /* vendor: Sony  */
        vm_write16(gaddr + 0x10A + i * 2, on ? 0x0268 : 0);   /* product: DS3  */
        vm_write8(gaddr + 0x208 + i, (u8)((on ? CELL_PAD_STATUS_CONNECTED : 0) | pad_status_take(i)));
    }
    return CELL_OK;
}

s32 cellPadGetInfo2(CellPadInfo2* info_guest)
{
    if (!s_pad_initialized)
        return CELL_PAD_ERROR_NOT_OPENED;

    if (!info_guest)
        return CELL_PAD_ERROR_INVALID_PARAMETER;

    /* Poll to get latest connection state */
    pad_poll_backend();

    /* Build in a local host struct, then copy to guest memory big-endian. */
    CellPadInfo2 _in; CellPadInfo2* info = &_in;
    memset(info, 0, sizeof(CellPadInfo2));
    info->max_connect = s_max_connect;

    u32 connected = 0;
    for (u32 i = 0; i < s_max_connect && i < PAD_MAX_HOST_PORTS; i++) {
        /* Always present a virtual pad on port 0 (standard emulator behavior) so
         * boot flows that block until a controller is connected can proceed even
         * with no physical device attached. */
        if (s_host_state[i].connected || i == 0) {
            info->port_status[i]       = CELL_PAD_STATUS_CONNECTED | pad_status_take(i);
            info->port_setting[i]      = s_port_setting[i];
            info->device_capability[i] = CELL_PAD_CAPABILITY_PS3_CONFORMITY
                                       | CELL_PAD_CAPABILITY_PRESS_MODE
                                       | CELL_PAD_CAPABILITY_SENSOR_MODE
                                       | CELL_PAD_CAPABILITY_HP_ANALOG_STICK
                                       | CELL_PAD_CAPABILITY_ACTUATOR;
            info->device_type[i]       = CELL_PAD_DEV_TYPE_STANDARD;
            connected++;
        } else {
            info->port_status[i] = CELL_PAD_STATUS_DISCONNECTED | pad_status_take(i);
        }
    }
    info->now_connect = connected;

    /* copy to guest big-endian (CellPadInfo2 is all u32 fields) */
    {
        unsigned int ea = (unsigned int)(uintptr_t)info_guest;
        for (unsigned int _o = 0; _o < sizeof(CellPadInfo2); _o += 4)
            vm_write32((unsigned long long)ea + _o, *(u32*)((char*)info + _o));
    }
    return CELL_OK;
}

/* cellPadPeriphGetInfo -- the peripheral-class view of the pad ports (NID
 * 0x4CC9B68D). LBP polls this during boot; leaving it unresolved returned
 * CELL_OK with the caller's CellPadPeriphInfo left as uninitialised stack
 * garbage, so the game read junk port_status/pclass_type for all 7 ports.
 *
 * Layout (SDK cell/pad/pad_codes.h) -- all u32, so the whole struct marshals
 * big-endian in one sweep like cellPadGetInfo2:
 *   max_connect, now_connect, system_info,
 *   port_status[7], port_setting[7], device_capability[7],
 *   device_type[7], pclass_type[7], pclass_profile[7]
 * We report a standard DUALSHOCK-class pad (pclass_type STANDARD = 0, no
 * profile bits) mirroring the ports cellPadGetInfo2 already reports. */
s32 cellPadPeriphGetInfo(CellPadPeriphInfo* info_guest)
{
    if (!s_pad_initialized)
        return CELL_PAD_ERROR_NOT_OPENED;
    if (!info_guest)
        return CELL_PAD_ERROR_INVALID_PARAMETER;

    pad_poll_backend();

    CellPadPeriphInfo _in; CellPadPeriphInfo* info = &_in;
    memset(info, 0, sizeof(CellPadPeriphInfo));
    info->max_connect = s_max_connect;
    info->system_info = 0;

    u32 connected = 0;
    for (u32 i = 0; i < s_max_connect && i < CELL_PAD_MAX_PORT_NUM; i++) {
        /* Same virtual-pad-on-port-0 rule as cellPadGetInfo2, so both views of
         * the ports agree (a game that cross-checks them must see one truth). */
        if ((i < PAD_MAX_HOST_PORTS && s_host_state[i].connected) || i == 0) {
            info->port_status[i]       = CELL_PAD_STATUS_CONNECTED | pad_status_take(i);
            info->port_setting[i]      = s_port_setting[i];
            info->device_capability[i] = CELL_PAD_CAPABILITY_PS3_CONFORMITY
                                       | CELL_PAD_CAPABILITY_PRESS_MODE
                                       | CELL_PAD_CAPABILITY_SENSOR_MODE
                                       | CELL_PAD_CAPABILITY_HP_ANALOG_STICK
                                       | CELL_PAD_CAPABILITY_ACTUATOR;
            info->device_type[i]       = CELL_PAD_DEV_TYPE_STANDARD;
            info->pclass_type[i]       = CELL_PAD_PCLASS_TYPE_STANDARD;
            info->pclass_profile[i]    = 0;
            connected++;
        } else {
            info->port_status[i] = CELL_PAD_STATUS_DISCONNECTED | pad_status_take(i);
        }
    }
    info->now_connect = connected;

    { unsigned int ea = (unsigned int)(uintptr_t)info_guest;
      for (unsigned int _o = 0; _o < sizeof(CellPadPeriphInfo); _o += 4)
          vm_write32((unsigned long long)ea + _o, *(u32*)((char*)info + _o)); }

    { static int _n = 0;
      if (PS3_LOG_FIRST(_n, 2))
          printf("[cellPad] PeriphGetInfo(max=%u now=%u) -> STANDARD class\n",
                 info->max_connect, connected); }
    return CELL_OK;
}

s32 cellPadSetPortSetting(u32 port_no, u32 port_setting)
{
    printf("[cellPad] SetPortSetting(port=%u, setting=0x%X)\n",
           port_no, port_setting);

    if (!s_pad_initialized)
        return CELL_PAD_ERROR_NOT_OPENED;

    if (port_no >= CELL_PAD_MAX_PORT_NUM)
        return CELL_PAD_ERROR_INVALID_PARAMETER;

    s_port_setting[port_no] = port_setting;
    return CELL_OK;
}

/* cellPadInfoPressMode (NID 0x0E2DFAAD) / cellPadSetPressMode (NID 0xF83F8182)
 *
 * Press mode makes the digital buttons report an analog 0..255 pressure in the
 * extended part of CellPadData. A DS3 supports it, and the virtual pad reported
 * by cellPadGetInfo/GetInfo2 already advertises CELL_PAD_CAPABILITY_PRESS_MODE,
 * so these have to agree with that claim rather than fall through to the
 * unresolved-NID path.
 *
 * Info returns 1 when the port supports press mode; Set toggles it, recorded in
 * the same s_port_setting word cellPadSetPortSetting uses so a title that reads
 * the setting back sees what it asked for. */
s32 cellPadInfoPressMode(u32 port_no)
{
    if (!s_pad_initialized)
        return CELL_PAD_ERROR_NOT_OPENED;
    if (port_no >= CELL_PAD_MAX_PORT_NUM)
        return CELL_PAD_ERROR_INVALID_PARAMETER;
    return 1;                                  /* supported */
}

s32 cellPadSetPressMode(u32 port_no, u32 mode)
{
    if (!s_pad_initialized)
        return CELL_PAD_ERROR_NOT_OPENED;
    if (port_no >= CELL_PAD_MAX_PORT_NUM)
        return CELL_PAD_ERROR_INVALID_PARAMETER;
    if (mode)
        s_port_setting[port_no] |= CELL_PAD_SETTING_PRESS_ON;
    else
        s_port_setting[port_no] &= ~(u32)CELL_PAD_SETTING_PRESS_ON;
    return CELL_OK;
}

/* cellPadInfoSensorMode (NID 0x78200559) / cellPadSetSensorMode (NID 0xBE5BE3BA)
 *
 * The same pair for the SIXAXIS sensors, which the virtual pad also advertises
 * (CELL_PAD_CAPABILITY_SENSOR_MODE). Unresolved, Info answered 0 -- "no
 * sensors" from a pad whose capability word says otherwise -- and Set never
 * turned the sensor block of CellPadData on. With SENSOR_ON the packet grows to
 * CELL_PAD_LEN_CHANGE_SENSOR_ON and carries the at-rest readings (see
 * cellPadGetData), which is what a DS3 lying still reports. */
s32 cellPadInfoSensorMode(u32 port_no)
{
    if (!s_pad_initialized)
        return CELL_PAD_ERROR_NOT_OPENED;
    if (port_no >= CELL_PAD_MAX_PORT_NUM)
        return CELL_PAD_ERROR_INVALID_PARAMETER;
    return 1;                                  /* supported */
}

s32 cellPadSetSensorMode(u32 port_no, u32 mode)
{
    if (!s_pad_initialized)
        return CELL_PAD_ERROR_NOT_OPENED;
    if (port_no >= CELL_PAD_MAX_PORT_NUM)
        return CELL_PAD_ERROR_INVALID_PARAMETER;
    const u32 was = s_port_setting[port_no] & CELL_PAD_SETTING_SENSOR_ON;
    if (mode)
        s_port_setting[port_no] |= CELL_PAD_SETTING_SENSOR_ON;
    else
        s_port_setting[port_no] &= ~(u32)CELL_PAD_SETTING_SENSOR_ON;
    if (was != (s_port_setting[port_no] & CELL_PAD_SETTING_SENSOR_ON))
        printf("[cellPad] SetSensorMode(port %u) -> %s\n", port_no, mode ? "on" : "off");
    return CELL_OK;
}

s32 cellPadGetCapabilityInfo(u32 port_no, CellPadCapabilityInfo* info)
{
    if (!s_pad_initialized)
        return CELL_PAD_ERROR_NOT_OPENED;

    if (port_no >= CELL_PAD_MAX_PORT_NUM || !info)
        return CELL_PAD_ERROR_INVALID_PARAMETER;

    u32 info_ea = (u32)(uintptr_t)info;
    for (u32 i = 0; i < CELL_PAD_MAX_CODES; i++)
        vm_write32(info_ea + i * 4, 0);

    /* Report standard DualShock 3 capabilities */
    vm_write32(info_ea, CELL_PAD_CAPABILITY_PS3_CONFORMITY
                        | CELL_PAD_CAPABILITY_PRESS_MODE
                        | CELL_PAD_CAPABILITY_SENSOR_MODE
                        | CELL_PAD_CAPABILITY_HP_ANALOG_STICK
                        | CELL_PAD_CAPABILITY_ACTUATOR);

    return CELL_OK;
}

s32 cellPadSetActDirect(u32 port_no, CellPadActParam* param)
{
    if (!s_pad_initialized)
        return CELL_PAD_ERROR_NOT_OPENED;

    if (port_no >= CELL_PAD_MAX_PORT_NUM || !param)
        return CELL_PAD_ERROR_INVALID_PARAMETER;

    param = GUEST_PTR(param, CellPadActParam*);

    if (port_no >= PAD_MAX_HOST_PORTS)
        return CELL_OK;

    u16 lo, hi;
    pad_rumble_levels(param->motor[CELL_PAD_ACTUATOR_PARAM_SMALL],
                      param->motor[CELL_PAD_ACTUATOR_PARAM_LARGE], &lo, &hi);
    lo = pad_rumble_scale(lo);   /* PS3_PAD_RUMBLE */
    hi = pad_rumble_scale(hi);
    s_rumble[port_no].lo = lo;
    s_rumble[port_no].hi = hi;

#if PAD_BACKEND_XINPUT
    /* XInput keeps the motors at what they were last set to: no renewal. */
    if (s_host_state[port_no].connected) {
        XINPUT_VIBRATION vib;
        vib.wLeftMotorSpeed  = (WORD)lo;
        vib.wRightMotorSpeed = (WORD)hi;
        if (p_xi_set_state) p_xi_set_state((DWORD)port_no, &vib);
    }
#endif

#if PAD_BACKEND_SDL2
    pad_lock();
    pad_sdl_rumble_send((int)port_no);
    pad_unlock();
#endif

    return CELL_OK;
}

s32 cellPadClearBuf(u32 port_no)
{
    if (!s_pad_initialized)
        return CELL_PAD_ERROR_NOT_OPENED;

    if (port_no >= CELL_PAD_MAX_PORT_NUM)
        return CELL_PAD_ERROR_INVALID_PARAMETER;

    /* Nothing to clear in our implementation -- state is polled fresh */
    return CELL_OK;
}
