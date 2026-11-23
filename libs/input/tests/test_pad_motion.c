/* cellPad's motion on the SDL path: a controller's accelerometer and gyro
 * reach the guest's sensor block as DualShock 3 counts (at rest, +1 g and
 * -1 g on each axis, clamping, yaw), SDL's sensors are enabled only while the
 * guest has sensor mode on and PS3_PAD_MOTION is on, a controller without
 * sensors reads at rest, PS3_PAD_MOTION=off reads at rest, and the shake
 * button (PS3_PAD_SHAKE_BUTTON) makes a shake that PAIN's own detector counts
 * at 15, 30 and 60 game frames per second, then returns to rest on release.
 *
 *   clang -std=gnu17 -I include $(sdl2-config --cflags) -I "$(sdl2-config --prefix)/include" \
 *         libs/input/tests/test_pad_motion.c $(sdl2-config --libs) -lm -o /tmp/test_pad_motion && /tmp/test_pad_motion
 *
 * Host time is a fake clock (as in test_pad_quit.c); no controller is opened:
 * the test defines the SDL calls the backend makes over fake pads.
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

/* A fake controller: buttons as SDL_GameControllerButton bits, which sensors
 * it has, which are enabled, and what they read. */
typedef struct {
    uint32_t buttons;
    int has_accel, has_gyro;
    int accel_on, gyro_on;
    int enable_calls;        /* SDL_GameControllerSetSensorEnabled calls */
    float accel[3], gyro[3];
} FakePad;
static FakePad s_fake[2];
#define FAKE(gc) ((FakePad*)(gc))
#define BIT(b) (1u << (b))

int SDL_GameControllerRumble(SDL_GameController* gc, Uint16 lo, Uint16 hi, Uint32 ms) { (void)gc; (void)lo; (void)hi; (void)ms; return 0; }
void SDL_GameControllerUpdate(void) {}
SDL_bool SDL_IsGameController(int i) { (void)i; return SDL_FALSE; }
SDL_bool SDL_GameControllerGetAttached(SDL_GameController* gc) { (void)gc; return SDL_TRUE; }
Uint8 SDL_GameControllerGetButton(SDL_GameController* gc, SDL_GameControllerButton b) { return (Uint8)((FAKE(gc)->buttons >> b) & 1u); }
Sint16 SDL_GameControllerGetAxis(SDL_GameController* gc, SDL_GameControllerAxis a) { (void)gc; (void)a; return 0; }
void SDL_GameControllerClose(SDL_GameController* gc) { (void)gc; }
SDL_bool SDL_GameControllerHasSensor(SDL_GameController* gc, SDL_SensorType t)
{
    return (t == SDL_SENSOR_ACCEL ? FAKE(gc)->has_accel : t == SDL_SENSOR_GYRO ? FAKE(gc)->has_gyro : 0) ? SDL_TRUE : SDL_FALSE;
}
int SDL_GameControllerSetSensorEnabled(SDL_GameController* gc, SDL_SensorType t, SDL_bool on)
{
    FakePad* f = FAKE(gc);
    f->enable_calls++;
    if (t == SDL_SENSOR_ACCEL && f->has_accel) { f->accel_on = on; return 0; }
    if (t == SDL_SENSOR_GYRO && f->has_gyro) { f->gyro_on = on; return 0; }
    return -1;
}
int SDL_GameControllerGetSensorData(SDL_GameController* gc, SDL_SensorType t, float* data, int n)
{
    FakePad* f = FAKE(gc);
    assert(n == 3);
    if (t == SDL_SENSOR_ACCEL && f->accel_on) { memcpy(data, f->accel, sizeof f->accel); return 0; }
    if (t == SDL_SENSOR_GYRO && f->gyro_on) { memcpy(data, f->gyro, sizeof f->gyro); return 0; }
    return -1;
}

static const float G = SDL_STANDARD_GRAVITY;
static const double PI = 3.14159265358979323846;
static CellPadData* const guest = (CellPadData*)(uintptr_t)0x100;

/* One host report later (at least PAD_POLL_INTERVAL_MS on), port `port`'s
 * packet: its length, and the four sensor words in `s` when it has them. */
static s32 report(unsigned port, unsigned ms, unsigned s[4])
{
    fake_ms += ms;
    assert(cellPadGetData(port, guest) == CELL_OK);
    const s32 len = (s32)vm_read32(0x100);
    for (int k = 0; k < 4; k++) s[k] = vm_read16(0x104 + (CELL_PAD_BTN_OFFSET_SENSOR_X + k) * 2);
    return len;
}
static unsigned digital(void)
{
    return vm_read16(0x104 + CELL_PAD_BTN_OFFSET_DIGITAL1 * 2) | vm_read16(0x104 + CELL_PAD_BTN_OFFSET_DIGITAL2 * 2) << 8;
}
static void expect(const char* what, const unsigned s[4], unsigned x, unsigned y, unsigned z, unsigned g)
{
    if (s[0] != x || s[1] != y || s[2] != z || s[3] != g) {
        printf("FAIL %s: (%u, %u, %u, %u), want (%u, %u, %u, %u)\n", what, s[0], s[1], s[2], s[3], x, y, z, g);
        assert(0);
    }
}

/* The SDL reading -> DS3 counts mapping alone. */
static void conv(const char* what, float ax, float ay, float az, float yaw, unsigned x, unsigned y, unsigned z, unsigned g)
{
    const float a[3] = { ax, ay, az };
    u16 o[4];
    pad_ds3_sensors(a, yaw, o);
    const unsigned s[4] = { o[0], o[1], o[2], o[3] };
    expect(what, s, x, y, z, g);
}
static void test_mapping(void)
{
    conv("flat, at rest", 0, G, 0, 0, 512, 399, 512, 512);
    conv("+1 g on X", G, 0, 0, 0, 399, 512, 512, 512);
    conv("-1 g on X", -G, 0, 0, 0, 625, 512, 512, 512);
    conv("-1 g on Y (upside down)", 0, -G, 0, 0, 512, 625, 512, 512);
    conv("+1 g on Z", 0, 0, G, 0, 512, 512, 399, 512);
    conv("-1 g on Z", 0, 0, -G, 0, 512, 512, 625, 512);
    conv("half a g", G / 2, 0, 0, 0, 456, 512, 512, 512);      /* 512 - 56.5, rounded away from 0 */
    conv("clamped high and low", -6 * G, 6 * G, -4 * G, 0, 1023, 0, 964, 512);
    conv("yaw +90 deg/s", 0, G, 0, (float)(PI / 2), 512, 399, 512, 635);
    conv("yaw -90 deg/s", 0, G, 0, (float)(-PI / 2), 512, 399, 512, 389);
    conv("yaw clamped", 0, G, 0, 20, 512, 399, 512, 1023);
    conv("yaw clamped low", 0, G, 0, -20, 512, 399, 512, 0);
}

static void reset_cfg(void)
{
    s_cfg_motion = -1;
    s_cfg_shake = -2;
}

static void test_env(void)
{
    static const struct { const char* v; int want; } m[] = {
        { NULL, 1 }, { "1", 1 }, { "on", 1 }, { "0", 0 }, { "off", 0 }, { "no", 0 }, { "false", 0 }, { "", 0 },
    };
    for (unsigned k = 0; k < sizeof m / sizeof m[0]; k++) {
        if (m[k].v) setenv("PS3_PAD_MOTION", m[k].v, 1); else unsetenv("PS3_PAD_MOTION");
        reset_cfg();
        assert(pad_motion_enabled() == m[k].want);
    }
    unsetenv("PS3_PAD_MOTION");
    static const struct { const char* v; int want; } b[] = {
        { NULL, -1 }, { "off", -1 }, { "l3", SDL_CONTROLLER_BUTTON_LEFTSTICK },
        { "r3", SDL_CONTROLLER_BUTTON_RIGHTSTICK }, { "touchpad", SDL_CONTROLLER_BUTTON_TOUCHPAD },
        { "R3", -1 }, { "x", -1 }, { "", -1 },
    };
    for (unsigned k = 0; k < sizeof b / sizeof b[0]; k++) {
        if (b[k].v) setenv("PS3_PAD_SHAKE_BUTTON", b[k].v, 1); else unsetenv("PS3_PAD_SHAKE_BUTTON");
        reset_cfg();
        assert(pad_shake_button() == b[k].want);
    }
    unsetenv("PS3_PAD_SHAKE_BUTTON");
    reset_cfg();
}

/* A DualShock 4: sensors only while the guest has sensor mode on. */
static void test_sensor_gating(void)
{
    FakePad* f = &s_fake[0];
    memset(f, 0, sizeof *f);
    f->has_accel = f->has_gyro = 1;
    f->accel[0] = G / 2; f->accel[1] = G * 0.866f;        /* tilted 30 degrees */
    s_sdl_controllers[0] = (SDL_GameController*)f;
    unsigned s[4];

    assert(cellPadSetPortSetting(0, CELL_PAD_SETTING_PRESS_ON) == CELL_OK);
    for (int n = 0; n < 5; n++) assert(report(0, 10, s) == CELL_PAD_LEN_CHANGE_PRESS_ON);
    assert(f->enable_calls == 0 && !f->accel_on && !f->gyro_on);   /* never asked for */
    assert(s[0] == 0 && s[1] == 0 && s[2] == 0 && s[3] == 0);      /* no sensor block */

    assert(cellPadSetPortSetting(0, CELL_PAD_SETTING_PRESS_ON | CELL_PAD_SETTING_SENSOR_ON) == CELL_OK);
    assert(report(0, 10, s) == CELL_PAD_LEN_CHANGE_SENSOR_ON);
    assert(f->accel_on && f->gyro_on && f->enable_calls == 2);
    expect("tilted 30 degrees about Z", s, 456, 414, 512, 512);
    f->accel[0] = 0; f->accel[1] = G; f->gyro[1] = (float)(PI / 2);
    report(0, 10, s);
    expect("flat, turning at 90 deg/s", s, 512, 399, 512, 635);
    f->accel[0] = 0; f->accel[1] = 0; f->accel[2] = G; f->gyro[1] = 0;
    report(0, 10, s);
    expect("standing on its end, +1 g on Z", s, 512, 512, 399, 512);
    f->accel[2] = 0; f->accel[1] = 6 * G;
    report(0, 10, s);
    expect("a 6 g jolt clamps", s, 512, 0, 512, 512);
    /* No report yet from the sensor (all zero): at rest, not free fall. */
    f->accel[1] = 0;
    report(0, 10, s);
    expect("before the first sensor report", s, 512, 399, 512, 512);
    assert(f->enable_calls == 2);                                   /* asked once, not per poll */

    /* Sensor mode off again: SDL's sensors are switched off. */
    assert(cellPadSetPortSetting(0, CELL_PAD_SETTING_PRESS_ON) == CELL_OK);
    assert(report(0, 10, s) == CELL_PAD_LEN_CHANGE_PRESS_ON);
    assert(!f->accel_on && !f->gyro_on && f->enable_calls == 4);
    assert(s[0] == 0 && s[1] == 0);
    for (int n = 0; n < 3; n++) report(0, 10, s);
    assert(f->enable_calls == 4);

    /* A controller with an accelerometer and no gyro: the gyro reads still. */
    memset(f, 0, sizeof *f);
    f->has_accel = 1;
    f->accel[1] = -G;
    pad_sdl_motion_reset(0);
    assert(cellPadSetPortSetting(0, CELL_PAD_SETTING_SENSOR_ON) == CELL_OK);
    assert(report(0, 10, s) == CELL_PAD_LEN_CHANGE_SENSOR_ON);
    expect("accelerometer only, upside down", s, 512, 625, 512, 512);
    assert(cellPadSetPortSetting(0, 0) == CELL_OK);
    report(0, 10, s);
}

/* PS3_PAD_MOTION=off: at rest whatever the controller reads, and its sensors
 * are never switched on. */
static void test_motion_off(void)
{
    FakePad* f = &s_fake[0];
    memset(f, 0, sizeof *f);
    f->has_accel = f->has_gyro = 1;
    f->accel[0] = G; f->gyro[1] = 1;
    s_sdl_controllers[0] = (SDL_GameController*)f;
    pad_sdl_motion_reset(0);
    setenv("PS3_PAD_MOTION", "off", 1);
    reset_cfg();
    unsigned s[4];
    assert(cellPadSetPortSetting(0, CELL_PAD_SETTING_PRESS_ON | CELL_PAD_SETTING_SENSOR_ON) == CELL_OK);
    for (int n = 0; n < 5; n++) {
        assert(report(0, 10, s) == CELL_PAD_LEN_CHANGE_SENSOR_ON);
        expect("PS3_PAD_MOTION=off", s, 512, 399, 512, 512);
    }
    assert(f->enable_calls == 0 && !f->accel_on);
    unsetenv("PS3_PAD_MOTION");
    reset_cfg();
    assert(cellPadSetPortSetting(0, 0) == CELL_OK);
    report(0, 10, s);
}

/* An Xbox pad: no sensors, so at rest, and nothing is switched on. */
static void test_no_sensors(void)
{
    FakePad* f = &s_fake[1];
    memset(f, 0, sizeof *f);
    s_sdl_controllers[1] = (SDL_GameController*)f;
    unsigned s[4];
    assert(cellPadSetPortSetting(1, CELL_PAD_SETTING_PRESS_ON | CELL_PAD_SETTING_SENSOR_ON) == CELL_OK);
    for (int n = 0; n < 5; n++) {
        assert(report(1, 10, s) == CELL_PAD_LEN_CHANGE_SENSOR_ON);
        expect("no sensors", s, 512, 399, 512, 512);
    }
    assert(f->enable_calls == 0);
    assert(cellPadSetPortSetting(1, 0) == CELL_OK);
}

/* PAIN's shake detector, as lifted: per sample, sensor Y goes through the
 * game's 2nd-order low-pass (coefficient set 0, a Butterworth at a quarter of
 * the sample rate, set up by func_004ABD00 after cellPadSetPortSetting and run
 * by func_004ABD68: Q14 feedback, Q18 output, +512, clamped 0..1023), then
 * func_0021D8EC keeps |Y - previous Y| for the last four samples and counts
 * up while their mean is at least 20 (float at 0xC6F164), else back to 0. */
typedef struct { int32_t s1, s2; float prev, ring[4]; int idx, count; } Detector;
static void detect(Detector* d, unsigned y)
{
    static const int32_t c[5] = { 0, -2811, 4798, 9597, 4798 };   /* 0xA479D4 in BCES00548 */
    const int32_t x = ((int32_t)y - 512) * (1 << 18);
    const int32_t w = (int32_t)((int64_t)x + (int64_t)d->s1 * c[0] + (int64_t)d->s2 * c[1] + 0x2000) >> 14;
    int32_t out = (int32_t)(((int64_t)d->s1 * c[3] + (int64_t)d->s2 * c[4] + (int64_t)w * c[2] + 0x2000) >> 18) + 512;
    d->s2 = d->s1; d->s1 = w;
    if (out < 0) out = 0;
    if (out > 1023) out = 1023;
    const float delta = fabsf((float)out - d->prev);
    d->prev = (float)out;
    d->ring[d->idx] = delta;
    d->idx = (d->idx + 1) & 3;
    const float mean = (d->ring[0] + d->ring[1] + d->ring[2] + d->ring[3]) * 0.25f;
    d->count = mean >= 20.0f ? d->count + 1 : 0;
}

/* Hold the shake button for 2 s at `fps` game frames per second, on `port`:
 * the detector counts every frame once it has started, and never drops back. */
static void shake_at(unsigned port, unsigned fps, uint32_t button)
{
    FakePad* f = &s_fake[port];
    Detector d;
    memset(&d, 0, sizeof d);
    unsigned s[4];
    uint64_t t0 = fake_ms;
    int frame = 0;
#define NEXT_FRAME() do { frame++; fake_ms = t0 + (uint64_t)frame * 1000u / fps; \
                          assert(report(port, 0, s) == CELL_PAD_LEN_CHANGE_SENSOR_ON); } while (0)
    for (int n = 0; n < 2 * (int)fps; n++) { NEXT_FRAME(); detect(&d, s[1]); }
    assert(d.count == 0);                       /* still: never a shake */
    f->buttons |= button;
    int started = -1, lo = 1023, hi = 0;
    for (int n = 0; n < 2 * (int)fps; n++) {
        NEXT_FRAME();
        assert(s[0] == 512 && s[2] == 512 && s[3] == 512);   /* only Y moves */
        if (n == 0) assert(s[1] == 399);                      /* each press starts at rest */
        if ((int)s[1] < lo) lo = (int)s[1];
        if ((int)s[1] > hi) hi = (int)s[1];
        detect(&d, s[1]);
        if (started < 0 && d.count > 0) started = n;
        if (started >= 0 && d.count != n - started + 1) {
            printf("FAIL shake at %u fps: detector count %d at frame %d, started at %d\n", fps, d.count, n, started);
            assert(0);
        }
    }
    if (started < 0 || started > 6) { printf("FAIL shake at %u fps: started at frame %d\n", fps, started); assert(0); }
    assert(lo >= 399 - 3 * 113 && hi <= 399 + 3 * 113 && hi - lo > 3 * 113);
    if (button == BIT(SDL_CONTROLLER_BUTTON_RIGHTSTICK)) assert(digital() & CELL_PAD_CTRL_R3);   /* its press still reported */
    if (button == BIT(SDL_CONTROLLER_BUTTON_LEFTSTICK)) assert(digital() & CELL_PAD_CTRL_L3);
    f->buttons &= ~button;
    NEXT_FRAME();
    expect("released: rest at once", s, 512, 399, 512, 512);
    detect(&d, s[1]);
    for (int n = 0; n < 8; n++) { NEXT_FRAME(); detect(&d, s[1]); }
    assert(d.count == 0);
    printf("shake at %u fps: counted from frame %d, Y %d..%d\n", fps, started, lo, hi);
#undef NEXT_FRAME
}

static void test_shake(void)
{
    /* Xbox pad on port 1 (no sensors), R3 the shake button. */
    setenv("PS3_PAD_SHAKE_BUTTON", "r3", 1);
    reset_cfg();
    memset(&s_fake[1], 0, sizeof s_fake[1]);
    s_sdl_controllers[1] = (SDL_GameController*)&s_fake[1];
    assert(cellPadSetPortSetting(1, CELL_PAD_SETTING_PRESS_ON | CELL_PAD_SETTING_SENSOR_ON) == CELL_OK);
    shake_at(1, 30, BIT(SDL_CONTROLLER_BUTTON_RIGHTSTICK));
    shake_at(1, 60, BIT(SDL_CONTROLLER_BUTTON_RIGHTSTICK));
    shake_at(1, 15, BIT(SDL_CONTROLLER_BUTTON_RIGHTSTICK));
    /* L3 does nothing while R3 is the button. */
    unsigned s[4];
    s_fake[1].buttons = BIT(SDL_CONTROLLER_BUTTON_LEFTSTICK);
    for (int n = 0; n < 10; n++) { report(1, 33, s); expect("L3 is not the shake button", s, 512, 399, 512, 512); }
    s_fake[1].buttons = 0;

    /* A DualShock 4 on port 0 with real sensors and its touchpad as the button:
     * the shake replaces the sensors while held, then they come back. */
    setenv("PS3_PAD_SHAKE_BUTTON", "touchpad", 1);
    reset_cfg();
    FakePad* f = &s_fake[0];
    memset(f, 0, sizeof *f);
    f->has_accel = f->has_gyro = 1;
    f->accel[1] = G;
    s_sdl_controllers[0] = (SDL_GameController*)f;
    pad_sdl_motion_reset(0);
    assert(cellPadSetPortSetting(0, CELL_PAD_SETTING_PRESS_ON | CELL_PAD_SETTING_SENSOR_ON) == CELL_OK);
    shake_at(0, 30, BIT(SDL_CONTROLLER_BUTTON_TOUCHPAD));
    assert(f->accel_on);
    f->accel[1] = -G;
    report(0, 33, s);
    expect("real sensors back after the shake", s, 512, 625, 512, 512);

    /* L3, and with PS3_PAD_MOTION=off the button still shakes. */
    setenv("PS3_PAD_SHAKE_BUTTON", "l3", 1);
    setenv("PS3_PAD_MOTION", "off", 1);
    reset_cfg();
    shake_at(1, 30, BIT(SDL_CONTROLLER_BUTTON_LEFTSTICK));

    /* Off (the default): holding R3 or the touchpad does nothing. */
    unsetenv("PS3_PAD_SHAKE_BUTTON");
    unsetenv("PS3_PAD_MOTION");
    reset_cfg();
    s_fake[1].buttons = BIT(SDL_CONTROLLER_BUTTON_RIGHTSTICK) | BIT(SDL_CONTROLLER_BUTTON_TOUCHPAD) |
                        BIT(SDL_CONTROLLER_BUTTON_LEFTSTICK);
    for (int n = 0; n < 10; n++) { report(1, 33, s); expect("no shake button", s, 512, 399, 512, 512); }
    s_fake[1].buttons = 0;

    /* Sensor mode off: the shake is not sent either (no sensor block). */
    setenv("PS3_PAD_SHAKE_BUTTON", "r3", 1);
    reset_cfg();
    assert(cellPadSetPortSetting(1, CELL_PAD_SETTING_PRESS_ON) == CELL_OK);
    s_fake[1].buttons = BIT(SDL_CONTROLLER_BUTTON_RIGHTSTICK);
    for (int n = 0; n < 10; n++) {
        assert(report(1, 33, s) == CELL_PAD_LEN_CHANGE_PRESS_ON);
        assert(s[0] == 0 && s[1] == 0 && s[2] == 0 && s[3] == 0);
    }
    s_fake[1].buttons = 0;
    unsetenv("PS3_PAD_SHAKE_BUTTON");
    reset_cfg();
}

int main(void)
{
    vm_base = mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    assert(vm_base != MAP_FAILED);
    ppu_vm_size = 0x10000;
    setenv("PAD_NO_KEYBOARD", "1", 1);
    unsetenv("PS3_PAD_MOTION");
    unsetenv("PS3_PAD_SHAKE_BUTTON");
    s_pad_initialized = 1;
    s_max_connect = 4;
    test_mapping();
    test_env();
    test_sensor_gating();
    test_motion_off();
    test_no_sensors();
    test_shake();
    printf("test_pad_motion: ok\n");
    return 0;
}
