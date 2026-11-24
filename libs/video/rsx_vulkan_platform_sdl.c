/* Linux Vulkan loader and SDL2 window integration for the RSX backend. */
#define VK_NO_PROTOTYPES
#include "rsx_vulkan_platform.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_vulkan.h>
#include <dlfcn.h>
#include <stdlib.h>

#define VK_LOG RSX_VK_LOG

static void* s_loader;
static SDL_Window* s_window;

PFN_vkGetInstanceProcAddr rsx_vk_platform_load(void)
{
    static const char* names[] = { "libvulkan.so.1", "libvulkan.so" };
    for (size_t i = 0; i < sizeof names / sizeof names[0] && !s_loader; i++)
        s_loader = dlopen(names[i], RTLD_NOW | RTLD_LOCAL);
    if (!s_loader) {
        VK_LOG("no Vulkan loader found (dlopen libvulkan.so.1: %s)\n", dlerror());
        return NULL;
    }
    PFN_vkGetInstanceProcAddr get_proc =
        (PFN_vkGetInstanceProcAddr)dlsym(s_loader, "vkGetInstanceProcAddr");
    if (!get_proc) VK_LOG("libvulkan has no vkGetInstanceProcAddr\n");
    return get_proc;
}

void rsx_vk_platform_unload(void)
{
    if (s_loader) dlclose(s_loader);
    s_loader = NULL;
}

static int vk_env_on(const char* name)
{
    const char* v = getenv(name);
    return v && v[0] && v[0] != '0';
}

/* Auto follows the desktop. Explicit sizes request an exact fullscreen
 * display mode only when the window's current display advertises one. */
static int vk_window_fullscreen(SDL_Window* window)
{
    const char* setting = getenv("PS3RECOMP_FULLSCREEN_SIZE");
    unsigned w, h;
    char extra;
    if (setting && sscanf(setting, "%ux%u%c", &w, &h, &extra) == 2 &&
        w >= 160 && h >= 90 && w <= 16384 && h <= 16384) {
        const int display = SDL_GetWindowDisplayIndex(window);
        SDL_DisplayMode requested = { .w = (int)w, .h = (int)h };
        SDL_DisplayMode closest;
        if (display >= 0 && SDL_GetClosestDisplayMode(display, &requested, &closest) &&
            closest.w == requested.w && closest.h == requested.h &&
            SDL_SetWindowDisplayMode(window, &closest) == 0 &&
            SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN) == 0)
            return 0;
        SDL_SetWindowDisplayMode(window, NULL);
        VK_LOG("fullscreen %ux%u unavailable; using desktop mode\n", w, h);
    }
    return SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);
}

int rsx_vk_platform_output_open(const char* title, uint32_t default_width,
                                uint32_t default_height, rsx_vk_output* output)
{
    if (!vk_env_on("PS3RECOMP_VK_WINDOW")) return -1;
    SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");   /* SIGTERM/SIGINT: ps3_host_quit_on_signals */
    if (SDL_InitSubSystem(SDL_INIT_VIDEO) != 0) {
        VK_LOG("window disabled, continuing headless: SDL video init failed: %s\n", SDL_GetError());
        return -1;
    }
    /* Resizable and maximizable; F11 or Alt+Enter switches to fullscreen and
     * back. The frame is fitted to the swapchain extent by vk_window_fit. */
    const char* size = getenv("PS3RECOMP_WINDOW_SIZE");
    unsigned width = default_width, height = default_height;
    unsigned w, h;
    if (size && sscanf(size, "%ux%u", &w, &h) == 2 &&
        w >= 160 && h >= 90 && w <= 16384 && h <= 16384) {
        width = w;
        height = h;
    }
    const Uint32 flags = SDL_WINDOW_VULKAN | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE;
    const int fullscreen = vk_env_on("PS3RECOMP_FULLSCREEN") || vk_env_on("PS3RECOMP_VK_FULLSCREEN");
    s_window = SDL_CreateWindow(title && title[0] ? title : "ps3recomp",
                                SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                (int)width, (int)height, flags);
    if (!s_window) {
        char why[256];
        snprintf(why, sizeof why, "SDL_CreateWindow failed: %s", SDL_GetError());
        VK_LOG("window disabled, continuing headless: %s\n", why);
        return -1;
    }
    SDL_SetWindowMinimumSize(s_window, 320, 180);
    if (fullscreen) {
        if (vk_window_fullscreen(s_window) == 0) SDL_ShowCursor(SDL_DISABLE);
        else VK_LOG("fullscreen failed; keeping window: %s\n", SDL_GetError());
    }
    unsigned n = 0;
    if (!SDL_Vulkan_GetInstanceExtensions(s_window, &n, NULL) || n == 0 ||
        n > sizeof output->extensions / sizeof output->extensions[0] ||
        !SDL_Vulkan_GetInstanceExtensions(s_window, &n, output->extensions)) {
        char why[256];
        snprintf(why, sizeof why, "no Vulkan surface extensions (%s)", SDL_GetError());
        VK_LOG("window disabled, continuing headless: %s\n", why);
        rsx_vk_platform_output_close();
        return -1;
    }
    output->extension_count = n;
    output->surface_per_device = 0;
    VK_LOG("window: SDL video driver '%s', %u surface extension(s)\n",
           SDL_GetCurrentVideoDriver(), n);
    return 0;
}

void rsx_vk_platform_output_close(void)
{
    if (s_window) {
        SDL_DestroyWindow(s_window);
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
        s_window = NULL;
    }
}

int rsx_vk_platform_create_surface(PFN_vkGetInstanceProcAddr get_proc,
                                  VkInstance instance, VkPhysicalDevice physical_device,
                                  VkSurfaceKHR* surface, char* why, size_t why_size)
{
    (void)get_proc;
    (void)physical_device;
    if (SDL_Vulkan_CreateSurface(s_window, instance, surface)) return 0;
    snprintf(why, why_size, "SDL_Vulkan_CreateSurface failed: %s", SDL_GetError());
    return -1;
}

VkExtent2D rsx_vk_platform_drawable_extent(VkPhysicalDevice physical_device)
{
    (void)physical_device;
    int w = 0, h = 0;
    SDL_Vulkan_GetDrawableSize(s_window, &w, &h);
    return (VkExtent2D){ (uint32_t)w, (uint32_t)h };
}

int rsx_vk_platform_output_hidden(void)
{
    int w = 0, h = 0;
    if (SDL_GetWindowFlags(s_window) & SDL_WINDOW_MINIMIZED) return 1;
    SDL_Vulkan_GetDrawableSize(s_window, &w, &h);
    return w <= 0 || h <= 0;
}

int rsx_vk_platform_pump_messages(int* resized)
{
    int quit = 0;
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        if (e.type == SDL_QUIT) quit = 1;
        else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) quit = 1;
        else if (e.type == SDL_KEYDOWN && !e.key.repeat && s_window &&
                 (e.key.keysym.sym == SDLK_F11 ||
                  (e.key.keysym.sym == SDLK_RETURN && (e.key.keysym.mod & KMOD_ALT)))) {
            const int full = (SDL_GetWindowFlags(s_window) & SDL_WINDOW_FULLSCREEN) != 0;
            if ((full ? SDL_SetWindowFullscreen(s_window, 0) : vk_window_fullscreen(s_window)) == 0)
                SDL_ShowCursor(full ? SDL_ENABLE : SDL_DISABLE);
            *resized = 1;
        } else if (e.type == SDL_WINDOWEVENT &&
                   (e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED || e.window.event == SDL_WINDOWEVENT_RESTORED ||
                    e.window.event == SDL_WINDOWEVENT_MAXIMIZED)) {
            *resized = 1;
        }
    }
    return quit ? -1 : 0;
}

uint32_t rsx_vk_platform_ticks(void)
{
    return SDL_GetTicks();
}

VkResult rsx_vk_platform_present(PFN_vkQueuePresentKHR present, VkQueue queue,
                                const VkPresentInfoKHR* info)
{
    return present(queue, info);
}
