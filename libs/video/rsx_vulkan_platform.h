/* Internal loader and WSI interface. Link one implementation with the
 * singleton RSX Vulkan backend. The backend owns all Vulkan objects. */
#ifndef RSX_VULKAN_PLATFORM_H
#define RSX_VULKAN_PLATFORM_H

#include <stdint.h>
#include <stdio.h>
#include <vulkan/vulkan.h>

#define RSX_VK_LOG(...) fprintf(stderr, "[RSX vulkan] " __VA_ARGS__)

typedef struct {
    const char* extensions[16];
    uint32_t extension_count;
    /* An SDL surface is shared by device candidates. VK_KHR_display needs
     * a separate surface from a mode/plane of each physical device. */
    int surface_per_device;
} rsx_vk_output;

PFN_vkGetInstanceProcAddr rsx_vk_platform_load(void);
void rsx_vk_platform_unload(void);

/* Return 0 for an available output, -1 to continue offscreen. Extension
 * strings must remain valid until output_close. */
int rsx_vk_platform_output_open(const char* title, uint32_t width, uint32_t height,
                                rsx_vk_output* output);
void rsx_vk_platform_output_close(void);

/* Pass VK_NULL_HANDLE as physical_device for a shared surface. Return 0
 * with a valid surface or -1 with a message in why. The backend destroys
 * the surface before closing the native output. */
int rsx_vk_platform_create_surface(PFN_vkGetInstanceProcAddr get_proc,
                                  VkInstance instance, VkPhysicalDevice physical_device,
                                  VkSurfaceKHR* surface, char* why, size_t why_size);

VkExtent2D rsx_vk_platform_drawable_extent(VkPhysicalDevice physical_device);
int rsx_vk_platform_output_hidden(void);
int rsx_vk_platform_pump_messages(int* resized);
uint32_t rsx_vk_platform_ticks(void);

/* The backend holds its queue lock and handles swapchain loss. */
VkResult rsx_vk_platform_present(PFN_vkQueuePresentKHR present, VkQueue queue,
                                const VkPresentInfoKHR* info);

#endif
