/*
 * ps3recomp - the GPU vertex decode on Metal
 *
 * rsx_vertex_pull_hlsl through the same rewrite, translation and safe-math
 * compile the backend uses, reading blocks laid out by
 * rsx_vertex_packed_triangles_write: every pixel is one triangle's decoded
 * ATTR0 as raw bits, compared with rsx_vertex_decode_element. Each row is
 * one packet: one format and component count, an unaligned offset, an index
 * order that is not monotonic, and a base index that wraps on every other row.
 *
 * macOS only; needs a Metal device and the HLSL -> MSL translation.
 */
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rsx_shader_msl.h"
#include "rsx_vertex_compact.h"
#include "rsx_vertex_formats.h"

enum { WIDTH = 24, ROWS = 32, CORNERS = WIDTH * 3, INDEX_START = 8192,
       ATTR_STRIDE = 24, PACKET_STRIDE = 4096 };

/* A stand-in guest program: places each triangle by its element and passes
 * ATTR0 through unconverted. */
static const char vertex_hlsl[] =
    "struct VSInput { float4 a0:ATTR0; };\n"
    "struct VSOutput { float4 pos:SV_Position; nointerpolation uint4 value:TEXCOORD0; };\n"
    "VSOutput main(VSInput input) {\n"
    "    uint4 header = rsx_pull_packet.Load4(0);\n"
    "    uint element = ((vertex_id + header.y) & 0xfffffu) - header.x;\n"
    "    uint corner = element % 3u;\n"
    "    float x = float(element / 3u) + 0.5 + (corner == 0u ? -0.4 : corner == 1u ? 0.4 : 0.0);\n"
    "    float y = 0.5 + (corner == 2u ? 0.4 : -0.4);\n"
    "    VSOutput result;\n"
    "    result.pos = float4(x * (2.0 / 24.0) - 1.0, 1.0 - y * 2.0, 0.0, 1.0);\n"
    "    result.value = asuint(input.a0);\n"
    "    return result;\n"
    "}\n";

static const char fragment_hlsl[] =
    "struct PSInput { float4 pos:SV_Position; nointerpolation uint4 value:TEXCOORD0; };\n"
    "uint4 main(PSInput input):SV_Target0 { return input.value; }\n";

static u32 random_bits(void)
{
    static u32 seed = 0x8cc79af3u;
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;
    return seed;
}

static id<MTLFunction> compile(id<MTLDevice> device, const char* hlsl, int stage)
{
    static char msl[512 * 1024];
    char diagnostic[4096];
    if (rsx_hlsl_to_msl(hlsl, stage, msl, sizeof msl, diagnostic, sizeof diagnostic)) {
        fprintf(stderr, "translation failed: %s\n", diagnostic);
        return nil;
    }
    if (stage == RSX_SHADER_STAGE_VERTEX && !strstr(msl, "[[buffer(28)]]")) {
        fprintf(stderr, "the packet is not at buffer 28\n");
        return nil;
    }
    /* The backend's guest programs compile with safe math; so does this. */
    MTLCompileOptions* options = [MTLCompileOptions new];
    options.fastMathEnabled = NO;
    NSError* error = nil;
    id<MTLLibrary> library = [device newLibraryWithSource:[NSString stringWithUTF8String:msl]
                                                  options:options error:&error];
    if (!library) fprintf(stderr, "%s\n", error.localizedDescription.UTF8String);
    return [library newFunctionWithName:@"main0"];
}

static rsx_packed_triangles g_packet;
static char g_pull_hlsl[64 * 1024];

int main(void)
{
    rsx_vertex_layout_plan layout;
    rsx_vertex_layout_plan_init(&layout, 1u);
    if (!rsx_vertex_pull_shader(vertex_hlsl, &layout, g_pull_hlsl, sizeof g_pull_hlsl)) {
        fprintf(stderr, "rewrite failed\n");
        return 2;
    }

    @autoreleasepool {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        id<MTLCommandQueue> queue = [device newCommandQueue];
        MTLRenderPipelineDescriptor* pd = [MTLRenderPipelineDescriptor new];
        pd.vertexFunction = compile(device, g_pull_hlsl, RSX_SHADER_STAGE_VERTEX);
        pd.fragmentFunction = compile(device, fragment_hlsl, RSX_SHADER_STAGE_FRAGMENT);
        pd.colorAttachments[0].pixelFormat = MTLPixelFormatRGBA32Uint;
        NSError* error = nil;
        id<MTLRenderPipelineState> pipeline = pd.vertexFunction && pd.fragmentFunction
            ? [device newRenderPipelineStateWithDescriptor:pd error:&error] : nil;
        if (!queue || !pipeline) {
            fprintf(stderr, "pipeline creation failed: %s\n",
                    error ? error.localizedDescription.UTF8String : "missing shader");
            return 2;
        }
        id<MTLBuffer> packets = [device newBufferWithLength:ROWS * PACKET_STRIDE
                                                    options:MTLResourceStorageModeShared];
        MTLTextureDescriptor* td = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA32Uint
                                         width:WIDTH height:ROWS mipmapped:NO];
        td.storageMode = MTLStorageModeShared;
        td.usage = MTLTextureUsageRenderTarget;
        id<MTLTexture> target = [device newTextureWithDescriptor:td];
        if (!packets || !target) return 2;

        static u32 expected[ROWS * WIDTH][4];
        u32 index_offset[ROWS];
        for (u32 row = 0; row < ROWS; row++) {
            rsx_packed_triangles* p = &g_packet;
            memset(p, 0, sizeof *p);
            const u32 base = row & 1u ? 0xFFF40u : 19u + row;
            const u32 type = row % 8u, size = type == 6u ? 1u : row / 8u + 1u;
            const u32 offset = 1u + row % 3u;
            p->base_index = base;
            p->first = (INDEX_START + base) & 0xFFFFFu;
            p->last = p->first + CORNERS - 1u;
            p->count = CORNERS;
            p->slots = 1;
            p->attributes[0] = (rsx_packed_attribute){ offset, ATTR_STRIDE, type, size,
                { random_bits(), random_bits(), random_bits(), random_bits() } };
            /* Triangle i is drawn at x = i * 5 % WIDTH; its three corners
             * carry the same bytes so the flat value is the decode. */
            for (u32 i = 0; i < WIDTH; i++)
                for (u32 corner = 0; corner < 3; corner++)
                    p->indices[3 * i + corner] =
                        (u16)(INDEX_START + (i * 5u % WIDTH) * 3u + corner);
            for (u32 x = 0; x < WIDTH; x++) {
                u8* data = p->data + offset + x * 3u * ATTR_STRIDE;
                for (u32 byte = 0; byte < 16; byte++) data[byte] = (u8)random_bits();
                memcpy(data + ATTR_STRIDE, data, 16);
                memcpy(data + 2 * ATTR_STRIDE, data, 16);
                float decoded[4];
                if (type && !rsx_vertex_decode_element(type, size, data, decoded)) return 2;
                memcpy(expected[row * WIDTH + x],
                       type ? (const void*)decoded : (const void*)p->attributes[0].defaults, 16);
            }
            p->bytes = offset + (CORNERS - 1u) * ATTR_STRIDE + 16u;
            const u32 bytes = rsx_vertex_packed_triangles_size(p, &index_offset[row]);
            if (bytes > PACKET_STRIDE) return 2;
            rsx_vertex_packed_triangles_write(p, (u8*)packets.contents + row * PACKET_STRIDE);
        }

        MTLRenderPassDescriptor* pass = [MTLRenderPassDescriptor renderPassDescriptor];
        pass.colorAttachments[0].texture = target;
        pass.colorAttachments[0].loadAction = MTLLoadActionClear;
        pass.colorAttachments[0].storeAction = MTLStoreActionStore;
        pass.colorAttachments[0].clearColor = MTLClearColorMake(44, 55, 66, 77);
        id<MTLCommandBuffer> cb = [queue commandBuffer];
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:pass];
        [enc setRenderPipelineState:pipeline];
        for (u32 row = 0; row < ROWS; row++) {
            [enc setViewport:(MTLViewport){ 0, row, WIDTH, 1, 0, 1 }];
            [enc setVertexBuffer:packets offset:row * PACKET_STRIDE atIndex:28];
            [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle indexCount:CORNERS
                             indexType:MTLIndexTypeUInt16 indexBuffer:packets
                     indexBufferOffset:row * PACKET_STRIDE + index_offset[row]];
        }
        [enc endEncoding];
        [cb commit];
        [cb waitUntilCompleted];
        if (cb.status != MTLCommandBufferStatusCompleted) {
            fprintf(stderr, "%s\n", cb.error.localizedDescription.UTF8String);
            return 2;
        }
        static u32 actual[ROWS * WIDTH][4];
        [target getBytes:actual bytesPerRow:WIDTH * 16u
              fromRegion:MTLRegionMake2D(0, 0, WIDTH, ROWS) mipmapLevel:0];
        for (u32 pixel = 0; pixel < ROWS * WIDTH; pixel++)
            for (u32 channel = 0; channel < 4; channel++)
                if (actual[pixel][channel] != expected[pixel][channel]) {
                    const u32 row = pixel / WIDTH;
                    fprintf(stderr, "[FAIL] type %u size %u x %u channel %u: CPU %08x, Metal %08x\n",
                            row % 8u, row % 8u == 6u ? 1u : row / 8u + 1u, pixel % WIDTH,
                            channel, expected[pixel][channel], actual[pixel][channel]);
                    return 1;
                }
        printf("[PASS] %u packed vertices decoded on the GPU match the CPU decode bit for bit, "
               "every format and component count\n", ROWS * WIDTH);
    }
    return 0;
}
