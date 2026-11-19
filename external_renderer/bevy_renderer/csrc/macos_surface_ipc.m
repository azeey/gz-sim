#import <CoreFoundation/CoreFoundation.h>
#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>
#import <IOSurface/IOSurface.h>
#import <Metal/Metal.h>

#include <mach/mach.h>
#include <pthread.h>
#include <servers/bootstrap.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SWAPCHAIN_SLOTS 3
#define MSG_ID_FETCH_SWAPCHAIN_REQ 0x475A0001
#define MSG_ID_FETCH_SWAPCHAIN_REP 0x475A0002
#define MSG_ID_PING_REQ 0x475A0003
#define MSG_ID_PING_REP 0x475A0004

typedef struct {
    mach_msg_header_t header;
    uint32_t requested_generation;
    uint32_t reserved;
    mach_msg_trailer_t trailer;
} FetchRequestRecvMsg;

typedef struct {
    mach_msg_header_t header;
    mach_msg_body_t body;
    mach_msg_port_descriptor_t ports[SWAPCHAIN_SLOTS];
    uint32_t generation;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint32_t status;
    uint32_t reserved;
} FetchReplySendMsg;

typedef struct {
    mach_msg_header_t header;
    uint32_t generation;
    uint32_t status;
} PingReplySendMsg;

typedef struct MacosSwapchainServer {
    pthread_mutex_t mtx;
    pthread_t thread;
    bool running;
    mach_port_t recv_port;
    char service_name[256];

    uint32_t generation;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    IOSurfaceRef surfaces[SWAPCHAIN_SLOTS];
} MacosSwapchainServer;

static void *mach_server_loop(void *arg) {
    MacosSwapchainServer *srv = (MacosSwapchainServer *)arg;
    while (srv->running) {
        FetchRequestRecvMsg req;
        memset(&req, 0, sizeof(req));
        kern_return_t kr = mach_msg(
            &req.header,
            MACH_RCV_MSG | MACH_RCV_TIMEOUT,
            0,
            sizeof(req),
            srv->recv_port,
            250,
            MACH_PORT_NULL);
        if (kr == MACH_RCV_TIMED_OUT) {
            continue;
        }
        if (kr != KERN_SUCCESS) {
            if (!srv->running) break;
            continue;
        }

        mach_port_t reply_port = req.header.msgh_remote_port;
        if (reply_port == MACH_PORT_NULL) continue;

        if (req.header.msgh_id == MSG_ID_PING_REQ) {
            PingReplySendMsg rep;
            memset(&rep, 0, sizeof(rep));
            rep.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_MOVE_SEND_ONCE, 0);
            rep.header.msgh_size = sizeof(rep);
            rep.header.msgh_remote_port = reply_port;
            rep.header.msgh_id = MSG_ID_PING_REP;
            pthread_mutex_lock(&srv->mtx);
            rep.generation = srv->generation;
            rep.status = 0;
            pthread_mutex_unlock(&srv->mtx);
            mach_msg(&rep.header, MACH_SEND_MSG | MACH_SEND_TIMEOUT, sizeof(rep), 0,
                     MACH_PORT_NULL, 500, MACH_PORT_NULL);
            continue;
        }

        if (req.header.msgh_id == MSG_ID_FETCH_SWAPCHAIN_REQ) {
            FetchReplySendMsg rep;
            memset(&rep, 0, sizeof(rep));
            rep.header.msgh_bits =
                MACH_MSGH_BITS_COMPLEX | MACH_MSGH_BITS(MACH_MSG_TYPE_MOVE_SEND_ONCE, 0);
            rep.header.msgh_size = sizeof(rep);
            rep.header.msgh_remote_port = reply_port;
            rep.header.msgh_id = MSG_ID_FETCH_SWAPCHAIN_REP;

            pthread_mutex_lock(&srv->mtx);
            bool ready = (srv->generation > 0 && srv->surfaces[0] != NULL);
            if (ready) {
                rep.body.msgh_descriptor_count = SWAPCHAIN_SLOTS;
                for (int i = 0; i < SWAPCHAIN_SLOTS; ++i) {
                    mach_port_t sp = IOSurfaceCreateMachPort(srv->surfaces[i]);
                    rep.ports[i].name = sp;
                    rep.ports[i].disposition = MACH_MSG_TYPE_MOVE_SEND;
                    rep.ports[i].type = MACH_MSG_PORT_DESCRIPTOR;
                }
                rep.generation = srv->generation;
                rep.width = srv->width;
                rep.height = srv->height;
                rep.pixel_format = srv->pixel_format;
                rep.status = 0;
            } else {
                rep.body.msgh_descriptor_count = 0;
                rep.status = 1;
            }
            pthread_mutex_unlock(&srv->mtx);

            kern_return_t skr = mach_msg(
                &rep.header,
                MACH_SEND_MSG | MACH_SEND_TIMEOUT,
                sizeof(rep),
                0,
                MACH_PORT_NULL,
                1000,
                MACH_PORT_NULL);
            if (skr != KERN_SUCCESS && rep.body.msgh_descriptor_count == SWAPCHAIN_SLOTS) {
                for (int i = 0; i < SWAPCHAIN_SLOTS; ++i) {
                    if (rep.ports[i].name != MACH_PORT_NULL) {
                        mach_port_deallocate(mach_task_self(), rep.ports[i].name);
                    }
                }
            }
        }
    }
    return NULL;
}

MacosSwapchainServer *macos_swapchain_server_start(const char *service_name) {
    MacosSwapchainServer *srv = (MacosSwapchainServer *)calloc(1, sizeof(MacosSwapchainServer));
    if (!srv) return NULL;
    pthread_mutex_init(&srv->mtx, NULL);
    snprintf(srv->service_name, sizeof(srv->service_name), "%s", service_name);

    kern_return_t kr = mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &srv->recv_port);
    if (kr != KERN_SUCCESS) {
        fprintf(stderr, "[macos_surface_ipc] mach_port_allocate failed: %d\n", kr);
        pthread_mutex_destroy(&srv->mtx);
        free(srv);
        return NULL;
    }
    kr = mach_port_insert_right(mach_task_self(), srv->recv_port, srv->recv_port, MACH_MSG_TYPE_MAKE_SEND);
    if (kr != KERN_SUCCESS) {
        fprintf(stderr, "[macos_surface_ipc] mach_port_insert_right failed: %d\n", kr);
        mach_port_destruct(mach_task_self(), srv->recv_port, 0, 0);
        pthread_mutex_destroy(&srv->mtx);
        free(srv);
        return NULL;
    }
    kr = bootstrap_register(bootstrap_port, srv->service_name, srv->recv_port);
    if (kr != KERN_SUCCESS) {
        fprintf(stderr, "[macos_surface_ipc] bootstrap_register(%s) failed: %d (%s)\n",
                srv->service_name, kr, mach_error_string(kr));
        mach_port_deallocate(mach_task_self(), srv->recv_port);
        mach_port_destruct(mach_task_self(), srv->recv_port, 0, 0);
        pthread_mutex_destroy(&srv->mtx);
        free(srv);
        return NULL;
    }

    srv->running = true;
    pthread_create(&srv->thread, NULL, mach_server_loop, srv);
    return srv;
}

// Allocate 3 IOSurface-backed BGRA8Unorm_sRGB MTLTextures on the given raw MTLDevice.
// out_mtl_textures[0..3] receive +1 retained (void*) id<MTLTexture> pointers owned by caller.
bool macos_swapchain_allocate(
    MacosSwapchainServer *srv,
    void *raw_mtl_device,
    uint32_t width,
    uint32_t height,
    uint32_t generation,
    void *out_mtl_textures[SWAPCHAIN_SLOTS]) {
    if (!srv || !raw_mtl_device || width == 0 || height == 0) return false;

    @autoreleasepool {
        id<MTLDevice> dev = (__bridge id<MTLDevice>)raw_mtl_device;
        IOSurfaceRef new_surfs[SWAPCHAIN_SLOTS] = {NULL, NULL, NULL};
        void *new_texs[SWAPCHAIN_SLOTS] = {NULL, NULL, NULL};

        NSDictionary *props = @{
            (id)kIOSurfaceWidth: @(width),
            (id)kIOSurfaceHeight: @(height),
            (id)kIOSurfaceBytesPerElement: @(4),
            (id)kIOSurfacePixelFormat: @(kCVPixelFormatType_32BGRA),
        };

        MTLTextureDescriptor *desc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm_sRGB
                                         width:width
                                        height:height
                                     mipmapped:NO];
        desc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead | MTLTextureUsagePixelFormatView;
        desc.storageMode = MTLStorageModeShared;

        for (int i = 0; i < SWAPCHAIN_SLOTS; ++i) {
            new_surfs[i] = IOSurfaceCreate((__bridge CFDictionaryRef)props);
            if (!new_surfs[i]) {
                for (int j = 0; j < i; ++j) {
                    if (new_texs[j]) CFRelease(new_texs[j]);
                    if (new_surfs[j]) CFRelease(new_surfs[j]);
                }
                return false;
            }
            id<MTLTexture> tex = [dev newTextureWithDescriptor:desc iosurface:new_surfs[i] plane:0];
            if (!tex) {
                CFRelease(new_surfs[i]);
                for (int j = 0; j < i; ++j) {
                    if (new_texs[j]) CFRelease(new_texs[j]);
                    if (new_surfs[j]) CFRelease(new_surfs[j]);
                }
                return false;
            }
            new_texs[i] = (__bridge_retained void *)tex;
        }

        pthread_mutex_lock(&srv->mtx);
        for (int i = 0; i < SWAPCHAIN_SLOTS; ++i) {
            if (srv->surfaces[i]) {
                CFRelease(srv->surfaces[i]);
            }
            srv->surfaces[i] = new_surfs[i];
            out_mtl_textures[i] = new_texs[i];
        }
        srv->width = width;
        srv->height = height;
        srv->pixel_format = kCVPixelFormatType_32BGRA;
        srv->generation = generation;
        pthread_mutex_unlock(&srv->mtx);
    }
    return true;
}

// Read pixels from a completed IOSurface slot directly into an RGBA8 tightly packed buffer.
// Zero GPU readback pass needed on Apple Silicon unified memory!
bool macos_swapchain_read_slot_rgba(
    MacosSwapchainServer *srv,
    uint32_t slot,
    uint8_t *out_rgba,
    size_t out_capacity,
    uint32_t *out_width,
    uint32_t *out_height) {
    if (!srv || slot >= SWAPCHAIN_SLOTS || !out_rgba) return false;

    IOSurfaceRef surf = NULL;
    uint32_t w = 0, h = 0;
    pthread_mutex_lock(&srv->mtx);
    if (srv->surfaces[slot]) {
        surf = srv->surfaces[slot];
        CFRetain(surf);
        w = srv->width;
        h = srv->height;
    }
    pthread_mutex_unlock(&srv->mtx);

    if (!surf || w == 0 || h == 0 || out_capacity < (size_t)w * (size_t)h * 4) {
        if (surf) CFRelease(surf);
        return false;
    }

    IOSurfaceLock(surf, kIOSurfaceLockReadOnly, NULL);
    const uint8_t *base = (const uint8_t *)IOSurfaceGetBaseAddress(surf);
    size_t stride = IOSurfaceGetBytesPerRow(surf);
    for (uint32_t y = 0; y < h; ++y) {
        const uint8_t *src = base + (size_t)y * stride;
        uint8_t *dst = out_rgba + (size_t)y * (size_t)w * 4;
        for (uint32_t x = 0; x < w; ++x) {
            dst[x * 4 + 0] = src[x * 4 + 2];  // R <- B
            dst[x * 4 + 1] = src[x * 4 + 1];  // G <- G
            dst[x * 4 + 2] = src[x * 4 + 0];  // B <- R
            dst[x * 4 + 3] = 255;             // Opaque alpha for PNG display
        }
    }
    IOSurfaceUnlock(surf, kIOSurfaceLockReadOnly, NULL);
    CFRelease(surf);

    if (out_width) *out_width = w;
    if (out_height) *out_height = h;
    return true;
}

void macos_swapchain_server_stop(MacosSwapchainServer *srv) {
    if (!srv) return;
    srv->running = false;
    pthread_join(srv->thread, NULL);
    pthread_mutex_lock(&srv->mtx);
    for (int i = 0; i < SWAPCHAIN_SLOTS; ++i) {
        if (srv->surfaces[i]) {
            CFRelease(srv->surfaces[i]);
            srv->surfaces[i] = NULL;
        }
    }
    pthread_mutex_unlock(&srv->mtx);
    if (srv->recv_port != MACH_PORT_NULL) {
        mach_port_deallocate(mach_task_self(), srv->recv_port);
        mach_port_destruct(mach_task_self(), srv->recv_port, 0, 0);
    }
    pthread_mutex_destroy(&srv->mtx);
    free(srv);
}
