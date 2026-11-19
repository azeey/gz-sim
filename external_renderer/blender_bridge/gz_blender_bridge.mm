#ifdef __APPLE__

#import <CoreFoundation/CoreFoundation.h>
#import <CoreGraphics/CoreGraphics.h>
#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>
#import <IOSurface/IOSurface.h>
#import <ImageIO/ImageIO.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include <mach/mach.h>
#include <pthread.h>
#include <servers/bootstrap.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "protocol.hh"
#include "zenoh_transport.hh"

namespace
{

constexpr uint32_t kSlots = gz::ext_render::kSwapchainSlots;
constexpr mach_msg_id_t kMsgIdFetchSwapchainReq = 0x475A0001;
constexpr mach_msg_id_t kMsgIdFetchSwapchainRep = 0x475A0002;
constexpr mach_msg_id_t kMsgIdPingReq = 0x475A0003;
constexpr mach_msg_id_t kMsgIdPingRep = 0x475A0004;

struct FetchRequestRecvMsg
{
  mach_msg_header_t header;
  uint32_t requested_generation;
  uint32_t reserved;
  mach_msg_trailer_t trailer;
};

struct FetchReplySendMsg
{
  mach_msg_header_t header;
  mach_msg_body_t body;
  mach_msg_port_descriptor_t ports[kSlots];
  uint32_t generation;
  uint32_t width;
  uint32_t height;
  uint32_t pixel_format;
  uint32_t status;
  uint32_t reserved;
};

struct PingReplySendMsg
{
  mach_msg_header_t header;
  uint32_t generation;
  uint32_t status;
};

struct BridgePacket
{
  std::vector<uint8_t> data;
  bool is_shm = false;
};

struct GzBlenderBridge
{
  std::string session;
  std::string endpoint;
  std::string service_name;

  // Mach bootstrap IOSurface swapchain server
  pthread_mutex_t surf_mtx = PTHREAD_MUTEX_INITIALIZER;
  pthread_t mach_thread{};
  std::atomic<bool> mach_running{false};
  mach_port_t recv_port = MACH_PORT_NULL;

  uint32_t generation = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t pixel_format = kCVPixelFormatType_32BGRA;
  IOSurfaceRef surfaces[kSlots] = {nullptr, nullptr, nullptr};

  // Triple-buffer mailbox coordination
  std::atomic<uint32_t> qt_active_gen{0};
  std::atomic<uint32_t> qt_active_slot{kSlots};
  uint32_t last_ready_slot = kSlots;
  uint64_t frame_id = 0;

  // Zenoh 1.8 POSIX SHM session
  gz::ext_render::ZenohNode zenoh;
  std::shared_ptr<gz::ext_render::ZenohNode::Publisher> snapshot_pub;
  std::shared_ptr<gz::ext_render::ZenohNode::Publisher> frame_pub;
  gz::ext_render::PacketBuilder builder;

  // Thread-safe queues drained by Python
  std::mutex scene_mtx;
  std::deque<BridgePacket> scene_queue;
  std::vector<uint8_t> last_scene_buf;

  std::mutex input_mtx;
  std::deque<BridgePacket> input_queue;
  std::vector<uint8_t> last_input_buf;

  // Reusable file read scratch buffer
  std::vector<uint8_t> tga_scratch;
};

void *MachServerLoop(void *arg)
{
  auto *b = static_cast<GzBlenderBridge *>(arg);
  while (b->mach_running.load(std::memory_order_acquire))
  {
    FetchRequestRecvMsg req{};
    kern_return_t kr = mach_msg(
        &req.header,
        MACH_RCV_MSG | MACH_RCV_TIMEOUT,
        0,
        sizeof(req),
        b->recv_port,
        200,
        MACH_PORT_NULL);
    if (kr == MACH_RCV_TIMED_OUT) continue;
    if (kr != KERN_SUCCESS)
    {
      if (!b->mach_running.load(std::memory_order_acquire)) break;
      continue;
    }

    mach_port_t reply_port = req.header.msgh_remote_port;
    if (reply_port == MACH_PORT_NULL) continue;

    if (req.header.msgh_id == kMsgIdPingReq)
    {
      PingReplySendMsg rep{};
      rep.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_MOVE_SEND_ONCE, 0);
      rep.header.msgh_size = sizeof(rep);
      rep.header.msgh_remote_port = reply_port;
      rep.header.msgh_id = kMsgIdPingRep;
      pthread_mutex_lock(&b->surf_mtx);
      rep.generation = b->generation;
      rep.status = 0;
      pthread_mutex_unlock(&b->surf_mtx);
      mach_msg(&rep.header, MACH_SEND_MSG | MACH_SEND_TIMEOUT, sizeof(rep), 0,
               MACH_PORT_NULL, 500, MACH_PORT_NULL);
      continue;
    }

    if (req.header.msgh_id == kMsgIdFetchSwapchainReq)
    {
      FetchReplySendMsg rep{};
      rep.header.msgh_bits =
          MACH_MSGH_BITS_COMPLEX | MACH_MSGH_BITS(MACH_MSG_TYPE_MOVE_SEND_ONCE, 0);
      rep.header.msgh_size = sizeof(rep);
      rep.header.msgh_remote_port = reply_port;
      rep.header.msgh_id = kMsgIdFetchSwapchainRep;

      pthread_mutex_lock(&b->surf_mtx);
      bool ready = (b->generation > 0 && b->surfaces[0] != nullptr);
      if (ready)
      {
        rep.body.msgh_descriptor_count = kSlots;
        for (uint32_t i = 0; i < kSlots; ++i)
        {
          mach_port_t sp = IOSurfaceCreateMachPort(b->surfaces[i]);
          rep.ports[i].name = sp;
          rep.ports[i].disposition = MACH_MSG_TYPE_MOVE_SEND;
          rep.ports[i].type = MACH_MSG_PORT_DESCRIPTOR;
        }
        rep.generation = b->generation;
        rep.width = b->width;
        rep.height = b->height;
        rep.pixel_format = b->pixel_format;
        rep.status = 0;
      }
      else
      {
        rep.body.msgh_descriptor_count = 0;
        rep.status = 1;
      }
      pthread_mutex_unlock(&b->surf_mtx);

      kern_return_t skr = mach_msg(
          &rep.header,
          MACH_SEND_MSG | MACH_SEND_TIMEOUT,
          sizeof(rep),
          0,
          MACH_PORT_NULL,
          1000,
          MACH_PORT_NULL);
      if (skr != KERN_SUCCESS && rep.body.msgh_descriptor_count == kSlots)
      {
        for (uint32_t i = 0; i < kSlots; ++i)
        {
          if (rep.ports[i].name != MACH_PORT_NULL)
          {
            mach_port_deallocate(mach_task_self(), rep.ports[i].name);
          }
        }
      }
    }
  }
  return nullptr;
}

bool AllocateSwapchain(GzBlenderBridge *b, uint32_t width, uint32_t height)
{
  if (!b || width == 0 || height == 0) return false;

  @autoreleasepool
  {
    NSDictionary *props = @{
      (id)kIOSurfaceWidth : @(width),
      (id)kIOSurfaceHeight : @(height),
      (id)kIOSurfaceBytesPerElement : @(4),
      (id)kIOSurfacePixelFormat : @(kCVPixelFormatType_32BGRA),
    };

    IOSurfaceRef newSurfs[kSlots] = {nullptr, nullptr, nullptr};
    for (uint32_t i = 0; i < kSlots; ++i)
    {
      newSurfs[i] = IOSurfaceCreate((__bridge CFDictionaryRef)props);
      if (!newSurfs[i])
      {
        for (uint32_t j = 0; j < i; ++j) CFRelease(newSurfs[j]);
        return false;
      }
      // Initialize to dark opaque slate so unrendered slots are never transparent/garbage
      IOSurfaceLock(newSurfs[i], 0, nullptr);
      uint8_t *base = static_cast<uint8_t *>(IOSurfaceGetBaseAddress(newSurfs[i]));
      size_t stride = IOSurfaceGetBytesPerRow(newSurfs[i]);
      const uint32_t fillPixel = 0xFF241D1Au;  // BGRA=(0x1A, 0x1D, 0x24, 0xFF)
      for (uint32_t y = 0; y < height; ++y)
      {
        uint32_t *row = reinterpret_cast<uint32_t *>(base + static_cast<size_t>(y) * stride);
        std::fill_n(row, width, fillPixel);
      }
      IOSurfaceUnlock(newSurfs[i], 0, nullptr);
    }

    pthread_mutex_lock(&b->surf_mtx);
    for (uint32_t i = 0; i < kSlots; ++i)
    {
      if (b->surfaces[i]) CFRelease(b->surfaces[i]);
      b->surfaces[i] = newSurfs[i];
    }
    b->width = width;
    b->height = height;
    b->generation += 1;
    b->last_ready_slot = kSlots;
    pthread_mutex_unlock(&b->surf_mtx);
  }
  return true;
}

uint32_t PickNextSlot(GzBlenderBridge *b)
{
  const uint32_t qtGen = b->qt_active_gen.load(std::memory_order_acquire);
  const uint32_t qtSlot =
      (qtGen == b->generation) ? b->qt_active_slot.load(std::memory_order_acquire) : kSlots;
  for (uint32_t s = 0; s < kSlots; ++s)
  {
    if (s != qtSlot && s != b->last_ready_slot)
    {
      return s;
    }
  }
  return (b->last_ready_slot + 1) % kSlots;
}

// Fast bilinear upscale or exact-size blit of BGRA8 (or BGR8) pixels into an IOSurface.
void BlitBgraToSurface(IOSurfaceRef surf,
                       uint32_t dstW,
                       uint32_t dstH,
                       const uint8_t *srcPixels,
                       uint32_t srcW,
                       uint32_t srcH,
                       uint32_t srcBppBytes,
                       bool srcTopDown)
{
  IOSurfaceLock(surf, 0, nullptr);
  uint8_t *dstBase = static_cast<uint8_t *>(IOSurfaceGetBaseAddress(surf));
  const size_t dstStride = IOSurfaceGetBytesPerRow(surf);
  const size_t srcRowBytes = static_cast<size_t>(srcW) * srcBppBytes;

  if (srcW == dstW && srcH == dstH && srcBppBytes == 4)
  {
    for (uint32_t y = 0; y < dstH; ++y)
    {
      const uint32_t sy = srcTopDown ? y : (srcH - 1 - y);
      const uint32_t *srcRow =
          reinterpret_cast<const uint32_t *>(srcPixels + static_cast<size_t>(sy) * srcRowBytes);
      uint32_t *dstRow =
          reinterpret_cast<uint32_t *>(dstBase + static_cast<size_t>(y) * dstStride);
      for (uint32_t x = 0; x < dstW; ++x)
      {
        dstRow[x] = srcRow[x] | 0xFF000000u;  // Force opaque alpha for Qt compositing
      }
    }
  }
  else
  {
    // Bilinear upscale/downscale (keeps fixed-size IOSurface with zero Mach churn when
    // Blender renders interactive frames at 55-70% resolution scale!)
    const float scaleX = (dstW > 1) ? static_cast<float>(srcW - 1) / static_cast<float>(dstW - 1) : 0.0f;
    const float scaleY = (dstH > 1) ? static_cast<float>(srcH - 1) / static_cast<float>(dstH - 1) : 0.0f;

    std::vector<uint32_t> x0Arr(dstW), x1Arr(dstW), wxArr(dstW);
    for (uint32_t x = 0; x < dstW; ++x)
    {
      const float fx = static_cast<float>(x) * scaleX;
      const uint32_t x0 = std::min(static_cast<uint32_t>(fx), srcW - 1);
      const uint32_t x1 = std::min(x0 + 1, srcW - 1);
      x0Arr[x] = x0 * srcBppBytes;
      x1Arr[x] = x1 * srcBppBytes;
      wxArr[x] = static_cast<uint32_t>((fx - static_cast<float>(x0)) * 256.0f + 0.5f);
    }

    for (uint32_t y = 0; y < dstH; ++y)
    {
      const float fy = static_cast<float>(y) * scaleY;
      const uint32_t y0Raw = std::min(static_cast<uint32_t>(fy), srcH - 1);
      const uint32_t y1Raw = std::min(y0Raw + 1, srcH - 1);
      const uint32_t wy = static_cast<uint32_t>((fy - static_cast<float>(y0Raw)) * 256.0f + 0.5f);
      const uint32_t wyInv = 256u - wy;

      const uint32_t sy0 = srcTopDown ? y0Raw : (srcH - 1 - y0Raw);
      const uint32_t sy1 = srcTopDown ? y1Raw : (srcH - 1 - y1Raw);
      const uint8_t *row0 = srcPixels + static_cast<size_t>(sy0) * srcRowBytes;
      const uint8_t *row1 = srcPixels + static_cast<size_t>(sy1) * srcRowBytes;
      uint8_t *dstRow = dstBase + static_cast<size_t>(y) * dstStride;

      for (uint32_t x = 0; x < dstW; ++x)
      {
        const uint32_t o0 = x0Arr[x];
        const uint32_t o1 = x1Arr[x];
        const uint32_t wx = wxArr[x];
        const uint32_t wxInv = 256u - wx;

        const uint32_t w00 = wxInv * wyInv;
        const uint32_t w10 = wx * wyInv;
        const uint32_t w01 = wxInv * wy;
        const uint32_t w11 = wx * wy;

        const uint8_t *p00 = row0 + o0;
        const uint8_t *p10 = row0 + o1;
        const uint8_t *p01 = row1 + o0;
        const uint8_t *p11 = row1 + o1;

        uint8_t *d = dstRow + static_cast<size_t>(x) * 4;
        d[0] = static_cast<uint8_t>((p00[0] * w00 + p10[0] * w10 + p01[0] * w01 + p11[0] * w11) >> 16);
        d[1] = static_cast<uint8_t>((p00[1] * w00 + p10[1] * w10 + p01[1] * w01 + p11[1] * w11) >> 16);
        d[2] = static_cast<uint8_t>((p00[2] * w00 + p10[2] * w10 + p01[2] * w01 + p11[2] * w11) >> 16);
        d[3] = 255;
      }
    }
  }

  IOSurfaceUnlock(surf, 0, nullptr);
}

void PublishFrameReady(GzBlenderBridge *b,
                       uint32_t slot,
                       float render_ms,
                       float fps,
                       uint32_t entity_count,
                       uint32_t quality_flags,
                       uint8_t refining,
                       uint16_t sample_count)
{
  b->last_ready_slot = slot;
  b->frame_id += 1;

  if (!b->frame_pub) return;

  gz::ext_render::FrameReadyWire fw{};
  fw.generation = b->generation;
  fw.slot = slot;
  fw.width = b->width;
  fw.height = b->height;
  fw.frame_id = b->frame_id;
  fw.render_ms = render_ms;
  fw.fps = fps;
  fw.entity_count = entity_count;
  fw.quality_flags = quality_flags;
  fw.backend = static_cast<uint8_t>(gz::ext_render::RendererBackend::Blender);
  fw.refining = refining;
  fw.sample_count = sample_count;

  b->builder.Reset(gz::ext_render::MsgType::FrameReady, 1);
  b->builder.AppendPod(fw);
  b->frame_pub->Publish(b->builder.Finish(), false);
}

}  // namespace

extern "C" {

GzBlenderBridge *gz_blender_bridge_create(const char *session,
                                          const char *endpoint,
                                          uint32_t width,
                                          uint32_t height)
{
  auto b = std::make_unique<GzBlenderBridge>();
  b->session = (session && session[0]) ? session : "default";
  b->endpoint = (endpoint && endpoint[0]) ? endpoint : "tcp/127.0.0.1:17447";
  b->service_name = "org.gazebosim.ext_renderer." + b->session;

  // 1. Start Mach bootstrap swapchain server
  kern_return_t kr = mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &b->recv_port);
  if (kr != KERN_SUCCESS)
  {
    std::cerr << "[gz_blender_bridge] mach_port_allocate failed: " << kr << std::endl;
    return nullptr;
  }
  kr = mach_port_insert_right(mach_task_self(), b->recv_port, b->recv_port, MACH_MSG_TYPE_MAKE_SEND);
  if (kr != KERN_SUCCESS)
  {
    std::cerr << "[gz_blender_bridge] mach_port_insert_right failed: " << kr << std::endl;
    mach_port_destruct(mach_task_self(), b->recv_port, 0, 0);
    return nullptr;
  }
  kr = bootstrap_register(bootstrap_port, const_cast<char *>(b->service_name.c_str()), b->recv_port);
  if (kr != KERN_SUCCESS)
  {
    std::cerr << "[gz_blender_bridge] bootstrap_register(" << b->service_name
              << ") failed: " << kr << " (" << mach_error_string(kr) << ")" << std::endl;
    mach_port_deallocate(mach_task_self(), b->recv_port);
    mach_port_destruct(mach_task_self(), b->recv_port, 0, 0);
    return nullptr;
  }

  if (!AllocateSwapchain(b.get(), std::max(width, 64u), std::max(height, 64u)))
  {
    std::cerr << "[gz_blender_bridge] Initial IOSurface allocation failed" << std::endl;
    mach_port_deallocate(mach_task_self(), b->recv_port);
    mach_port_destruct(mach_task_self(), b->recv_port, 0, 0);
    return nullptr;
  }

  b->mach_running.store(true, std::memory_order_release);
  pthread_create(&b->mach_thread, nullptr, MachServerLoop, b.get());

  // 2. Open Zenoh 1.8 listener session with POSIX SHM enabled
  if (!b->zenoh.Open(b->endpoint, true, 0))
  {
    std::cerr << "[gz_blender_bridge] Zenoh open failed on " << b->endpoint << std::endl;
    b->mach_running.store(false, std::memory_order_release);
    pthread_join(b->mach_thread, nullptr);
    mach_port_deallocate(mach_task_self(), b->recv_port);
    mach_port_destruct(mach_task_self(), b->recv_port, 0, 0);
    return nullptr;
  }

  const std::string prefix = "gz_ext/" + b->session;
  b->snapshot_pub = b->zenoh.DeclarePublisher(prefix + "/scene/snapshot_req");
  b->frame_pub = b->zenoh.DeclarePublisher(prefix + "/gui/frame_ready");

  GzBlenderBridge *raw = b.get();
  b->zenoh.DeclareSubscriber(
      prefix + "/scene/events",
      [raw](const uint8_t *data, size_t len, bool is_shm) {
        BridgePacket pkt;
        pkt.data.assign(data, data + len);
        pkt.is_shm = is_shm;
        std::lock_guard<std::mutex> lk(raw->scene_mtx);
        raw->scene_queue.push_back(std::move(pkt));
      });

  b->zenoh.DeclareSubscriber(
      prefix + "/gui/input",
      [raw](const uint8_t *data, size_t len, bool is_shm) {
        BridgePacket pkt;
        pkt.data.assign(data, data + len);
        pkt.is_shm = is_shm;
        std::lock_guard<std::mutex> lk(raw->input_mtx);
        raw->input_queue.push_back(std::move(pkt));
      });

  b->zenoh.DeclareSubscriber(
      prefix + "/gui/slot_acquired",
      [raw](const uint8_t *data, size_t len, bool /*is_shm*/) {
        if (len < sizeof(gz::ext_render::MsgHeader) + sizeof(gz::ext_render::SlotAcquiredWire))
          return;
        gz::ext_render::MsgHeader hdr{};
        std::memcpy(&hdr, data, sizeof(hdr));
        if (hdr.magic != gz::ext_render::kProtocolMagic ||
            hdr.msg_type != static_cast<uint16_t>(gz::ext_render::MsgType::SlotAcquired))
          return;
        gz::ext_render::SlotAcquiredWire wire{};
        std::memcpy(&wire, data + sizeof(hdr), sizeof(wire));
        raw->qt_active_gen.store(wire.generation, std::memory_order_release);
        raw->qt_active_slot.store(wire.active_slot, std::memory_order_release);
      });

  std::cerr << "[gz_blender_bridge] Ready: service='" << b->service_name
            << "' zenoh='" << b->endpoint << "' swapchain="
            << b->width << "x" << b->height << " (gen=" << b->generation << ")" << std::endl;

  return b.release();
}

void gz_blender_bridge_destroy(GzBlenderBridge *b)
{
  if (!b) return;
  b->zenoh.Close();
  b->mach_running.store(false, std::memory_order_release);
  if (b->mach_thread)
  {
    pthread_join(b->mach_thread, nullptr);
  }
  pthread_mutex_lock(&b->surf_mtx);
  for (uint32_t i = 0; i < kSlots; ++i)
  {
    if (b->surfaces[i])
    {
      CFRelease(b->surfaces[i]);
      b->surfaces[i] = nullptr;
    }
  }
  pthread_mutex_unlock(&b->surf_mtx);
  if (b->recv_port != MACH_PORT_NULL)
  {
    mach_port_deallocate(mach_task_self(), b->recv_port);
    mach_port_destruct(mach_task_self(), b->recv_port, 0, 0);
  }
  pthread_mutex_destroy(&b->surf_mtx);
  delete b;
}

bool gz_blender_bridge_resize(GzBlenderBridge *b, uint32_t width, uint32_t height)
{
  if (!b) return false;
  width = std::clamp(width, 64u, 3840u);
  height = std::clamp(height, 64u, 2160u);
  if (width == b->width && height == b->height) return true;
  bool ok = AllocateSwapchain(b, width, height);
  if (ok)
  {
    std::cerr << "[gz_blender_bridge] Resized triple-buffered IOSurface swapchain to "
              << width << "x" << height << " (gen=" << b->generation << ")" << std::endl;
  }
  return ok;
}

uint32_t gz_blender_bridge_width(GzBlenderBridge *b) { return b ? b->width : 0; }
uint32_t gz_blender_bridge_height(GzBlenderBridge *b) { return b ? b->height : 0; }

bool gz_blender_bridge_request_snapshot(GzBlenderBridge *b, uint64_t client_id)
{
  if (!b || !b->snapshot_pub) return false;
  gz::ext_render::SnapshotRequestWire req{client_id};
  b->builder.Reset(gz::ext_render::MsgType::SnapshotRequest, 1);
  b->builder.AppendPod(req);
  return b->snapshot_pub->Publish(b->builder.Finish(), false);
}

bool gz_blender_bridge_pop_scene(GzBlenderBridge *b,
                                 const uint8_t **out_data,
                                 uint32_t *out_len,
                                 uint8_t *out_is_shm)
{
  if (!b || !out_data || !out_len) return false;
  std::lock_guard<std::mutex> lk(b->scene_mtx);
  if (b->scene_queue.empty()) return false;
  BridgePacket pkt = std::move(b->scene_queue.front());
  b->scene_queue.pop_front();
  b->last_scene_buf = std::move(pkt.data);
  *out_data = b->last_scene_buf.data();
  *out_len = static_cast<uint32_t>(b->last_scene_buf.size());
  if (out_is_shm) *out_is_shm = pkt.is_shm ? 1 : 0;
  return true;
}

bool gz_blender_bridge_pop_input(GzBlenderBridge *b,
                                 const uint8_t **out_data,
                                 uint32_t *out_len)
{
  if (!b || !out_data || !out_len) return false;
  std::lock_guard<std::mutex> lk(b->input_mtx);
  if (b->input_queue.empty()) return false;
  BridgePacket pkt = std::move(b->input_queue.front());
  b->input_queue.pop_front();
  b->last_input_buf = std::move(pkt.data);
  *out_data = b->last_input_buf.data();
  *out_len = static_cast<uint32_t>(b->last_input_buf.size());
  return true;
}

bool gz_blender_bridge_has_pending(GzBlenderBridge *b)
{
  if (!b) return false;
  {
    std::lock_guard<std::mutex> lk(b->input_mtx);
    if (!b->input_queue.empty()) return true;
  }
  {
    std::lock_guard<std::mutex> lk(b->scene_mtx);
    if (!b->scene_queue.empty()) return true;
  }
  return false;
}

bool gz_blender_bridge_present_tga_file(GzBlenderBridge *b,
                                        const char *tga_path,
                                        float render_ms,
                                        float fps,
                                        uint32_t entity_count,
                                        uint32_t quality_flags,
                                        uint8_t refining,
                                        uint16_t sample_count)
{
  if (!b || !tga_path) return false;

  std::ifstream ifs(tga_path, std::ios::binary | std::ios::ate);
  if (!ifs) return false;
  const std::streamsize sz = ifs.tellg();
  if (sz < 18) return false;
  ifs.seekg(0, std::ios::beg);
  b->tga_scratch.resize(static_cast<size_t>(sz));
  if (!ifs.read(reinterpret_cast<char *>(b->tga_scratch.data()), sz)) return false;

  const uint8_t *hdr = b->tga_scratch.data();
  const uint8_t idLen = hdr[0];
  const uint8_t imgType = hdr[2];
  const uint32_t srcW = static_cast<uint32_t>(hdr[12]) | (static_cast<uint32_t>(hdr[13]) << 8);
  const uint32_t srcH = static_cast<uint32_t>(hdr[14]) | (static_cast<uint32_t>(hdr[15]) << 8);
  const uint8_t bpp = hdr[16];
  const bool topDown = (hdr[17] & 0x20u) != 0;

  if (imgType != 2 || srcW == 0 || srcH == 0 || (bpp != 32 && bpp != 24))
  {
    std::cerr << "[gz_blender_bridge] Unsupported TGA format: type=" << int(imgType)
              << " bpp=" << int(bpp) << " size=" << srcW << "x" << srcH << std::endl;
    return false;
  }

  const uint32_t bppBytes = bpp / 8;
  const size_t dataOffset = 18u + idLen;
  const size_t needed = dataOffset + static_cast<size_t>(srcW) * srcH * bppBytes;
  if (b->tga_scratch.size() < needed) return false;

  const uint32_t slot = PickNextSlot(b);
  pthread_mutex_lock(&b->surf_mtx);
  IOSurfaceRef surf = b->surfaces[slot];
  if (surf) CFRetain(surf);
  const uint32_t dstW = b->width;
  const uint32_t dstH = b->height;
  pthread_mutex_unlock(&b->surf_mtx);
  if (!surf) return false;

  BlitBgraToSurface(surf, dstW, dstH, b->tga_scratch.data() + dataOffset,
                    srcW, srcH, bppBytes, topDown);
  CFRelease(surf);

  PublishFrameReady(b, slot, render_ms, fps, entity_count, quality_flags, refining, sample_count);
  return true;
}

bool gz_blender_bridge_reannounce_frame(GzBlenderBridge *b,
                                        float render_ms,
                                        float fps,
                                        uint32_t entity_count,
                                        uint32_t quality_flags,
                                        uint8_t refining,
                                        uint16_t sample_count)
{
  if (!b || b->last_ready_slot >= kSlots) return false;
  PublishFrameReady(b, b->last_ready_slot, render_ms, fps, entity_count,
                    quality_flags, refining, sample_count);
  return true;
}

bool gz_blender_bridge_save_png(GzBlenderBridge *b, const char *png_path)
{
  if (!b || !png_path || !png_path[0]) return false;

  const uint32_t slot = (b->last_ready_slot < kSlots) ? b->last_ready_slot : 0;
  pthread_mutex_lock(&b->surf_mtx);
  IOSurfaceRef surf = b->surfaces[slot];
  if (surf) CFRetain(surf);
  const uint32_t w = b->width;
  const uint32_t h = b->height;
  pthread_mutex_unlock(&b->surf_mtx);
  if (!surf || w == 0 || h == 0)
  {
    if (surf) CFRelease(surf);
    return false;
  }

  bool ok = false;
  @autoreleasepool
  {
    IOSurfaceLock(surf, kIOSurfaceLockReadOnly, nullptr);
    const void *base = IOSurfaceGetBaseAddress(surf);
    const size_t stride = IOSurfaceGetBytesPerRow(surf);

    CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGContextRef ctx = CGBitmapContextCreate(
        const_cast<void *>(base),
        w,
        h,
        8,
        stride,
        cs,
        kCGBitmapByteOrder32Little | kCGImageAlphaNoneSkipFirst);
    if (ctx)
    {
      CGImageRef cgImg = CGBitmapContextCreateImage(ctx);
      if (cgImg)
      {
        NSString *nsPath = [NSString stringWithUTF8String:png_path];
        NSURL *url = [NSURL fileURLWithPath:nsPath];
        CGImageDestinationRef dest = CGImageDestinationCreateWithURL(
            (__bridge CFURLRef)url,
            (__bridge CFStringRef)UTTypePNG.identifier,
            1,
            nullptr);
        if (dest)
        {
          CGImageDestinationAddImage(dest, cgImg, nullptr);
          ok = CGImageDestinationFinalize(dest);
          CFRelease(dest);
        }
        CGImageRelease(cgImg);
      }
      CGContextRelease(ctx);
    }
    if (cs) CGColorSpaceRelease(cs);
    IOSurfaceUnlock(surf, kIOSurfaceLockReadOnly, nullptr);
  }
  CFRelease(surf);
  return ok;
}

}  // extern "C"

#endif  // __APPLE__
