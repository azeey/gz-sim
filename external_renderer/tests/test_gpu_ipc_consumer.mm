#import <CoreFoundation/CoreFoundation.h>
#import <IOSurface/IOSurface.h>
#import <Metal/Metal.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "gpu_surface_ipc.hh"
#include "protocol.hh"
#include "zenoh_transport.hh"

// Standalone cross-process consumer verifier:
// 1. Connects to Bevy over Zenoh + Mach bootstrap service
// 2. Receives FrameReady events from Bevy
// 3. Imports the 3x IOSurface Mach ports & wraps each as an id<MTLTexture>
// 4. Sends SlotAcquired feedback back to Bevy
// 5. Inspects pixel statistics directly from the shared IOSurface GPU buffer
//    to prove real rendered frames arrive zero-copy across processes.
int main(int argc, char **argv)
{
  std::string session = gz::ext_render::DefaultSessionName();
  std::string endpoint = gz::ext_render::DefaultZenohEndpoint();
  int timeoutSec = 30;
  uint64_t minFrames = 2;

  for (int i = 1; i + 1 < argc; ++i)
  {
    if (std::string(argv[i]) == "--session") session = argv[++i];
    else if (std::string(argv[i]) == "--endpoint") endpoint = argv[++i];
    else if (std::string(argv[i]) == "--timeout") timeoutSec = std::stoi(argv[++i]);
    else if (std::string(argv[i]) == "--min-frames") minFrames = std::stoull(argv[++i]);
  }

  const std::string serviceName = gz::ext_render::GpuIpcServiceName(session);
  const std::string prefix = "gz_ext/" + session;

  gz::ext_render::ZenohNode zenoh;
  if (!zenoh.Open(endpoint, false, 0))
  {
    std::cerr << "[consumer_test] Zenoh open failed\n";
    return 1;
  }

  auto slotPub = zenoh.DeclarePublisher(prefix + "/gui/slot_acquired");
  std::mutex mtx;
  gz::ext_render::FrameReadyWire latest{};
  std::atomic<uint64_t> framesReceived{0};

  zenoh.DeclareSubscriber(
      prefix + "/gui/frame_ready",
      [&](const uint8_t *data, size_t len, bool) {
        if (len < sizeof(gz::ext_render::MsgHeader) + 40)
          return;
        gz::ext_render::MsgHeader hdr{};
        std::memcpy(&hdr, data, sizeof(hdr));
        if (hdr.magic != gz::ext_render::kProtocolMagic ||
            hdr.msg_type != static_cast<uint16_t>(gz::ext_render::MsgType::FrameReady))
          return;
        gz::ext_render::FrameReadyWire fw{};
        const size_t avail = len - sizeof(hdr);
        std::memcpy(&fw, data + sizeof(hdr), std::min(avail, sizeof(fw)));
        {
          std::lock_guard<std::mutex> lk(mtx);
          latest = fw;
        }
        framesReceived.fetch_add(1, std::memory_order_release);
      });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeoutSec);
  while (framesReceived.load(std::memory_order_acquire) < minFrames &&
         std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (framesReceived.load() == 0)
  {
    std::cerr << "[consumer_test] Timed out waiting for FrameReady from external renderer\n";
    return 2;
  }

  gz::ext_render::FrameReadyWire fw{};
  {
    std::lock_guard<std::mutex> lk(mtx);
    fw = latest;
  }

  const char *backendStr = (fw.backend == static_cast<uint8_t>(gz::ext_render::RendererBackend::Blender))
                               ? "Blender"
                               : "Bevy";
  std::cout << "[consumer_test] Received " << framesReceived.load()
            << " FrameReady events (" << backendStr << "). Latest: gen=" << fw.generation
            << " slot=" << fw.slot << " size=" << fw.width << "x" << fw.height
            << " fps=" << fw.fps << " render_ms=" << fw.render_ms
            << " spp=" << fw.sample_count << " refining=" << static_cast<int>(fw.refining)
            << " entities=" << fw.entity_count << std::endl;

  if (!gz::ext_render::IsExternalRendererAlive(serviceName))
  {
    std::cerr << "[consumer_test] Mach ping failed for " << serviceName << std::endl;
    return 3;
  }

  gz::ext_render::ImportedSwapchainInfo info{};
  if (!gz::ext_render::FetchExternalSwapchain(serviceName, fw.generation, info, 3000))
  {
    std::cerr << "[consumer_test] FetchExternalSwapchain failed\n";
    return 4;
  }

  id<MTLDevice> dev = MTLCreateSystemDefaultDevice();
  MTLTextureDescriptor *desc = [MTLTextureDescriptor
      texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                   width:info.width
                                  height:info.height
                               mipmapped:NO];
  desc.usage = MTLTextureUsageShaderRead;
  desc.storageMode = MTLStorageModeShared;

  for (uint32_t i = 0; i < gz::ext_render::kSwapchainSlots; ++i)
  {
    IOSurfaceRef surf = static_cast<IOSurfaceRef>(info.native_handles[i]);
    id<MTLTexture> tex = [dev newTextureWithDescriptor:desc iosurface:surf plane:0];
    if (!tex)
    {
      std::cerr << "[consumer_test] Failed creating MTLTexture for slot " << i << std::endl;
      gz::ext_render::ReleaseImportedSwapchain(info);
      return 5;
    }
  }
  std::cout << "[consumer_test] Imported all 3 IOSurface Mach ports & created 3 MTLTextures ("
            << info.width << "x" << info.height << ", gen=" << info.generation << ")\n";

  // Send SlotAcquired ack back to Bevy
  {
    gz::ext_render::SlotAcquiredWire ack{info.generation, fw.slot};
    gz::ext_render::PacketBuilder pb;
    pb.Reset(gz::ext_render::MsgType::SlotAcquired, 1);
    pb.AppendPod(ack);
    slotPub->Publish(pb.Finish(), false);
  }

  // Inspect pixel variance on the completed slot directly from IOSurface
  IOSurfaceRef activeSurf = static_cast<IOSurfaceRef>(info.native_handles[fw.slot]);
  IOSurfaceLock(activeSurf, kIOSurfaceLockReadOnly, nullptr);
  const uint8_t *base = static_cast<const uint8_t *>(IOSurfaceGetBaseAddress(activeSurf));
  const size_t stride = IOSurfaceGetBytesPerRow(activeSurf);
  uint64_t sumR = 0, sumG = 0, sumB = 0;
  uint64_t nonZeroPixels = 0;
  uint8_t minLuma = 255, maxLuma = 0;
  const uint32_t step = 4;
  uint64_t sampled = 0;
  for (uint32_t y = 0; y < info.height; y += step)
  {
    const uint8_t *row = base + static_cast<size_t>(y) * stride;
    for (uint32_t x = 0; x < info.width; x += step)
    {
      uint8_t b = row[x * 4 + 0];
      uint8_t g = row[x * 4 + 1];
      uint8_t r = row[x * 4 + 2];
      sumR += r;
      sumG += g;
      sumB += b;
      if (r || g || b) ++nonZeroPixels;
      uint8_t luma = static_cast<uint8_t>((r + g + b) / 3);
      if (luma < minLuma) minLuma = luma;
      if (luma > maxLuma) maxLuma = luma;
      ++sampled;
    }
  }
  IOSurfaceUnlock(activeSurf, kIOSurfaceLockReadOnly, nullptr);
  gz::ext_render::ReleaseImportedSwapchain(info);

  std::cout << "[consumer_test] Shared IOSurface slot=" << fw.slot
            << " meanRGB=(" << (sumR / sampled) << "," << (sumG / sampled) << "," << (sumB / sampled)
            << ") lumaRange=[" << static_cast<int>(minLuma) << "," << static_cast<int>(maxLuma)
            << "] nonZero=" << nonZeroPixels << "/" << sampled << std::endl;

  if (nonZeroPixels == 0 || maxLuma == minLuma)
  {
    std::cerr << "[consumer_test] Image appears blank or uniform!\n";
    return 6;
  }

  std::cout << "[consumer_test] PASS: Cross-process " << backendStr
            << " -> IOSurface -> Metal verified!\n";
  return 0;
}
