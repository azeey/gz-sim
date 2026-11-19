#ifndef GZ_EXTERNAL_RENDERER_GPU_SURFACE_IPC_HH_
#define GZ_EXTERNAL_RENDERER_GPU_SURFACE_IPC_HH_

#include <array>
#include <cstdint>
#include <string>

#include "protocol.hh"

namespace gz::ext_render
{

struct ImportedSwapchainInfo
{
  uint32_t generation = 0;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t pixel_format = 0;  // Platform native format (e.g. 'BGRA' on macOS)
  // Platform-owned opaque handles:
  //   macOS: IOSurfaceRef (retained; release with ReleaseImportedSwapchain)
  //   Linux (future): DMA-BUF fd cast to intptr_t
  std::array<void *, kSwapchainSlots> native_handles = {nullptr, nullptr, nullptr};
};

// Build the deterministic OS service name for GPU surface handle exchange.
inline std::string GpuIpcServiceName(const std::string &session)
{
  return "org.gazebosim.ext_renderer." + session;
}

// Check if the external renderer's GPU handle IPC endpoint is already live.
bool IsExternalRendererAlive(const std::string &service_name);

// Request the triple-buffered native GPU surface handles from the external renderer
// for the latest (or requested) swapchain generation.
bool FetchExternalSwapchain(
    const std::string &service_name,
    uint32_t requested_generation,
    ImportedSwapchainInfo &out_info,
    uint32_t timeout_ms = 2000);

// Release platform handles inside ImportedSwapchainInfo.
void ReleaseImportedSwapchain(ImportedSwapchainInfo &info);

}  // namespace gz::ext_render

#endif  // GZ_EXTERNAL_RENDERER_GPU_SURFACE_IPC_HH_
