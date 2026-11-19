#ifndef GZ_GUI_PLUGINS_EXTERNAL_RENDERER_METAL_TEXTURE_BRIDGE_HH_
#define GZ_GUI_PLUGINS_EXTERNAL_RENDERER_METAL_TEXTURE_BRIDGE_HH_

#include <cstdint>
#include <memory>

#include "gpu_surface_ipc.hh"

class QQuickWindow;
class QSGTexture;

namespace gz::gui::plugins
{

/// \brief Cross-platform interface for wrapping externally allocated GPU swapchain
/// handles (IOSurface on macOS Metal, DMA-BUF on Linux Vulkan) into Qt Quick
/// QSGTexture objects with zero CPU copies.
class ExternalTextureBridge
{
 public:
  virtual ~ExternalTextureBridge() = default;

  /// \brief Import a 3-slot swapchain from native handles on Qt's scene graph thread.
  virtual bool ImportSwapchain(
      QQuickWindow *window,
      const gz::ext_render::ImportedSwapchainInfo &info) = 0;

  /// \brief Return cached QSGTexture for the given swapchain slot (0..2).
  virtual QSGTexture *TextureForSlot(uint32_t slot) const = 0;

  /// \brief Currently imported swapchain generation ID.
  virtual uint32_t Generation() const = 0;

  /// \brief Release all imported native textures and QSGTexture wrappers.
  virtual void Release() = 0;
};

std::unique_ptr<ExternalTextureBridge> CreatePlatformTextureBridge();

}  // namespace gz::gui::plugins

#endif  // GZ_GUI_PLUGINS_EXTERNAL_RENDERER_METAL_TEXTURE_BRIDGE_HH_
