#include "MetalTextureBridge.hh"

#ifdef __APPLE__

#import <IOSurface/IOSurface.h>
#import <Metal/Metal.h>

#include <QQuickWindow>
#include <QSGRendererInterface>
#include <QSGTexture>
#include <QSize>

#include <array>
#include <iostream>

#if !__has_feature(objc_arc)
#error "MetalTextureBridge.mm must be compiled with -fobjc-arc"
#endif

namespace gz::gui::plugins
{

class MetalTextureBridge final : public ExternalTextureBridge
{
 public:
  ~MetalTextureBridge() override { Release(); }

  bool ImportSwapchain(
      QQuickWindow *window,
      const gz::ext_render::ImportedSwapchainInfo &info) override
  {
    if (!window || info.width == 0 || info.height == 0) return false;

    auto *rif = window->rendererInterface();
    if (!rif || rif->graphicsApi() != QSGRendererInterface::MetalRhi)
    {
      std::cerr << "[ExternalRendererGui] Qt Quick window is not using MetalRhi!" << std::endl;
      return false;
    }

    id<MTLDevice> dev = (__bridge id<MTLDevice>)rif->getResource(
        window, QSGRendererInterface::DeviceResource);
    if (!dev)
    {
      dev = MTLCreateSystemDefaultDevice();
    }
    if (!dev)
    {
      std::cerr << "[ExternalRendererGui] Could not acquire MTLDevice" << std::endl;
      return false;
    }

    Release();

    // IMPORTANT: Bevy renders into MTLPixelFormatBGRA8Unorm_sRGB, so the IOSurface
    // already holds sRGB-encoded bytes. Wrapping it as MTLPixelFormatBGRA8Unorm (non-sRGB)
    // matches MinimalSceneRhiMetal.mm and ensures Qt Quick's Scene Graph passes the
    // sRGB bytes through bit-exact without double-darkening.
    MTLTextureDescriptor *desc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                     width:info.width
                                    height:info.height
                                 mipmapped:NO];
    desc.usage = MTLTextureUsageShaderRead;
    desc.storageMode = MTLStorageModeShared;

    const QSize sz(static_cast<int>(info.width), static_cast<int>(info.height));
    for (uint32_t i = 0; i < gz::ext_render::kSwapchainSlots; ++i)
    {
      IOSurfaceRef surf = static_cast<IOSurfaceRef>(info.native_handles[i]);
      if (!surf)
      {
        Release();
        return false;
      }
      id<MTLTexture> tex = [dev newTextureWithDescriptor:desc iosurface:surf plane:0];
      if (!tex)
      {
        std::cerr << "[ExternalRendererGui] newTextureWithDescriptor:iosurface failed for slot "
                  << i << std::endl;
        Release();
        return false;
      }
      QSGTexture *qsgTex = QNativeInterface::QSGMetalTexture::fromNative(tex, window, sz);
      if (!qsgTex)
      {
        std::cerr << "[ExternalRendererGui] QSGMetalTexture::fromNative failed for slot "
                  << i << std::endl;
        Release();
        return false;
      }
      metalTextures_[i] = tex;
      qsgTextures_[i] = qsgTex;
    }

    generation_ = info.generation;
    return true;
  }

  QSGTexture *TextureForSlot(uint32_t slot) const override
  {
    if (slot >= gz::ext_render::kSwapchainSlots) return nullptr;
    return qsgTextures_[slot];
  }

  uint32_t Generation() const override { return generation_; }

  void Release() override
  {
    for (auto &t : qsgTextures_)
    {
      delete t;
      t = nullptr;
    }
    for (auto &mt : metalTextures_)
    {
      mt = nil;
    }
    generation_ = 0;
  }

 private:
  uint32_t generation_ = 0;
  std::array<id<MTLTexture>, gz::ext_render::kSwapchainSlots> metalTextures_ = {nil, nil, nil};
  std::array<QSGTexture *, gz::ext_render::kSwapchainSlots> qsgTextures_ = {
      nullptr, nullptr, nullptr};
};

std::unique_ptr<ExternalTextureBridge> CreatePlatformTextureBridge()
{
  return std::make_unique<MetalTextureBridge>();
}

}  // namespace gz::gui::plugins

#endif  // __APPLE__
