#include "ExternalRendererGui.hh"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QQmlEngine>
#include <QQuickWindow>
#include <QSGSimpleTextureNode>
#include <QWheelEvent>

#include <dlfcn.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <sstream>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

namespace fs = std::filesystem;

namespace gz::gui::plugins
{

namespace
{

uint32_t QtButtonToWire(Qt::MouseButton btn)
{
  switch (btn)
  {
    case Qt::LeftButton: return 1;
    case Qt::RightButton: return 2;
    case Qt::MiddleButton: return 4;
    default: return 0;
  }
}

uint32_t QtButtonsToMask(Qt::MouseButtons btns)
{
  uint32_t m = 0;
  if (btns & Qt::LeftButton) m |= 1;
  if (btns & Qt::RightButton) m |= 2;
  if (btns & Qt::MiddleButton) m |= 4;
  return m;
}

uint32_t QtModifiersToWire(Qt::KeyboardModifiers mods)
{
  uint32_t m = 0;
  if (mods & Qt::ShiftModifier) m |= 1;
  if (mods & Qt::ControlModifier) m |= 2;
  if (mods & Qt::AltModifier) m |= 4;
  if (mods & Qt::MetaModifier) m |= 8;
  return m;
}

bool ParsePose6(const std::string &str, gz::math::Pose3d &outPose)
{
  std::istringstream iss(str);
  double x, y, z, r, p, yaw;
  if (!(iss >> x >> y >> z >> r >> p >> yaw)) return false;
  outPose.Set(x, y, z, r, p, yaw);
  return true;
}

}  // namespace

ExternalViewportItem::ExternalViewportItem(QQuickItem *parent)
    : QQuickItem(parent),
      textureBridge_(CreatePlatformTextureBridge())
{
  setFlag(ItemHasContents, true);
  setAcceptedMouseButtons(Qt::AllButtons);
  setAcceptHoverEvents(true);
  setActiveFocusOnTab(true);
  connect(this, &QQuickItem::windowChanged, this, &ExternalViewportItem::OnWindowChanged);
}

ExternalViewportItem::~ExternalViewportItem()
{
  zenoh_.Close();
  if (childProcess_ && childProcess_->state() != QProcess::NotRunning)
  {
    childProcess_->terminate();
    if (!childProcess_->waitForFinished(2500))
    {
      childProcess_->kill();
    }
  }
  gz::ext_render::ReleaseImportedSwapchain(importedInfo_);
}

void ExternalViewportItem::OnWindowChanged(QQuickWindow *win)
{
  if (win)
  {
    connect(win, &QQuickWindow::sceneGraphInvalidated,
            this, &ExternalViewportItem::OnSceneGraphInvalidated,
            Qt::DirectConnection);
    if (!started_)
    {
      StartSession();
    }
    SendResizeIfValid();
  }
}

void ExternalViewportItem::OnSceneGraphInvalidated()
{
  if (textureBridge_)
  {
    textureBridge_->Release();
  }
  gz::ext_render::ReleaseImportedSwapchain(importedInfo_);
}

void ExternalViewportItem::releaseResources()
{
  if (textureBridge_)
  {
    textureBridge_->Release();
  }
  gz::ext_render::ReleaseImportedSwapchain(importedInfo_);
}

void ExternalViewportItem::Configure(const std::string &session,
                                     const std::string &endpoint,
                                     const std::string &rendererBin,
                                     bool autoSpawn,
                                     bool hasCameraPose,
                                     const gz::math::Pose3d &cameraPose,
                                     float cameraHfov)
{
  sessionName_ = session;
  zenohEndpoint_ = endpoint;
  rendererBin_ = rendererBin;
  autoSpawn_ = autoSpawn;
  hasCameraPose_ = hasCameraPose;
  initialCameraPose_ = cameraPose;
  initialCameraHfov_ = cameraHfov;

  if (!started_)
  {
    StartSession();
  }
}

std::string ExternalViewportItem::FindRendererBinary() const
{
  if (!rendererBin_.empty() && fs::exists(rendererBin_)) return rendererBin_;
  if (const char *env = std::getenv("GZ_EXT_RENDER_BIN"))
  {
    if (env[0] != '\0' && fs::exists(env)) return env;
  }

  Dl_info dlInfo{};
  if (dladdr(reinterpret_cast<const void *>(&ParsePose6), &dlInfo) != 0 && dlInfo.dli_fname)
  {
    fs::path libDir = fs::path(dlInfo.dli_fname).parent_path();
    const std::vector<fs::path> candidates = {
        libDir / "../blender_renderer/gz_blender_renderer",
        libDir / "gz_bevy_renderer",
        libDir / "../bevy_renderer/target/release/gz_bevy_renderer",
        libDir / "../bevy_renderer/target/debug/gz_bevy_renderer",
    };
    for (const auto &c : candidates)
    {
      if (fs::exists(c)) return fs::weakly_canonical(c).string();
    }
  }
  return {};
}

void ExternalViewportItem::StartSession()
{
  if (started_) return;
  started_ = true;

  const std::string serviceName = gz::ext_render::GpuIpcServiceName(sessionName_);
  const bool alreadyRunning = gz::ext_render::IsExternalRendererAlive(serviceName);

  if (alreadyRunning)
  {
    gzmsg << "[ExternalRendererGui] Attaching to running external renderer ('"
          << serviceName << "')" << std::endl;
    statusText_ = "Connected (external renderer process)";
  }
  else if (autoSpawn_)
  {
    const std::string bin = FindRendererBinary();
    if (!bin.empty())
    {
      childProcess_ = std::make_unique<QProcess>();
      childProcess_->setProcessChannelMode(QProcess::ForwardedChannels);
      QStringList args;
      args << "--session" << QString::fromStdString(sessionName_)
           << "--endpoint" << QString::fromStdString(zenohEndpoint_)
           << "--width" << QString::number(std::max(320, static_cast<int>(width() > 64 ? width() : 1280)))
           << "--height" << QString::number(std::max(240, static_cast<int>(height() > 64 ? height() : 720)));
      if (hasCameraPose_)
      {
        const auto &p = initialCameraPose_.Pos();
        const auto r = initialCameraPose_.Rot().Euler();
        args << "--camera-pose"
             << QString::number(p.X()) << QString::number(p.Y()) << QString::number(p.Z())
             << QString::number(r.X()) << QString::number(r.Y()) << QString::number(r.Z())
             << "--camera-hfov" << QString::number(initialCameraHfov_);
      }
      // Forward optional extra CLI flags (e.g. "--engine cycles --show-blender")
      if (const char *extra = std::getenv("GZ_EXT_RENDER_ARGS"))
      {
        std::istringstream iss(extra);
        std::string tok;
        while (iss >> tok)
        {
          args << QString::fromStdString(tok);
        }
      }
      gzmsg << "[ExternalRendererGui] Auto-spawning external renderer: " << bin << std::endl;
      childProcess_->start(QString::fromStdString(bin), args);
      statusText_ = "Starting external renderer...";
    }
    else
    {
      gzwarn << "[ExternalRendererGui] Could not locate external renderer binary. "
             << "Launch it manually or set GZ_EXT_RENDER_BIN." << std::endl;
      statusText_ = "Waiting for external renderer process...";
    }
  }
  else
  {
    statusText_ = "Waiting for external renderer process...";
  }

  // Open Zenoh peer connecting to external renderer listener
  if (!zenoh_.Open(zenohEndpoint_, false, 0))
  {
    statusText_ = "Zenoh connection error";
    emit StatsChanged();
    return;
  }

  const std::string prefix = "gz_ext/" + sessionName_;
  inputPub_ = zenoh_.DeclarePublisher(prefix + "/gui/input");
  slotPub_ = zenoh_.DeclarePublisher(prefix + "/gui/slot_acquired");

  zenoh_.DeclareSubscriber(
      prefix + "/gui/frame_ready",
      [this](const uint8_t *data, size_t len, bool /*is_shm*/) {
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
          std::lock_guard<std::mutex> lk(stateMtx_);
          latestFrame_ = fw;
          hasFrame_ = true;
        }
        QMetaObject::invokeMethod(this, "OnFrameArrivedOnMainThread", Qt::QueuedConnection);
      });

  emit StatsChanged();
}

void ExternalViewportItem::RefreshEngineLabel()
{
  if (IsBlender())
  {
    const bool cycles = (qualityFlags_ & gz::ext_render::kBlenderCycles) != 0;
    engineLabel_ = cycles ? "Blender 5.0 | Cycles Metal" : "Blender 5.0 | EEVEE Metal";
    statusText_ = engineLabel_ + " | Metal IOSurface (3x) | Zenoh SHM";
  }
  else
  {
    engineLabel_ = "Bevy 0.19";
    statusText_ = "Bevy 0.19 | Metal IOSurface (3x) | Zenoh SHM";
  }
}

void ExternalViewportItem::OnFrameArrivedOnMainThread()
{
  gz::ext_render::FrameReadyWire fw{};
  {
    std::lock_guard<std::mutex> lk(stateMtx_);
    if (!hasFrame_) return;
    fw = latestFrame_;
  }

  if (!sentInitialResize_)
  {
    sentInitialResize_ = true;
    SendResizeIfValid();
  }

  if (!sentInitialCamera_ && hasCameraPose_)
  {
    sentInitialCamera_ = true;
    gz::ext_render::GuiInputWire w{};
    w.kind = static_cast<uint8_t>(gz::ext_render::InputKind::SetCameraPose);
    w.camera_pose.px = static_cast<float>(initialCameraPose_.Pos().X());
    w.camera_pose.py = static_cast<float>(initialCameraPose_.Pos().Y());
    w.camera_pose.pz = static_cast<float>(initialCameraPose_.Pos().Z());
    w.camera_pose.qx = static_cast<float>(initialCameraPose_.Rot().X());
    w.camera_pose.qy = static_cast<float>(initialCameraPose_.Rot().Y());
    w.camera_pose.qz = static_cast<float>(initialCameraPose_.Rot().Z());
    w.camera_pose.qw = static_cast<float>(initialCameraPose_.Rot().W());
    w.camera_hfov = initialCameraHfov_;
    SendInput(w);
  }

  update();

  const bool isBlenderFrame =
      fw.backend == static_cast<uint8_t>(gz::ext_render::RendererBackend::Blender);
  const bool shouldRefreshHud =
      !connected_ || isBlenderFrame || fw.frame_id >= lastHudFrame_ + 10 ||
      fw.quality_flags != qualityFlags_ || fw.backend != backend_ ||
      (fw.refining != 0) != refining_ || fw.sample_count != sampleCount_;

  if (shouldRefreshHud)
  {
    connected_ = true;
    lastHudFrame_ = fw.frame_id;
    fps_ = fw.fps;
    renderMs_ = fw.render_ms;
    viewportWidth_ = static_cast<int>(fw.width);
    viewportHeight_ = static_cast<int>(fw.height);
    entityCount_ = static_cast<int>(fw.entity_count);
    qualityFlags_ = fw.quality_flags;
    backend_ = fw.backend;
    refining_ = (fw.refining != 0);
    sampleCount_ = fw.sample_count;
    RefreshEngineLabel();
    emit StatsChanged();
  }
}

QSGNode *ExternalViewportItem::updatePaintNode(QSGNode *oldNode, UpdatePaintNodeData *)
{
  gz::ext_render::FrameReadyWire fw{};
  {
    std::lock_guard<std::mutex> lk(stateMtx_);
    if (!hasFrame_) return oldNode;
    fw = latestFrame_;
  }

  if (!window() || !textureBridge_) return oldNode;

  if (fw.generation != textureBridge_->Generation())
  {
    const std::string serviceName = gz::ext_render::GpuIpcServiceName(sessionName_);
    if (gz::ext_render::FetchExternalSwapchain(serviceName, fw.generation, importedInfo_, 1500))
    {
      if (!textureBridge_->ImportSwapchain(window(), importedInfo_))
      {
        gzerr << "[ExternalRendererGui] Failed to import Metal swapchain gen="
              << importedInfo_.generation << std::endl;
      }
      else
      {
        gzmsg << "[ExternalRendererGui] Imported zero-copy Metal IOSurface swapchain "
              << importedInfo_.width << "x" << importedInfo_.height
              << " (gen=" << importedInfo_.generation << ")" << std::endl;
      }
    }
  }

  QSGTexture *tex = textureBridge_->TextureForSlot(fw.slot);
  if (!tex) return oldNode;

  auto *node = static_cast<QSGSimpleTextureNode *>(oldNode);
  if (!node)
  {
    node = new QSGSimpleTextureNode();
    node->setOwnsTexture(false);
    node->setFiltering(QSGTexture::Linear);
  }
  node->setTexture(tex);
  node->setRect(boundingRect());
  node->markDirty(QSGNode::DirtyMaterial);

  if (slotPub_)
  {
    gz::ext_render::SlotAcquiredWire ack{};
    ack.generation = textureBridge_->Generation();
    ack.active_slot = fw.slot;
    gz::ext_render::PacketBuilder pb;
    pb.Reset(gz::ext_render::MsgType::SlotAcquired, 1);
    pb.AppendPod(ack);
    slotPub_->Publish(pb.Finish(), false);
  }

  return node;
}

void ExternalViewportItem::SendInput(const gz::ext_render::GuiInputWire &wire,
                                     const std::string &path)
{
  if (!inputPub_) return;
  auto w = wire;
  w.path_len = static_cast<uint32_t>(path.size());
  gz::ext_render::PacketBuilder pb;
  pb.Reset(gz::ext_render::MsgType::GuiInput, 1);
  pb.AppendPod(w);
  if (!path.empty()) pb.AppendString(path);
  inputPub_->Publish(pb.Finish(), false);
}

void ExternalViewportItem::SendResizeIfValid()
{
  if (!window()) return;
  const int w = static_cast<int>(width());
  const int h = static_cast<int>(height());
  if (w < 64 || h < 64) return;
  gz::ext_render::GuiInputWire ev{};
  ev.kind = static_cast<uint8_t>(gz::ext_render::InputKind::Resize);
  ev.width = static_cast<uint32_t>(w);
  ev.height = static_cast<uint32_t>(h);
  ev.dpr = static_cast<float>(window()->devicePixelRatio());
  SendInput(ev);
}

void ExternalViewportItem::geometryChange(const QRectF &newGeometry, const QRectF &oldGeometry)
{
  QQuickItem::geometryChange(newGeometry, oldGeometry);
  if (newGeometry.size() != oldGeometry.size())
  {
    SendResizeIfValid();
  }
}

void ExternalViewportItem::mousePressEvent(QMouseEvent *event)
{
  forceActiveFocus();
  lastMousePos_ = event->position();
  gz::ext_render::GuiInputWire ev{};
  ev.kind = static_cast<uint8_t>(gz::ext_render::InputKind::MouseDown);
  ev.x = static_cast<float>(event->position().x());
  ev.y = static_cast<float>(event->position().y());
  ev.button = QtButtonToWire(event->button());
  ev.buttons_mask = QtButtonsToMask(event->buttons());
  ev.modifiers = QtModifiersToWire(event->modifiers());
  SendInput(ev);
  event->accept();
}

void ExternalViewportItem::mouseReleaseEvent(QMouseEvent *event)
{
  gz::ext_render::GuiInputWire ev{};
  ev.kind = static_cast<uint8_t>(gz::ext_render::InputKind::MouseUp);
  ev.x = static_cast<float>(event->position().x());
  ev.y = static_cast<float>(event->position().y());
  ev.button = QtButtonToWire(event->button());
  ev.buttons_mask = QtButtonsToMask(event->buttons());
  ev.modifiers = QtModifiersToWire(event->modifiers());
  SendInput(ev);
  event->accept();
}

void ExternalViewportItem::mouseMoveEvent(QMouseEvent *event)
{
  const QPointF pos = event->position();
  const QPointF delta = pos - lastMousePos_;
  lastMousePos_ = pos;
  gz::ext_render::GuiInputWire ev{};
  ev.kind = static_cast<uint8_t>(gz::ext_render::InputKind::MouseMove);
  ev.x = static_cast<float>(pos.x());
  ev.y = static_cast<float>(pos.y());
  ev.dx = static_cast<float>(delta.x());
  ev.dy = static_cast<float>(delta.y());
  ev.buttons_mask = QtButtonsToMask(event->buttons());
  ev.modifiers = QtModifiersToWire(event->modifiers());
  SendInput(ev);
  event->accept();
}

void ExternalViewportItem::wheelEvent(QWheelEvent *event)
{
  gz::ext_render::GuiInputWire ev{};
  ev.kind = static_cast<uint8_t>(gz::ext_render::InputKind::Wheel);
  ev.x = static_cast<float>(event->position().x());
  ev.y = static_cast<float>(event->position().y());
  ev.dx = static_cast<float>(event->angleDelta().x());
  ev.dy = static_cast<float>(event->angleDelta().y());
  ev.modifiers = QtModifiersToWire(event->modifiers());
  SendInput(ev);
  event->accept();
}

void ExternalViewportItem::keyPressEvent(QKeyEvent *event)
{
  gz::ext_render::GuiInputWire ev{};
  ev.kind = static_cast<uint8_t>(gz::ext_render::InputKind::KeyDown);
  ev.key_code = static_cast<uint32_t>(event->key());
  ev.modifiers = QtModifiersToWire(event->modifiers());
  SendInput(ev);
  event->accept();
}

void ExternalViewportItem::keyReleaseEvent(QKeyEvent *event)
{
  gz::ext_render::GuiInputWire ev{};
  ev.kind = static_cast<uint8_t>(gz::ext_render::InputKind::KeyUp);
  ev.key_code = static_cast<uint32_t>(event->key());
  ev.modifiers = QtModifiersToWire(event->modifiers());
  SendInput(ev);
  event->accept();
}

void ExternalViewportItem::toggleQualityFlag(int bit)
{
  qualityFlags_ ^= static_cast<uint32_t>(bit);
  gz::ext_render::GuiInputWire ev{};
  ev.kind = static_cast<uint8_t>(gz::ext_render::InputKind::QualityConfig);
  ev.quality_flags = qualityFlags_;
  SendInput(ev);
  RefreshEngineLabel();
  emit StatsChanged();
}

void ExternalViewportItem::setBlenderEngine(bool useCycles)
{
  if (useCycles)
    qualityFlags_ |= gz::ext_render::kBlenderCycles;
  else
    qualityFlags_ &= ~gz::ext_render::kBlenderCycles;

  gz::ext_render::GuiInputWire ev{};
  ev.kind = static_cast<uint8_t>(gz::ext_render::InputKind::QualityConfig);
  ev.quality_flags = qualityFlags_;
  SendInput(ev);
  RefreshEngineLabel();
  emit StatsChanged();
}

void ExternalViewportItem::setQualityPreset(const QString &preset)
{
  if (IsBlender())
  {
    if (preset == "interactive")
    {
      qualityFlags_ = gz::ext_render::kBlenderIbl | gz::ext_render::kBlenderRtGi |
                      gz::ext_render::kBlenderDenoise | gz::ext_render::kBlenderAgx |
                      gz::ext_render::kBlenderShadows;
    }
    else if (preset == "photoreal")
    {
      qualityFlags_ = gz::ext_render::kBlenderCycles | gz::ext_render::kBlenderIbl |
                      gz::ext_render::kBlenderRtGi | gz::ext_render::kBlenderDenoise |
                      gz::ext_render::kBlenderAgx | gz::ext_render::kBlenderShadows |
                      gz::ext_render::kBlenderHighQuality;
    }
    else
    {
      qualityFlags_ = gz::ext_render::kBlenderDefault;
    }
  }
  else
  {
    if (preset == "strict")
    {
      qualityFlags_ = gz::ext_render::kQualityShadows | gz::ext_render::kQualityMsaa;
    }
    else if (preset == "ssao")
    {
      qualityFlags_ = gz::ext_render::kQualityIbl | gz::ext_render::kQualitySsao |
                      gz::ext_render::kQualityBloom | gz::ext_render::kQualityAgx |
                      gz::ext_render::kQualityShadows;
    }
    else
    {
      qualityFlags_ = gz::ext_render::kQualityDefault;
    }
  }
  gz::ext_render::GuiInputWire ev{};
  ev.kind = static_cast<uint8_t>(gz::ext_render::InputKind::QualityConfig);
  ev.quality_flags = qualityFlags_;
  SendInput(ev);
  RefreshEngineLabel();
  emit StatsChanged();
}

void ExternalViewportItem::saveScreenshot()
{
  gz::ext_render::GuiInputWire ev{};
  ev.kind = static_cast<uint8_t>(gz::ext_render::InputKind::SaveScreenshot);
  SendInput(ev);
}

void ExternalViewportItem::saveScene()
{
  gz::ext_render::GuiInputWire ev{};
  ev.kind = static_cast<uint8_t>(gz::ext_render::InputKind::SaveScene);
  SendInput(ev);
}

void ExternalViewportItem::resetCamera()
{
  gz::ext_render::GuiInputWire ev{};
  ev.kind = static_cast<uint8_t>(gz::ext_render::InputKind::ResetCamera);
  SendInput(ev);
}

ExternalRendererGui::ExternalRendererGui() : Plugin()
{
  qmlRegisterType<ExternalViewportItem>("ExternalRenderer", 1, 0, "ExternalViewport");
}

ExternalRendererGui::~ExternalRendererGui() = default;

void ExternalRendererGui::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty()) this->title = "External Renderer";

  auto *viewport = this->PluginItem()
                       ? this->PluginItem()->findChild<ExternalViewportItem *>()
                       : nullptr;
  if (!viewport)
  {
    gzerr << "[ExternalRendererGui] Could not find ExternalViewport item in QML" << std::endl;
    return;
  }

  std::string session = gz::ext_render::DefaultSessionName();
  std::string endpoint = gz::ext_render::DefaultZenohEndpoint();
  std::string rendererBin;
  bool autoSpawn = true;
  bool hasCamPose = false;
  gz::math::Pose3d camPose;
  float camHfov = 1.0472f;

  if (_pluginElem)
  {
    if (auto *el = _pluginElem->FirstChildElement("session"); el && el->GetText())
      session = el->GetText();
    if (auto *el = _pluginElem->FirstChildElement("endpoint"); el && el->GetText())
      endpoint = el->GetText();
    if (auto *el = _pluginElem->FirstChildElement("renderer_bin"); el && el->GetText())
      rendererBin = el->GetText();
    if (auto *el = _pluginElem->FirstChildElement("auto_spawn"); el && el->GetText())
      autoSpawn = (std::string(el->GetText()) != "false" && std::string(el->GetText()) != "0");
    if (auto *el = _pluginElem->FirstChildElement("camera_pose"); el && el->GetText())
      hasCamPose = ParsePose6(el->GetText(), camPose);
    if (auto *el = _pluginElem->FirstChildElement("horizontal_fov"); el && el->GetText())
    {
      double degOrRad = std::stod(el->GetText());
      camHfov = static_cast<float>(degOrRad > 3.2 ? degOrRad * M_PI / 180.0 : degOrRad);
    }
  }

  viewport->Configure(session, endpoint, rendererBin, autoSpawn, hasCamPose, camPose, camHfov);
}

}  // namespace gz::gui::plugins

GZ_ADD_PLUGIN(
    gz::gui::plugins::ExternalRendererGui,
    gz::gui::Plugin)
