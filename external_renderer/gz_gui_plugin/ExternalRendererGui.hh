#ifndef GZ_GUI_PLUGINS_EXTERNAL_RENDERER_GUI_HH_
#define GZ_GUI_PLUGINS_EXTERNAL_RENDERER_GUI_HH_

#include <QProcess>
#include <QQuickItem>
#include <QString>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <gz/gui/Plugin.hh>
#include <gz/math/Pose3.hh>

#include "MetalTextureBridge.hh"
#include "gpu_surface_ipc.hh"
#include "protocol.hh"
#include "zenoh_transport.hh"

namespace gz::gui::plugins
{

class ExternalViewportItem : public QQuickItem
{
  Q_OBJECT

  Q_PROPERTY(QString statusText READ StatusText NOTIFY StatsChanged)
  Q_PROPERTY(double fps READ Fps NOTIFY StatsChanged)
  Q_PROPERTY(double renderMs READ RenderMs NOTIFY StatsChanged)
  Q_PROPERTY(int viewportWidth READ ViewportWidth NOTIFY StatsChanged)
  Q_PROPERTY(int viewportHeight READ ViewportHeight NOTIFY StatsChanged)
  Q_PROPERTY(int entityCount READ EntityCount NOTIFY StatsChanged)
  Q_PROPERTY(int qualityFlags READ QualityFlags NOTIFY StatsChanged)
  Q_PROPERTY(bool connected READ Connected NOTIFY StatsChanged)
  Q_PROPERTY(int backend READ Backend NOTIFY StatsChanged)
  Q_PROPERTY(bool isBlender READ IsBlender NOTIFY StatsChanged)
  Q_PROPERTY(bool refining READ Refining NOTIFY StatsChanged)
  Q_PROPERTY(int sampleCount READ SampleCount NOTIFY StatsChanged)
  Q_PROPERTY(QString engineLabel READ EngineLabel NOTIFY StatsChanged)

 public:
  explicit ExternalViewportItem(QQuickItem *parent = nullptr);
  ~ExternalViewportItem() override;

  void Configure(const std::string &session,
                 const std::string &endpoint,
                 const std::string &rendererBin,
                 bool autoSpawn,
                 bool hasCameraPose,
                 const gz::math::Pose3d &cameraPose,
                 float cameraHfov);

  QString StatusText() const { return statusText_; }
  double Fps() const { return fps_; }
  double RenderMs() const { return renderMs_; }
  int ViewportWidth() const { return viewportWidth_; }
  int ViewportHeight() const { return viewportHeight_; }
  int EntityCount() const { return entityCount_; }
  int QualityFlags() const { return static_cast<int>(qualityFlags_); }
  bool Connected() const { return connected_; }
  int Backend() const { return static_cast<int>(backend_); }
  bool IsBlender() const
  {
    return backend_ == static_cast<uint8_t>(gz::ext_render::RendererBackend::Blender);
  }
  bool Refining() const { return refining_; }
  int SampleCount() const { return static_cast<int>(sampleCount_); }
  QString EngineLabel() const { return engineLabel_; }

  Q_INVOKABLE void toggleQualityFlag(int bit);
  Q_INVOKABLE void setBlenderEngine(bool useCycles);
  Q_INVOKABLE void setQualityPreset(const QString &preset);
  Q_INVOKABLE void saveScreenshot();
  Q_INVOKABLE void saveScene();
  Q_INVOKABLE void resetCamera();

 signals:
  void StatsChanged();

 protected:
  QSGNode *updatePaintNode(QSGNode *oldNode, UpdatePaintNodeData *data) override;
  void geometryChange(const QRectF &newGeometry, const QRectF &oldGeometry) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;
  void keyReleaseEvent(QKeyEvent *event) override;
  void releaseResources() override;

 private slots:
  void OnFrameArrivedOnMainThread();
  void OnWindowChanged(QQuickWindow *win);
  void OnSceneGraphInvalidated();

 private:
  void StartSession();
  void SendInput(const gz::ext_render::GuiInputWire &wire, const std::string &path = {});
  void SendResizeIfValid();
  std::string FindRendererBinary() const;
  void RefreshEngineLabel();

  std::string sessionName_ = gz::ext_render::DefaultSessionName();
  std::string zenohEndpoint_ = gz::ext_render::DefaultZenohEndpoint();
  std::string rendererBin_;
  bool autoSpawn_ = true;
  bool hasCameraPose_ = false;
  gz::math::Pose3d initialCameraPose_;
  float initialCameraHfov_ = 1.0472f;
  bool started_ = false;
  bool sentInitialCamera_ = false;
  bool sentInitialResize_ = false;

  std::unique_ptr<QProcess> childProcess_;
  gz::ext_render::ZenohNode zenoh_;
  std::shared_ptr<gz::ext_render::ZenohNode::Publisher> inputPub_;
  std::shared_ptr<gz::ext_render::ZenohNode::Publisher> slotPub_;

  // Cross-thread state from Zenoh subscriber -> Qt Render & Main threads
  std::mutex stateMtx_;
  gz::ext_render::FrameReadyWire latestFrame_{};
  bool hasFrame_ = false;
  QPointF lastMousePos_;

  // Owned & mutated exclusively on Qt's Scene Graph render thread
  gz::ext_render::ImportedSwapchainInfo importedInfo_;
  std::unique_ptr<ExternalTextureBridge> textureBridge_;

  // Main-thread HUD properties
  QString statusText_ = "Connecting to external renderer...";
  QString engineLabel_ = "External Renderer";
  double fps_ = 0.0;
  double renderMs_ = 0.0;
  int viewportWidth_ = 0;
  int viewportHeight_ = 0;
  int entityCount_ = 0;
  uint32_t qualityFlags_ = gz::ext_render::kQualityDefault;
  uint8_t backend_ = 0;
  bool refining_ = false;
  uint16_t sampleCount_ = 0;
  bool connected_ = false;
  uint64_t lastHudFrame_ = 0;
};

class ExternalRendererGui : public gz::gui::Plugin
{
  Q_OBJECT

 public:
  ExternalRendererGui();
  ~ExternalRendererGui() override;

  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;
};

}  // namespace gz::gui::plugins

#endif  // GZ_GUI_PLUGINS_EXTERNAL_RENDERER_GUI_HH_
