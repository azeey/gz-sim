#include <iostream>
#include <mutex>
#include <thread>
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>

#include <QEvent>
#include <QTimer>
#include <QMainWindow>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/StringUtils.hh>
#include <gz/common/SystemPaths.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>

#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include "gz/sim/InstallationDirectories.hh"
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/rendering/RenderUtil.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Visual.hh>

#include <sdf/Root.hh>
#include <sdf/Error.hh>
#include <sdf/parser.hh>

using namespace gz;
using namespace sim;

class SdfViewer : public QObject
{
  public: explicit SdfViewer(const std::string &_path)
    : sdfPath(_path)
  {
    this->renderUtil = std::make_unique<RenderUtil>();
    this->eventMgr = std::make_unique<EventManager>();
    this->entityCreator = std::make_unique<SdfEntityCreator>(this->ecm, *this->eventMgr);

    // Setup file watcher
    this->lastWriteTime = std::filesystem::last_write_time(this->sdfPath);
    this->watcherThread = std::thread([this]() {
      while (!this->stopWatcher)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        try
        {
          auto currentWriteTime = std::filesystem::last_write_time(this->sdfPath);
          if (currentWriteTime != this->lastWriteTime)
          {
            this->lastWriteTime = currentWriteTime;
            this->reloadNeeded = true;
          }
        }
        catch (const std::exception &e)
        {
          gzerr << "Error watching file: " << e.what() << std::endl;
        }
      }
    });
  }

  public: ~SdfViewer()
  {
    this->stopWatcher = true;
    if (this->watcherThread.joinable())
      this->watcherThread.join();
  }

  protected: bool eventFilter(QObject *_obj, QEvent *_event) override
  {
    if (_event->type() == gz::gui::events::Render::kType)
    {
      if (!this->initialized)
      {
        this->Initialize();
        this->reloadNeeded = true; // Trigger initial load
      }

      if (this->reloadNeeded)
      {
        this->Reload();
        this->reloadNeeded = false;
      }

      if (this->initialized)
      {
         // Update the scene
         // UpdateInfo info;
         // info.simTime = std::chrono::steady_clock::duration::zero();
         // info.dt = std::chrono::steady_clock::duration::zero();
         // this->renderUtil->UpdateFromECM(info, this->ecm);
         this->renderUtil->Update();
      }
    }
    return QObject::eventFilter(_obj, _event);
  }

  private: void Initialize()
  {
    auto scene = rendering::sceneFromFirstRenderEngine();
    if (!scene)
    {
      return;
    }

    this->renderUtil->SetScene(scene);
    this->renderUtil->Init();
    this->initialized = true;
  }

  private: void Reload()
  {
    if (!this->initialized)
      return;

    gzmsg << "Reloading SDF: " << this->sdfPath << std::endl;

    // Remove all entities from ECM.
    this->ecm.Each<components::Name>([&](const Entity &_entity, const components::Name *)->bool{
      this->ecm.RequestRemoveEntity(_entity, true);
      return true;
    });
    
    // Process the removal in RenderUtil
    UpdateInfo info;
    this->renderUtil->UpdateFromECM(info, this->ecm);
    
    this->ecm.ProcessRemoveEntityRequests();

    // Now load the new SDF
    sdf::Root root;
    sdf::Errors errors = root.Load(this->sdfPath);
    if (!errors.empty())
    {
      for (const auto &e : errors)
        gzerr << e << std::endl;
      return;
    }

    // Create entities
    if (root.WorldCount() > 0)
    {
      for (uint64_t i = 0; i < root.WorldCount(); ++i)
      {
        auto world = root.WorldByIndex(i);
        if (world)
          this->entityCreator->CreateEntities(world);
      }
    }
    else if (root.Model())
    {
      this->entityCreator->CreateEntities(root.Model());
    }
    
    gzmsg << "Created entities. Total entities: " << this->ecm.EntityCount() << std::endl;
    
    // Check new entities
    int newCount = 0;
    this->ecm.EachNew<components::Name>([&](const Entity &, const components::Name *){
      newCount++;
      return true;
    });
    gzmsg << "New entities count: " << newCount << std::endl;

    // Create visuals for the new entities
    this->renderUtil->UpdateFromECM(info, this->ecm);
    this->renderUtil->Update();
    
    gzmsg << "Visual count in scene: " << this->renderUtil->Scene()->VisualCount() << std::endl;
  }

  private: std::string sdfPath;
  private: EntityComponentManager ecm;
  private: std::unique_ptr<EventManager> eventMgr;
  private: std::unique_ptr<SdfEntityCreator> entityCreator;
  private: std::unique_ptr<RenderUtil> renderUtil;
  
  private: std::thread watcherThread;
  private: std::atomic<bool> stopWatcher{false};
  private: std::atomic<bool> reloadNeeded{false};
  private: bool initialized{false};
  private: std::filesystem::file_time_type lastWriteTime;
};

int main(int argc, char **argv)
{
  std::string file_path = "";
  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    if ((arg == "--file" || arg == "-f") && i + 1 < argc)
    {
      file_path = argv[i + 1];
      break;
    }
  }

  if (file_path.empty())
  {
    std::cerr << "Usage: viewer --file <path_to_sdf>" << std::endl;
    return 1;
  }
  
  if (!std::filesystem::exists(file_path))
  {
    std::cerr << "File not found: " << file_path << std::endl;
    return 1;
  }

  gz::common::Console::SetVerbosity(4);

  // Initialize GUI application
  gz::gui::Application app(argc, argv);

  app.AddPluginPath(gz::sim::getGUIPluginInstallDir());

  // Configure the GUI
  std::string config = R"(
    <?xml version="1.0"?>
    <plugin filename="MinimalScene" name="3D View">
      <gz-gui>
        <title>SDF Viewer</title>
        <property type="bool" key="showTitleBar">false</property>
        <property type="string" key="state">docked</property>
      </gz-gui>
      <engine>ogre2</engine>
      <scene>scene</scene>
      <ambient_light>0.4 0.4 0.4</ambient_light>
      <background_color>0.8 0.8 0.8</background_color>
      <camera_pose>-6 0 6 0 0.5 0</camera_pose>
    </plugin>
    <plugin filename="InteractiveViewControl" name="Interactive view control">
      <gz-gui>
        <property key="resizable" type="bool">false</property>
        <property key="width" type="double">5</property>
        <property key="height" type="double">5</property>
        <property key="state" type="string">floating</property>
        <property key="showTitleBar" type="bool">false</property>
      </gz-gui>
    </plugin>
    <plugin filename="SelectEntities" name="Select Entities">
      <gz-gui>
        <property key="resizable" type="bool">false</property>
        <property key="width" type="double">5</property>
        <property key="height" type="double">5</property>
        <property key="state" type="string">floating</property>
        <property key="showTitleBar" type="bool">false</property>
      </gz-gui>
    </plugin>
  )";

  // Write config to a temporary file
  std::string tempConfigPath = std::filesystem::temp_directory_path() / "viewer_config.config";
  std::ofstream configFile(tempConfigPath);
  if (configFile.is_open())
  {
    configFile << config;
    configFile.close();
  }
  else
  {
    std::cerr << "Failed to write config to " << tempConfigPath << std::endl;
    return 1;
  }

  if (!app.LoadConfig(tempConfigPath))
  {
    std::cerr << "Failed to load config" << std::endl;
    return 1;
  }

  // Initialize and install the viewer
  SdfViewer viewer(file_path);
  auto win = app.findChild<gz::gui::MainWindow *>();
  if (!win)
  {
    std::cerr << "Main window not found" << std::endl;
    return 1;
  }
  
  win->installEventFilter(&viewer);

  return app.exec();
}
