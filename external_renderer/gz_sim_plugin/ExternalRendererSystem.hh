#ifndef GZ_SIM_SYSTEMS_EXTERNAL_RENDERER_SYSTEM_HH_
#define GZ_SIM_SYSTEMS_EXTERNAL_RENDERER_SYSTEM_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz::sim::systems
{

class ExternalRendererSystemPrivate;

/// \brief Gazebo Sim system plugin that streams simulation scene state,
/// hybrid geometry (primitives, native glTF paths, and raw gz-common meshes/textures),
/// lights, and high-frequency pose updates to an external Bevy renderer over Zenoh SHM.
class ExternalRendererSystem
    : public System,
      public ISystemConfigure,
      public ISystemPostUpdate
{
 public:
  ExternalRendererSystem();
  ~ExternalRendererSystem() override;

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  void PostUpdate(const UpdateInfo &_info,
                  const EntityComponentManager &_ecm) override;

 private:
  std::unique_ptr<ExternalRendererSystemPrivate> dataPtr;
};

}  // namespace gz::sim::systems

#endif  // GZ_SIM_SYSTEMS_EXTERNAL_RENDERER_SYSTEM_HH_
