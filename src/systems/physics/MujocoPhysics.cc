#include <gz/plugin/Register.hh>
#include "gz/sim/System.hh"
#include "gz/sim/config.hh"

// namespace components = gz::sim::components;

namespace gz
{
namespace sim
{
namespace systems
{

class MujocoPhysics : public System,
                      public ISystemConfigure,
                      public ISystemUpdate
{
  // Documentation inherited
  public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm, EventManager &_eventMgr) final;

  // Documentation inherited
  public:
  void Update(const UpdateInfo &_info, EntityComponentManager &_ecm) final;
};

void MujocoPhysics::Configure(const Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              EntityComponentManager &_ecm,
                              EventManager &_eventMgr)
{
}

void MujocoPhysics::Update(const UpdateInfo &_info,
                           EntityComponentManager &_ecm)
{
}

GZ_ADD_PLUGIN(MujocoPhysics, System, MujocoPhysics::ISystemConfigure,
              MujocoPhysics::ISystemUpdate)
}  // namespace systems
}  // namespace sim
}  // namespace gz
