/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <mujoco/mujoco.h>

#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>

#include <gz/common/Console.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/plugin/Register.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Component.hh"
#include "gz/sim/components/Factory.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityReset.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/config.hh"

namespace components = gz::sim::components;

namespace gz
{
namespace sim
{
// Define custom components
namespace components
{
  using MujocoBodyId = gz::sim::components::Component<int, class MujocoBodyIdTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.MujocoBodyId", MujocoBodyId)

  using MujocoJointId = gz::sim::components::Component<int, class MujocoJointIdTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.MujocoJointId", MujocoJointId)
}

namespace systems
{

template<typename T>
void SetMujocoPose(T* _mujocoObject, const gz::math::Pose3d& _pose)
{
    _mujocoObject->pos[0] = _pose.Pos().X();
    _mujocoObject->pos[1] = _pose.Pos().Y();
    _mujocoObject->pos[2] = _pose.Pos().Z();
    _mujocoObject->quat[0] = _pose.Rot().W();
    _mujocoObject->quat[1] = _pose.Rot().X();
    _mujocoObject->quat[2] = _pose.Rot().Y();
    _mujocoObject->quat[3] = _pose.Rot().Z();
}

class MujocoPhysics : public System,
                      public ISystemConfigure,
                      public ISystemUpdate
{
  public: MujocoPhysics() = default;
  public: ~MujocoPhysics() override
  {
    if (this->data) mj_deleteData(this->data);
    if (this->model) mj_deleteModel(this->model);
    if (this->spec) mj_deleteSpec(this->spec);
  }

  public: void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm, EventManager &_eventMgr) final;

  public: void Update(const UpdateInfo &_info, EntityComponentManager &_ecm) final;

  private: bool NeedsRebuild(const EntityComponentManager &_ecm);
  private: void RebuildModel(EntityComponentManager &_ecm);
  
  // Helper for building spec
  struct LinkNode
  {
    Entity entity;
    std::vector<Entity> children;
    std::vector<Entity> joints;
    bool processed = false;
  };
  
  private: void AddModelToSpec(const EntityComponentManager &_ecm, mjSpec *spec, mjsBody *worldbody, Entity modelEntity);
  private: void AddBodyRecursive(const EntityComponentManager &_ecm, 
                                 mjSpec *spec,
                                 mjsBody *parentBody,
                                 Entity linkEntity,
                                 Entity jointEntity,
                                 std::unordered_map<Entity, LinkNode> &nodes,
                                 const math::Pose3d &parentPoseInModel,
                                 const math::Pose3d &modelPose,
                                 bool modelIsStatic);

  private: gz::math::Pose3d GetLinkWorldPose(int _bodyId) const;


  // Caches
  private: std::unordered_map<Entity, Entity> modelToCanonicalLink;


  // Mujoco State
  private: mjSpec *spec = nullptr;
  private: mjModel *model = nullptr;
  private: mjData *data = nullptr;
};

void MujocoPhysics::Configure(const Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              EntityComponentManager &_ecm,
                              EventManager &_eventMgr)
{
}

bool MujocoPhysics::NeedsRebuild(const EntityComponentManager &_ecm)
{
  // 1. Check for new entities using EachNew
  bool rebuild = false;
  auto check = [&](const Entity &, const auto *)
  {
    rebuild = true;
    return false;
  };

  _ecm.EachNew<components::Model>(check);
  if (rebuild) return true;

  // 2. Check for removed entities
  if (this->model)
  {
    _ecm.EachRemoved<components::Model>(check);
    if (rebuild) return true;
  }

  return false;
}

void MujocoPhysics::Update(const UpdateInfo &_info,
                           EntityComponentManager &_ecm)
{
  if (this->NeedsRebuild(_ecm))
  {
    this->RebuildModel(_ecm);
  }

  if (!this->model || !this->data)
    return;

  auto start_time = std::chrono::steady_clock::now();

  // Sync ECM -> Mujoco (Forces)
  _ecm.Each<components::Joint, components::JointForceCmd, components::MujocoJointId>(
    [&](const Entity &, const components::Joint *, const components::JointForceCmd *cmd, const components::MujocoJointId *jntIdComp)
    {
        int jntId = jntIdComp->Data();
        if (jntId >= 0 && jntId < this->model->njnt)
        {
            int dofAdr = this->model->jnt_dofadr[jntId];
            if (cmd->Data().size() > 0)
                this->data->qfrc_applied[dofAdr] = cmd->Data()[0];
        }
        return true;
    });

  auto ecm_to_mj_done_time = std::chrono::steady_clock::now();

  // Step
  if (!_info.paused)
  {
    mj_step(this->model, this->data);
  }

  auto mj_step_done_time = std::chrono::steady_clock::now();

  // Sync Mujoco -> ECM
  
  // 1. Joint Positions & Velocities
  _ecm.Each<components::Joint, components::MujocoJointId>(
    [&](const Entity &entity, const components::Joint *, const components::MujocoJointId *jntIdComp)
    {
        int jntId = jntIdComp->Data();
        if (jntId >= 0 && jntId < this->model->njnt)
        {
            int qposAdr = this->model->jnt_qposadr[jntId];
            int dofAdr = this->model->jnt_dofadr[jntId];
            
            double pos = this->data->qpos[qposAdr];
            double vel = this->data->qvel[dofAdr];
            
            auto jointPosComp = _ecm.Component<components::JointPosition>(entity);
            if (jointPosComp)
            {
              if (jointPosComp->Data().empty()) jointPosComp->Data().resize(1);
              jointPosComp->Data()[0] = pos;
              _ecm.SetChanged(entity, components::JointPosition::typeId, ComponentState::PeriodicChange);
            }
            else
            {
              _ecm.CreateComponent(entity, components::JointPosition({pos}));
            }
            
            auto jointVelComp = _ecm.Component<components::JointVelocity>(entity);
            if (jointVelComp)
            {
              if (jointVelComp->Data().empty()) jointVelComp->Data().resize(1);
              jointVelComp->Data()[0] = vel;
              _ecm.SetChanged(entity, components::JointVelocity::typeId, ComponentState::PeriodicChange);
            }
            else
            {
              _ecm.CreateComponent(entity, components::JointVelocity({vel}));
            }
        }
        return true;
    });

  // 2. Link Poses
  _ecm.Each<components::Model, components::Pose>(
    [&](const Entity &modelEntity, const components::Model *, components::Pose *modelPoseComp)
    {
      auto it = this->modelToCanonicalLink.find(modelEntity);
      if (it == this->modelToCanonicalLink.end())
      {
        return true;
      }
      Entity linkEntity = it->second;
      auto bodyIdComp = _ecm.Component<components::MujocoBodyId>(linkEntity);
      if (!bodyIdComp)
      {
        return true;
      }

      int bodyId = bodyIdComp->Data();
      if (bodyId > 0 && bodyId < this->model->nbody)
      {
          gz::math::Pose3d linkWorldPose = this->GetLinkWorldPose(bodyId);

          auto linkPoseInModelComp = _ecm.Component<components::Pose>(linkEntity);
          if (linkPoseInModelComp)
          {
            modelPoseComp->Data() = linkWorldPose * linkPoseInModelComp->Data().Inverse();
            _ecm.SetChanged(modelEntity, components::Pose::typeId, ComponentState::PeriodicChange);
          }
      }
      return true;
    });

  _ecm.Each<components::Link, components::MujocoBodyId, components::Pose>(
    [&](const Entity &entity, const components::Link *,
        const components::MujocoBodyId *bodyIdComp,
        components::Pose *poseComp)
    {
        if (_ecm.Component<components::CanonicalLink>(entity))
        {
          return true;
        }

        int bodyId = bodyIdComp->Data();
        if (bodyId > 0 && bodyId < this->model->nbody)
        {
          // Get the link's world pose from Mujoco
          gz::math::Pose3d linkWorldPose = this->GetLinkWorldPose(bodyId);

          // Get the parent model's world pose
          auto parent = _ecm.Component<components::ParentEntity>(entity);
          if (!parent)
            return true;

          auto modelWorldPoseComp = _ecm.Component<components::Pose>(parent->Data());
          if (!modelWorldPoseComp)
            return true;

          // Convert link's world pose to be relative to the model's world pose
          gz::math::Pose3d relativePose = modelWorldPoseComp->Data().Inverse() * linkWorldPose;

          // Update the component with the relative pose
          poseComp->Data() = relativePose;

          _ecm.SetChanged(entity, components::Pose::typeId,
              ComponentState::PeriodicChange);
        }
        return true;
    });

  auto mj_to_ecm_done_time = std::chrono::steady_clock::now();

  auto ecm_to_mj_dur = std::chrono::duration_cast<std::chrono::microseconds>(ecm_to_mj_done_time - start_time);
  auto mj_step_dur = std::chrono::duration_cast<std::chrono::microseconds>(mj_step_done_time - ecm_to_mj_done_time);
  auto mj_to_ecm_dur = std::chrono::duration_cast<std::chrono::microseconds>(mj_to_ecm_done_time - mj_step_done_time);

  gzdbg << "MujocoPhysics::Update timings:"
        << " ECM->MJ: " << ecm_to_mj_dur.count() << " us |"
        << " mj_step: " << mj_step_dur.count() << " us |"
        << " MJ->ECM: " << mj_to_ecm_dur.count() << " us" << std::endl;
}

void MujocoPhysics::RebuildModel(EntityComponentManager &_ecm)
{
  // if (this->model && this->data)
  // {
  //   this->Update(gz::sim::UpdateInfo{}, _ecm);
  // }

  if (this->spec) mj_deleteSpec(this->spec);
  this->spec = mj_makeSpec();
  if (!this->spec) return;

  // Set Global Options
  Entity worldEntity = _ecm.EntityByComponents(components::World());
  if (worldEntity != kNullEntity)
  {
      auto gravityComp = _ecm.Component<components::Gravity>(worldEntity);
      if (gravityComp)
      {
          this->spec->option.gravity[0] = gravityComp->Data().X();
          this->spec->option.gravity[1] = gravityComp->Data().Y();
          this->spec->option.gravity[2] = gravityComp->Data().Z();
      }
  }

  mjsBody *worldbody = mjs_findBody(this->spec, "world");
  if (!worldbody)
  {
      mjsElement *firstBody = mjs_firstElement(this->spec, mjOBJ_BODY);
      if (firstBody) worldbody = mjs_asBody(firstBody);
  }

  if (!worldbody)
  {
      gzerr << "Mujoco spec has no world body!" << std::endl;
      mj_deleteSpec(this->spec);
      this->spec = nullptr;
      return;
  }

  // Add Models
  _ecm.Each<components::Model>([&](const Entity &modelEntity, const components::Model *){
      this->AddModelToSpec(_ecm, this->spec, worldbody, modelEntity);
      return true;
  });

  // Compile / Recompile
  if (this->model)
  {
      mj_recompile(this->spec, nullptr, this->model, this->data);
  }
  else
  {
      this->model = mj_compile(this->spec, nullptr);
      if (this->model) this->data = mj_makeData(this->model);
  }

  if (!this->model)
  {
    gzerr << "Mujoco compile error: " << mjs_getError(this->spec) << std::endl;
    return;
  }

  // Save the model to XML for inspection.
  const char* mjcf_path = "model.xml";
  char error[1000] = {0};
  if (mj_saveXML(this->spec, mjcf_path, error, 1000) != 0)
  {
    gzerr << "Failed to save MJCF XML: " << error << std::endl;
  }
  else
  {
    gzdbg << "Saved MJCF model to " << mjcf_path << std::endl;
  }
  mj_forward(this->model, this->data);

  // Cache canonical links
  this->modelToCanonicalLink.clear();
  _ecm.Each<components::Model>([&](const Entity &modelEntity, const components::Model *)
  {
    auto canonicalLinks = _ecm.ChildrenByComponents(modelEntity, components::CanonicalLink());
    if (!canonicalLinks.empty())
    {
      this->modelToCanonicalLink[modelEntity] = canonicalLinks[0];
    }
    return true;
  });

  // Map Entities & Restore State
  std::unordered_map<std::string, Entity> bodyMap;
  std::unordered_map<std::string, Entity> jointMap;
  _ecm.Each<components::Model>([&](const Entity &modelEntity, const components::Model *)
  {
    auto modelName = _ecm.Component<components::Name>(modelEntity)->Data();
    auto links = _ecm.ChildrenByComponents(modelEntity, components::Link());
    for (const auto &linkEntity : links)
    {
      auto linkName = _ecm.Component<components::Name>(linkEntity)->Data();
      bodyMap[modelName + "::" + linkName] = linkEntity;
    }
    auto joints = _ecm.ChildrenByComponents(modelEntity, components::Joint());
    for (const auto &jointEntity : joints)
    {
      auto jointName = _ecm.Component<components::Name>(jointEntity)->Data();
      jointMap[modelName + "::" + jointName] = jointEntity;
    }
    return true;
  });

  for (int i = 0; i < this->model->nbody; ++i)
  {
    const char* name = mj_id2name(this->model, mjOBJ_BODY, i);
    if (name && bodyMap.count(name))
    {
      Entity e = bodyMap.at(name);
      auto comp = _ecm.Component<components::MujocoBodyId>(e);
      if (comp) comp->Data() = i;
      else _ecm.CreateComponent(e, components::MujocoBodyId(i));
    }
  }

  for (int i = 0; i < this->model->njnt; ++i)
  {
    const char* name = mj_id2name(this->model, mjOBJ_JOINT, i);
    if (name && jointMap.count(name))
    {
      Entity e = jointMap.at(name);
      auto comp = _ecm.Component<components::MujocoJointId>(e);
      if (comp) comp->Data() = i;
      else _ecm.CreateComponent(e, components::MujocoJointId(i));
          
      auto posComp = _ecm.Component<components::JointPosition>(e);
      if (posComp && !posComp->Data().empty())
      {
          this->data->qpos[this->model->jnt_qposadr[i]] = posComp->Data()[0];
      }
      auto velComp = _ecm.Component<components::JointVelocity>(e);
      if (velComp && !velComp->Data().empty())
      {
          this->data->qvel[this->model->jnt_dofadr[i]] = velComp->Data()[0];
      }
    }
  }
}

void MujocoPhysics::AddModelToSpec(const EntityComponentManager &_ecm, mjSpec *spec, mjsBody *worldbody, Entity modelEntity)
{
    std::unordered_map<Entity, LinkNode> nodes;
    std::vector<Entity> links;
    
    auto linksComp = _ecm.ChildrenByComponents(modelEntity, components::Link());
    for (const auto &linkEntity : linksComp)
    {
      LinkNode node;
      node.entity = linkEntity;
      nodes[linkEntity] = node;
      links.push_back(linkEntity);
    }

    auto jointsComp = _ecm.ChildrenByComponents(modelEntity, components::Joint());
    for (const auto &jointEntity : jointsComp)
    {
      auto parentLinkComp = _ecm.Component<components::ParentLinkName>(jointEntity);
      auto childLinkComp = _ecm.Component<components::ChildLinkName>(jointEntity);
      if (!parentLinkComp || !childLinkComp) continue;

      Entity parentLink = kNullEntity;
      Entity childLink = kNullEntity;

      for (const auto &l : links)
      {
        auto nameComp = _ecm.Component<components::Name>(l);
        if (nameComp && nameComp->Data() == parentLinkComp->Data()) parentLink = l;
        if (nameComp && nameComp->Data() == childLinkComp->Data()) childLink = l;
      }

      if (parentLink != kNullEntity && childLink != kNullEntity)
      {
        nodes[parentLink].children.push_back(childLink);
        nodes[parentLink].joints.push_back(jointEntity);
      }
    }

    std::set<Entity> children;
    for (const auto &pair : nodes)
    {
      for (const auto &child : pair.second.children)
        children.insert(child);
    }

    auto modelPoseComp = _ecm.Component<components::Pose>(modelEntity);
    math::Pose3d modelPose = modelPoseComp ? modelPoseComp->Data() : math::Pose3d::Zero;
    auto staticComp = _ecm.Component<components::Static>(modelEntity);
    bool isStatic = staticComp ? staticComp->Data() : false;

    for (const auto &linkEntity : links)
    {
      if (children.find(linkEntity) == children.end())
      {
        this->AddBodyRecursive(_ecm, spec, worldbody, linkEntity, kNullEntity, nodes, math::Pose3d::Zero, modelPose, isStatic);
      }
    }
}

void MujocoPhysics::AddBodyRecursive(const EntityComponentManager &_ecm, 
                                 mjSpec *spec,
                                 mjsBody *parentBody,
                                 Entity linkEntity,
                                 Entity jointEntity,
                                 std::unordered_map<Entity, LinkNode> &nodes,
                                 const math::Pose3d &parentPoseInModel,
                                 const math::Pose3d &modelPose,
                                 bool modelIsStatic)
{
    auto &node = nodes[linkEntity];
    if (node.processed) return;
    node.processed = true;

    auto modelName = _ecm.Component<components::Name>(
        _ecm.Component<components::ParentEntity>(linkEntity)->Data()
    )->Data();

    auto linkPoseComp = _ecm.Component<components::Pose>(linkEntity);
    math::Pose3d linkPoseInModel = linkPoseComp ? linkPoseComp->Data() : math::Pose3d::Zero;

    math::Pose3d relPose;
    // A root link is identified by not being the child of any joint.
    if (jointEntity == kNullEntity)
    {
      // For a root link, its parent in the spec is the world. Its pose should
      // be its absolute world pose.
      relPose = modelPose * linkPoseInModel;
    }
    else
    {
      // For a child link, its pose in the spec should be relative to its
      // parent link.
      relPose = parentPoseInModel.Inverse() * linkPoseInModel;
    }
    gzdbg << "link: " << _ecm.Component<components::Name>(linkEntity)->Data()
          << " modelPose: " << modelPose
          << " link in model: " << linkPoseInModel
          << " parent in model: " << parentPoseInModel
          << " relPose: " << relPose << std::endl;

    mjsBody *linkBody = mjs_addBody(parentBody, nullptr);
    if (!linkBody)
    {
      gzerr << "Failed to add body for entity " << linkEntity << std::endl;
      return;
    }
    std::string linkName = _ecm.Component<components::Name>(linkEntity)->Data();
    std::string bodyName = modelName + "::" + linkName;
    mjs_setName(linkBody->element, bodyName.c_str());
    
    SetMujocoPose(linkBody, relPose);

    if (jointEntity == kNullEntity)
    {
        if (!modelIsStatic) mjs_addFreeJoint(linkBody);
    }
    else
    {
        auto jointTypeComp = _ecm.Component<components::JointType>(jointEntity);
        auto axisComp = _ecm.Component<components::JointAxis>(jointEntity);
        auto poseComp = _ecm.Component<components::Pose>(jointEntity); 
        math::Pose3d jointPose = poseComp ? poseComp->Data() : math::Pose3d::Zero;
        
        if (jointTypeComp && jointTypeComp->Data() != sdf::JointType::FIXED)
        {
            mjsJoint *joint = mjs_addJoint(linkBody, nullptr);
            std::string jointName = _ecm.Component<components::Name>(jointEntity)->Data();
            std::string fullJointName = modelName + "::" + jointName;
            mjs_setName(joint->element, fullJointName.c_str());
            
            if (jointTypeComp->Data() == sdf::JointType::REVOLUTE) joint->type = mjJNT_HINGE;
            else if (jointTypeComp->Data() == sdf::JointType::PRISMATIC) joint->type = mjJNT_SLIDE;
            else if (jointTypeComp->Data() == sdf::JointType::BALL) joint->type = mjJNT_BALL;
            
            joint->pos[0] = jointPose.Pos().X();
            joint->pos[1] = jointPose.Pos().Y();
            joint->pos[2] = jointPose.Pos().Z();
            
            if (axisComp)
            {
                math::Vector3d axis = jointPose.Rot() * axisComp->Data().Xyz();
                joint->axis[0] = axis.X();
                joint->axis[1] = axis.Y();
                joint->axis[2] = axis.Z();
            }
        }
    }

    auto inertialComp = _ecm.Component<components::Inertial>(linkEntity);
    if (inertialComp && !modelIsStatic)
    {
        const auto &inertial = inertialComp->Data();
        linkBody->mass = inertial.MassMatrix().Mass();
        linkBody->ipos[0] = inertial.Pose().Pos().X();
        linkBody->ipos[1] = inertial.Pose().Pos().Y();
        linkBody->ipos[2] = inertial.Pose().Pos().Z();
        linkBody->iquat[0] = inertial.Pose().Rot().W();
        linkBody->iquat[1] = inertial.Pose().Rot().X();
        linkBody->iquat[2] = inertial.Pose().Rot().Y();
        linkBody->iquat[3] = inertial.Pose().Rot().Z();
        linkBody->inertia[0] = inertial.MassMatrix().Ixx();
        linkBody->inertia[1] = inertial.MassMatrix().Iyy();
        linkBody->inertia[2] = inertial.MassMatrix().Izz();
    }
    else
    {
      // For static models, or dynamic models without an <inertial> tag,
      // let mass be the default (0). Mujoco treats bodies with 0 mass as
      // being fixed to the world.
      linkBody->mass = 0;
    }

    auto collisions = _ecm.ChildrenByComponents(linkEntity, components::Collision());
    for (const auto &colEntity : collisions)
    {
        auto geomComp = _ecm.Component<components::Geometry>(colEntity);
        auto colPoseComp = _ecm.Component<components::Pose>(colEntity);
        math::Pose3d colPose = colPoseComp ? colPoseComp->Data() : math::Pose3d::Zero;
        
        if (geomComp)
        {
            const auto &geom = geomComp->Data();
            mjsGeom *mjsg = mjs_addGeom(linkBody, nullptr);
            std::string colName = _ecm.Component<components::Name>(colEntity)->Data();
            std::string fullGeomName = bodyName + "::" + colName;
            mjs_setName(mjsg->element, fullGeomName.c_str());
            
            SetMujocoPose(mjsg, colPose);
            
            if (geom.Type() == sdf::GeometryType::BOX)
            {
                mjsg->type = mjGEOM_BOX;
                mjsg->size[0] = geom.BoxShape()->Size().X()/2.0;
                mjsg->size[1] = geom.BoxShape()->Size().Y()/2.0;
                mjsg->size[2] = geom.BoxShape()->Size().Z()/2.0;
            }
            else if (geom.Type() == sdf::GeometryType::SPHERE)
            {
                mjsg->type = mjGEOM_SPHERE;
                mjsg->size[0] = geom.SphereShape()->Radius();
            }
            else if (geom.Type() == sdf::GeometryType::CYLINDER)
            {
                mjsg->type = mjGEOM_CYLINDER;
                mjsg->size[0] = geom.CylinderShape()->Radius();
                mjsg->size[1] = geom.CylinderShape()->Length()/2.0;
            }
            else if (geom.Type() == sdf::GeometryType::PLANE)
            {
              mjsg->type = mjGEOM_PLANE;

              // For planes, we must orient the geom's Z-axis to match the
              // plane's normal vector. The normal vector from SDF is in the
              // geometry's frame.
              const auto normalInGeomFrame = geom.PlaneShape()->Normal();

              // The final orientation of the normal in the link's frame is
              // found by composing the geometry's pose rotation with the normal
              // vector.
              const auto normalInLinkFrame = colPose.Rot() * normalInGeomFrame;

              // We compute the rotation that aligns Mujoco's default normal
              // (Z-axis) with our desired normal in the link frame.
              gz::math::Quaterniond mujocoRot;
              mujocoRot.SetFrom2Axes(gz::math::Vector3d::UnitZ, normalInLinkFrame);

              mjsg->quat[0] = mujocoRot.W();
              mjsg->quat[1] = mujocoRot.X();
              mjsg->quat[2] = mujocoRot.Y();
              mjsg->quat[3] = mujocoRot.Z();

              for (int j = 0; j < 2; ++j)
              {
                mjsg->size[j] = geom.PlaneShape()->Size()[j] / 2.0;
              }
              mjsg->size[2] = 1.0;
            }
            else if (geom.Type() == sdf::GeometryType::MESH && geom.MeshShape())
            {
                const ::sdf::Mesh *meshSdf = geom.MeshShape();
                auto &meshManager = *gz::common::MeshManager::Instance();
                auto *mesh = meshManager.Load(meshSdf->Uri());

                if (nullptr == mesh)
                {
                    gzwarn << "Failed to load mesh from [" << meshSdf->Uri() << "]." << std::endl;
                    mjsg->type = mjGEOM_NONE;
                    continue;
                }

                mjsg->type = mjGEOM_MESH;

                // Use the mesh's name (typically its URI) as the asset name.
                // This allows reusing the asset if the same mesh is used multiple times.
                const std::string meshName = mesh->Name();
                mjs_setString(mjsg->meshname, meshName.c_str());

                // Create the mesh asset if it doesn't exist yet.
                if (!mjs_findElement(spec, mjOBJ_MESH, meshName.c_str()))
                {
                    auto *muMesh = mjs_addMesh(spec, nullptr);
                    mjs_setName(muMesh->element, meshName.c_str());

                    const auto &scale = meshSdf->Scale();
                    muMesh->scale[0] = scale.X();
                    muMesh->scale[1] = scale.Y();
                    muMesh->scale[2] = scale.Z();

                    double *verts{nullptr};
                    int *indices{nullptr};

                    mesh->FillArrays(&verts, &indices);

                    auto nverts = mesh->VertexCount();
                    if (nverts > 0 && verts)
                    {
                      muMesh->uservert->assign(verts, verts + 3 * nverts);
                    }

                    auto nfaces = mesh->IndexCount();
                    if (nfaces > 0 && indices)
                    {
                      mjs_setInt(muMesh->userface, indices, nfaces);
                    }

                    delete[] verts;
                    delete[] indices;
                }
            }
        }
    }

    for (size_t i=0; i < node.children.size(); ++i)
    {
        this->AddBodyRecursive(_ecm, spec, linkBody, node.children[i], node.joints[i], nodes, linkPoseInModel, modelPose, modelIsStatic);
    }
}


gz::math::Pose3d MujocoPhysics::GetLinkWorldPose(int _bodyId) const
{
  gz::math::Pose3d pose;
  if (_bodyId > 0 && _bodyId < this->model->nbody)
  {
    pose.Pos().Set(
        this->data->xpos[3*_bodyId + 0],
        this->data->xpos[3*_bodyId + 1],
        this->data->xpos[3*_bodyId + 2]
    );
    pose.Rot().Set(
        this->data->xquat[4*_bodyId + 0],
        this->data->xquat[4*_bodyId + 1],
        this->data->xquat[4*_bodyId + 2],
        this->data->xquat[4*_bodyId + 3]
    );
  }
  return pose;
}

GZ_ADD_PLUGIN(MujocoPhysics, System, MujocoPhysics::ISystemConfigure,
              MujocoPhysics::ISystemUpdate)

}  // namespace systems
}  // namespace sim
}  // namespace gz
