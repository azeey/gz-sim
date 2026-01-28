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

#include <gz/common/Console.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SystemPaths.hh>
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

  // Step
  if (!_info.paused)
  {
    mj_step(this->model, this->data);
  }

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

  // 2. Root Link Poses (update associated Model Poses)
  for (int i = 1; i < this->model->nbody; ++i)
  {
      if (this->model->body_parentid[i] == 0)
      {
          const char* name = mj_id2name(this->model, mjOBJ_BODY, i);
          if (name && std::string(name).find("body_") == 0)
          {
              try {
                  Entity linkEntity = std::stoull(name + 5);
                  auto parentComp = _ecm.Component<components::ParentEntity>(linkEntity);
                  if (parentComp)
                  {
                      Entity modelEntity = parentComp->Data();
                      math::Vector3d pos(
                          this->data->xpos[3*i + 0],
                          this->data->xpos[3*i + 1],
                          this->data->xpos[3*i + 2]
                      );
                      math::Quaterniond rot(
                          this->data->xquat[4*i + 0],
                          this->data->xquat[4*i + 1],
                          this->data->xquat[4*i + 2],
                          this->data->xquat[4*i + 3]
                      );
                      
                      // _ecm.SetComponentData<components::Pose>(modelEntity, math::Pose3d(pos, rot));
                      // _ecm.SetChanged(modelEntity, components::Pose::typeId,
                      //                 ComponentState::PeriodicChange);
                  }
              } catch (...) {}
          }
      }
  }
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

  // Map Entities & Restore State
  for (int i = 0; i < this->model->nbody; ++i)
  {
    const char* name = mj_id2name(this->model, mjOBJ_BODY, i);
    if (name && std::string(name).find("body_") == 0)
    {
      try {
          Entity e = std::stoull(name + 5);
          auto comp = _ecm.Component<components::MujocoBodyId>(e);
          if (comp) comp->Data() = i;
          else _ecm.CreateComponent(e, components::MujocoBodyId(i));
      } catch (...) {}
    }
  }

  for (int i = 0; i < this->model->njnt; ++i)
  {
    const char* name = mj_id2name(this->model, mjOBJ_JOINT, i);
    if (name && std::string(name).find("joint_") == 0)
    {
      try {
          Entity e = std::stoull(name + 6);
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
      } catch (...) {}
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

    auto linkPoseComp = _ecm.Component<components::Pose>(linkEntity);
    math::Pose3d linkPoseInModel = linkPoseComp ? linkPoseComp->Data() : math::Pose3d::Zero;
    math::Pose3d worldPoseOfLink = modelPose * linkPoseInModel;
    math::Pose3d worldPoseOfParent = modelPose * parentPoseInModel;
    // math::Pose3d relPose = worldPoseOfParent.Inverse() * worldPoseOfLink;
    math::Pose3d relPose = worldPoseOfLink;
    gzdbg << "link: " << linkPoseInModel << " world: " << worldPoseOfLink << " parent: " << worldPoseOfParent << " relPose: " << relPose << std::endl;

    mjsBody *linkBody = mjs_addBody(parentBody, nullptr);
    if (!linkBody)
    {
      gzerr << "Failed to add body for entity " << linkEntity << std::endl;
      return;
    }
    mjs_setName(linkBody->element, ("body_" + std::to_string(linkEntity)).c_str());
    
    linkBody->pos[0] = relPose.Pos().X();
    linkBody->pos[1] = relPose.Pos().Y();
    linkBody->pos[2] = relPose.Pos().Z();
    linkBody->quat[0] = relPose.Rot().W();
    linkBody->quat[1] = relPose.Rot().X();
    linkBody->quat[2] = relPose.Rot().Y();
    linkBody->quat[3] = relPose.Rot().Z();

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
            mjs_setName(joint->element, ("joint_" + std::to_string(jointEntity)).c_str());
            
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
    if (inertialComp)
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
            mjs_setName(mjsg->element, ("geom_" + std::to_string(colEntity)).c_str());
            
            mjsg->pos[0] = colPose.Pos().X();
            mjsg->pos[1] = colPose.Pos().Y();
            mjsg->pos[2] = colPose.Pos().Z();
            mjsg->quat[0] = colPose.Rot().W();
            mjsg->quat[1] = colPose.Rot().X();
            mjsg->quat[2] = colPose.Rot().Y();
            mjsg->quat[3] = colPose.Rot().Z();
            
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

              // Set mass to 0 to mark the body as static
              mjsg->type = mjGEOM_PLANE;
              for (int j = 0; j < 2; ++j)
              {
                mjsg->size[j] = geom.PlaneShape()->Size()[j] / 2.0;
              }
              mjsg->size[2] = 1.0;
              break;
            }
            else if (geom.Type() == sdf::GeometryType::MESH && geom.MeshShape())
            {
                mjsMesh *meshSpec = mjs_addMesh(spec, nullptr);
                std::string uri = geom.MeshShape()->Uri();
                std::string path = asFullPath(uri, "");
                mjs_setName(meshSpec->element, uri.c_str());
                mjs_setString(meshSpec->file, path.c_str());
                meshSpec->scale[0] = geom.MeshShape()->Scale().X();
                meshSpec->scale[1] = geom.MeshShape()->Scale().Y();
                meshSpec->scale[2] = geom.MeshShape()->Scale().Z();
                
                mjsg->type = mjGEOM_MESH;
                mjs_setString(mjsg->meshname, uri.c_str());
            }
        }
    }

    for (size_t i=0; i < node.children.size(); ++i)
    {
        this->AddBodyRecursive(_ecm, spec, linkBody, node.children[i], node.joints[i], nodes, linkPoseInModel, modelPose, modelIsStatic);
    }
}

GZ_ADD_PLUGIN(MujocoPhysics, System, MujocoPhysics::ISystemConfigure,
              MujocoPhysics::ISystemUpdate)

}  // namespace systems
}  // namespace sim
}  // namespace gz
