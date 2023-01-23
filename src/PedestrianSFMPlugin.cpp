/***********************************************************************/
/**                                                                    */
/** PedestrianSFMPlugin.cpp                                            */
/**                                                                    */
/** Copyright (c) 2022, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#include "gazebo_sfm_plugin/PedestrianSFMPlugin.h"

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

using namespace gz;
using namespace sim;
using namespace systems;

class PedestrianSFMPlugin::Implementation
{
  public:
    /// \brief Helper function to detect the closest obstacles.
    void HandleObstacles(const UpdateInfo& _info, const EntityComponentManager& _ecm);

    /// \brief Helper function to detect the nearby pedestrians (other actors).
    void HandlePedestrians(const UpdateInfo& _info, const EntityComponentManager& _ecm);

    void Reset();

    /// \brief this actor as a SFM agent
    sfm::Agent sfmActor;

    /// \brief this actor gazebo pose
    math::Pose3<double> actorPose;

    /// \brief names of the other models in my walking group.
    std::vector<std::string> groupNames;

    /// \brief vector of pedestrians detected.
    std::vector<sfm::Agent> otherActors;

    /// \brief Maximum distance to detect nearby pedestrians.
    double peopleDistance;

    /// \brief Pointer to the parent actor.
    Entity actorEntity{ kNullEntity };

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    double animationFactor{ 1.0 };

    /// \brief List of models to ignore. Used for vector field
    std::unordered_map<gz::sim::Entity, std::string> ignoreModels;

    /// \brief Time of the last update.
    std::chrono::steady_clock::duration lastUpdate{ 0 };
};

//////////////////////////////////////////////////
void PedestrianSFMPlugin::Implementation::HandleObstacles(const UpdateInfo& _info,
                                                          const EntityComponentManager& _ecm)
{
    (void)_info;
    (void)_ecm;

    double minDist = std::numeric_limits<double>::max();
    math::Vector3d closest_obs;

    this->sfmActor.obstacles1.clear();
    _ecm.Each<components::Model>(
      [&](const Entity& _entity, const components::Model* _model) -> bool {
          (void)_model;
          if (this->ignoreModels.find(_entity) == this->ignoreModels.end()) {
              auto modelPose = _ecm.Component<components::Pose>(_entity)->Data();

              auto distance = actorPose.Pos().Distance(modelPose.Pos());
              if (distance < minDist) {
                  minDist = distance;
                  closest_obs = modelPose.Pos();
              }
          }
          return true;
      });

    ::utils::Vector2d ob(closest_obs.X(), closest_obs.Y());
    this->sfmActor.obstacles1.push_back(ob);

    GZ_PROFILE("PedestrianSFMPlugin::HandleObstacles");
}

//////////////////////////////////////////////////
void PedestrianSFMPlugin::Implementation::HandlePedestrians(const UpdateInfo& _info,
                                                            const EntityComponentManager& _ecm)
{
    (void)_info;

    GZ_PROFILE("PedestrianSFMPlugin::HandlePedestrians");

    math::Pose3<double> entityPose;
    this->otherActors.clear();
    _ecm.Each<components::Actor>(
      [&](const Entity& _entity, const components::Actor* _actor) -> bool {
          auto trajPoseComp = _ecm.Component<components::TrajectoryPose>(_entity);
          auto poseComp = _ecm.Component<components::Pose>(_entity);
          if (trajPoseComp) {
              entityPose = trajPoseComp->Data();
          } else if (poseComp) {
              entityPose = poseComp->Data();
          } else {
              gzerr << "actor dosnt have TrajectoryPose or Pose" << std::endl;
              return true;
          }

          auto distance = entityPose.Pos().Distance(actorPose.Pos());
          if (distance <= this->peopleDistance) {
              sfm::Agent ped;
              ped.id = _entity;
              ped.position.set(entityPose.Pos().X(), entityPose.Pos().Y());
              ped.yaw.fromRadian(entityPose.Rot().Yaw());

              ped.radius = this->sfmActor.radius;

              // ignition::math::Vector3d linvel = model->WorldLinearVel();
              // ped.velocity.set(linvel.X(), linvel.Y());
              // ped.linearVelocity = linvel.Length();
              // ignition::math::Vector3d angvel = model->WorldAngularVel();
              // ped.angularVelocity = angvel.Z(); // Length()

              // check if the ped belongs to my group
              if (this->sfmActor.groupId != -1) {
                  std::vector<std::string>::iterator it;
                  it = find(groupNames.begin(), groupNames.end(), _actor->Data().Name());
                  if (it != groupNames.end())
                      ped.groupId = this->sfmActor.groupId;
                  else
                      ped.groupId = -1;
              }
              this->otherActors.push_back(ped);
          }
          return true;
      });
}

//////////////////////////////////////////////////
PedestrianSFMPlugin::PedestrianSFMPlugin()
  : System()
  , dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{}

//////////////////////////////////////////////////
void PedestrianSFMPlugin::Configure(const gz::sim::Entity& _entity,
                                    const std::shared_ptr<const sdf::Element>& _sdf,
                                    gz::sim::EntityComponentManager& _ecm,
                                    gz::sim::EventManager& _eventMgr)
{
    (void)_eventMgr;

    GZ_PROFILE("PedestrianSFMPlugin::Configure");

    auto actorComp = _ecm.Component<components::Actor>(_entity);
    if (!actorComp) {
        gzerr << "Entity [" << _entity << "] is not an actor." << std::endl;
        return;
    }

    this->dataPtr->actorEntity = _entity;

    // Initialize sfmActor id
    this->dataPtr->sfmActor.id = this->dataPtr->actorEntity;

    // Current world pose
    math::Pose3d initialPose;
    auto poseComp = _ecm.Component<components::Pose>(this->dataPtr->actorEntity);
    if (!poseComp) {
        _ecm.CreateComponent(_entity, components::Pose(math::Pose3d::Zero));
    } else {
        initialPose = poseComp->Data();

        // We'll be setting the actor's X/Y pose with respect to the world. So we
        // zero the current values.
        auto newPose = initialPose;
        newPose.Pos().X(0);
        newPose.Pos().Y(0);
        *poseComp = components::Pose(newPose);
    }
    auto pos = initialPose.Pos();
    auto rot = initialPose.Rot();

    // Initialize sfmActor position
    this->dataPtr->sfmActor.position.set(pos.X(), pos.Y());
    this->dataPtr->sfmActor.yaw.fromRadian(rot.Yaw()); // yaw
    this->dataPtr->sfmActor.velocity.set(0.0, 0.0);
    this->dataPtr->sfmActor.linearVelocity = 0.0;
    this->dataPtr->sfmActor.angularVelocity = 0.0; // Length()

    // Read in the maximum velocity of the pedestrian
    if (_sdf->HasElement("velocity"))
        this->dataPtr->sfmActor.desiredVelocity = _sdf->Get<double>("velocity");
    else
        this->dataPtr->sfmActor.desiredVelocity = 0.8;

    // Read in the target weight
    if (_sdf->HasElement("goal_weight"))
        this->dataPtr->sfmActor.params.forceFactorDesired = _sdf->Get<double>("goal_weight");
    // Read in the obstacle weight
    if (_sdf->HasElement("obstacle_weight"))
        this->dataPtr->sfmActor.params.forceFactorObstacle = _sdf->Get<double>("obstacle_weight");
    // Read in the social weight
    if (_sdf->HasElement("social_weight"))
        this->dataPtr->sfmActor.params.forceFactorSocial = _sdf->Get<double>("social_weight");
    // Read in the group gaze weight
    if (_sdf->HasElement("group_gaze_weight"))
        this->dataPtr->sfmActor.params.forceFactorGroupGaze =
          _sdf->Get<double>("group_gaze_weight");
    // Read in the group coherence weight
    if (_sdf->HasElement("group_coh_weight"))
        this->dataPtr->sfmActor.params.forceFactorGroupCoherence =
          _sdf->Get<double>("group_coh_weight");
    // Read in the group repulsion weight
    if (_sdf->HasElement("group_rep_weight"))
        this->dataPtr->sfmActor.params.forceFactorGroupRepulsion =
          _sdf->Get<double>("group_rep_weight");

    // Read in the animation factor (applied in the OnUpdate function).
    if (_sdf->HasElement("animation_factor"))
        this->dataPtr->animationFactor = _sdf->Get<double>("animation_factor");
    else
        this->dataPtr->animationFactor = 1.0;

    if (_sdf->HasElement("people_distance"))
        this->dataPtr->peopleDistance = _sdf->Get<double>("people_distance");
    else
        this->dataPtr->peopleDistance = 5.0;

    // Ugly, but needed because the sdf::Element::GetElement is not a const
    // function and _sdf is a const shared pointer to a const sdf::Element.
    auto sdfPtr = const_cast<sdf::Element*>(_sdf.get());

    // Read in the pedestrians in your walking group
    if (_sdf->HasElement("group")) {
        auto modelElem = sdfPtr->GetElement("group")->GetElement("model");
        while (modelElem) {
            this->dataPtr->groupNames.push_back(modelElem->Get<std::string>());
            modelElem = modelElem->GetNextElement("model");
        }
        this->dataPtr->sfmActor.groupId = this->dataPtr->sfmActor.id;
    } else {
        this->dataPtr->sfmActor.groupId = -1;
    }

    // Read in the goals to reach
    if (_sdf->HasElement("trajectory")) {
        auto sdfElem = sdfPtr->GetElement("trajectory");
        this->dataPtr->sfmActor.cyclicGoals = sdfElem->Get<double>("cyclic");

        auto waypointElem = sdfElem->GetElement("waypoint");
        while (waypointElem) {
            auto g = waypointElem->Get<gz::math::Vector3d>();
            sfm::Goal goal;
            goal.center.set(g.X(), g.Y());
            goal.radius = 0.3;
            this->dataPtr->sfmActor.goals.push_back(goal);
            waypointElem = waypointElem->GetNextElement("waypoint");
        }
    }

    // Read in the other obstacles to ignore
    if (_sdf->HasElement("ignore_obstacles")) {
        auto modelElem = sdfPtr->GetElement("ignore_obstacles")->GetElement("model");
        while (modelElem) {
            auto modelName = modelElem->Get<std::string>();
            auto modelEntity = _ecm.EntityByComponents(components::Name(modelName));
            this->dataPtr->ignoreModels[modelEntity] = modelName;
            modelElem = modelElem->GetNextElement("model");
        }
    }
    // Add our own name to models we should ignore when avoiding obstacles.
    this->dataPtr->ignoreModels[_entity] = actorComp->Data().Name();

    // Set custom animation time from this plugin
    auto animTimeComp = _ecm.Component<components::AnimationTime>(_entity);
    if (!animTimeComp) {
        _ecm.CreateComponent(_entity, components::AnimationTime());
    }

    // Having a trajectory pose prevents the actor from moving with the
    // SDF script
    auto trajPoseComp = _ecm.Component<components::TrajectoryPose>(_entity);
    if (!trajPoseComp) {
        // Leave Z to the pose component, control only 2D with Trajectory
        initialPose.Pos().Z(0);
        _ecm.CreateComponent(_entity, components::TrajectoryPose(initialPose));
    }
}

//////////////////////////////////////////////////
void PedestrianSFMPlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                                    gz::sim::EntityComponentManager& _ecm)

{
    GZ_PROFILE("PedestrianSFMPlugin::PreUpdate");

    if (_info.paused)
        return;

    // \TODO(anyone) Support rewind
    if (_info.dt < std::chrono::steady_clock::duration::zero()) {
        gzwarn << "Detected jump back in time ["
               << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
               << "s]. System may not work properly." << std::endl;
    }

    // Time delta
    std::chrono::duration<double> dtDuration = _info.simTime - this->dataPtr->lastUpdate;
    double dt = dtDuration.count();

    this->dataPtr->lastUpdate = _info.simTime;

    // Add the other pedestrians to the ignored obstacles
    _ecm.EachNew<components::Actor>(
      [&](const Entity& _entity, const components::Actor* _actor) -> bool {
          this->dataPtr->ignoreModels[_entity] = _actor->Data().Name();
          return true;
      });

    // Update closest obstacle
    this->dataPtr->HandleObstacles(_info, _ecm);

    // Update pedestrian around
    this->dataPtr->HandlePedestrians(_info, _ecm);

    // Compute Social Forces
    sfm::SFM.computeForces(this->dataPtr->sfmActor, this->dataPtr->otherActors);
    // Update model
    sfm::SFM.updatePosition(this->dataPtr->sfmActor, dt);
    this->dataPtr->actorPose.SetX(this->dataPtr->sfmActor.position.getX());
    this->dataPtr->actorPose.SetY(this->dataPtr->sfmActor.position.getY());
    this->dataPtr->actorPose.Rot() =
      math::Quaterniond(0, 0, this->dataPtr->sfmActor.yaw.toRadian());

    // Current world pose
    auto trajPoseComp = _ecm.Component<components::TrajectoryPose>(this->dataPtr->actorEntity);
    auto actorPos = trajPoseComp->Data();
    auto initialPose = actorPos;

    // Move accordingly to sfm
    actorPos = this->dataPtr->actorPose;

    // Distance traveled is used to coordinate motion with the walking
    // animation
    double distanceTraveled = (actorPos.Pos() - initialPose.Pos()).Length();

    // Update actor root pose
    *trajPoseComp = components::TrajectoryPose(actorPos);
    // Mark as a one-time-change so that the change is propagated to the GUI
    _ecm.SetChanged(this->dataPtr->actorEntity,
                    components::TrajectoryPose::typeId,
                    ComponentState::OneTimeChange);

    // Update actor bone trajectories based on animation time
    auto animTimeComp = _ecm.Component<components::AnimationTime>(this->dataPtr->actorEntity);

    auto animTime =
      animTimeComp->Data() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(distanceTraveled * this->dataPtr->animationFactor));
    *animTimeComp = components::AnimationTime(animTime);
    // Mark as a one-time-change so that the change is propagated to the GUI
    _ecm.SetChanged(
      this->dataPtr->actorEntity, components::AnimationTime::typeId, ComponentState::OneTimeChange);
}

GZ_ADD_PLUGIN(PedestrianSFMPlugin,
              System,
              PedestrianSFMPlugin::ISystemConfigure,
              PedestrianSFMPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::PedestrianSFMPlugin, "gz::sim::systems::PedestrianSFMPlugin")
