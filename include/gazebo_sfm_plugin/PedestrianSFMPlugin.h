/***********************************************************************/
/**                                                                    */
/** PedestrianSFMPlugin.h                                              */
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

#pragma once

// C++
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// Gazebo
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

// Social Force Model
#include <lightsfm/sfm.hpp>

namespace gz {
namespace sim {
namespace systems {
class PedestrianSFMPlugin
  : public gz::sim::System
  , public gz::sim::ISystemConfigure
  , public gz::sim::ISystemPreUpdate
{
    // \brief Constructor.
  public:
    PedestrianSFMPlugin();

    /// \brief Destructor.
    ~PedestrianSFMPlugin() override = default;

    // Documentation inherited.
    void Configure(const gz::sim::Entity& _entity,
                   const std::shared_ptr<const sdf::Element>& _sdf,
                   gz::sim::EntityComponentManager& _ecm,
                   gz::sim::EventManager& _eventMgr) override;

    // Documentation inherited.
    void PreUpdate(const gz::sim::UpdateInfo& _info,
                   gz::sim::EntityComponentManager& _ecm) override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
};
} // namespace gz
} // namespace sim
} // namespace systems
