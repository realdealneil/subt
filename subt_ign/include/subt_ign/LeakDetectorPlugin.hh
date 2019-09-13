/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef SUBT_IGN_LEAKDETECTORPLUGIN_HH_
#define SUBT_IGN_LEAKDETECTORPLUGIN_HH_

#include <memory>

#include <ignition/gazebo/System.hh>

// Inline bracket to help doxygen filtering.
namespace subt
{
  // Forward declarations.
  class LeakDetectorPluginPrivate;

  /// \brief A leak detector that uses a logical camera sensor to identify
  /// the specified leak within the sensor frustum
  /// Parameters:
  ///      leak_name: Name of the leak model (required)
  ///      min_value: Minimum value of the leak reading,
  ///                 i.e. when no leak is detected
  ///      topic:     Topic to publish the leak readings to (msgs::Float).
  class LeakDetectorPlugin :
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure
  {
    /// \brief Constructor
    public: LeakDetectorPlugin();

    /// \brief Destructor
    public: ~LeakDetectorPlugin();

    // Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<LeakDetectorPluginPrivate> dataPtr;
  };
}

#endif

