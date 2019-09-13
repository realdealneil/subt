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

#include <ignition/msgs/logical_camera_image.pb.h>
#include <ignition/msgs/float.pb.h>

#include <ignition/gazebo/components/LogicalCamera.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/common/Console.hh>

#include <ignition/transport/Node.hh>
#include "subt_ign/LeakDetectorPlugin.hh"

IGNITION_ADD_PLUGIN(
    subt::LeakDetectorPlugin,
    ignition::gazebo::System,
    subt::LeakDetectorPlugin::ISystemConfigure)

using namespace ignition;
using namespace subt;

/// \brief Private LeakDetector data class.
class subt::LeakDetectorPluginPrivate
{
  /// \brief Callback for logical camera messages
  /// \param[in] _msg LogicalCameraImage message
  public: void OnLogicalCameraMsg(const msgs::LogicalCameraImage &_msg);

  // \brienf transport node used to subscribe to logical camera messages
  public: transport::Node node;

  /// \brief Logical camera near plane
  public: float camNear = 0;

  /// \brief Logical camera far plane
  public: float camFar = 0;

  /// \brief Factor used when calculating the intensity of the leak reading
  public: double factor = 0;

  /// \brief Min value if no leak is detected
  public: double minValue = 0;

  /// \brief Name of the leak model
  public: std::string leakName;

  /// \brief Leak msg to publish
  public: msgs::Float leakMsg;

  /// \brief publisher to publish leak messages.
  public: transport::Node::Publisher pub;
};

//////////////////////////////////////////////////
LeakDetectorPlugin::LeakDetectorPlugin() :
    dataPtr(std::make_unique<LeakDetectorPluginPrivate>())
{
}

//////////////////////////////////////////////////
LeakDetectorPlugin::~LeakDetectorPlugin() = default;

/////////////////////////////////////////////////
void LeakDetectorPlugin::Configure(const ignition::gazebo::Entity & _entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager & /*_eventMgr*/)
{
  // leak name is required
  if (!_sdf->HasElement("leak_name"))
  {
    ignerr << "Unable to initialize LeakDetectorPlugin: missing 'leak_name'"
           << std::endl;
    return;
  }
  this->dataPtr->leakName = _sdf->Get<std::string>("leak_name");

  double defaultMinVal = 0.01;
  this->dataPtr->minValue = _sdf->Get<double>("min_value", defaultMinVal).first;
  if (this->dataPtr->minValue <= 0.0)
  {
    ignwarn << "min_value must be greater than 0. Defaulting to "
            << defaultMinVal << std::endl;
    this->dataPtr->minValue = defaultMinVal;
  }

  // leak messages publisher
  std::string leakDetectorTopic = _sdf->Get<std::string>("topic",
      scopedName(_entity, _ecm) + "/leak").first;
  this->dataPtr->pub = this->dataPtr->node.Advertise<ignition::msgs::Float>(
      leakDetectorTopic);

  // parse parent sensor values
  auto logicalCamComp =
      _ecm.Component<ignition::gazebo::components::LogicalCamera>(_entity);
  sdf::ElementPtr sensorElem = logicalCamComp->Data();
  sdf::ElementPtr logicalCamElem = sensorElem->GetElement("logical_camera");
  this->dataPtr->camNear = logicalCamElem->Get<double>("near");
  this->dataPtr->camFar = logicalCamElem->Get<double>("far");
  double hfov = logicalCamElem->Get<double>("horizontal_fov");

  // subscribe to parent sensor topic
  std::string sensorTopic = sensorElem->Get<std::string>("topic",
      scopedName(_entity, _ecm) + "/logical_camera").first;
  this->dataPtr->node.Subscribe(sensorTopic,
    &LeakDetectorPluginPrivate::OnLogicalCameraMsg, this->dataPtr.get());

  // Calculate factor
  // Furthest detectable point (frustum corner)
  double c = this->dataPtr->camFar * tan(hfov / 2.0);
  ignition::math::Vector3d corner(this->dataPtr->camFar, c, c);

  // Maximum detectable distance
  ignition::math::Vector3d antenaPos(this->dataPtr->camNear, 0, 0);
  auto maxDist = corner.Distance(antenaPos);

  // output = factor ^ distance
  // We chose the factor so that the maximum distance results in the
  // minimum value.
  this->dataPtr->factor = pow(this->dataPtr->minValue, 1 / maxDist);
}

//////////////////////////////////////////////////
void LeakDetectorPluginPrivate::OnLogicalCameraMsg(
    const msgs::LogicalCameraImage &_msg)
{
  ignition::math::Vector3d leakPos = ignition::math::Vector3d::Zero;

  for (const auto &model : _msg.model())
  {
    if (model.name() != leakName)
      continue;

    leakPos = ignition::msgs::Convert(model.pose().position());
    break;
  }

  double value = this->minValue;

  // If leak within frustum
  if (leakPos != ignition::math::Vector3d::Zero)
  {
    // Get distance from leak to sensor
    ignition::math::Vector3d sensorPos(this->camNear, 0, 0);
    float distance = leakPos.Distance(sensorPos);

    // Exclude some partial inclusion
    if (leakPos.X() >= this->camNear)
      value = std::max(pow(this->factor, distance), this->minValue);
  }

  this->leakMsg.set_data(value);
  this->pub.Publish(this->leakMsg);
}
