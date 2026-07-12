// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "auto_apms_px4_interfaces/action/takeoff.hpp"

#include "auto_apms_behavior_tree_core/node.hpp"

#define INPUT_KEY_ALTITUDE "alt"
#define INPUT_KEY_USE_AMSL "use_amsl"
#define INPUT_KEY_HEADING "heading"

namespace auto_apms_px4
{

class TakeoffAction : public auto_apms_behavior_tree::core::RosActionNode<auto_apms_px4_interfaces::action::Takeoff>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<float>(INPUT_KEY_ALTITUDE, "Target altitude for takeoff in meters"),
      BT::InputPort<bool>(
        INPUT_KEY_USE_AMSL, false,
        "If true, altitude is interpreted as above mean sea level (AMSL), otherwise as altitude above takeoff point"),
      BT::InputPort<float>(INPUT_KEY_HEADING, 0.0, "Heading after takeoff in radians from north in NED frame")};
  }

  bool setGoal(Goal & goal)
  {
    if (const BT::Expected<float> expected = getInput<float>(INPUT_KEY_ALTITUDE)) {
      goal.alt = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s", expected.error().c_str());
      return false;
    }
    goal.use_amsl = getInput<bool>(INPUT_KEY_USE_AMSL).value();
    goal.heading_rad = getInput<float>(INPUT_KEY_HEADING).value();
    return true;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::TakeoffAction)
