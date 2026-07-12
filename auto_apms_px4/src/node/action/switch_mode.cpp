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

#include <cstdint>

#include "auto_apms_px4/send_vehicle_command.hpp"

#define INPUT_KEY_NAV_STATE "nav_state"

namespace auto_apms_px4
{

/**
 * @ingroup auto_apms_px4
 * @brief Behavior tree node that switches the active PX4 flight mode while respecting mode-executor ownership.
 *
 * A specialization of %SendVehicleCommand that fixes the command to `VEHICLE_CMD_SET_NAV_STATE` and exposes the
 * target navigation state through a single, friendly `nav_state` input port instead of the raw command parameters.
 *
 * As with any %SendVehicleCommand, the command's `source_component` is resolved from the global blackboard so that an
 * in-charge `auto_apms_px4::BehaviorModeExecutor` can attribute the mode change to itself. If no executor is in
 * charge, the command is sent with the default source component (0).
 *
 * @note This node only requests the mode change; it returns SUCCESS as soon as the command has been published and
 * does not wait for the vehicle to actually enter the requested mode. Pair it with %CheckNavState to wait for the
 * transition to complete.
 */
class SwitchMode : public SendVehicleCommand
{
public:
  using SendVehicleCommand::SendVehicleCommand;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<int>(INPUT_KEY_NAV_STATE, "Target PX4 navigation state (mode id) to switch to."),
    });
  }

  bool setMessage(px4_msgs::msg::VehicleCommand & msg) override final
  {
    const BT::Expected<int> expected_nav_state = getInput<int>(INPUT_KEY_NAV_STATE);
    if (!expected_nav_state) {
      RCLCPP_ERROR(
        logger_, "%s - Missing required input '%s': %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        INPUT_KEY_NAV_STATE, expected_nav_state.error().c_str());
      return false;
    }

    msg = px4_msgs::msg::VehicleCommand{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
    msg.param1 = static_cast<float>(expected_nav_state.value());
    msg.source_component = resolveSourceComponent();
    msg.timestamp = 0;  // Let PX4 set the timestamp.

    RCLCPP_DEBUG(
      logger_, "%s - Switching to nav_state %d (source_component %u).",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), expected_nav_state.value(), msg.source_component);
    return true;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::SwitchMode)
