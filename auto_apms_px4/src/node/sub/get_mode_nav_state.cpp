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
#include <map>
#include <memory>
#include <string>

#include "auto_apms_behavior_tree_core/node.hpp"
#include "auto_apms_px4_interfaces/msg/registered_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#define INPUT_KEY_MODE_NAME "mode_name"
#define OUTPUT_KEY_NAV_STATE "nav_state"

namespace auto_apms_px4
{

/**
 * @ingroup auto_apms_px4
 * @brief Resolves a PX4 mode's name to its nav_state (mode id).
 *
 * The builtin PX4 flight modes have fixed nav_states and are known out of the box (see standardModes below), so they
 * can be resolved by name without any mode component running. Custom/external modes, on the other hand, are assigned a
 * nav_state dynamically by PX4 at registration time, so it cannot be hard-coded in a behavior tree. For those, this
 * node subscribes to the `registered_modes` topic - on which every mode brought up via an
 * `auto_apms_px4::ModeRegistrationHandler` (i.e. through ModeRegistrationFactory or ModeProxyActionFactory) announces
 * its name-to-nav_state mapping - and merges the announcements with the builtin table. The nav_state of the mode named
 * by the `mode_name` input port is written to the `nav_state` output port.
 *
 * Announcements from all mode components are accumulated across ticks, so the node reliably resolves any announced
 * mode regardless of tick rate or the order in which components come up. It returns SUCCESS once the requested mode is
 * known, otherwise FAILURE. Wrap it in a retry decorator to wait for a custom mode to become available, then feed the
 * resolved `nav_state` into %SendCmdSetNavState (and %CheckNavState) to switch to the mode.
 */
class GetModeNavState
: public auto_apms_behavior_tree::core::RosSubscriberNode<auto_apms_px4_interfaces::msg::RegisteredMode>
{
  /// Builtin PX4 flight modes with their fixed nav_states (px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_*).
  static std::map<std::string, uint8_t> standardModes()
  {
    using VehicleStatus = px4_msgs::msg::VehicleStatus;
    return {
      {"Manual", VehicleStatus::NAVIGATION_STATE_MANUAL},
      {"Altitude", VehicleStatus::NAVIGATION_STATE_ALTCTL},
      {"Position", VehicleStatus::NAVIGATION_STATE_POSCTL},
      {"Mission", VehicleStatus::NAVIGATION_STATE_AUTO_MISSION},
      {"Hold", VehicleStatus::NAVIGATION_STATE_AUTO_LOITER},
      {"RTL", VehicleStatus::NAVIGATION_STATE_AUTO_RTL},
      {"Acro", VehicleStatus::NAVIGATION_STATE_ACRO},
      {"Descend", VehicleStatus::NAVIGATION_STATE_DESCEND},
      {"Termination", VehicleStatus::NAVIGATION_STATE_TERMINATION},
      {"Offboard", VehicleStatus::NAVIGATION_STATE_OFFBOARD},
      {"Stabilized", VehicleStatus::NAVIGATION_STATE_STAB},
      {"Takeoff", VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF},
      {"Land", VehicleStatus::NAVIGATION_STATE_AUTO_LAND},
      {"FollowTarget", VehicleStatus::NAVIGATION_STATE_AUTO_FOLLOW_TARGET},
      {"PrecisionLand", VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND},
      {"Orbit", VehicleStatus::NAVIGATION_STATE_ORBIT},
      {"VtolTakeoff", VehicleStatus::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF},
    };
  }

  // Seeded with the builtin modes; announcements from custom mode components are merged in on tick.
  std::map<std::string, uint8_t> known_modes_ = standardModes();

public:
  GetModeNavState(
    const std::string & instance_name, const BT::NodeConfig & config,
    const auto_apms_behavior_tree::core::RosNodeContext & context)
  : RosSubscriberNode{instance_name, config, context, rclcpp::QoS(10).transient_local()}
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(INPUT_KEY_MODE_NAME, "Name of the registered PX4 mode to resolve."),
      BT::OutputPort<int>(OUTPUT_KEY_NAV_STATE, "Resolved PX4 navigation state (mode id) of the named mode."),
    };
  }

  BT::NodeStatus onTick(const std::shared_ptr<auto_apms_px4_interfaces::msg::RegisteredMode> & last_msg_ptr) final
  {
    // Accumulate the latest announcement (each mode component publishes its own mapping on the shared topic).
    if (last_msg_ptr) known_modes_[last_msg_ptr->name] = last_msg_ptr->nav_state;

    const BT::Expected<std::string> expected_mode_name = getInput<std::string>(INPUT_KEY_MODE_NAME);
    if (!expected_mode_name) {
      RCLCPP_ERROR(
        logger_, "%s - Missing required input '%s': %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        INPUT_KEY_MODE_NAME, expected_mode_name.error().c_str());
      return BT::NodeStatus::FAILURE;
    }

    const auto it = known_modes_.find(expected_mode_name.value());
    if (it == known_modes_.end()) {
      // Not a builtin mode and not announced (yet) - retry to keep waiting for a custom mode to register.
      return BT::NodeStatus::FAILURE;
    }

    setOutput(OUTPUT_KEY_NAV_STATE, static_cast<int>(it->second));
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::GetModeNavState)
