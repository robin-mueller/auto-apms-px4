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

#define INPUT_KEY_MODE_NAME "mode_name"
#define OUTPUT_KEY_NAV_STATE "nav_state"

namespace auto_apms_px4
{

/**
 * @ingroup auto_apms_px4
 * @brief Resolves a registered PX4 mode's name to the nav_state (mode id) PX4 assigned it during registration.
 *
 * PX4 assigns the nav_state of a custom/external mode dynamically at registration time, so it cannot be hard-coded in
 * a behavior tree. This node subscribes to the `registered_modes` topic - on which every mode brought up via an
 * `auto_apms_px4::ModeRegistrationHandler` (i.e. through ModeRegistrationFactory or ModeProxyActionFactory)
 * announces its name-to-nav_state mapping - and writes the nav_state of the mode named by the `mode_name` input port
 * to the `nav_state` output port.
 *
 * Announcements from all mode components are accumulated across ticks, so the node reliably resolves any announced
 * mode regardless of tick rate or the order in which components come up. It returns SUCCESS once the requested mode
 * has been seen, otherwise FAILURE. Wrap it in a retry decorator to wait for a mode to become available, then feed the
 * resolved `nav_state` into %SwitchMode (and %CheckNavState) to switch to the custom mode.
 */
class GetModeNavState
: public auto_apms_behavior_tree::core::RosSubscriberNode<auto_apms_px4_interfaces::msg::RegisteredMode>
{
  std::map<std::string, uint8_t> known_modes_;

public:
  GetModeNavState(
    const std::string & instance_name, const BT::NodeConfig & config,
    const auto_apms_behavior_tree::core::RosNodeContext & context)
  : RosSubscriberNode{instance_name, config, context, rclcpp::QoS(10).transient_local()}
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>(INPUT_KEY_MODE_NAME, "Name of the registered PX4 mode to resolve."),
      BT::OutputPort<int>(OUTPUT_KEY_NAV_STATE, "Resolved PX4 navigation state (mode id) of the named mode."),
    });
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
      // Mode not announced (yet) - retry to keep waiting for it to register.
      return BT::NodeStatus::FAILURE;
    }

    setOutput(OUTPUT_KEY_NAV_STATE, static_cast<int>(it->second));
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::GetModeNavState)
