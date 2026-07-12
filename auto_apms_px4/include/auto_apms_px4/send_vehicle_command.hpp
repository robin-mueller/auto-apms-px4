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

#pragma once

#include <cstdint>
#include <string>

#include "auto_apms_behavior_tree_core/node.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

namespace auto_apms_px4
{

/**
 * @ingroup auto_apms_px4
 * @brief Generic behavior tree node that publishes a PX4 `VehicleCommand`.
 *
 * This is the common building block for all command-based PX4 interactions. It exposes the raw MAVLink-style
 * `VehicleCommand` fields (`command` id and `param1`..`param7`) as input ports so that any command - switching the
 * flight mode, arming/disarming, starting a mission, triggering a servo, ... - can be represented in a behavior tree
 * without a dedicated C++ node. Purpose-built nodes (e.g. %SwitchMode) specialize this node by fixing the `command`
 * id (and possibly some parameters) and exposing a friendlier port vocabulary.
 *
 * To respect the ownership of an in-charge mode executor, the command's `source_component` is resolved from the
 * global blackboard entry an active `auto_apms_px4::BehaviorModeExecutor` publishes. If it is undefined (behavior run
 * standalone, no executor in charge), the command is sent with the default source component (0) and the blackboard is
 * left untouched.
 *
 * The target topic is likewise chosen at construction from the global blackboard: while a `BehaviorModeExecutor` is in
 * charge (it sets a boolean flag), commands are published on the mode-executor command topic
 * (`fmu/in/vehicle_command_mode_executor`); otherwise the default external command topic (`fmu/in/vehicle_command`)
 * is used. The message version suffix is appended at runtime in either case.
 *
 * @note This node only publishes the command; it returns SUCCESS as soon as the message has been sent and does not
 * wait for the FMU to acknowledge or complete it. Compose it with a condition node (e.g. %CheckNavState) inside a
 * retry loop to build a behavior that only returns once the commanded action has finished.
 */
class SendVehicleCommand : public auto_apms_behavior_tree::core::RosPublisherNode<px4_msgs::msg::VehicleCommand>
{
public:
  static constexpr auto PORT_KEY_COMMAND = "command";
  static constexpr auto PORT_KEY_PARAM1 = "param1";
  static constexpr auto PORT_KEY_PARAM2 = "param2";
  static constexpr auto PORT_KEY_PARAM3 = "param3";
  static constexpr auto PORT_KEY_PARAM4 = "param4";
  static constexpr auto PORT_KEY_PARAM5 = "param5";
  static constexpr auto PORT_KEY_PARAM6 = "param6";
  static constexpr auto PORT_KEY_PARAM7 = "param7";
  static constexpr auto PORT_KEY_CONFIRMATION = "confirmation";
  static constexpr auto PORT_KEY_TARGET_SYSTEM = "target_system";
  static constexpr auto PORT_KEY_TARGET_COMPONENT = "target_component";

  SendVehicleCommand(const std::string & instance_name, const Config & config, const Context & context);

  static BT::PortsList providedPorts();

  bool setMessage(px4_msgs::msg::VehicleCommand & msg) override;

protected:
  /**
   * @brief Transitively resolve the in-charge mode executor's source component from the global blackboard.
   * @return The executor's `VehicleCommand` source component, or 0 if no executor published one (normal external
   * command).
   */
  uint16_t resolveSourceComponent();

private:
  /**
   * @brief Determine the full topic (base + runtime message version suffix) to publish on.
   *
   * Reads the mode-executor flag from the global blackboard: while a `BehaviorModeExecutor` is in charge, the
   * mode-executor command topic is used, otherwise the default external command topic. Evaluated once at construction.
   */
  std::string resolveTopicName();
};

}  // namespace auto_apms_px4
