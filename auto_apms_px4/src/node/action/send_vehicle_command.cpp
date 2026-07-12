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

#include "auto_apms_px4/send_vehicle_command.hpp"

#include "px4_ros2/utils/message_version.hpp"

// AUTO_APMS_PX4_SOURCE_COMPONENT_GLOBAL_KEY is injected as a compile definition (see CMakeLists.txt). It is the
// global blackboard entry an in-charge auto_apms_px4::BehaviorModeExecutor publishes its VehicleCommand source
// component under. The '@' prefix accesses the root (global) blackboard transitively, regardless of the (sub)tree the
// node lives in.
#ifndef AUTO_APMS_PX4_SOURCE_COMPONENT_GLOBAL_KEY
#error "AUTO_APMS_PX4_SOURCE_COMPONENT_GLOBAL_KEY must be defined (see CMakeLists.txt)"
#endif
#ifndef AUTO_APMS_PX4_MODE_EXECUTOR_ACTIVE_GLOBAL_KEY
#error "AUTO_APMS_PX4_MODE_EXECUTOR_ACTIVE_GLOBAL_KEY must be defined (see CMakeLists.txt)"
#endif

namespace auto_apms_px4
{

SendVehicleCommand::SendVehicleCommand(
  const std::string & instance_name, const Config & config, const Context & context)
: RosPublisherNode(instance_name, config, context)
{
  // The topic is determined here in the implementation rather than from the node manifest: its base is chosen from the
  // mode-executor flag on the global blackboard and its message version suffix is resolved at runtime (see
  // resolveTopicName). Since the manifest does not fix a topic, the base class cannot resolve one at construction and
  // defers creation to this constructor. SwitchMode inherits this constructor and the same topic.
  createPublisher(resolveTopicName());
}

std::string SendVehicleCommand::resolveTopicName()
{
  // While a BehaviorModeExecutor is in charge it sets this flag on the global blackboard; route the command through
  // the mode-executor command topic so PX4 attributes it to the executor. Otherwise use the default external command
  // topic. The '@'-prefixed key resolves transitively to the root (global) blackboard.
  bool via_mode_executor = false;
  (void)config().blackboard->get<bool>(AUTO_APMS_PX4_MODE_EXECUTOR_ACTIVE_GLOBAL_KEY, via_mode_executor);
  const std::string base = via_mode_executor ? "fmu/in/vehicle_command_mode_executor" : "fmu/in/vehicle_command";

  // The message version suffix is resolved at runtime from the message definition rather than baked into the manifest.
  return base + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommand>();
}

BT::PortsList SendVehicleCommand::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<int>(PORT_KEY_COMMAND, "VehicleCommand command id (see px4_msgs/msg/VehicleCommand)."),
    BT::InputPort<double>(PORT_KEY_PARAM1, 0.0, "Command parameter 1."),
    BT::InputPort<double>(PORT_KEY_PARAM2, 0.0, "Command parameter 2."),
    BT::InputPort<double>(PORT_KEY_PARAM3, 0.0, "Command parameter 3."),
    BT::InputPort<double>(PORT_KEY_PARAM4, 0.0, "Command parameter 4."),
    BT::InputPort<double>(PORT_KEY_PARAM5, 0.0, "Command parameter 5."),
    BT::InputPort<double>(PORT_KEY_PARAM6, 0.0, "Command parameter 6."),
    BT::InputPort<double>(PORT_KEY_PARAM7, 0.0, "Command parameter 7."),
    BT::InputPort<int>(PORT_KEY_CONFIRMATION, 0, "Confirmation count (0 = first transmission of this command)."),
    BT::InputPort<int>(PORT_KEY_TARGET_SYSTEM, 0, "System that should execute the command."),
    BT::InputPort<int>(PORT_KEY_TARGET_COMPONENT, 0, "Component that should execute the command (0 = all)."),
  });
}

bool SendVehicleCommand::setMessage(px4_msgs::msg::VehicleCommand & msg)
{
  const BT::Expected<int> expected_command = getInput<int>(PORT_KEY_COMMAND);
  if (!expected_command) {
    RCLCPP_ERROR(
      logger_, "%s - Missing required input '%s': %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
      PORT_KEY_COMMAND, expected_command.error().c_str());
    return false;
  }

  msg = px4_msgs::msg::VehicleCommand{};
  msg.command = static_cast<uint32_t>(expected_command.value());
  msg.param1 = static_cast<float>(getInput<double>(PORT_KEY_PARAM1).value_or(0.0));
  msg.param2 = static_cast<float>(getInput<double>(PORT_KEY_PARAM2).value_or(0.0));
  msg.param3 = static_cast<float>(getInput<double>(PORT_KEY_PARAM3).value_or(0.0));
  msg.param4 = static_cast<float>(getInput<double>(PORT_KEY_PARAM4).value_or(0.0));
  msg.param5 = getInput<double>(PORT_KEY_PARAM5).value_or(0.0);
  msg.param6 = getInput<double>(PORT_KEY_PARAM6).value_or(0.0);
  msg.param7 = static_cast<float>(getInput<double>(PORT_KEY_PARAM7).value_or(0.0));
  msg.confirmation = static_cast<uint8_t>(getInput<int>(PORT_KEY_CONFIRMATION).value_or(0));
  msg.target_system = static_cast<uint8_t>(getInput<int>(PORT_KEY_TARGET_SYSTEM).value_or(0));
  msg.target_component = static_cast<uint8_t>(getInput<int>(PORT_KEY_TARGET_COMPONENT).value_or(0));
  msg.source_component = resolveSourceComponent();
  msg.from_external = true;
  msg.timestamp = 0;  // Let PX4 set the timestamp.

  RCLCPP_DEBUG(
    logger_, "%s - Sending VehicleCommand %u (source_component %u).",
    context_.getFullyQualifiedTreeNodeName(this).c_str(), msg.command, msg.source_component);
  return true;
}

uint16_t SendVehicleCommand::resolveSourceComponent()
{
  // Transitively resolve the executor's source component from the global blackboard. If undefined, fall back to 0 (a
  // normal external command) without modifying the blackboard.
  int source_component = 0;
  (void)config().blackboard->get<int>(AUTO_APMS_PX4_SOURCE_COMPONENT_GLOBAL_KEY, source_component);
  return static_cast<uint16_t>(source_component);
}

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::SendVehicleCommand)
