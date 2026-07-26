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

#include "auto_apms_px4/node/send_vehicle_command.hpp"

#include <algorithm>

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

SendVehicleCommand::AckSubscription::AckSubscription(
  rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, const std::string & topic_name)
: name(topic_name)
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  // The single subscription callback fans every acknowledgement out to all the command nodes that registered a
  // listener. Each listener keeps its own matching state (see SendVehicleCommand's listener below).
  subscription = node->create_subscription<Ack>(
    topic_name, rclcpp::SensorDataQoS(), [this](Ack::SharedPtr msg) { broadcast(msg); }, options);
}

void SendVehicleCommand::AckSubscription::addListener(
  const void * owner, std::function<void(const Ack::SharedPtr &)> callback)
{
  listeners.emplace_back(owner, std::move(callback));
}

void SendVehicleCommand::AckSubscription::removeListener(const void * owner)
{
  listeners.erase(
    std::remove_if(listeners.begin(), listeners.end(), [owner](const auto & pair) { return pair.first == owner; }),
    listeners.end());
}

void SendVehicleCommand::AckSubscription::broadcast(const Ack::SharedPtr & msg)
{
  for (const auto & [owner, callback] : listeners) callback(msg);
}

SendVehicleCommand::SendVehicleCommand(
  const std::string & instance_name, const Config & config, const Context & context)
: RosActionNodeBase(instance_name, config, context)
{
  // The base class stores the context and logger and applies the node manifest 'port_alias' feature.
  const rclcpp::Node::SharedPtr node = context_.getRosNode();
  const rclcpp::CallbackGroup::SharedPtr group = context_.getWaitablesCallbackGroup();

  // Publisher for the command. The topic is resolved here rather than fixed in the manifest: its base depends on the
  // mode-executor flag on the global blackboard (available at construction) and its message version suffix is resolved
  // at runtime. A manifest 'topic' still takes precedence (see resolveCommandTopicName). Obtained from the RosNodeBase
  // shared-entity registry so command nodes publishing on the same topic reuse a single publisher.
  const std::string command_topic = resolveCommandTopicName();
  command_pub_ = getSharedEntity<rclcpp::Publisher<px4_msgs::msg::VehicleCommand>>(
    command_topic, [&] { return node->create_publisher<px4_msgs::msg::VehicleCommand>(command_topic, 10); });

  // Subscription to the command acknowledgements so we can confirm PX4 accepted the command. All command nodes share a
  // single subscription (from the shared-entity registry) that broadcasts every ack to the per-node listeners; it is
  // created up front (before any command is sent) so no acknowledgement is missed. Our listener only latches
  // acknowledgements for the command currently in flight, and only while we are waiting - this correlates the ack to
  // the send without any timestamp bookkeeping (there is no spin between arming the matcher and publishing, so a stale
  // ack cannot slip in).
  const std::string ack_topic =
    "fmu/out/vehicle_command_ack" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommandAck>();
  ack_sub_ = getSharedEntity<AckSubscription>(
    ack_topic, [&] { return std::make_shared<AckSubscription>(node, group, ack_topic); });
  ack_sub_->addListener(this, [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr & msg) {
    if (waiting_for_ack_ && msg->command == tx_msg_.command) last_ack_ = msg;
  });

  // Timeout is inferred from the `request_timeout` field of the node registration options.
  ack_timeout_s_ = std::chrono::duration<double>(context_.getRegistrationOptions().request_timeout).count();

  RCLCPP_DEBUG(
    logger_, "%s - Using command publisher on '%s' and acknowledgement subscription on '%s'.",
    context_.getFullyQualifiedTreeNodeName(this).c_str(), command_pub_->get_topic_name(),
    ack_sub_->subscription->get_topic_name());
}

SendVehicleCommand::~SendVehicleCommand()
{
  // Deregister our listener so the shared subscription doesn't invoke a dangling callback after we are destroyed.
  if (ack_sub_) ack_sub_->removeListener(this);
}

std::string SendVehicleCommand::resolveCommandTopicName()
{
  // A manifest-configured topic takes precedence (supports the node manifest 'topic' feature, incl. (input:port)
  // substitutions).
  if (const BT::Expected<std::string> expected = context_.getTopicName(this); expected && !expected.value().empty()) {
    return expected.value();
  }

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
  return {
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
  };
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
  msg.source_system = 0;
  msg.source_component = resolveSourceComponent();
  msg.from_external = true;
  msg.timestamp = 0;  // Let PX4 set the timestamp.

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

BT::NodeStatus SendVehicleCommand::tick()
{
  if (!rclcpp::ok()) {
    halt();
    return BT::NodeStatus::FAILURE;
  }

  // Phase 1 (first tick, status IDLE): build and publish the command, then start waiting for the acknowledgement.
  if (status() == BT::NodeStatus::IDLE) {
    setStatus(BT::NodeStatus::RUNNING);

    if (!setMessage(tx_msg_)) return BT::NodeStatus::FAILURE;

    last_ack_.reset();
    waiting_for_ack_ = true;
    send_time_ = context_.getCurrentTime();
    command_pub_->publish(tx_msg_);

    RCLCPP_DEBUG(
      logger_, "%s - Sent VehicleCommand %u and waiting for acknowledgement (timeout %.2fs).",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), tx_msg_.command, ack_timeout_s_);

    return BT::NodeStatus::RUNNING;
  }

  // Phase 2 (subsequent ticks, status RUNNING): wait for the matching acknowledgement or time out.
  if (last_ack_) {
    const uint8_t result = last_ack_->result;
    waiting_for_ack_ = false;
    using Ack = px4_msgs::msg::VehicleCommandAck;
    if (result == Ack::VEHICLE_CMD_RESULT_ACCEPTED || result == Ack::VEHICLE_CMD_RESULT_IN_PROGRESS) {
      RCLCPP_DEBUG(
        logger_, "%s - VehicleCommand %u accepted (result %u).", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        tx_msg_.command, result);
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(
      logger_, "%s - VehicleCommand %u was not accepted (result %u).",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), tx_msg_.command, result);
    return BT::NodeStatus::FAILURE;
  }

  if ((context_.getCurrentTime() - send_time_).seconds() > ack_timeout_s_) {
    waiting_for_ack_ = false;
    RCLCPP_ERROR(
      logger_, "%s - Timed out after %.2fs waiting for acknowledgement of VehicleCommand %u.",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), ack_timeout_s_, tx_msg_.command);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void SendVehicleCommand::halt()
{
  // Stop waiting and drop any pending acknowledgement, then return to IDLE so a later tick starts a fresh send.
  waiting_for_ack_ = false;
  last_ack_.reset();
  resetStatus();
}

}  // namespace auto_apms_px4

// This translation unit is compiled into two libraries (see CMakeLists.txt): the exported auto_apms_px4 library, which
// provides the SendVehicleCommand implementation to other targets (e.g. the generated node model header), and the
// auto_apms_px4_behavior_tree_nodes pluginlib plugin. Only the plugin library must export the pluginlib factory.
// Exporting it from the exported library as well would register the same factory a second time whenever a process links
// that library directly (e.g. the behavior_mode_executor executable) in addition to loading the plugin, which triggers
// a class_loader namespace-collision warning. AUTO_APMS_PX4_EXPORT_NODE_PLUGIN is defined only on the plugin target.
#ifdef AUTO_APMS_PX4_EXPORT_NODE_PLUGIN
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::SendVehicleCommand)
#endif
