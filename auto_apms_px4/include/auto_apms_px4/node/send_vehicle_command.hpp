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
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "auto_apms_behavior_tree_core/node.hpp"
#include "auto_apms_behavior_tree_core/node/base/ros_action_node_base.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_px4
{

/**
 * @ingroup auto_apms_px4
 * @brief Generic behavior tree node that publishes a PX4 `VehicleCommand` and waits for its acknowledgement.
 *
 * This is the common building block for all command-based PX4 interactions. It exposes the raw MAVLink-style
 * `VehicleCommand` fields (`command` id and `param1`..`param7`) as input ports so that any command - switching the
 * flight mode, arming/disarming, starting a mission, triggering a servo, ... - can be represented in a behavior tree
 * without a dedicated C++ node. Purpose-built nodes (e.g. %SendCmdSetNavState) specialize this node by fixing the
 * `command` id (and possibly some parameters) and exposing a friendlier port vocabulary.
 *
 * It derives from the generic `RosActionNodeBase` and runs its own two-phase state machine in `tick()`: on the first
 * tick (status IDLE) it publishes the command and starts waiting; on every subsequent tick (status RUNNING) it checks
 * for PX4's reply on `fmu/out/vehicle_command_ack`. It returns SUCCESS once an acknowledgement for this command is
 * received with result `ACCEPTED` (or `IN_PROGRESS`), and FAILURE if the command is rejected/denied or no
 * acknowledgement arrives within the timeout (derived from the registration option `request_timeout`). This validation
 * is built into the node so that a silently rejected command fails the behavior instead of leaving a downstream wait
 * loop hanging forever.
 *
 * To respect the ownership of an in-charge mode executor, the command's `source_component` is resolved from the global
 * blackboard entry an active `auto_apms_px4::BehaviorModeExecutor` publishes. If it is undefined (behavior run
 * standalone, no executor in charge), the command is sent with the default source component (0) and the blackboard is
 * left untouched.
 *
 * The command topic is chosen from the global blackboard: while a `BehaviorModeExecutor` is in charge (it sets a
 * boolean flag), commands are published on the mode-executor command topic (`fmu/in/vehicle_command_mode_executor`);
 * otherwise the default external command topic (`fmu/in/vehicle_command`) is used. A manifest-configured `topic`
 * overrides this default. The message version suffix is appended at runtime in either case.
 */
class SendVehicleCommand : public auto_apms_behavior_tree::core::RosActionNodeBase
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

  ~SendVehicleCommand() override;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  void halt() override;

  /**
   * @brief Populate the `VehicleCommand` message to publish.
   *
   * Specializations override this to fix the command id and expose friendlier ports. Return `false` to abort, in
   * which case the node returns FAILURE without publishing.
   */
  virtual bool setMessage(px4_msgs::msg::VehicleCommand & msg);

protected:
  /**
   * @brief Transitively resolve the in-charge mode executor's source component from the global blackboard.
   * @return The executor's `VehicleCommand` source component, or 0 if no executor published one (normal external
   * command).
   */
  uint16_t resolveSourceComponent();

private:
  /**
   * @brief Shared acknowledgement subscription reused across all %SendVehicleCommand instances on the same ROS 2 node.
   *
   * PX4 publishes every command acknowledgement on a single topic, so one subscription serves every command node. The
   * instance owns the subscription and broadcasts each received message to the per-node listeners registered via
   * addListener(); every node keeps its own acknowledgement-matching state. Obtained from the RosNodeBase shared-entity
   * registry so it is created once and shared while at least one node is alive.
   */
  struct AckSubscription
  {
    using Ack = px4_msgs::msg::VehicleCommandAck;

    AckSubscription(
      rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, const std::string & topic_name);

    void addListener(const void * owner, std::function<void(const Ack::SharedPtr &)> callback);
    void removeListener(const void * owner);
    void broadcast(const Ack::SharedPtr & msg);

    rclcpp::Subscription<Ack>::SharedPtr subscription;
    std::vector<std::pair<const void *, std::function<void(const Ack::SharedPtr &)>>> listeners;
    std::string name;
  };

  /**
   * @brief Determine the command topic (base + runtime message version suffix) to publish on.
   *
   * A manifest-configured `topic` takes precedence. Otherwise the base is chosen from the mode-executor flag on the
   * global blackboard: while a `BehaviorModeExecutor` is in charge, the mode-executor command topic is used, otherwise
   * the default external command topic.
   */
  std::string resolveCommandTopicName();

  /// Command publisher shared across command nodes publishing on the same topic (from the shared-entity registry).
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  /// Acknowledgement subscription shared across all command nodes (from the shared-entity registry).
  std::shared_ptr<AckSubscription> ack_sub_;

  /// Command message to send
  px4_msgs::msg::VehicleCommand tx_msg_;
  /// Latest acknowledgement matching the in-flight command (set asynchronously by the subscription callback).
  std::shared_ptr<px4_msgs::msg::VehicleCommandAck> last_ack_;
  bool waiting_for_ack_ = false;
  rclcpp::Time send_time_;
  double ack_timeout_s_;
};

}  // namespace auto_apms_px4
