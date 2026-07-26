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
#include <memory>

#include "auto_apms_behavior_tree_core/node.hpp"
#include "px4_msgs/msg/mode_completed.hpp"
#include "px4_ros2/utils/message_version.hpp"

#define INPUT_KEY_NAV_STATE "nav_state"

namespace auto_apms_px4
{

/**
 * @ingroup auto_apms_px4
 * @brief Condition node that succeeds once the mode with the expected nav_state reports successful completion.
 *
 * Subscribes to `fmu/out/mode_completed` (message version suffix resolved at runtime). PX4's auto modes (e.g. Takeoff,
 * Land, RTL, Mission) publish a single `ModeCompleted` message when the maneuver finishes; this node returns SUCCESS on
 * the tick that delivers a completion for the `nav_state` input port with result `RESULT_SUCCESS`, otherwise FAILURE.
 *
 * Because `mode_completed` is published only once (not continuously like `vehicle_status`), wrap this node in an
 * infinite `RetryUntilSuccessful` decorator: the subscription callback buffers the one-shot message and the retry loop
 * catches it on the next tick. Unlike %CheckNavState (which confirms a mode merely became active), this node waits for
 * the mode to actually run to completion - use it to build the execution ("Do") behaviors such as DoTakeoff/DoLanding.
 *
 * @note A completion with a non-success result is treated as "not done yet" (FAILURE), so the retry loop keeps waiting.
 */
class CheckModeCompleted : public auto_apms_behavior_tree::core::RosSubscriberNode<px4_msgs::msg::ModeCompleted>
{
public:
  CheckModeCompleted(
    const std::string & instance_name, const BT::NodeConfig & config,
    const auto_apms_behavior_tree::core::RosNodeContext & context)
  : RosSubscriberNode{instance_name, config, context, rclcpp::SensorDataQoS{}}
  {
    // Resolve the topic's message version suffix at runtime from the message definition, rather than baking it into
    // the node manifest at configure time. Since the manifest does not fix a topic, the base class cannot resolve one
    // at construction and defers creation to this constructor.
    createSubscriber("fmu/out/mode_completed" + px4_ros2::getMessageNameVersion<px4_msgs::msg::ModeCompleted>());
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<int>(INPUT_KEY_NAV_STATE, "PX4 navigation state (mode id) whose completion is awaited."),
    });
  }

  BT::NodeStatus onTick(const std::shared_ptr<px4_msgs::msg::ModeCompleted> & last_msg_ptr) final
  {
    // No completion message delivered on this tick - keep waiting (the one-shot message is buffered by the base class
    // and delivered on the tick following its arrival).
    if (!last_msg_ptr) return BT::NodeStatus::FAILURE;

    const BT::Expected<int> expected_nav_state = getInput<int>(INPUT_KEY_NAV_STATE);
    if (!expected_nav_state) {
      RCLCPP_ERROR(
        logger_, "%s - Missing required input '%s': %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        INPUT_KEY_NAV_STATE, expected_nav_state.error().c_str());
      return BT::NodeStatus::FAILURE;
    }

    // A completion for a different mode is irrelevant - keep waiting for ours.
    if (last_msg_ptr->nav_state != static_cast<uint8_t>(expected_nav_state.value())) return BT::NodeStatus::FAILURE;

    if (last_msg_ptr->result != px4_msgs::msg::ModeCompleted::RESULT_SUCCESS) {
      RCLCPP_WARN(
        logger_, "%s - Mode nav_state %d reported completion with non-success result %u.",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), expected_nav_state.value(), last_msg_ptr->result);
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::CheckModeCompleted)
