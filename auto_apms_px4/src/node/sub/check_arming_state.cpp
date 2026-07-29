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
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_ros2/utils/message_version.hpp"

#define INPUT_KEY_ARMING_STATE "arming_state"

namespace auto_apms_px4
{

/**
 * @ingroup auto_apms_px4
 * @brief Condition node that succeeds once the vehicle's arming state matches the expected one.
 *
 * Subscribes to `fmu/out/vehicle_status` (message version suffix resolved at runtime) and compares the current
 * `arming_state` against the `arming_state` input port (px4_msgs::msg::VehicleStatus::ARMING_STATE_*: 1 = DISARMED,
 * 2 = ARMED). Returns SUCCESS if they match, otherwise FAILURE (also while no vehicle status has been received yet).
 *
 * Wrap this node in a retry decorator to wait for an arm/disarm command - issued via %SendVehicleCommand
 * (VEHICLE_CMD_COMPONENT_ARM_DISARM) - to actually take effect on the vehicle (see the SetArm/SetDisarm behaviors).
 */
class CheckArmingState : public auto_apms_behavior_tree::core::RosSubscriberNode<px4_msgs::msg::VehicleStatus>
{
  std::shared_ptr<px4_msgs::msg::VehicleStatus> last_msg_;

public:
  CheckArmingState(
    const std::string & instance_name, const BT::NodeConfig & config,
    const auto_apms_behavior_tree::core::RosNodeContext & context)
  : RosSubscriberNode{instance_name, config, context, rclcpp::SensorDataQoS{}}
  {
    // Resolve the topic's message version suffix at runtime from the message definition, rather than baking it into
    // the node manifest at configure time. Since the manifest does not fix a topic, the base class cannot resolve one
    // at construction and defers creation to this constructor.
    createSubscriber("fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>());
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>(
        INPUT_KEY_ARMING_STATE, "Expected PX4 arming state the vehicle should be in (1 = disarmed, 2 = armed)."),
    };
  }

  BT::NodeStatus onTick(const std::shared_ptr<px4_msgs::msg::VehicleStatus> & last_msg_ptr) final
  {
    if (last_msg_ptr) last_msg_ = last_msg_ptr;

    // No vehicle status received yet - the arming state cannot be confirmed.
    if (!last_msg_) return BT::NodeStatus::FAILURE;

    const BT::Expected<int> expected_arming_state = getInput<int>(INPUT_KEY_ARMING_STATE);
    if (!expected_arming_state) {
      RCLCPP_ERROR(
        logger_, "%s - Missing required input '%s': %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        INPUT_KEY_ARMING_STATE, expected_arming_state.error().c_str());
      return BT::NodeStatus::FAILURE;
    }

    return last_msg_->arming_state == static_cast<uint8_t>(expected_arming_state.value()) ? BT::NodeStatus::SUCCESS
                                                                                          : BT::NodeStatus::FAILURE;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::CheckArmingState)
