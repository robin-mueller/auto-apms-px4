// Copyright 2026 Robin Müller
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

#include "auto_apms_behavior_tree_core/node.hpp"
#include "auto_apms_px4_interfaces/action/set_actuators.hpp"

#define INPUT_KEY_MOTOR_COMMANDS "motor_commands"
#define INPUT_KEY_SERVO_COMMANDS "servo_commands"
#define INPUT_KEY_HOLD_PERIOD_MS "hold_period_ms"

namespace auto_apms_px4_behavior
{

class SetActuatorsAction
: public auto_apms_behavior_tree::core::RosActionNode<auto_apms_px4_interfaces::action::SetActuators>
{
public:
  using RosActionNode::RosActionNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<float>>(
        INPUT_KEY_MOTOR_COMMANDS,
        "Motor commands as a semicolon-separated list. Range: [-1, 1], 'nan'|'NaN'|'NAN' = disarmed"),
      BT::InputPort<std::vector<float>>(
        INPUT_KEY_SERVO_COMMANDS,
        "Servo commands as a semicolon-separated list. Range: [-1, 1], 'nan'|'NaN'|'NAN' = disarmed"),
      BT::InputPort<uint32_t>(
        INPUT_KEY_HOLD_PERIOD_MS, static_cast<uint32_t>(0),
        "Hold the commands for this duration [ms] before stopping. 0 = send once then stop immediately")};
  }

  bool setGoal(Goal & goal)
  {
    auto read_vector = [this](const char * key, std::vector<float> & out) -> bool {
      BT::PortsRemapping::iterator it = config().input_ports.find(key);
      if (it == config().input_ports.end() || it->second.empty()) {
        return true;  // optional port — leave empty
      }
      if (const BT::Expected<std::vector<float>> expected = getInput<std::vector<float>>(key)) {
        out = expected.value();
      } else {
        RCLCPP_ERROR(
          logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
        return false;
      }
      return true;
    };

    if (!read_vector(INPUT_KEY_MOTOR_COMMANDS, goal.motor_commands)) return false;
    if (!read_vector(INPUT_KEY_SERVO_COMMANDS, goal.servo_commands)) return false;

    if (const BT::Expected<uint32_t> expected = getInput<uint32_t>(INPUT_KEY_HOLD_PERIOD_MS)) {
      goal.hold_period_ms = expected.value();
    } else {
      RCLCPP_ERROR(logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected.error().c_str());
      return false;
    }

    return true;
  }
};

}  // namespace auto_apms_px4_behavior

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4_behavior::SetActuatorsAction)
