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

#include <limits>

#include "auto_apms_px4/mode.hpp"
#include "auto_apms_px4/mode_executor.hpp"
#include "auto_apms_px4_interfaces/action/set_actuators.hpp"
#include "px4_ros2/control/setpoint_types/direct_actuators.hpp"

namespace auto_apms_px4
{

using SetActuatorsActionType = auto_apms_px4_interfaces::action::SetActuators;

class SetActuatorsMode : public ModeBase<SetActuatorsActionType>
{
  std::shared_ptr<px4_ros2::DirectActuatorsSetpointType> actuator_setpoint_ptr_;
  rclcpp::Time activation_time_;

public:
  SetActuatorsMode(
    rclcpp::Node & node, px4_ros2::ModeBase::Settings settings, std::shared_ptr<ActionContextType> action_context_ptr)
  : ModeBase{node, settings, action_context_ptr}
  {
    actuator_setpoint_ptr_ = std::make_shared<px4_ros2::DirectActuatorsSetpointType>(*this);
  }

private:
  void onActivateWithGoal(std::shared_ptr<const Goal> /*goal_ptr*/) final
  {
    activation_time_ = node().get_clock()->now();
  }

  void stopActuators()
  {
    constexpr auto kNaN = std::numeric_limits<float>::quiet_NaN();
    constexpr int kMaxMotors = px4_ros2::DirectActuatorsSetpointType::kMaxNumMotors;
    constexpr int kMaxServos = px4_ros2::DirectActuatorsSetpointType::kMaxNumServos;
    actuator_setpoint_ptr_->updateMotors(Eigen::Matrix<float, kMaxMotors, 1>::Constant(kNaN));
    actuator_setpoint_ptr_->updateServos(Eigen::Matrix<float, kMaxServos, 1>::Constant(kNaN));
  }

  void onDeactivateWithGoal(std::shared_ptr<const Goal> /*goal_ptr*/) final { stopActuators(); }

  void updateSetpointOnCancel(
    float /*dt_s*/, std::shared_ptr<const Goal> /*goal_ptr*/, std::shared_ptr<Feedback> /*feedback_ptr*/,
    std::shared_ptr<Result> /*result_ptr*/) final
  {
    stopActuators();
    completed(px4_ros2::Result::Success);
  }

  void updateSetpointWithGoal(
    float /*dt_s*/, std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Feedback> /*feedback_ptr*/,
    std::shared_ptr<Result> /*result_ptr*/) final
  {
    constexpr auto kNaN = std::numeric_limits<float>::quiet_NaN();
    constexpr int kMaxMotors = px4_ros2::DirectActuatorsSetpointType::kMaxNumMotors;
    constexpr int kMaxServos = px4_ros2::DirectActuatorsSetpointType::kMaxNumServos;

    // Build motor command vector (NaN = disarmed for unspecified channels)
    Eigen::Matrix<float, kMaxMotors, 1> motor_cmds = Eigen::Matrix<float, kMaxMotors, 1>::Constant(kNaN);
    for (size_t i = 0; i < std::min(goal_ptr->motor_commands.size(), static_cast<size_t>(kMaxMotors)); ++i) {
      motor_cmds(i) = goal_ptr->motor_commands[i];
    }

    // Build servo command vector (NaN = disarmed for unspecified channels)
    Eigen::Matrix<float, kMaxServos, 1> servo_cmds = Eigen::Matrix<float, kMaxServos, 1>::Constant(kNaN);
    for (size_t i = 0; i < std::min(goal_ptr->servo_commands.size(), static_cast<size_t>(kMaxServos)); ++i) {
      servo_cmds(i) = goal_ptr->servo_commands[i];
    }

    const rclcpp::Duration elapsed = node().get_clock()->now() - activation_time_;
    const rclcpp::Duration hold_duration =
      rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(goal_ptr->hold_period_ms) * 1'000'000LL);

    if (elapsed < hold_duration) {
      // First call or still within hold period: send the requested commands
      actuator_setpoint_ptr_->updateMotors(motor_cmds);
      actuator_setpoint_ptr_->updateServos(servo_cmds);
    } else {
      // Hold period elapsed: stop all actuators and signal completion
      stopActuators();
      completed(px4_ros2::Result::Success);
    }
  }
};

class SetActuatorsSkill : public ModeExecutorFactory<SetActuatorsActionType, SetActuatorsMode>
{
public:
  explicit SetActuatorsSkill(const rclcpp::NodeOptions & options)
  : ModeExecutorFactory{
      _AUTO_APMS_PX4__SET_ACTUATORS_ACTION_NAME, options, VehicleCommandClient::FlightMode::Unset, true}
  {
  }
};

}  // namespace auto_apms_px4

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_px4::SetActuatorsSkill)
