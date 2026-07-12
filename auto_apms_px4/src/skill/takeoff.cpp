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

#include "auto_apms_px4_interfaces/action/takeoff.hpp"

#include "auto_apms_px4/mode_proxy_action.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_ros2/utils/message_version.hpp"

namespace auto_apms_px4
{

class TakeoffSkill : public ModeProxyAction<auto_apms_px4_interfaces::action::Takeoff>
{
public:
  explicit TakeoffSkill(const rclcpp::NodeOptions & options)
  : ModeProxyAction(_AUTO_APMS_PX4__TAKEOFF_ACTION_NAME, options, FlightMode::Takeoff)
  {
    vehicle_global_position_sub_ptr_ = this->node_ptr_->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      "fmu/out/vehicle_global_position" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleGlobalPosition>(),
      rclcpp::SensorDataQoS(), [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg_ptr) {
        last_vehicle_global_position_ = std::move(*msg_ptr);
      });
  }

private:
  bool sendActivationCommand(const VehicleCommandClient & client, std::shared_ptr<const Goal> goal_ptr) override final
  {
    float target_altitude = goal_ptr->alt;
    if (!goal_ptr->use_amsl) {
      target_altitude += last_vehicle_global_position_.alt;
    }
    RCLCPP_DEBUG(
      node_ptr_->get_logger(), "Target altitude: %f m (AMSL) - Heading: %f rad", target_altitude,
      goal_ptr->heading_rad);
    return client.takeoff(target_altitude, goal_ptr->heading_rad);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_ptr_;
  px4_msgs::msg::VehicleGlobalPosition last_vehicle_global_position_;
};

}  // namespace auto_apms_px4

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_px4::TakeoffSkill)
