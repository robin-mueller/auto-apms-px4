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

#include <cmath>
#include <cstdint>

#include "auto_apms_px4/node/send_vehicle_command.hpp"

#define INPUT_KEY_LATITUDE "latitude"
#define INPUT_KEY_LONGITUDE "longitude"
#define INPUT_KEY_ALTITUDE "altitude"
#define INPUT_KEY_YAW "yaw"
#define INPUT_KEY_GROUND_SPEED "ground_speed"

namespace auto_apms_px4
{

/**
 * @ingroup auto_apms_px4
 * @brief Behavior tree node that commands the vehicle to reposition to a global WGS84 location.
 *
 * A specialization of %SendVehicleCommand that fixes the command to `VEHICLE_CMD_DO_REPOSITION` (192) and exposes the
 * target latitude/longitude/altitude (plus optional yaw and ground speed) through friendly input ports instead of the
 * raw command parameters. PX4 switches into its Reposition flight mode and flies to the requested WGS84 position.
 *
 * As with any %SendVehicleCommand, the command's `source_component` is resolved from the global blackboard so that an
 * in-charge `auto_apms_px4::BehaviorModeExecutor` can attribute the reposition to itself. If no executor is in charge,
 * the command is sent with the default source component (0).
 *
 * @note This node only requests the reposition; it returns SUCCESS as soon as the command has been published and does
 * not wait for the vehicle to arrive. Pair it with %GetGlobalPosition in a retry loop to build a behavior that only
 * returns once the target location has actually been reached.
 */
class SendCmdDoReposition : public SendVehicleCommand
{
public:
  using SendVehicleCommand::SendVehicleCommand;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(INPUT_KEY_LATITUDE, "Target latitude [°] (WGS84)."),
      BT::InputPort<double>(INPUT_KEY_LONGITUDE, "Target longitude [°] (WGS84)."),
      BT::InputPort<double>(INPUT_KEY_ALTITUDE, "Target altitude [m] (AMSL)."),
      BT::InputPort<double>(
        INPUT_KEY_YAW, "Desired yaw [°] clockwise from north. Leave empty to keep the current heading mode."),
      BT::InputPort<double>(INPUT_KEY_GROUND_SPEED, -1.0, "Ground speed [m/s]. Negative selects the PX4 default."),
    };
  }

  bool setMessage(px4_msgs::msg::VehicleCommand & msg) override final
  {
    const BT::Expected<double> expected_lat = getInput<double>(INPUT_KEY_LATITUDE);
    const BT::Expected<double> expected_lon = getInput<double>(INPUT_KEY_LONGITUDE);
    const BT::Expected<double> expected_alt = getInput<double>(INPUT_KEY_ALTITUDE);
    if (!expected_lat) {
      RCLCPP_ERROR(
        logger_, "%s - Missing required input '%s': %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        INPUT_KEY_LATITUDE, expected_lat.error().c_str());
      return false;
    }
    if (!expected_lon) {
      RCLCPP_ERROR(
        logger_, "%s - Missing required input '%s': %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        INPUT_KEY_LONGITUDE, expected_lon.error().c_str());
      return false;
    }
    if (!expected_alt) {
      RCLCPP_ERROR(
        logger_, "%s - Missing required input '%s': %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        INPUT_KEY_ALTITUDE, expected_alt.error().c_str());
      return false;
    }

    msg = px4_msgs::msg::VehicleCommand{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
    msg.param1 = static_cast<float>(getInput<double>(INPUT_KEY_GROUND_SPEED).value_or(-1.0));  // Ground speed [m/s].
    msg.param2 = 1.0f;  // MAV_DO_REPOSITION_FLAGS: bit 0 set -> switch to (guided) Reposition mode.
    msg.param3 = 0.0f;  // Loiter radius (planes only), unused for multicopters.

    // Yaw is optional: an empty port leaves the heading unchanged (NaN tells PX4 to keep its current yaw mode).
    const BT::Expected<double> expected_yaw = getInput<double>(INPUT_KEY_YAW);
    msg.param4 = expected_yaw ? static_cast<float>(expected_yaw.value()) : std::numeric_limits<float>::quiet_NaN();

    msg.param5 = expected_lat.value();                      // Latitude [°] (float64, full precision).
    msg.param6 = expected_lon.value();                      // Longitude [°] (float64, full precision).
    msg.param7 = static_cast<float>(expected_alt.value());  // Altitude [m] (AMSL).
    msg.source_component = resolveSourceComponent();
    msg.from_external = true;
    msg.timestamp = 0;  // Let PX4 set the timestamp.

    RCLCPP_DEBUG(
      logger_, "%s - Repositioning to (%.7f, %.7f, %.2f) (source_component %u).",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), msg.param5, msg.param6, static_cast<double>(msg.param7),
      msg.source_component);
    return true;
  }
};

}  // namespace auto_apms_px4

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_px4::SendCmdDoReposition)
