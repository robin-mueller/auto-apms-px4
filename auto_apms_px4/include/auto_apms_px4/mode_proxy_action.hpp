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

#include <optional>

#include "auto_apms_px4/mode.hpp"
#include "auto_apms_px4/mode_registration.hpp"
#include "auto_apms_px4/vehicle_command_client.hpp"
#include "auto_apms_util/action_wrapper.hpp"
#include "px4_msgs/msg/mode_completed.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_ros2/components/wait_for_fmu.hpp"
#include "px4_ros2/utils/message_version.hpp"

/**
 * @defgroup auto_apms_px4 PX4 Integration
 * @brief Methods for using [PX4 Autopilot](https://px4.io/) together with AutoAPMS.
 *
 * We allow controlling autonomous sytems running PX4 by incorporating the [PX4/ROS2 Control
 * Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface.html). The user is able to define custom
 * modes that can be dynamically registered with the autopilot. These modes are written in ROS 2 and communicate
 * with PX4 using the internal [uORB messages](https://docs.px4.io/main/en/middleware/uorb.html).
 *
 * > [!note]
 * > The required packages are hosted in a separate repository thus not part of standard AutoAPMS.
 * > Visit [auto-apms-px4](https://github.com/AutoAPMS/auto-apms-px4) for more info.
 */

/**
 * @ingroup auto_apms_px4
 * @brief Implementation of PX4 mode peers offered by [px4_ros2_cpp](https://github.com/Auterion/px4-ros2-interface-lib)
 * enabling integration with AutoAPMS.
 */
namespace auto_apms_px4
{

/**
 * @ingroup auto_apms_px4
 * @brief Generic template class for executing a PX4 mode implementing the interface of a standard ROS 2 action.
 *
 * The modes to be executed must be registered with the PX4 autopilot server before any action goals are sent. By
 * default, only the standard PX4 modes may be executed, but the user may also implement custom modes using
 * ActionDrivenMode.
 * Refer to ModeProxyActionFactory if you want to set up a ROS 2 node for executing your custom modes.
 *
 * ## Usage
 *
 * To register a ROS 2 node component that is able to execute for example the [land
 * mode](https://docs.px4.io/main/en/flight_modes_mc/land.html) when requested, the corresponding executor is
 * implemented as follows:
 *
 * ```cpp
 * #include "auto_apms_px4_interfaces/action/takeoff.hpp"
 * #include "auto_apms_px4/mode_proxy_action.hpp"
 *
 * namespace my_namespace
 * {
 * class MyTakeoffModeProxyAction : public auto_apms_px4::ModeProxyAction<auto_apms_px4_interfaces::action::Takeoff>
 * {
 * public:
 *   explicit MyTakeoffModeProxyAction(const rclcpp::NodeOptions & options)
 *   : ModeProxyAction("my_proxy_action_name", options, FlightMode::Takeoff)
 *   {
 *   }
 *
 *   bool sendActivationCommand(const VehicleCommandClient & client,
 *                              std::shared_ptr<const Goal> goal_ptr) override final
 *   {
 *     return client.takeoff(goal_ptr->altitude_amsl_m, goal_ptr->heading_rad);
 *   }
 * }
 * }  // namespace my_namespace
 *
 * // Register the ROS 2 node component
 * #include "rclcpp_components/register_node_macro.hpp"
 * RCLCPP_COMPONENTS_REGISTER_NODE(my_namespace::MyTakeoffModeProxyAction)
 * ```
 *
 * @note The package `%auto_apms_px4` comes with ROS 2 node components for the most common standard modes and they work
 * out of the box.
 *
 * @tparam ActionT Type of the ROS 2 action.
 */
template <class ActionT>
class ModeProxyAction : public auto_apms_util::ActionWrapper<ActionT>
{
  enum class State : uint8_t
  {
    REQUEST_ACTIVATION,
    WAIT_FOR_ACTIVATION,
    WAIT_FOR_COMPLETION_SIGNAL,
    COMPLETE
  };

public:
  using VehicleCommandClient = auto_apms_px4::VehicleCommandClient;
  using FlightMode = VehicleCommandClient::FlightMode;
  using typename auto_apms_util::ActionWrapper<ActionT>::ActionContextType;
  using typename auto_apms_util::ActionWrapper<ActionT>::Goal;
  using typename auto_apms_util::ActionWrapper<ActionT>::Feedback;
  using typename auto_apms_util::ActionWrapper<ActionT>::Result;
  using ActionStatus = auto_apms_util::ActionStatus;

  explicit ModeProxyAction(
    const std::string & action_name, rclcpp::Node::SharedPtr node_ptr,
    std::shared_ptr<ActionContextType> action_context_ptr, uint8_t mode_id,
    FlightMode deactivation_flight_mode = FlightMode::Hold, bool disarm_after_completion = false);
  explicit ModeProxyAction(
    const std::string & action_name, const rclcpp::NodeOptions & options, uint8_t mode_id,
    FlightMode deactivation_flight_mode = FlightMode::Hold, bool disarm_after_completion = false);
  explicit ModeProxyAction(
    const std::string & action_name, const rclcpp::NodeOptions & options, FlightMode flight_mode,
    FlightMode deactivation_flight_mode = FlightMode::Hold, bool disarm_after_completion = false);

  static bool isExternalMode(uint8_t mode_id);

private:
  void setUp();
  auto_apms_util::ActionStatus asyncDeactivateFlightMode();
  bool onGoalRequest(std::shared_ptr<const Goal> goal_ptr) override final;
  bool onCancelRequest(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) override final;
  auto_apms_util::ActionStatus cancelGoal(
    std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) override final;
  auto_apms_util::ActionStatus executeGoal(
    std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Feedback> feedback_ptr,
    std::shared_ptr<Result> result_ptr) override final;

protected:
  bool isCurrentNavState(uint8_t nav_state);
  virtual bool sendActivationCommand(const VehicleCommandClient & client, std::shared_ptr<const Goal> goal_ptr);
  virtual bool isCompleted(std::shared_ptr<const Goal> goal_ptr, const px4_msgs::msg::VehicleStatus & vehicle_status);
  virtual void setFeedback(std::shared_ptr<Feedback> feedback_ptr, const px4_msgs::msg::VehicleStatus & vehicle_status);

  uint8_t getModeID() const;

private:
  const VehicleCommandClient vehicle_command_client_;
  const uint8_t mode_id_;
  bool is_custom_mode_{false};  // Whether the mode is a custom mode (i.e. not one of the standard PX4 modes)
  FlightMode deactivation_flight_mode_;
  bool disarm_after_completion_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_ptr_;
  rclcpp::Subscription<px4_msgs::msg::ModeCompleted>::SharedPtr mode_completed_sub_ptr_;
  px4_msgs::msg::VehicleStatus::SharedPtr last_vehicle_status_ptr_;
  std::optional<uint8_t> mode_completed_result_;
  bool deactivation_command_sent_{false};
  State state_{State::REQUEST_ACTIVATION};
  rclcpp::Time activation_command_sent_time_;
  rclcpp::Duration activation_timeout_{0, 0};
};

/**
 * @ingroup auto_apms_px4
 * @brief Helper template class that creates a ModeProxyAction for a custom PX4 mode implemented by inheriting from
 * ActionDrivenMode.
 *
 * Composes a ModeRegistrationHandler, so the mode is registered using the common registration sequence (wait for
 * FMU, register, announce the name-to-nav_state mapping on the `registered_modes` topic) just like with
 * ModeRegistrationFactory.
 * @tparam ActionT Type of the ROS 2 action. Must be the same as used by @p ModeT.
 * @tparam ModeT Custom PX4 mode class.
 */
template <class ActionT, class ModeT>
class ModeProxyActionFactory
{
public:
  ModeProxyActionFactory(
    const std::string & action_name, const rclcpp::NodeOptions & options,
    VehicleCommandClient::FlightMode deactivation_flight_mode = VehicleCommandClient::FlightMode::Hold,
    bool disarm_after_completion = false);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

private:
  rclcpp::Node::SharedPtr node_ptr_;
  std::unique_ptr<ActionDrivenMode<ActionT>> mode_ptr_;
  ModeRegistrationHandler registration_handler_;
  std::shared_ptr<ModeProxyAction<ActionT>> mode_proxy_action_ptr_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <class ActionT>
ModeProxyAction<ActionT>::ModeProxyAction(
  const std::string & action_name, rclcpp::Node::SharedPtr node_ptr,
  std::shared_ptr<ActionContextType> action_context_ptr, uint8_t mode_id, FlightMode deactivation_flight_mode,
  bool disarm_after_completion)
: auto_apms_util::ActionWrapper<ActionT>(action_name, node_ptr, action_context_ptr),
  vehicle_command_client_(*node_ptr),
  mode_id_(mode_id),
  deactivation_flight_mode_(deactivation_flight_mode),
  disarm_after_completion_(disarm_after_completion)
{
  setUp();
}

template <class ActionT>
ModeProxyAction<ActionT>::ModeProxyAction(
  const std::string & action_name, const rclcpp::NodeOptions & options, uint8_t mode_id,
  FlightMode deactivation_flight_mode, bool disarm_after_completion)
: auto_apms_util::ActionWrapper<ActionT>(action_name, options),
  vehicle_command_client_(*this->node_ptr_),
  mode_id_(mode_id),
  is_custom_mode_(isExternalMode(mode_id_)),
  deactivation_flight_mode_(deactivation_flight_mode),
  disarm_after_completion_(disarm_after_completion)
{
  setUp();
}

template <class ActionT>
ModeProxyAction<ActionT>::ModeProxyAction(
  const std::string & action_name, const rclcpp::NodeOptions & options, FlightMode flight_mode,
  FlightMode deactivation_flight_mode, bool disarm_after_completion)
: ModeProxyAction<ActionT>(
    action_name, options, static_cast<uint8_t>(flight_mode), deactivation_flight_mode, disarm_after_completion)
{
}

template <class ActionT>
inline bool ModeProxyAction<ActionT>::isExternalMode(uint8_t mode_id)
{
  const uint8_t first_external_mode_nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL1;
  const uint8_t max_external_mode_nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL8 + 1;
  return mode_id >= first_external_mode_nav_state && mode_id < max_external_mode_nav_state;
}

template <class ActionT>
void ModeProxyAction<ActionT>::setUp()
{
  vehicle_status_sub_ptr_ = this->node_ptr_->template create_subscription<px4_msgs::msg::VehicleStatus>(
    "fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(),
    rclcpp::QoS(1).best_effort(),
    [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) { last_vehicle_status_ptr_ = std::move(msg); });

  mode_completed_sub_ptr_ = this->node_ptr_->template create_subscription<px4_msgs::msg::ModeCompleted>(
    "fmu/out/mode_completed" + px4_ros2::getMessageNameVersion<px4_msgs::msg::ModeCompleted>(),
    rclcpp::QoS(1).best_effort(), [this](px4_msgs::msg::ModeCompleted::UniquePtr msg) {
      bool is_relevant = msg->nav_state == this->mode_id_;
      RCLCPP_DEBUG(
        this->node_ptr_->get_logger(),
        "Flight mode completion signal received by mode %i. Signal was: timestamp=%li, mode_id=%i, result=%i (is "
        "relevant for this mode: %i)",
        this->mode_id_, msg->timestamp, msg->nav_state, msg->result, is_relevant);
      if (is_relevant) {
        this->mode_completed_result_ = msg->result;
      }
    });
}

template <class ActionT>
auto_apms_util::ActionStatus ModeProxyAction<ActionT>::asyncDeactivateFlightMode()
{
  // If currently waiting for flight mode activation and the deactivation mode is already active, we need to wait for
  // the nav state to change before starting deactivation. Otherwise, we'll misinterpret the current nav state and
  // return success immediately.
  bool is_deactivation_mode_active = isCurrentNavState(static_cast<uint8_t>(deactivation_flight_mode_));
  if (state_ == State::WAIT_FOR_ACTIVATION) {
    if (is_deactivation_mode_active) {
      auto & clock = *this->node_ptr_->get_clock();
      RCLCPP_DEBUG_THROTTLE(
        this->node_ptr_->get_logger(), clock, 250, "Waiting for flight mode %i to become active before deactivating...",
        mode_id_);
      return ActionStatus::RUNNING;
    } else {
      state_ = State::COMPLETE;  // Change state to indicate that mode has been activated
    }
  }

  if (is_deactivation_mode_active) {
    RCLCPP_DEBUG(
      this->node_ptr_->get_logger(), "Deactivated flight mode successfully (deactivation mode %i is active)",
      static_cast<int>(deactivation_flight_mode_));
    return ActionStatus::SUCCESS;
  } else {
    // Only send command if not in deactivation mode already
    if (!deactivation_command_sent_) {
      if (!vehicle_command_client_.syncActivateFlightMode(deactivation_flight_mode_)) {
        RCLCPP_ERROR(
          this->node_ptr_->get_logger(), "Failed to send command to activate deactivation flight mode %i",
          static_cast<int>(deactivation_flight_mode_));
        return ActionStatus::FAILURE;
      }
      // Force to consider only new status messages after sending new command
      last_vehicle_status_ptr_ = nullptr;
      deactivation_command_sent_ = true;
    }
  }

  return ActionStatus::RUNNING;
}

template <class ActionT>
bool ModeProxyAction<ActionT>::onGoalRequest(const std::shared_ptr<const Goal> /*goal_ptr*/)
{
  state_ = State::REQUEST_ACTIVATION;
  deactivation_command_sent_ = false;
  mode_completed_result_ = std::nullopt;
  activation_timeout_ = rclcpp::Duration::from_seconds(fmin(this->param_listener_.get_params().loop_rate * 15, 1.5));
  return true;
}

template <class ActionT>
bool ModeProxyAction<ActionT>::onCancelRequest(
  std::shared_ptr<const Goal> /*goal_ptr*/, std::shared_ptr<Result> /*result_ptr*/)
{
  RCLCPP_INFO(this->node_ptr_->get_logger(), "Cancellation requested!");
  return true;
}

template <class ActionT>
auto_apms_util::ActionStatus ModeProxyAction<ActionT>::cancelGoal(
  std::shared_ptr<const Goal> /*goal_ptr*/, std::shared_ptr<Result> /*result_ptr*/)
{
  // The custom mode is responsible for managing the lifecycle of the cancellation via the onGoalCanceled callback.
  // Wait until the mode signals completion (by calling completed()) or PX4 deactivates it externally. For standard
  // modes, continue immediately with deactivation since we can't rely on the mode_completed topic for standard modes as
  // they usually don't publish to it.
  if (is_custom_mode_ && state_ != State::COMPLETE) {
    // During cancellation, we bypass the custom isCompleted method and simply wait for any mode completion signal
    // regardless the value
    if (mode_completed_result_.has_value()) {
      state_ = State::COMPLETE;
    } else {
      if (!isCurrentNavState(mode_id_)) {
        RCLCPP_WARN(
          this->node_ptr_->get_logger(), "Flight mode %i was deactivated externally during cancellation", mode_id_);
        // In this case we don't have to do anything afterwards
        return ActionStatus::SUCCESS;
      }
      return ActionStatus::RUNNING;
    }
  }

  ActionStatus ret = ActionStatus::SUCCESS;
  if (deactivation_flight_mode_ != FlightMode::Unset) {
    ret = asyncDeactivateFlightMode();
  }

  if (ret != ActionStatus::RUNNING) {
    if (disarm_after_completion_ && !vehicle_command_client_.disarm()) {
      RCLCPP_WARN(this->node_ptr_->get_logger(), "Failed to disarm after flight mode %i cancellation", mode_id_);
    }
    RCLCPP_INFO(this->node_ptr_->get_logger(), "Flight mode %i cancellation complete", mode_id_);
  }
  return ret;
}

template <class ActionT>
bool ModeProxyAction<ActionT>::isCurrentNavState(uint8_t nav_state)
{
  if (last_vehicle_status_ptr_ && last_vehicle_status_ptr_->nav_state == nav_state) {
    return true;
  }
  return false;
}

template <class ActionT>
auto_apms_util::ActionStatus ModeProxyAction<ActionT>::executeGoal(
  std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Feedback> feedback_ptr, std::shared_ptr<Result> /*result_ptr*/)
{
  switch (state_) {
    case State::REQUEST_ACTIVATION:
      if (!sendActivationCommand(vehicle_command_client_, goal_ptr)) {
        RCLCPP_ERROR(
          this->node_ptr_->get_logger(), "Failed to send activation command for flight mode %i. Aborting...", mode_id_);
        return ActionStatus::FAILURE;
      }
      // Force to consider only new status messages after sending new command
      last_vehicle_status_ptr_ = nullptr;
      state_ = State::WAIT_FOR_ACTIVATION;
      activation_command_sent_time_ = this->node_ptr_->now();
      RCLCPP_DEBUG(
        this->node_ptr_->get_logger(), "Activation command for flight mode %i was sent successfully", mode_id_);
      return ActionStatus::RUNNING;
    case State::WAIT_FOR_ACTIVATION:
      if (isCurrentNavState(mode_id_)) {
        RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Flight mode %i is active", mode_id_);
        state_ = State::WAIT_FOR_COMPLETION_SIGNAL;
      } else if (this->node_ptr_->now() - activation_command_sent_time_ > activation_timeout_) {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "Timeout activating flight mode %i. Aborting...", mode_id_);
        return ActionStatus::FAILURE;
      }
      return ActionStatus::RUNNING;
    case State::WAIT_FOR_COMPLETION_SIGNAL:
      // Populate feedback message
      setFeedback(feedback_ptr, *last_vehicle_status_ptr_);

      // Check if execution should be terminated
      if (isCompleted(goal_ptr, *last_vehicle_status_ptr_)) {
        state_ = State::COMPLETE;
        if (deactivation_flight_mode_ != FlightMode::Unset) {
          RCLCPP_DEBUG(
            this->node_ptr_->get_logger(),
            "Flight mode %i complete! Will deactivate before termination (switching to flight mode %i)...", mode_id_,
            static_cast<int>(deactivation_flight_mode_));
        } else {
          RCLCPP_DEBUG(
            this->node_ptr_->get_logger(),
            "Flight mode %i complete! Will leave current navigation state as is. User is "
            "responsible for initiating the next flight mode...",
            mode_id_);
        }

        // Don't return to complete in same iteration
        break;
      }
      // Check if nav state changed
      if (!isCurrentNavState(mode_id_)) {
        RCLCPP_WARN(this->node_ptr_->get_logger(), "Flight mode %i was deactivated externally. Aborting...", mode_id_);
        return ActionStatus::FAILURE;
      }
      return ActionStatus::RUNNING;
    case State::COMPLETE:
      break;
  }

  if (deactivation_flight_mode_ != FlightMode::Unset) {
    const auto deactivation_state = asyncDeactivateFlightMode();
    if (deactivation_state != ActionStatus::SUCCESS) {
      return deactivation_state;
    }
    // Don't return to complete in same iteration
  }

  RCLCPP_INFO(this->node_ptr_->get_logger(), "Flight mode %i execution complete", mode_id_);
  if (disarm_after_completion_ && !vehicle_command_client_.disarm()) {
    RCLCPP_WARN(this->node_ptr_->get_logger(), "Failed to disarm after flight mode %i completion", mode_id_);
  }
  return ActionStatus::SUCCESS;
}

template <class ActionT>
bool ModeProxyAction<ActionT>::sendActivationCommand(
  const VehicleCommandClient & client, std::shared_ptr<const Goal> /*goal_ptr*/)
{
  return client.syncActivateFlightMode(mode_id_);
}

template <class ActionT>
bool ModeProxyAction<ActionT>::isCompleted(
  std::shared_ptr<const Goal> /*goal_ptr*/, const px4_msgs::msg::VehicleStatus & /*vehicle_status*/)
{
  if (!mode_completed_result_.has_value()) {
    return false;
  }
  if (*mode_completed_result_ == px4_msgs::msg::ModeCompleted::RESULT_SUCCESS) {
    return true;
  } else {
    RCLCPP_INFO(
      this->node_ptr_->get_logger(), "Flight mode %i completed unsuccessfully (result: %i). Aborting...", mode_id_,
      static_cast<int>(*mode_completed_result_));
    this->action_context_ptr_->abort();
  }
  return false;
}

template <class ActionT>
void ModeProxyAction<ActionT>::setFeedback(
  std::shared_ptr<Feedback> /*feedback_ptr*/, const px4_msgs::msg::VehicleStatus & /*vehicle_status*/)
{
  return;
}

template <class ActionT>
uint8_t ModeProxyAction<ActionT>::getModeID() const
{
  return mode_id_;
}

template <class ActionT, class ModeT>
ModeProxyActionFactory<ActionT, ModeT>::ModeProxyActionFactory(
  const std::string & action_name, const rclcpp::NodeOptions & options,
  VehicleCommandClient::FlightMode deactivation_flight_mode, bool disarm_after_completion)
: node_ptr_(std::make_shared<rclcpp::Node>(action_name + "_node", options)), registration_handler_(node_ptr_)
{
  static_assert(
    std::is_base_of<ActionDrivenMode<ActionT>, ModeT>::value,
    "Template argument ModeT must inherit auto_apms_px4::ActionDrivenMode<ActionT> as public and with same type "
    "ActionT as auto_apms_util::ActionWrapper<ActionT>");

  const auto action_context_ptr = std::make_shared<auto_apms_util::ActionContext<ActionT>>(node_ptr_->get_logger());

  mode_ptr_ = std::make_unique<ModeT>(*node_ptr_, px4_ros2::ModeBase::Settings(action_name), action_context_ptr);

  // Wait for the FMU, register the mode and announce its name-to-nav_state mapping.
  registration_handler_.registerMode(*mode_ptr_, action_name);

  // AFTER (!) registration, the mode id can be queried to set up the proxy action
  mode_proxy_action_ptr_ = std::make_shared<ModeProxyAction<ActionT>>(
    action_name, node_ptr_, action_context_ptr, mode_ptr_->id(), deactivation_flight_mode, disarm_after_completion);
}

template <class ActionT, class ModeT>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ModeProxyActionFactory<ActionT, ModeT>::get_node_base_interface()
{
  return node_ptr_->get_node_base_interface();
}

}  // namespace auto_apms_px4
