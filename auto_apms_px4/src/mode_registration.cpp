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

#include "auto_apms_px4/mode_registration.hpp"

#include <chrono>
#include <stdexcept>

#include "px4_ros2/components/wait_for_fmu.hpp"

namespace auto_apms_px4
{

ModeRegistrationHandler::ModeRegistrationHandler(rclcpp::Node::SharedPtr node_ptr) : node_ptr_(node_ptr) {}

ModeRegistrationHandler::~ModeRegistrationHandler()
{
  if (registered_mode_pub_) {
    stopModeAnnouncement();
  }
}

void ModeRegistrationHandler::registerMode(px4_ros2::ModeBase & mode, const std::string & mode_name)
{
  waitForFmu();

  if (!mode.doRegister()) {
    RCLCPP_FATAL(node_ptr_->get_logger(), "Registration of mode with name '%s' failed.", mode_name.c_str());
    throw std::runtime_error("Mode registration failed");
  }

  // AFTER (!) registration, the nav_state (mode id) is known and can be announced for discovery by behaviors.
  announce(mode_name, mode.id());

  RCLCPP_INFO(node_ptr_->get_logger(), "Registered mode '%s' with nav_state %i.", mode_name.c_str(), mode.id());
}

void ModeRegistrationHandler::registerMode(px4_ros2::ModeExecutorBase & executor, const std::string & mode_name)
{
  waitForFmu();

  // Registers the executor together with its owned mode. No announcement: a mode executor cannot be put in charge by
  // another mode executor, so it is not relevant for native ROS 2 orchestration (which targets modes directly).
  if (!executor.doRegister()) {
    RCLCPP_FATAL(
      node_ptr_->get_logger(), "Registration of mode executor for mode with name '%s' failed.", mode_name.c_str());
    throw std::runtime_error("Mode executor registration failed");
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "Registered mode executor owning mode '%s'.", mode_name.c_str());
}

void ModeRegistrationHandler::waitForFmu()
{
  constexpr int max_retries = 5;
  for (int attempt = 0; attempt < max_retries; ++attempt) {
    if (px4_ros2::waitForFMU(*node_ptr_, std::chrono::seconds(3))) {
      RCLCPP_DEBUG(node_ptr_->get_logger(), "FMU availability test successful (attempt %d).", attempt + 1);
      return;
    }
    RCLCPP_WARN(node_ptr_->get_logger(), "No message from FMU (attempt %d/%d). Retrying...", attempt + 1, max_retries);
  }
  throw std::runtime_error("No message from FMU after multiple attempts");
}

void ModeRegistrationHandler::announce(const std::string & mode_name, uint8_t nav_state)
{
  registered_mode_msg_.name = mode_name;
  registered_mode_msg_.nav_state = nav_state;

  // Latched so late-joining subscribers immediately receive the mapping.
  registered_mode_pub_ = node_ptr_->create_publisher<auto_apms_px4_interfaces::msg::RegisteredMode>(
    TOPIC_NAME, rclcpp::QoS(1).transient_local());
  registered_mode_pub_->publish(registered_mode_msg_);

  // Re-announce periodically (availability heartbeat + robust discovery when multiple modes share the topic).
  announce_timer_ = node_ptr_->create_wall_timer(
    std::chrono::seconds(1), [this]() { registered_mode_pub_->publish(registered_mode_msg_); });
}

void ModeRegistrationHandler::stopModeAnnouncement()
{
  // Destroying the publisher retracts the latched announcement for late-joining subscribers.
  announce_timer_.reset();
  registered_mode_pub_.reset();
  RCLCPP_INFO(node_ptr_->get_logger(), "Stopped announcing mode '%s'.", registered_mode_msg_.name.c_str());
}

}  // namespace auto_apms_px4
