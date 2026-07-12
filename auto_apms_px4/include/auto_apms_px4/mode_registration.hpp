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
#include <memory>
#include <string>
#include <type_traits>

#include "auto_apms_px4_interfaces/msg/registered_mode.hpp"
#include "px4_ros2/components/mode.hpp"
#include "px4_ros2/components/mode_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_px4
{

/**
 * @ingroup auto_apms_px4
 * @brief Handles the registration of a PX4 mode with the FMU and announces its dynamically assigned nav_state.
 *
 * Designed for composition: any class that owns a `px4_ros2::ModeBase` instance can add a ModeRegistrationHandler
 * member and call registerMode() to execute the common registration sequence (wait for the FMU, register the mode,
 * announce the name-to-nav_state mapping).
 *
 * PX4 assigns the nav_state of a (custom/external) mode dynamically during registration, so the mapping from a mode's
 * name to its nav_state is only known at runtime and is not otherwise exposed on a topic that arbitrary subscribers
 * can read. This handler closes that gap: after successful registration it publishes an
 * `auto_apms_px4_interfaces::msg::RegisteredMode` on the shared `registered_modes` topic, so behaviors can discover
 * the mode (e.g. through the `GetModeNavState` behavior tree node). The announcement uses a transient-local (latched)
 * QoS so late-joining subscribers immediately receive it, and it is additionally re-published periodically. The
 * periodic re-announcement acts as an availability heartbeat and, because several mode components typically publish to
 * the same topic, ensures a behavior reliably observes every mode's mapping regardless of its tick rate.
 */
class ModeRegistrationHandler
{
public:
  /// Shared topic (relative to the node namespace) on which registered modes are announced.
  static constexpr auto TOPIC_NAME = "registered_modes";

  /**
   * @brief Constructor.
   * @param node_ptr ROS 2 node used to communicate with the FMU and announce the registered mode.
   */
  explicit ModeRegistrationHandler(rclcpp::Node::SharedPtr node_ptr);

  /// Stops announcing the mode (see stopModeAnnouncement()) if it was ever announced.
  ~ModeRegistrationHandler();

  /**
   * @brief Wait for the FMU, register @p mode with it and announce the mode's assigned nav_state.
   *
   * After successful registration the mode's dynamically assigned nav_state (px4_ros2::ModeBase::id) is announced on
   * the shared `registered_modes` topic (latched and periodically re-announced).
   * @param mode The mode instance to register. Must outlive this handler or be unregistered before its destruction.
   * @param mode_name Name under which the mode is announced.
   * @throw std::runtime_error if the FMU is not available or registration fails.
   */
  void registerMode(px4_ros2::ModeBase & mode, const std::string & mode_name);

  /**
   * @brief Wait for the FMU and register @p executor (together with its owned mode).
   *
   * Mode executors are registered through px4_ros2::ModeExecutorBase::doRegister, which registers the executor and its
   * owned mode in one step. Unlike registerMode(px4_ros2::ModeBase &, const std::string &), the owned mode is *not*
   * announced on the `registered_modes` topic: a mode executor cannot be put in charge by another mode executor, and
   * is intended as a mechanism to trigger automation from PX4 itself (RC switch, GCS), not as a target for native
   * ROS 2 orchestration. From ROS 2, prefer registering and switching plain modes directly.
   * @param executor The mode executor to register. Must outlive this handler.
   * @param mode_name Name of the executor's owned mode, used for logging only.
   * @throw std::runtime_error if the FMU is not available or registration fails.
   */
  void registerMode(px4_ros2::ModeExecutorBase & executor, const std::string & mode_name);

  /**
   * @brief Stop announcing the mode registered with registerMode().
   *
   * Retracts the latched announcement and stops the heartbeat, so behaviors no longer discover the mode. The FMU
   * itself only unregisters the mode when the `px4_ros2::ModeBase` instance is destroyed, which remains the
   * responsibility of the owning class (px4_ros2 exposes no separate unregistration call).
   *
   * Called automatically from the destructor, so an explicit call is only needed to stop the announcement before
   * this handler itself is destroyed (e.g. to retract it while the mode stays registered).
   */
  void stopModeAnnouncement();

private:
  /// Wait for the FMU to become available (with retries). Throws std::runtime_error on timeout.
  void waitForFmu();

  /// Start announcing @p mode_name with its assigned @p nav_state (latched + heartbeat).
  void announce(const std::string & mode_name, uint8_t nav_state);

  const rclcpp::Node::SharedPtr node_ptr_;
  auto_apms_px4_interfaces::msg::RegisteredMode registered_mode_msg_;
  rclcpp::Publisher<auto_apms_px4_interfaces::msg::RegisteredMode>::SharedPtr registered_mode_pub_;
  rclcpp::TimerBase::SharedPtr announce_timer_;
};

/**
 * @ingroup auto_apms_px4
 * @brief Composable ROS 2 component that registers a custom PX4 mode and announces its dynamically assigned nav_state.
 *
 * Use this to bring up a custom mode (a `px4_ros2::ModeBase` subclass constructible from `(rclcpp::Node &,
 * px4_ros2::ModeBase::Settings)`) that is meant to be activated declaratively from a behavior tree via the
 * `SwitchMode` node - as opposed to ModeProxyActionFactory, which drives a mode through a ROS 2 action.
 *
 * On construction it instantiates the mode and delegates the registration sequence (wait for FMU, register, announce)
 * to a ModeRegistrationHandler. Multiple factories can be composed into a single process through `rclcpp_components`;
 * each announces its own mode on the shared `registered_modes` topic.
 *
 * How the factory is deployed is up to the user. Within `auto_apms_px4` we stick to `rclcpp_components`: derive a
 * component class with an `explicit MyComponent(const rclcpp::NodeOptions &)` constructor that forwards the mode name,
 * apply `RCLCPP_COMPONENTS_REGISTER_NODE(MyComponent)` and register it in CMake with
 * `rclcpp_components_register_nodes(<target> "MyComponent")`.
 *
 * @tparam ModeT Custom PX4 mode class inheriting px4_ros2::ModeBase.
 */
template <class ModeT>
class ModeRegistrationFactory
{
public:
  ModeRegistrationFactory(const std::string & mode_name, const rclcpp::NodeOptions & options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

private:
  rclcpp::Node::SharedPtr node_ptr_;
  std::unique_ptr<ModeT> mode_ptr_;
  ModeRegistrationHandler registration_handler_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <class ModeT>
ModeRegistrationFactory<ModeT>::ModeRegistrationFactory(
  const std::string & mode_name, const rclcpp::NodeOptions & options)
: node_ptr_(std::make_shared<rclcpp::Node>(mode_name + "_node", options)),
  mode_ptr_(std::make_unique<ModeT>(*node_ptr_, px4_ros2::ModeBase::Settings(mode_name))),
  registration_handler_(node_ptr_)
{
  static_assert(
    std::is_base_of<px4_ros2::ModeBase, ModeT>::value,
    "Template argument ModeT must publicly inherit px4_ros2::ModeBase.");

  registration_handler_.registerMode(*mode_ptr_, mode_name);
}

template <class ModeT>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ModeRegistrationFactory<ModeT>::get_node_base_interface()
{
  return node_ptr_->get_node_base_interface();
}

}  // namespace auto_apms_px4
