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

#include <memory>
#include <string>

#include "auto_apms_behavior_tree/executor/generic_executor_node.hpp"
#include "auto_apms_px4/mode_registration.hpp"
#include "px4_ros2/components/mode.hpp"
#include "px4_ros2/components/mode_executor.hpp"
#include "px4_ros2/control/setpoint_types/direct_actuators.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_px4
{

/**
 * @ingroup auto_apms_px4
 * @brief Which behavior to run and how to build it.
 *
 * Mirrors the relevant fields of the `auto_apms_interfaces::action::StartTreeExecutor` goal. The behavior is always
 * assembled from standard AutoAPMS behavior tree nodes; no executor-specific node vocabulary is required.
 */
struct BehaviorSpec
{
  std::string build_request;  ///< Behavior build request (e.g. a registered behavior resource identity or XML).
  std::string entry_point;    ///< Single point of entry for behavior execution.
  std::string node_manifest;  ///< Encoded node manifest specifying additional nodes to load.
};

/**
 * @ingroup auto_apms_px4
 * @brief Registration placeholder PX4 mode owned by a BehaviorModeExecutor.
 *
 * This mode exists only because px4_ros2 requires every mode to declare at least one setpoint type and because a
 * mode executor must own a mode. It is never meant to actively fly: whenever the executor is in charge it keeps the
 * builtin, vehicle-type-aware Loiter mode scheduled, which both provides a fixed-wing- and multicopter-safe hold and
 * prevents this mode from becoming the active setpoint source. It declares a direct-actuators setpoint type only to
 * satisfy that requirement (it needs no position estimate to be valid) but never publishes a setpoint, so it drives no
 * airframe should the mode ever become active. The mode is also where behavior completion is reported to the FMU (see
 * BehaviorModeExecutor::CompletionReaction::COMPLETE).
 */
class BehaviorOwnedMode : public px4_ros2::ModeBase
{
public:
  BehaviorOwnedMode(rclcpp::Node & node, const px4_ros2::ModeBase::Settings & settings);

  void onActivate() override {}
  void onDeactivate() override {}
  void updateSetpoint(float dt_s) override;

  /**
   * @brief Report completion of the mode to the FMU. Only takes effect while the mode is active.
   * @param result PX4 result code.
   */
  void finish(px4_ros2::Result result) { completed(result); }

private:
  std::shared_ptr<px4_ros2::DirectActuatorsSetpointType> actuator_setpoint_ptr_;
};

/**
 * @ingroup auto_apms_px4
 * @brief PX4 mode executor that runs an AutoAPMS behavior in-process when put in charge by the FMU.
 *
 * When the owned mode is selected (RC switch, GCS or `ActivateImmediately`), PX4 puts this executor in charge and
 * `onActivate()` starts the configured behavior on the in-process behavior tree executor (@p engine). A pilot override
 * or failsafe triggers `onDeactivate()`, which halts the behavior. When the behavior finishes, a configurable
 * in-charge reaction (hold/RTL/land/disarm/complete/none) is performed.
 *
 * The behavior itself is built from standard AutoAPMS nodes and drives the vehicle through the regular PX4 skills.
 * Before starting, the executor publishes its `VehicleCommand` source component on the global blackboard (under the
 * key configured via the `AUTO_APMS_PX4_SOURCE_COMPONENT_GLOBAL_KEY` compile definition) so that ownership-aware
 * nodes (e.g. `%SwitchMode`) can attribute their commands to the executor.
 */
class BehaviorModeExecutor : public px4_ros2::ModeExecutorBase
{
public:
  using ExecutionResult = auto_apms_behavior_tree::TreeExecutorBase::ExecutionResult;

  /// Reaction performed when a behavior terminates or fails, using the in-charge executor API.
  enum class CompletionReaction
  {
    HOLD,      ///< Schedule the owned mode (hold position) and stay in charge.
    RTL,       ///< Return to launch.
    LAND,      ///< Land at the current position.
    DISARM,    ///< Disarm the vehicle.
    COMPLETE,  ///< Report the owned mode as completed to the FMU (relinquish charge).
    NONE       ///< Do nothing and stay in charge.
  };

  struct Config
  {
    BehaviorSpec spec;                                           ///< Behavior to run when activated.
    CompletionReaction on_completion{CompletionReaction::HOLD};  ///< Reaction after the behavior succeeds.
    CompletionReaction on_failure{CompletionReaction::RTL};      ///< Reaction after the behavior fails.
    bool defer_failsafes{false};  ///< Defer FMU failsafes while a behavior is running (in-charge privilege).
  };

  /**
   * @brief Constructor.
   * @param owned_mode Placeholder mode owned by this executor.
   * @param settings PX4 mode executor settings (activation policy).
   * @param config Behavior specification and completion reactions.
   * @param engine In-process behavior tree executor used to run the behavior.
   */
  BehaviorModeExecutor(
    BehaviorOwnedMode & owned_mode, const px4_ros2::ModeExecutorBase::Settings & settings, const Config & config,
    auto_apms_behavior_tree::GenericTreeExecutorNode & engine);

  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;

  /**
   * @brief Parse a string into a CompletionReaction.
   * @throw std::invalid_argument if the value is not a valid reaction.
   */
  static CompletionReaction reactionFromString(const std::string & str);

  /**
   * @brief Handle the termination of the behavior running on the in-process executor.
   *
   * Translates the tree execution result into a px4_ros2::Result and performs the configured completion reaction.
   * Intended to be called from the owning node's `TreeExecutorBase::onTermination` override.
   * @param result Final result of the tree execution.
   */
  void onExecutionResult(ExecutionResult result);

private:
  void onBehaviorTerminated(px4_ros2::Result result);
  void performReaction(CompletionReaction reaction, px4_ros2::Result result);

  /**
   * @brief Schedule the builtin PX4 Loiter mode.
   *
   * Loiter is the vehicle-type-aware hold mode (position hold for multicopters, orbit for fixed-wing), making it the
   * safe way to hold for both airframes. Scheduling it also prevents the owned placeholder mode from becoming the
   * active setpoint source while the executor is in charge.
   * @param on_completed Callback forwarded to px4_ros2::ModeExecutorBase::scheduleMode.
   */
  void scheduleLoiter(const px4_ros2::ModeExecutorBase::CompletedCallback & on_completed);

  rclcpp::Node & node_;
  BehaviorOwnedMode & owned_mode_;
  auto_apms_behavior_tree::GenericTreeExecutorNode & behavior_executor_;
  const Config config_;
};

/**
 * @ingroup auto_apms_px4
 * @brief ROS 2 component that hosts a BehaviorModeExecutor and its owned mode on an in-process behavior tree executor
 * node.
 *
 * Configuration is entirely parameter driven:
 * - `activation` (string): `armed`, `always` or `immediately` (see px4_ros2::ModeExecutorBase::Settings::Activation).
 * - `mode_name` (string): Registered name of the owned PX4 mode.
 * - `on_completion` / `on_failure` (string): Reaction after the behavior succeeds/fails
 *   (`hold`, `rtl`, `land`, `disarm`, `complete`, `none`).
 * - `defer_failsafes` (bool): Defer FMU failsafes while a behavior is running.
 * - `behavior.build_request` / `behavior.entry_point` / `behavior.node_manifest`: Which behavior to run.
 * - `build_handler` (string), `tick_rate` (double), `groot2_port` (int): Standard behavior tree executor parameters
 *   consumed via inherited GenericTreeExecutorNode.
 *
 * This node disables GenericTreeExecutorNode's strict unkown parameter removal via
 * TreeExecutorNodeOptions so these behavior-specific parameters can coexist on the same node.
 *
 * Registration with the FMU (a blocking operation) happens in the constructor and is delegated to a
 * ModeRegistrationHandler. Unlike a plain mode registered via ModeRegistrationFactory, the owned mode is not
 * announced on the `registered_modes` topic: a mode executor cannot be put in charge by another mode executor, so it
 * is not a target for `SwitchMode`-based orchestration from a behavior tree. Mode executors instead serve as a
 * mechanism to trigger automation from PX4 itself (RC switch, GCS, `ActivateImmediately`); native ROS 2
 * orchestration should activate modes directly.
 */
class BehaviorModeExecutorNode : public auto_apms_behavior_tree::GenericTreeExecutorNode
{
public:
  explicit BehaviorModeExecutorNode(const rclcpp::NodeOptions & options);

protected:
  void onTermination(const ExecutionResult & result) override;

private:
  static px4_ros2::ModeExecutorBase::Settings::Activation activationFromString(const std::string & str);

  std::unique_ptr<BehaviorOwnedMode> owned_mode_ptr_;
  std::unique_ptr<BehaviorModeExecutor> executor_ptr_;
  ModeRegistrationHandler registration_handler_;
};

}  // namespace auto_apms_px4
