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

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include "auto_apms_behavior_tree/executor/generic_executor_node.hpp"
#include "auto_apms_interfaces/action/start_tree_executor.hpp"
#include "auto_apms_interfaces/srv/set_behavior.hpp"
#include "auto_apms_px4/mode_registration.hpp"
#include "auto_apms_px4/vehicle_command_client.hpp"
#include "auto_apms_util/action_context.hpp"
#include "px4_ros2/components/mode.hpp"
#include "px4_ros2/components/mode_executor.hpp"
#include "px4_ros2/control/setpoint_types/direct_actuators.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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
  std::string build_handler;  ///< Fully qualified class name of the build handler plugin (empty keeps the current one).
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
 * The behavior tree is built ahead of time and kept detached (see prepareTree): the potentially expensive tree
 * construction (which instantiates the ROS 2 waitables of the behavior tree nodes) happens once when the node is
 * constructed and again after every run, off the activation path. When the owned mode is selected (RC switch, GCS or
 * `ActivateImmediately`), PX4 puts this executor in charge and `onActivate()` hands the pre-built tree to the
 * in-process executor (@p engine) via `startExecution`, so activation only recreates the lightweight execution timer
 * and starts ticking. A pilot override or failsafe triggers `onDeactivate()`, which terminates the behavior. When the
 * behavior finishes, a configurable in-charge reaction (hold/RTL/land/disarm/complete/none) is performed and a fresh
 * tree is prepared for the next activation.
 *
 * The behavior itself is built from standard AutoAPMS nodes and drives the vehicle through the regular PX4 skills.
 * Before starting, the executor publishes its `VehicleCommand` source component on the global blackboard (under the
 * key configured via the `AUTO_APMS_PX4_SOURCE_COMPONENT_GLOBAL_KEY` compile definition) so that ownership-aware
 * nodes (e.g. `%SendCmdSetNavState`) can attribute their commands to the executor.
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
    CompletionReaction on_completion{CompletionReaction::HOLD};  ///< Reaction after the behavior succeeds.
    CompletionReaction on_failure{CompletionReaction::RTL};      ///< Reaction after the behavior fails.
    bool defer_failsafes{false};  ///< Defer FMU failsafes while a behavior is running (in-charge privilege).
  };

  /// Default time to wait for the owned mode's nav_state to become active in requestAndVerifyActivation().
  static constexpr std::chrono::milliseconds DEFAULT_ACTIVATION_TIMEOUT{5000};

  /**
   * @brief Constructor.
   * @param owned_mode Placeholder mode owned by this executor.
   * @param settings PX4 mode executor settings (activation policy).
   * @param config_provider Callable returning the current Config (completion reactions and failsafe deferral). It is
   *   invoked at the start of every activation, so runtime parameter changes take effect on the next run. It must stay
   *   valid for the lifetime of this executor and is expected to keep any underlying parameter listener alive. The
   *   behavior to run is not part of this config; it is held internally and changed via setBehavior().
   * @param initial_spec Behavior to run initially (seeded from the node's read-only behavior parameters).
   * @param engine In-process behavior tree executor used to run the behavior.
   */
  BehaviorModeExecutor(
    BehaviorOwnedMode & owned_mode, const px4_ros2::ModeExecutorBase::Settings & settings,
    std::function<Config()> config_provider, BehaviorSpec initial_spec,
    auto_apms_behavior_tree::GenericTreeExecutorNode & engine);

  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;

  /**
   * @brief Build the current behavior tree and keep it detached, ready to be executed on the next activation.
   *
   * Builds the tree for the currently set behavior (see setBehavior) via the in-process executor's
   * makeTreeConstructor. This is the expensive step (it instantiates the ROS 2 waitables of the behavior tree nodes)
   * and is intentionally kept off the activation path: it is called once when the node is constructed (before FMU
   * registration, so a build failure prevents the mode from being registered) and again after every run.
   * @throw std::exception if the tree cannot be built (e.g. an invalid build request).
   */
  void prepareTree();

  /**
   * @brief Verify a candidate behavior and, if valid, make it the current behavior.
   *
   * Builds the candidate behavior's tree exactly as prepareTree() does. Building validates the tree structure and
   * instantiates its nodes, so a malformed or unresolvable behavior fails here. On success the freshly built tree and
   * the candidate spec are latched atomically (they become what the next activation runs); on failure the previously
   * prepared tree and spec are left untouched.
   * @param candidate Behavior to verify and set.
   * @param[out] message Diagnostic detail on failure (the build error).
   * @return `true` if the candidate was verified and set, `false` otherwise.
   */
  bool setBehavior(const BehaviorSpec & candidate, std::string & message);

  /**
   * @brief Request activation of the owned mode from ROS 2 and wait until it is confirmed in charge.
   *
   * Commands the FMU to switch to the owned mode and blocks until the vehicle reports the owned mode's nav_state as
   * active (i.e. this executor has been put in charge and onActivate() has run the prepared tree). Whether activation
   * is granted depends on the configured activation policy and the vehicle state (e.g. it must be armed under the
   * `armed` policy).
   * @param timeout Maximum time to wait for the owned mode to become active.
   * @return `true` if the owned mode became active within @p timeout, `false` otherwise.
   */
  bool requestAndVerifyActivation(std::chrono::milliseconds timeout = DEFAULT_ACTIVATION_TIMEOUT);

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
  /**
   * @brief Build a detached behavior tree for @p spec.
   *
   * Publishes the mode-executor-active flag on the global blackboard (so the behavior tree nodes pick it up as they
   * construct) and builds the tree via the in-process executor's makeTreeConstructor.
   * @throw std::exception if the tree cannot be built.
   */
  std::unique_ptr<auto_apms_behavior_tree::Tree> buildTree(const BehaviorSpec & spec);

  void performReaction(CompletionReaction reaction, ExecutionResult result);

  /**
   * @brief Schedule the builtin PX4 Loiter mode.
   *
   * Loiter is the vehicle-type-aware hold mode (position hold for multicopters, orbit for fixed-wing), making it the
   * safe way to hold for both airframes. It has no completion signal, so this schedule method takes no on_completed
   * callback.
   */
  void scheduleLoiter();

  rclcpp::Node & node_;
  BehaviorOwnedMode & owned_mode_;
  auto_apms_behavior_tree::GenericTreeExecutorNode & behavior_executor_;
  std::function<Config()> config_provider_;
  Config config_;              ///< Snapshot of the configuration, refreshed from config_provider_ on each activation.
  BehaviorSpec current_spec_;  ///< Currently set behavior; authoritative source for what prepareTree() builds.
  VehicleCommandClient vehicle_command_client_;  ///< Used to request activation of the owned mode from ROS 2.
  /// Detached, pre-built behavior tree handed to the executor on activation (see prepareTree).
  std::unique_ptr<auto_apms_behavior_tree::Tree> prepared_tree_ptr_;
};

/**
 * @ingroup auto_apms_px4
 * @brief ROS 2 component that hosts a BehaviorModeExecutor and its owned mode on an in-process behavior tree executor
 * node.
 *
 * Configuration is entirely parameter driven. Registration-time parameters are read once here in the constructor and
 * are declared read only; the remaining parameters are dynamic and re-read on each activation, so they can be changed
 * at runtime (e.g. via `ros2 param set`) and take effect the next time the executor is put in charge:
 * - `activation` (string, read only): `armed`, `always` or `immediately`
 *   (see px4_ros2::ModeExecutorBase::Settings::Activation). Latched at mode registration.
 * - `mode_name` (string, read only): Registered name of the owned PX4 mode. Latched at mode registration.
 * - `on_completion` / `on_failure` (string, dynamic): Reaction after the behavior succeeds/fails
 *   (`hold`, `rtl`, `land`, `disarm`, `complete`, `none`).
 * - `defer_failsafes` (bool, dynamic): Defer FMU failsafes while a behavior is running.
 * - `behavior.build_request` / `behavior.entry_point` / `behavior.node_manifest` (dynamic): Which behavior to run.
 * - `build_handler` (string), `tick_rate` (double), `groot2_port` (int): Standard behavior tree executor parameters
 *   consumed via inherited GenericTreeExecutorNode.
 *
 * This node disables GenericTreeExecutorNode's strict unkown parameter removal via
 * TreeExecutorNodeOptions so these behavior-specific parameters can coexist on the same node.
 *
 * Registration with the FMU (a blocking operation) happens in the constructor and is delegated to a
 * ModeRegistrationHandler. Unlike a plain mode registered via ModeRegistrationFactory, the owned mode is not
 * announced on the `registered_modes` topic: a mode executor cannot be put in charge by another mode executor, so it
 * is not a target for `SendCmdSetNavState`-based orchestration from a behavior tree. Mode executors instead serve as a
 * mechanism to trigger automation from PX4 itself (RC switch, GCS, `ActivateImmediately`).
 *
 * The behavior to run is not changed through the `behavior.*` parameters (which are read only and only seed the
 * initial behavior). Instead the node offers two interfaces that both verify a new behavior by building its tree
 * before latching it (an invalid behavior is rejected and the previously set one is kept):
 * - a `~/set_behavior` service (`auto_apms_interfaces::srv::SetBehavior`): safely change the current behavior without
 *   running it.
 * - a `~/start` action (`auto_apms_interfaces::action::StartTreeExecutor`): change the behavior and trigger it from
 *   ROS 2 by requesting activation of the owned mode. Activation is confirmed by waiting for the owned mode's
 *   nav_state to become active. Attached goals (`attach=true`) stay open and return the tree result on termination;
 *   detached goals succeed once activation is verified.
 */
class BehaviorModeExecutorNode : public auto_apms_behavior_tree::GenericTreeExecutorNode
{
public:
  using StartAction = auto_apms_interfaces::action::StartTreeExecutor;
  using StartActionContext = auto_apms_util::ActionContext<StartAction>;
  using StartGoalHandle = rclcpp_action::ServerGoalHandle<StartAction>;
  using SetBehaviorSrv = auto_apms_interfaces::srv::SetBehavior;

  explicit BehaviorModeExecutorNode(const rclcpp::NodeOptions & options);

protected:
  void onTermination(const ExecutionResult & result) override;

private:
  static px4_ros2::ModeExecutorBase::Settings::Activation activationFromString(const std::string & str);

  /**
   * @brief Apply the candidate's build handler (if any) and verify-and-latch the behavior.
   *
   * Mirrors the standard executor's goal handling: a non-empty `build_handler` is loaded when the
   * `allow_other_build_handlers` option is enabled, and otherwise only accepted if it equals the current handler. The
   * behavior is then verified and latched via BehaviorModeExecutor::setBehavior. If verification fails after the
   * handler was switched, the previous handler is restored so a rejected change leaves executor state untouched.
   * @param candidate Behavior to apply (its `build_handler` is resolved to the effective handler on success).
   * @param[out] message Diagnostic detail on failure.
   * @return `true` if the behavior was verified and set, `false` otherwise.
   */
  bool applyBehavior(BehaviorSpec candidate, std::string & message);

  /* set_behavior service */
  void handleSetBehavior(
    const std::shared_ptr<SetBehaviorSrv::Request> request, std::shared_ptr<SetBehaviorSrv::Response> response);

  /* StartTreeExecutor action */
  rclcpp_action::GoalResponse handleStartGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const StartAction::Goal> goal_ptr);
  rclcpp_action::CancelResponse handleStartCancel(std::shared_ptr<StartGoalHandle> goal_handle_ptr);
  void handleStartAccept(std::shared_ptr<StartGoalHandle> goal_handle_ptr);

  std::unique_ptr<BehaviorOwnedMode> owned_mode_ptr_;
  std::unique_ptr<BehaviorModeExecutor> mode_executor_ptr_;
  ModeRegistrationHandler registration_handler_;

  rclcpp::Service<SetBehaviorSrv>::SharedPtr set_behavior_service_ptr_;
  rclcpp_action::Server<StartAction>::SharedPtr start_action_ptr_;
  StartActionContext start_action_context_;
  /// Per-accepted-goal flags carried from goal acceptance to the accept handler.
  std::map<rclcpp_action::GoalUUID, bool> pending_attach_;
  std::map<rclcpp_action::GoalUUID, bool> pending_clear_blackboard_;
};

}  // namespace auto_apms_px4
