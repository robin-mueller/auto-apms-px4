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

#include "auto_apms_px4/behavior_mode_executor.hpp"

#include <cstdint>
#include <stdexcept>
#include <utility>

#include "auto_apms_behavior_tree/executor/options.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_px4/behavior_mode_executor_params.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

namespace auto_apms_px4
{

// #####################################################################################################################
// ##################################              BehaviorOwnedMode              ######################################
// #####################################################################################################################

BehaviorOwnedMode::BehaviorOwnedMode(rclcpp::Node & node, const px4_ros2::ModeBase::Settings & settings)
: px4_ros2::ModeBase(node, settings)
{
  actuator_setpoint_ptr_ = std::make_shared<px4_ros2::DirectActuatorsSetpointType>(*this);
}

void BehaviorOwnedMode::updateSetpoint(float /*dt_s*/)
{
  // Intentionally a no-op. This mode is a registration placeholder: px4_ros2 requires every mode to declare at least
  // one setpoint type (the DirectActuatorsSetpointType constructed above), but this mode is never meant to actively
  // fly.
}

// #####################################################################################################################
// #################################              BehaviorModeExecutor              ####################################
// #####################################################################################################################

BehaviorModeExecutor::BehaviorModeExecutor(
  BehaviorOwnedMode & owned_mode, const px4_ros2::ModeExecutorBase::Settings & settings,
  std::function<Config()> config_provider, BehaviorSpec initial_spec,
  auto_apms_behavior_tree::GenericTreeExecutorNode & engine)
: px4_ros2::ModeExecutorBase(settings, owned_mode),
  node_(owned_mode.node()),
  owned_mode_(owned_mode),
  behavior_executor_(engine),
  config_provider_(std::move(config_provider)),
  config_(config_provider_()),
  current_spec_(std::move(initial_spec)),
  vehicle_command_client_(node_)
{
}

void BehaviorModeExecutor::onExecutionResult(ExecutionResult result)
{
  RCLCPP_INFO(node_.get_logger(), "Behavior execution result: %s.", auto_apms_behavior_tree::toStr(result).c_str());

  // If we lost charge in the meantime (pilot/failsafe), don't fight the FMU: skip the completion reaction. We still
  // prepare a fresh tree below so the next activation stays lightweight.
  if (isInCharge()) {
    const CompletionReaction reaction =
      result == ExecutionResult::TREE_SUCCEEDED ? config_.on_completion : config_.on_failure;
    performReaction(reaction, result);
  } else {
    RCLCPP_INFO(node_.get_logger(), "No longer in charge. Skipping completion reaction");
  }

  // Rebuild the detached tree so the next time this executor is put in charge, activation does not pay the tree
  // construction cost. A build failure here (e.g. an invalid build request set at runtime) leaves no prepared tree;
  // the next activation then reports an error instead of running a stale one.
  try {
    prepareTree();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_.get_logger(), "Failed to prepare behavior tree for the next activation: %s", e.what());
  }
}

BehaviorModeExecutor::CompletionReaction BehaviorModeExecutor::reactionFromString(const std::string & str)
{
  if (str == "hold") return CompletionReaction::HOLD;
  if (str == "rtl") return CompletionReaction::RTL;
  if (str == "land") return CompletionReaction::LAND;
  if (str == "disarm") return CompletionReaction::DISARM;
  if (str == "complete") return CompletionReaction::COMPLETE;
  if (str == "none") return CompletionReaction::NONE;
  throw std::invalid_argument(
    "Invalid completion reaction '" + str + "' (expected one of: hold, rtl, land, disarm, complete, none)");
}

void BehaviorModeExecutor::onActivate()
{
  // Refresh the configuration from the current parameter values so runtime changes (e.g. via `ros2 param set`) to the
  // completion reactions and failsafe deferral take effect on this activation. The behavior itself was already built
  // (see prepareTree); its build request is latched at build time.
  config_ = config_provider_();

  if (!prepared_tree_ptr_) {
    RCLCPP_ERROR(node_.get_logger(), "Cannot start behavior: no pre-built behavior tree is available");
    onExecutionResult(ExecutionResult::ERROR);
    return;
  }

  RCLCPP_INFO(
    node_.get_logger(), "Behavior executor put in charge. Starting behavior '%s'", current_spec_.build_request.c_str());

  if (config_.defer_failsafes) {
    if (deferFailsafesSync(true)) {
      RCLCPP_INFO(node_.get_logger(), "Failsafes are now being deferred while the behavior is running");
    } else {
      RCLCPP_WARN(node_.get_logger(), "Failed to enable failsafe deferral");
    }
  }

  // Publish the executor's VehicleCommand source component on the global blackboard so that ownership-aware behavior
  // tree nodes (e.g. SendCmdSetNavState) can attribute their commands to this executor. Those nodes read it at tick
  // time, so setting it here (id() is valid once registered) is sufficient. Stored as int to match the type read by
  // the SendCmdSetNavState node. The global key already carries the '@' prefix; on the global blackboard (which is its
  // own root) this resolves to the same entry the tree nodes read transitively.
  const int source_component = static_cast<int>(px4_msgs::msg::VehicleCommand::COMPONENT_MODE_EXECUTOR_START) + id();
  behavior_executor_.getGlobalBlackboardPtr()->set(AUTO_APMS_PX4_SOURCE_COMPONENT_GLOBAL_KEY, source_component);

  // Hand the pre-built tree to the executor. This only (re)creates the lightweight execution timer and starts ticking;
  // the expensive tree construction already happened in prepareTree.
  const auto executor_params = behavior_executor_.getExecutorParameters();
  try {
    behavior_executor_.startExecution(
      std::move(prepared_tree_ptr_), executor_params.tick_rate, executor_params.groot2_port);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_.get_logger(), "Failed to start behavior: %s", e.what());
    onExecutionResult(ExecutionResult::ERROR);
  }
}

void BehaviorModeExecutor::onDeactivate(DeactivateReason reason)
{
  const char * reason_str = reason == DeactivateReason::FailsafeActivated ? "failsafe activated" : "other";
  RCLCPP_INFO(node_.get_logger(), "Behavior executor deactivating (reason: %s)", reason_str);

  // The FMU has already taken over (pilot override or failsafe). Halting the behavior is cleanup, not safety.
  if (behavior_executor_.isBusy()) {
    RCLCPP_INFO(node_.get_logger(), "Behavior is still running. Terminating it now...");
    behavior_executor_.setControlCommand(auto_apms_behavior_tree::TreeExecutorBase::ControlCommand::TERMINATE);
  }

  if (config_.defer_failsafes) {
    deferFailsafesSync(false);
  }
}

std::unique_ptr<auto_apms_behavior_tree::Tree> BehaviorModeExecutor::buildTree(const BehaviorSpec & spec)
{
  // Flag that a mode executor is in charge so SendVehicleCommand routes commands through the mode-executor command
  // topic. This must be set before the tree is built below, because the command nodes latch their command topic from
  // this flag as they construct. The behavior executor is dedicated to this mode executor, so the flag stays true for
  // the lifetime of its global blackboard.
  behavior_executor_.getGlobalBlackboardPtr()->set(AUTO_APMS_PX4_MODE_EXECUTOR_ACTIVE_GLOBAL_KEY, true);

  auto_apms_behavior_tree::core::NodeManifest node_manifest;
  if (!spec.node_manifest.empty()) {
    node_manifest = auto_apms_behavior_tree::core::NodeManifest::decode(spec.node_manifest);
  }

  // Build the tree now: this is the expensive step because it instantiates the ROS 2 waitables of the behavior tree
  // nodes. The tree is kept detached until the next activation hands it to the executor. Its blackboard is rooted at
  // the executor's global blackboard so that '@'-prefixed entries resolve at runtime (mirroring what
  // TreeExecutorBase::startExecution does for the TreeConstructor overloads).
  const auto_apms_behavior_tree::TreeConstructor make_tree =
    behavior_executor_.makeTreeConstructor(spec.build_request, spec.entry_point, node_manifest);
  const auto_apms_behavior_tree::TreeBlackboardSharedPtr main_tree_bb_ptr =
    auto_apms_behavior_tree::TreeBlackboard::create(behavior_executor_.getGlobalBlackboardPtr());
  return std::make_unique<auto_apms_behavior_tree::Tree>(make_tree(main_tree_bb_ptr));
}

void BehaviorModeExecutor::prepareTree()
{
  prepared_tree_ptr_ = buildTree(current_spec_);
  RCLCPP_INFO(
    node_.get_logger(), "Behavior tree '%s' built and ready for activation", current_spec_.build_request.c_str());
}

bool BehaviorModeExecutor::setBehavior(const BehaviorSpec & candidate, std::string & message)
{
  // Verify the candidate by building its tree. This validates the tree structure and instantiates its nodes, so a
  // malformed or unresolvable behavior throws here and the currently prepared tree and spec are left untouched.
  std::unique_ptr<auto_apms_behavior_tree::Tree> candidate_tree;
  try {
    candidate_tree = buildTree(candidate);
  } catch (const std::exception & e) {
    message = e.what();
    RCLCPP_WARN(node_.get_logger(), "Rejecting behavior '%s': %s", candidate.build_request.c_str(), message.c_str());
    return false;
  }

  // Latch the verified tree and spec atomically: from here on this is what the next activation runs.
  prepared_tree_ptr_ = std::move(candidate_tree);
  current_spec_ = candidate;
  message = "Behavior '" + candidate.build_request + "' verified and set";
  RCLCPP_INFO(node_.get_logger(), "%s", message.c_str());
  return true;
}

bool BehaviorModeExecutor::requestAndVerifyActivation(std::chrono::milliseconds timeout)
{
  RCLCPP_INFO(node_.get_logger(), "Requesting activation of owned mode (nav_state %d)", owned_mode_.id());
  return vehicle_command_client_.syncActivateFlightModeAndWait(&owned_mode_, timeout);
}

void BehaviorModeExecutor::performReaction(CompletionReaction reaction, ExecutionResult result)
{
  px4_ros2::Result px4_result = px4_ros2::Result::Success;
  switch (result) {
    case ExecutionResult::TREE_SUCCEEDED:
      px4_result = px4_ros2::Result::Success;
      break;
    case ExecutionResult::TERMINATED_PREMATURELY:
      px4_result = px4_ros2::Result::Deactivated;
      break;
    case ExecutionResult::TREE_FAILED:
    case ExecutionResult::ERROR:
    default:
      px4_result = px4_ros2::Result::ModeFailureOther;
      break;
  }

  const auto log_done = [this](px4_ros2::Result r) {
    RCLCPP_INFO(node_.get_logger(), "Completion reaction finished (%s)", px4_ros2::resultToString(r));
  };

  switch (reaction) {
    case CompletionReaction::HOLD:
      RCLCPP_INFO(node_.get_logger(), "Completion reaction: HOLD");
      scheduleLoiter();
      break;
    case CompletionReaction::RTL:
      RCLCPP_INFO(node_.get_logger(), "Completion reaction: RTL");
      rtl(log_done);
      break;
    case CompletionReaction::LAND:
      RCLCPP_INFO(node_.get_logger(), "Completion reaction: LAND");
      land(log_done);
      break;
    case CompletionReaction::DISARM:
      RCLCPP_INFO(node_.get_logger(), "Completion reaction: DISARM");
      disarm(log_done);
      break;
    case CompletionReaction::COMPLETE:
      RCLCPP_INFO(node_.get_logger(), "Completion reaction: COMPLETE (reporting owned mode completion)");
      owned_mode_.finish(px4_result);
      break;
    case CompletionReaction::NONE:
      RCLCPP_INFO(node_.get_logger(), "Completion reaction: NONE");
      break;
  }
}

void BehaviorModeExecutor::scheduleLoiter()
{
  scheduleMode(px4_ros2::ModeBase::kModeIDLoiter, [](px4_ros2::Result) {
    // Loiter mode has no completion signal, so callback is no-op.
  });
}

// #####################################################################################################################
// ###############################              BehaviorModeExecutorNode ###################################
// #####################################################################################################################

px4_ros2::ModeExecutorBase::Settings::Activation BehaviorModeExecutorNode::activationFromString(const std::string & str)
{
  using Activation = px4_ros2::ModeExecutorBase::Settings::Activation;
  if (str == "armed") return Activation::ActivateOnlyWhenArmed;
  if (str == "always") return Activation::ActivateAlways;
  if (str == "immediately") return Activation::ActivateImmediately;
  throw std::invalid_argument("Invalid activation '" + str + "' (expected one of: armed, always, immediately)");
}

BehaviorModeExecutorNode::BehaviorModeExecutorNode(const rclcpp::NodeOptions & options)
: GenericTreeExecutorNode(
    "behavior_mode_executor",
    // Disable declaring parameters from overrides for the scripting-enum and blackboard groups (runtime/dynamic use is
    // kept). Without this the base would set automatically_declare_parameters_from_overrides(true), which declares the
    // launch-provided `behavior.*` parameters as writable before generate_parameter_library runs - making it skip its
    // `read_only` descriptor. With auto-declaration off, generate_parameter_library declares `behavior.*` itself (read
    // only) while still picking up the launch-provided initial values. All other launch parameters are explicitly
    // declared, so none rely on override auto-declaration.
    auto_apms_behavior_tree::TreeExecutorNodeOptions(options)
      .enableStrictUnkownParameterRemoval(false)
      .enableScriptingEnumParameters(false, true)
      .enableGlobalBlackboardParameters(false, true)),
  registration_handler_(getNodePtr()),
  start_action_context_(logger_)
{
  // Declares all behavior-specific parameters via generate_parameter_library. The listener is kept alive for the
  // lifetime of the executor (captured by the config provider below) so its parameter-validation callback stays
  // registered and get_params() keeps reflecting runtime changes. The behavior parameters (`behavior.*`) are read
  // only: they only seed the initial behavior below. The behavior is changed at runtime exclusively through the
  // `set_behavior` service and the `StartTreeExecutor` action, which verify a candidate before latching it. Only the
  // reaction parameters remain writable and are re-read on each activation via the config provider.
  const auto param_listener_ptr = std::make_shared<behavior_mode_executor_params::ParamListener>(getNodePtr());
  const behavior_mode_executor_params::Params params = param_listener_ptr->get_params();

  // Builds a fresh Config (completion reactions and failsafe deferral only) from the current parameter values. Invoked
  // by the executor on every activation so runtime parameter changes take effect. The reaction strings are constrained
  // to valid values by the parameter validation (one_of<>), so reactionFromString never sees an invalid value here.
  auto config_provider = [param_listener_ptr]() -> BehaviorModeExecutor::Config {
    const behavior_mode_executor_params::Params p = param_listener_ptr->get_params();
    BehaviorModeExecutor::Config config;
    config.on_completion = BehaviorModeExecutor::reactionFromString(p.on_completion);
    config.on_failure = BehaviorModeExecutor::reactionFromString(p.on_failure);
    config.defer_failsafes = p.defer_failsafes;
    return config;
  };

  const px4_ros2::ModeExecutorBase::Settings settings{activationFromString(params.activation)};

  owned_mode_ptr_ = std::make_unique<BehaviorOwnedMode>(*getNodePtr(), px4_ros2::ModeBase::Settings{params.mode_name});

  BehaviorSpec initial_spec;
  initial_spec.build_request = params.behavior.build_request;
  initial_spec.build_handler =
    current_build_handler_name_;  // Handler loaded by the base from the 'build_handler' param.
  initial_spec.entry_point = params.behavior.entry_point;
  initial_spec.node_manifest = params.behavior.node_manifest;

  mode_executor_ptr_ = std::make_unique<BehaviorModeExecutor>(
    *owned_mode_ptr_, settings, std::move(config_provider), std::move(initial_spec), *this);

  // Build the initial behavior tree up front so the expensive tree construction (instantiating the behavior tree
  // nodes' ROS 2 waitables) happens here instead of on the activation path. If building fails, the exception
  // propagates and the mode is never registered with the FMU.
  mode_executor_ptr_->prepareTree();

  // Wait for the FMU and register the executor together with its owned mode
  registration_handler_.registerMode(*mode_executor_ptr_, params.mode_name);

  // Interfaces for changing the behavior at runtime. Both verify a candidate behavior (by building its tree) before
  // latching it; the action additionally triggers execution by requesting activation of the owned mode.
  set_behavior_service_ptr_ = getNodePtr()->create_service<SetBehaviorSrv>(
    "~/set_behavior",
    std::bind(&BehaviorModeExecutorNode::handleSetBehavior, this, std::placeholders::_1, std::placeholders::_2));

  start_action_ptr_ = rclcpp_action::create_server<StartAction>(
    getNodePtr(), "~/start",
    std::bind(&BehaviorModeExecutorNode::handleStartGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&BehaviorModeExecutorNode::handleStartCancel, this, std::placeholders::_1),
    std::bind(&BehaviorModeExecutorNode::handleStartAccept, this, std::placeholders::_1));
}

bool BehaviorModeExecutorNode::applyBehavior(BehaviorSpec candidate, std::string & message)
{
  // Apply the requested build handler (mirroring the standard executor's goal handling), remembering the previous one
  // so it can be restored if the subsequent build fails.
  const std::string previous_handler = current_build_handler_name_;
  bool switched = false;
  if (!candidate.build_handler.empty() && candidate.build_handler != current_build_handler_name_) {
    if (!getExecutorParameters().allow_other_build_handlers) {
      message = "Build handler '" + candidate.build_handler +
                "' rejected: the 'Allow other build handlers' option is "
                "disabled (current: '" +
                current_build_handler_name_ + "')";
      RCLCPP_WARN(logger_, "%s", message.c_str());
      return false;
    }
    try {
      loadBuildHandler(candidate.build_handler);
      switched = true;
    } catch (const std::exception & e) {
      message = std::string("Failed to load build handler '") + candidate.build_handler + "': " + e.what();
      RCLCPP_WARN(logger_, "%s", message.c_str());
      return false;
    }
  }

  // Record the effective handler so the latched spec reflects reality even when the request kept the current one.
  candidate.build_handler = current_build_handler_name_;

  if (mode_executor_ptr_->setBehavior(candidate, message)) return true;

  // Verification failed: undo the handler switch so executor state is left untouched.
  if (switched) {
    try {
      loadBuildHandler(previous_handler);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to restore previous build handler '%s': %s", previous_handler.c_str(), e.what());
    }
  }
  return false;
}

void BehaviorModeExecutorNode::handleSetBehavior(
  const std::shared_ptr<SetBehaviorSrv::Request> request, std::shared_ptr<SetBehaviorSrv::Response> response)
{
  // Changing the behavior rebuilds the prepared tree, which must not race a running behavior (the prepared tree is
  // moved into the executor while it runs and rebuilt on termination).
  if (isBusy() || mode_executor_ptr_->isInCharge()) {
    response->success = false;
    response->message = "Cannot change behavior while a behavior is running or the executor is in charge";
    RCLCPP_WARN(logger_, "%s", response->message.c_str());
    return;
  }

  BehaviorSpec candidate;
  candidate.build_request = request->build_request;
  candidate.build_handler = request->build_handler;
  candidate.entry_point = request->entry_point;
  candidate.node_manifest = request->node_manifest;

  std::string message;
  response->success = applyBehavior(candidate, message);
  response->message = message;
}

rclcpp_action::GoalResponse BehaviorModeExecutorNode::handleStartGoal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const StartAction::Goal> goal_ptr)
{
  if (isBusy() || mode_executor_ptr_->isInCharge()) {
    RCLCPP_WARN(
      logger_, "Goal %s REJECTED: a behavior is already running or the executor is in charge",
      rclcpp_action::to_string(uuid).c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Apply the requested build handler (if any) and verify+latch the behavior; reject the goal if it does not build.
  BehaviorSpec candidate;
  candidate.build_request = goal_ptr->build_request;
  candidate.build_handler = goal_ptr->build_handler;
  candidate.entry_point = goal_ptr->entry_point;
  candidate.node_manifest = goal_ptr->node_manifest;

  std::string message;
  if (!applyBehavior(candidate, message)) {
    RCLCPP_WARN(logger_, "Goal %s REJECTED: %s", rclcpp_action::to_string(uuid).c_str(), message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  pending_attach_[uuid] = goal_ptr->attach;
  pending_clear_blackboard_[uuid] = goal_ptr->clear_blackboard;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BehaviorModeExecutorNode::handleStartCancel(
  std::shared_ptr<StartGoalHandle> /*goal_handle_ptr*/)
{
  setControlCommand(ControlCommand::TERMINATE);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BehaviorModeExecutorNode::handleStartAccept(std::shared_ptr<StartGoalHandle> goal_handle_ptr)
{
  const rclcpp_action::GoalUUID uuid = goal_handle_ptr->get_goal_id();
  const bool attach = pending_attach_.extract(uuid).mapped();
  const bool clear_blackboard = pending_clear_blackboard_.extract(uuid).mapped();

  if (clear_blackboard) clearGlobalBlackboard();

  // Request activation of the owned mode and wait until it is confirmed active (the executor is put in charge and the
  // prepared tree starts running). This uses the client's own wait set, so it does not depend on this node being spun
  // by an external executor.
  auto result_ptr = std::make_shared<StartAction::Result>();
  if (!mode_executor_ptr_->requestAndVerifyActivation()) {
    result_ptr->tree_result = StartAction::Result::TREE_RESULT_NOT_SET;
    result_ptr->message = "Failed to activate the owned mode (not put in charge within the timeout)";
    RCLCPP_ERROR(logger_, "%s", result_ptr->message.c_str());
    goal_handle_ptr->abort(result_ptr);
    return;
  }

  if (!attach) {
    // Detached: succeed as soon as activation is verified; the behavior keeps running under the mode executor.
    result_ptr->tree_result = StartAction::Result::TREE_RESULT_NOT_SET;
    result_ptr->message = "Owned mode activated; behavior running detached";
    goal_handle_ptr->succeed(result_ptr);
    return;
  }

  // Attached: keep the goal open and resolve it in onTermination with the tree result.
  start_action_context_.setUp(goal_handle_ptr);
  RCLCPP_INFO(logger_, "Owned mode activated; tracking behavior execution for attached goal.");
}

void BehaviorModeExecutorNode::onTermination(const ExecutionResult & result)
{
  mode_executor_ptr_->onExecutionResult(result);

  // Resolve an attached StartTreeExecutor goal, if one is open. Behaviors started from a GCS/RC (no open goal) leave
  // the context invalid and are unaffected.
  if (!start_action_context_.isValid()) return;

  const std::shared_ptr<StartAction::Result> result_ptr = start_action_context_.getResultPtr();
  result_ptr->terminated_tree_identity = getTreeName();
  switch (result) {
    case ExecutionResult::TREE_SUCCEEDED:
      result_ptr->tree_result = StartAction::Result::TREE_RESULT_SUCCESS;
      result_ptr->message = "Tree execution finished with status SUCCESS";
      start_action_context_.succeed();
      break;
    case ExecutionResult::TREE_FAILED:
      result_ptr->tree_result = StartAction::Result::TREE_RESULT_FAILURE;
      result_ptr->message = "Tree execution finished with status FAILURE";
      start_action_context_.abort();
      break;
    case ExecutionResult::TERMINATED_PREMATURELY:
      result_ptr->tree_result = StartAction::Result::TREE_RESULT_NOT_SET;
      if (start_action_context_.getGoalHandlePtr()->is_canceling()) {
        result_ptr->message = "Tree execution canceled successfully";
        start_action_context_.cancel();
      } else {
        result_ptr->message = "Tree execution terminated prematurely";
        start_action_context_.abort();
      }
      break;
    case ExecutionResult::ERROR:
    default:
      result_ptr->tree_result = StartAction::Result::TREE_RESULT_NOT_SET;
      result_ptr->message = "An unexpected error occurred during tree execution";
      start_action_context_.abort();
      break;
  }
  start_action_context_.invalidate();
}

}  // namespace auto_apms_px4

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_px4::BehaviorModeExecutorNode)
