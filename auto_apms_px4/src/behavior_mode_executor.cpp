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
  std::function<Config()> config_provider, auto_apms_behavior_tree::GenericTreeExecutorNode & engine)
: px4_ros2::ModeExecutorBase(settings, owned_mode),
  node_(owned_mode.node()),
  owned_mode_(owned_mode),
  behavior_executor_(engine),
  config_provider_(std::move(config_provider)),
  config_(config_provider_())
{
}

void BehaviorModeExecutor::onExecutionResult(ExecutionResult result)
{
  RCLCPP_INFO(node_.get_logger(), "Behavior execution result: %s.", auto_apms_behavior_tree::toStr(result).c_str());

  // If we lost charge in the meantime (pilot/failsafe), don't fight the FMU.
  if (!isInCharge()) {
    RCLCPP_INFO(node_.get_logger(), "No longer in charge. Skipping completion reaction");
    return;
  }

  const CompletionReaction reaction =
    result == ExecutionResult::TREE_SUCCEEDED ? config_.on_completion : config_.on_failure;
  performReaction(reaction, result);
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
  // Refresh the configuration from the current parameter values so runtime changes (e.g. via `ros2 param set`) take
  // effect on this activation. The registration-time parameters (activation policy, mode name) are latched when the
  // mode is registered with the FMU and are not part of this snapshot.
  config_ = config_provider_();

  if (config_.spec.build_request.empty()) {
    RCLCPP_ERROR(node_.get_logger(), "Cannot start behavior: parameter 'behavior.build_request' must not be empty");
    onExecutionResult(ExecutionResult::ERROR);
    return;
  }

  RCLCPP_INFO(
    node_.get_logger(), "Behavior executor put in charge. Starting behavior '%s'", config_.spec.build_request.c_str());

  if (config_.defer_failsafes) {
    if (deferFailsafesSync(true)) {
      RCLCPP_INFO(node_.get_logger(), "Failsafes are now being deferred while the behavior is running");
    } else {
      RCLCPP_WARN(node_.get_logger(), "Failed to enable failsafe deferral");
    }
  }

  // Publish the executor's VehicleCommand source component on the global blackboard so that ownership-aware behavior
  // tree nodes (e.g. SendCmdSetNavState) can attribute their commands to this executor. Stored as int to match the type
  // read by the SendCmdSetNavState node. The global key already carries the '@' prefix; on the global blackboard (which
  // is its own root) this resolves to the same entry the tree nodes read transitively.
  const int source_component = static_cast<int>(px4_msgs::msg::VehicleCommand::COMPONENT_MODE_EXECUTOR_START) + id();
  behavior_executor_.getGlobalBlackboardPtr()->set(AUTO_APMS_PX4_SOURCE_COMPONENT_GLOBAL_KEY, source_component);

  // Flag that a mode executor is in charge so SendVehicleCommand routes commands through the mode-executor command
  // topic. Set before the behavior tree is built (below), so the flag is already readable when its nodes construct.
  behavior_executor_.getGlobalBlackboardPtr()->set(AUTO_APMS_PX4_MODE_EXECUTOR_ACTIVE_GLOBAL_KEY, true);

  auto_apms_behavior_tree::core::NodeManifest node_manifest;
  if (!config_.spec.node_manifest.empty()) {
    node_manifest = auto_apms_behavior_tree::core::NodeManifest::decode(config_.spec.node_manifest);
  }

  try {
    behavior_executor_.startExecution(config_.spec.build_request, config_.spec.entry_point, node_manifest);
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
    auto_apms_behavior_tree::TreeExecutorNodeOptions(options).enableStrictUnkownParameterRemoval(false)),
  registration_handler_(getNodePtr())
{
  // Declares all behavior-specific parameters via generate_parameter_library. The listener is kept alive for the
  // lifetime of the executor (captured by the config provider below) so its parameter-validation callback stays
  // registered and get_params() keeps reflecting runtime changes. The behavior parameters are declared writable (not
  // read_only), so they can be updated at runtime; the new values are picked up on the next activation (see
  // BehaviorModeExecutor::onActivate). Only `activation` and `mode_name` are read only, because the owned mode is
  // registered with the FMU once, here in the constructor.
  const auto param_listener_ptr = std::make_shared<behavior_mode_executor_params::ParamListener>(getNodePtr());
  const behavior_mode_executor_params::Params params = param_listener_ptr->get_params();

  if (params.behavior.build_request.empty()) {
    throw std::invalid_argument("Parameter 'behavior.build_request' must not be empty.");
  }

  // Builds a fresh Config from the current parameter values. Invoked by the executor on every activation so runtime
  // parameter changes take effect. The reaction strings are constrained to valid values by the parameter validation
  // (one_of<>), so reactionFromString never sees an invalid value here.
  auto config_provider = [param_listener_ptr]() -> BehaviorModeExecutor::Config {
    const behavior_mode_executor_params::Params p = param_listener_ptr->get_params();
    BehaviorModeExecutor::Config config;
    config.spec.build_request = p.behavior.build_request;
    config.spec.entry_point = p.behavior.entry_point;
    config.spec.node_manifest = p.behavior.node_manifest;
    config.on_completion = BehaviorModeExecutor::reactionFromString(p.on_completion);
    config.on_failure = BehaviorModeExecutor::reactionFromString(p.on_failure);
    config.defer_failsafes = p.defer_failsafes;
    return config;
  };

  const px4_ros2::ModeExecutorBase::Settings settings{activationFromString(params.activation)};

  owned_mode_ptr_ = std::make_unique<BehaviorOwnedMode>(*getNodePtr(), px4_ros2::ModeBase::Settings{params.mode_name});

  executor_ptr_ = std::make_unique<BehaviorModeExecutor>(*owned_mode_ptr_, settings, std::move(config_provider), *this);

  // Wait for the FMU and register the executor together with its owned mode
  registration_handler_.registerMode(*executor_ptr_, params.mode_name);
}

void BehaviorModeExecutorNode::onTermination(const ExecutionResult & result)
{
  executor_ptr_->onExecutionResult(result);
}

}  // namespace auto_apms_px4

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_px4::BehaviorModeExecutorNode)
