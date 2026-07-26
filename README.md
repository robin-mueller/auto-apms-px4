<p align="center">
  <img width="30%" src="https://github.com/PX4/PX4-graphics/blob/master/PX4_Logo_Black_RGB.png?raw=true">
  <img width="50%" src="https://autoapms.github.io/auto-apms-guide/logo/autoapms_logo.svg">
</p>
<div align="center">

<a href="https://autoapms.github.io/auto-apms-guide/concept/px4-integration">![Docs](https://img.shields.io/website?url=https%3A%2F%2Fautoapms.github.io%2Fauto-apms-guide&label=🎓Documentation)</a>
<a href="https://doi.org/10.5281/zenodo.14790307">![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.14790307.svg)</a>

</div>

# auto_apms_px4

**A modular Mission Architecture for PX4 Autopilot using ROS 2**

This projects integrates the [PX4 Autopilot](https://px4.io/) with [AutoAPMS](https://github.com/AutoAPMS/auto-apms). It offers an architecture and tooling to allow a companion computer register custom PX4 modes and orchestrate everything from a behavior tree using ROS 2.

It builds on the [PX4/ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface.html)
(`px4_ros2_cpp`) and communicates with the FMU over the standard uORB/`px4_msgs` topics.

> [!NOTE]
> This project follows the same branch naming convention as the rest of the PX4 ecosystem (e.g. "release/XX"). To ensure compatibility, make sure to checkout the same branch for [px4_msgs](https://github.com/PX4/px4_msgs) and [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib)
>
> The dependency versions are pinned for CI inside [dependencies.repos](./dependencies.repos) and should be used for local development as well.

## What's inside?

- **Custom mode infrastructure**: base classes and composable helpers for writing, registering and deploying custom PX4 modes. See [Writing custom modes](#writing-custom-px4-modes) below.
- **Skills**: ready-to-use `rclcpp_components` nodes that wrap the common standard PX4 modes as ROS 2 action servers. Bring them up with [`launch/skills_launch.py`](auto_apms_px4/launch/skills_launch.py).
- **Behavior tree nodes**: plugins for orchestrating PX4 from a behavior tree, exported under the node manifest alias `auto_apms_px4::behavior_tree_nodes`. The full port-level reference is published in the guide: [Behavior Tree Nodes › auto_apms_px4](https://autoapms.github.io/auto-apms-guide/reference/behavior-tree-nodes#overview-auto-apms-px4).
- **Behavior mode executor**: a PX4 mode executor that runs an AutoAPMS behavior in-process when put in charge by the FMU. Configured entirely via ROS 2 parameters; see [`config/behavior_mode_executor_params.yaml`](auto_apms_px4/config/behavior_mode_executor_params.yaml) and [`launch/behavior_mode_executor_launch.py`](auto_apms_px4/launch/behavior_mode_executor_launch.py).

## Writing custom PX4 modes

A custom mode is a subclass of `px4_ros2::ModeBase`. This package offers two composable helpers depending on how the
mode is meant to be activated. Both wait for the FMU, register the mode, and announce the mode's
dynamically assigned `nav_state` on the `registered_modes` topic so behaviors can discover it by name.

| Helper | Base class for your mode | How the mode is activated | Typical use |
| --- | --- | --- | --- |
| `ModeRegistrationFactory<ModeT>` | `px4_ros2::ModeBase` | By switching to its `nav_state` (e.g. from a behavior tree via `SendCmdSetNavState`/`GetModeNavState`, an RC switch or the GCS) | A self-contained mode that runs autonomously once selected |
| `ModeProxyActionFactory<ActionT, ModeT>` | `auto_apms_px4::ActionDrivenMode<ActionT>` | By sending a goal to the ROS 2 action server the factory creates | A mode driven by a request with a goal/feedback/result (a "skill") |

Under the hood both compose a
[`ModeRegistrationHandler`](auto_apms_px4/include/auto_apms_px4/mode_registration.hpp), which owns the registration sequence (wait
for FMU → register → announce). You can also use `ModeRegistrationHandler` in your custom class directly if you want to manage the mode's lifetime
yourself.

### 1. A plain mode — `ModeRegistrationFactory`

Write the mode as a regular `px4_ros2::ModeBase` subclass constructible from `(rclcpp::Node &,
px4_ros2::ModeBase::Settings)`:

```cpp
#include "auto_apms_px4/mode_registration.hpp"
#include "px4_ros2/components/mode.hpp"

namespace my_pkg
{

class MyMode : public px4_ros2::ModeBase
{
public:
  MyMode(rclcpp::Node & node, const px4_ros2::ModeBase::Settings & settings) : px4_ros2::ModeBase(node, settings)
  {
    // Construct your px4_ros2 setpoint/odometry helpers here.
  }

  void onActivate() override {}
  void onDeactivate() override {}
  void updateSetpoint(float dt_s) override { /* publish your setpoint */ }
};

// Deploy as an rclcpp_components node: forward the registered mode name to the factory.
class MyModeComponent : public auto_apms_px4::ModeRegistrationFactory<MyMode>
{
public:
  explicit MyModeComponent(const rclcpp::NodeOptions & options) : ModeRegistrationFactory("my_mode", options) {}
};

}  // namespace my_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_pkg::MyModeComponent)
```

Once registered, the mode is announced on `registered_modes`. A behavior tree can then resolve its `nav_state` with
`GetModeNavState` and switch to it with `SendCmdSetNavState` (see the `SwitchToNamedModeAndWait` tree in
`behavior/vehicle_command.xml`).

### 2. An action-driven mode ("skill") — `ModeProxyActionFactory`

Define a ROS 2 action for the goal/feedback/result, then write the mode as an `auto_apms_px4::ActionDrivenMode<ActionT>`
subclass (which exposes the `onActivateWithGoal` / `updateSetpointWithGoal` / ... hooks). The factory pairs the mode
with a `ModeProxyAction` action server so the mode can be triggered by sending an action goal — for example from a
`RosActionNode` in a behavior tree.

```cpp
#include "auto_apms_px4/mode.hpp"
#include "auto_apms_px4/mode_proxy_action.hpp"
#include "my_pkg_interfaces/action/my_skill.hpp"

namespace my_pkg
{

using MySkillAction = my_pkg_interfaces::action::MySkill;

class MySkillMode : public auto_apms_px4::ActionDrivenMode<MySkillAction>
{
public:
  MySkillMode(
    rclcpp::Node & node, px4_ros2::ModeBase::Settings settings,
    std::shared_ptr<ActionContextType> action_context_ptr)
  : ActionDrivenMode{node, settings, action_context_ptr}
  {
    // Construct setpoint helpers here.
  }

private:
  void updateSetpointWithGoal(
    float dt_s, std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Feedback> feedback_ptr,
    std::shared_ptr<Result> result_ptr) final
  {
    // Drive the vehicle towards the goal, then call completed(px4_ros2::Result::Success) when done.
  }
};

class MySkill : public auto_apms_px4::ModeProxyActionFactory<MySkillAction, MySkillMode>
{
public:
  explicit MySkill(const rclcpp::NodeOptions & options) : ModeProxyActionFactory{"my_skill", options} {}
};

}  // namespace my_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_pkg::MySkill)
```

The skill sources under [`src/skill/`](auto_apms_px4/src/skill) are complete, working examples of this pattern.

## The behavior mode executor

The [`BehaviorModeExecutor`](auto_apms_px4/include/auto_apms_px4/behavior_mode_executor.hpp) is a
`px4_ros2::ModeExecutorBase` that owns a placeholder PX4 mode and runs an AutoAPMS behavior *in-process* whenever the
FMU puts it in charge (the mode is selected via RC switch, GCS or `immediately`). It is configured entirely through
ROS 2 parameters (see [`config/behavior_mode_executor_params.yaml`](auto_apms_px4/config/behavior_mode_executor_params.yaml)):
which behavior to build, when it may be activated, and how to react once the behavior succeeds or fails
(`hold`/`rtl`/`land`/`disarm`/`complete`/`none`). All parameters except `activation` and `mode_name` are dynamic and
may be changed at runtime (e.g. `ros2 param set`); the new values take effect the next time the executor is put in
charge. `activation` and `mode_name` are read only because the owned mode is registered with the FMU once at startup.

Unlike a plain custom mode, a mode executor is *not* announced on `registered_modes` — it cannot be targeted by
`SwitchMode` from another behavior tree. It is the mechanism for triggering automation *from PX4 itself*.

### Deploying a behavior

The bundled example [`behavior/example.xml`](auto_apms_px4/behavior/example.xml) takes off and immediately lands
again. It reuses the `DoTakeoff` and `DoLanding` subtrees from
[`behavior/vehicle_command.xml`](auto_apms_px4/behavior/vehicle_command.xml) by pulling them in with `<include>`
elements:

```xml
<include autoapms="auto_apms_px4::vehicle_command::DoTakeoff"/>
<include autoapms="auto_apms_px4::vehicle_command::DoLanding"/>
```

An AutoAPMS `<include>` supports three ways to locate the included trees:

- `autoapms="<package>::<file_stem>::<tree_name>"` — resolve a registered tree resource by its identity (no knowledge
  of any install location needed). This **cherry-picks the individual tree** named by the identity plus the trees it
  transitively depends on via `<SubTree>` (so `DoTakeoff` also brings `SetModeTakeoff` and `SwitchModeAndWait`), rather
  than the whole file. A helper shared between two includes (here `SwitchModeAndWait`) is merged only once.
- `path="..." ros_pkg="<package>"` — a path relative to the package's share directory.
- `path="..."` — a plain filesystem path (fragile: relative paths resolve against the current working directory).

Behaviors are registered as tree resources via `auto_apms_behavior_tree_register_trees()` in CMake. The resulting resource identity is `<package>::<file_stem>::<tree_name>`, so this example is addressed as `auto_apms_px4::example::TakeoffAndLand`.

**How to launch the behavior mode executor with the example behavior:**

The `behavior` argument takes the resource identity:

```bash
ros2 launch auto_apms_px4 behavior_mode_executor_launch.py \
  behavior:=auto_apms_px4::example::TakeoffAndLand
```

Other arguments (inspect using the `-s` argument) map directly to the executor's parameters. Once the owned mode is selected in the GCS (or with `activation:=immediately`), the executor takes charge and runs the behavior.

## Further reading

- [PX4 integration concept](https://autoapms.github.io/auto-apms-guide/concept/px4-integration)
- [Behavior tree node reference](https://autoapms.github.io/auto-apms-guide/reference/behavior-tree-nodes#overview-auto-apms-px4)
