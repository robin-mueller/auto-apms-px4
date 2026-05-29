# Copyright 2024 Robin Müller
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ALL_SKILLS = {
    "arm_disarm_node": "auto_apms_px4::ArmDisarmSkill",
    "enable_hold_node": "auto_apms_px4::EnableHoldSkill",
    "goto_global_node": "auto_apms_px4::GoToGlobalSkill",
    "goto_local_node": "auto_apms_px4::GoToLocalSkill",
    "land_node": "auto_apms_px4::LandSkill",
    "takeoff_node": "auto_apms_px4::TakeoffSkill",
    "rtl_node": "auto_apms_px4::RTLSkill",
    # "mission_node": "auto_apms_px4::MissionSkill",
    "set_actuators_node": "auto_apms_px4::SetActuatorsSkill",
}


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)

    ros_arguments = [arg for name in ALL_SKILLS for arg in ["--log-level", f"{namespace}.{name}:={log_level}"]]

    return [
        ComposableNodeContainer(
            name="skill_container",
            namespace=namespace,
            exec_name="skill_container",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(package="auto_apms_px4", plugin=plugin, name=name, namespace=namespace)
                for name, plugin in ALL_SKILLS.items()
            ],
            ros_arguments=ros_arguments,
            output="screen",
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for the nodes",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level for all skill nodes",
                choices=["debug", "info", "warn", "error", "fatal"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
