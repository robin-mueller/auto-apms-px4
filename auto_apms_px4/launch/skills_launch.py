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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ALL_SKILL_NAMES = [
    "auto_apms_px4::ArmDisarmSkill",
    "auto_apms_px4::EnableHoldSkill",
    "auto_apms_px4::GoToGlobalSkill",
    "auto_apms_px4::GoToLocalSkill",
    "auto_apms_px4::LandSkill",
    "auto_apms_px4::TakeoffSkill",
    "auto_apms_px4::RTLSkill",
    # "auto_apms_px4::MissionSkill",
    "auto_apms_px4::SetActuatorsSkill",
]


def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the nodes",
    )
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            ns_arg,
            ComposableNodeContainer(
                name="skill_container",
                namespace=namespace,
                exec_name="skill_container",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(package="auto_apms_px4", plugin=name, namespace=namespace)
                    for name in ALL_SKILL_NAMES
                ],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
