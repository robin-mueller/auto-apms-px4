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
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="", description="Namespace for the nodes"),
            DeclareLaunchArgument(
                "behavior",
                description="Resource identity of the behavior to execute",
            ),
            DeclareLaunchArgument(
                "activation",
                default_value="armed",
                description="When the executor may be activated by the FMU",
                choices=["armed", "always", "immediately"],
            ),
            DeclareLaunchArgument(
                "mode_name",
                default_value="AutoAPMS Behavior",
                description="Registered name of the owned PX4 mode",
            ),
            DeclareLaunchArgument(
                "on_completion",
                default_value="hold",
                description="Reaction after the behavior succeeds",
                choices=["hold", "rtl", "land", "disarm", "complete", "none"],
            ),
            DeclareLaunchArgument(
                "on_failure",
                default_value="rtl",
                description="Reaction after the behavior fails",
                choices=["hold", "rtl", "land", "disarm", "complete", "none"],
            ),
            Node(
                package="auto_apms_px4",
                executable="behavior_mode_executor",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "activation": LaunchConfiguration("activation"),
                        "mode_name": LaunchConfiguration("mode_name"),
                        "on_completion": LaunchConfiguration("on_completion"),
                        "on_failure": LaunchConfiguration("on_failure"),
                        "behavior.build_request": LaunchConfiguration("behavior"),
                        "build_handler": "auto_apms_behavior_tree::TreeFromResourceBuildHandler",
                    }
                ],
            ),
        ]
    )
