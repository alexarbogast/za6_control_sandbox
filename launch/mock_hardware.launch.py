# Copyright 2026 Alex Arbogast
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    description_package_share = FindPackageShare("za6_description")

    prefix = LaunchConfiguration("prefix")
    robot_description_content = LaunchConfiguration("robot_description_content")

    description_launch_py = PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [
                description_package_share,
                "launch",
                "robot_description.launch.py",
            ]
        )
    )

    ros2_control_plugin = "mock_components/GenericSystem"

    launch_entities = [
        DeclareLaunchArgument(
            "description_package",
            default_value="za6_description",
            description="Description package with robot URDF/XACRO files",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="za6.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description=(
                "Prefix of joint names for multi-robot setup; "
                "if changed, controller configuration joint names "
                "must also be updated"
            ),
        ),
        DeclareLaunchArgument(
            "ros2_controllers_yaml",
            description="ROS2 controller manager configuration YAML.",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("za6_control_sandbox"),
                    "config",
                    "za6_controllers.yaml",
                ]
            ),
        ),
        IncludeLaunchDescription(
            description_launch_py,
            launch_arguments=dict(
                description_package=LaunchConfiguration("description_package"),
                description_file=LaunchConfiguration("description_file"),
                prefix=prefix,
                ros2_control_plugin=ros2_control_plugin,
            ).items(),
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                dict(
                    # Expanded robot description URDF
                    robot_description=ParameterValue(
                        robot_description_content, value_type=str
                    ),
                ),
                LaunchConfiguration("ros2_controllers_yaml"),
            ],
            output="screen",
        ),
    ]

    return LaunchDescription(launch_entities)
