from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    description_pkg_share = FindPackageShare("za6_description")
    this_package_share = FindPackageShare("za6_control_sandbox")

    default_rviz_config = PathJoinSubstitution(
        [this_package_share, "config", "za6_control_sandbox.rviz"]
    )

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Should mock (simulated) hardware be used?",
        )
    )
    declared_arguments.append(
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
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller",
            default_value="pose_controller",
            description="Which controller should be started?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            description="Path to the initial positions configuration",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("za6_control_sandbox"),
                    "config",
                    "initial_positions.yaml",
                ]
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_frequency",
            default_value="100.0",
            description="The rate in (Hz) of the reobot state publisher",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hal_debug_output",
            default_value="true",
            description="Output HAL debug messages to screen (console)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hal_debug_level",
            default_value="1",
            description="Set HAL debug level, 0-5",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Should RViz be launched",
        )
    )

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    ros2_control_name = PythonExpression(
        [
            "'mock_hardware' if '",
            use_mock_hardware,
            "' == 'true' else ",
            "'hal_hw_interface'",
        ]
    )
    ros2_control_plugin = PythonExpression(
        [
            "'mock_components/GenericSystem' if '",
            use_mock_hardware,
            "' == 'true' else ",
            "'hal_system_interface/HalSystemInterface'",
        ]
    )
    controller = LaunchConfiguration("controller")
    initial_positions_file = LaunchConfiguration("initial_positions_file")

    hal_debug_output = LaunchConfiguration("hal_debug_output")
    hal_debug_level = LaunchConfiguration("hal_debug_level")
    publish_frequency = LaunchConfiguration("publish_frequency")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [description_pkg_share, "launch", "robot_description.launch.py"]
            )
        ),
        launch_arguments={
            "ros2_control_name": ros2_control_name,
            "ros2_control_plugin": ros2_control_plugin,
            "initial_positions_file": initial_positions_file,
        }.items(),
    )
    robot_description_content = LaunchConfiguration("robot_description_content")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    robot_description_content, value_type=str
                )
            },
            {"publish_frequency": publish_frequency},  # Hz
        ],
    )

    # Hardware bringup
    hal_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [this_package_share, "launch", "hal_hardware.launch.py"]
            )
        ),
        launch_arguments={
            "hal_debug_output": hal_debug_output,
            "hal_debug_level": hal_debug_level,
        }.items(),
        condition=UnlessCondition(use_mock_hardware),
    )

    mock_hardware_node = Node(
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
        condition=IfCondition(use_mock_hardware)
    )

    # Controllers bringup
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "--controller-manager", "controller_manager"],
    )

    # Rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_config],
        condition=IfCondition(use_rviz),
    )

    launch_description = [
        hal_hardware_launch,
        robot_description_launch,
        robot_state_publisher_node,
        mock_hardware_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
    ]
    return LaunchDescription(declared_arguments + launch_description)
