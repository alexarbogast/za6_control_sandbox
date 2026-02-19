from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller",
            choices=["pose_controller", "as_nullspace_controller"],
            default_value="pose_controller",
            description="Which controller should be started?",
        )
    )
    controller = LaunchConfiguration("controller")

    control_demo_node = Node(
        package="za6_control_sandbox",
        executable="control_demo.py",
        name=f"za6_control_demo",
        output="screen",
        parameters=[{"controller": controller}],
    )

    return LaunchDescription(declared_arguments + [control_demo_node])
