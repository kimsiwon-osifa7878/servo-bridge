from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="COM3"),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("servo_bridge"), "config", "servos.yaml"]
                ),
            ),
            Node(
                package="servo_bridge",
                executable="bridge_node",
                name="servo_bridge",
                output="screen",
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                        "config_file": LaunchConfiguration("config_file"),
                    }
                ],
            ),
        ]
    )
