import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    default_config_file=os.path.join(
        get_package_share_directory("ros2socketcan_bridge"),
        "config",
        "ros2socketcan_bridge_parameter.yaml",
    )

    config_file = LaunchConfiguration('config_file', default=default_config_file)

    ros2socketcan_bridge_node = Node(
        package="ros2socketcan_bridge",
        name="ros2socketcan_bridge",
        executable="ros2socketcan_bridge",
        parameters=[config_file],
    )

    ld.add_action(ros2socketcan_bridge_node)
    return ld
