"""Launch px4_bridge control node + odom bridge."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="px4_bridge",
                executable="bridge",
                name="px4_bridge",
                output="screen",
            ),
            Node(
                package="px4_bridge",
                executable="odom_bridge",
                name="odom_to_vehicle_odometry_bridge",
                output="screen",
            ),
        ]
    )
