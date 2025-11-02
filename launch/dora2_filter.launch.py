from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                name="dora2_filter_node",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    PathJoinSubstitution(
                        [get_package_share_directory("dora2filter"), "config", "dora2_filter.yaml"]
                    )
                ],
            )
        ]
    )
