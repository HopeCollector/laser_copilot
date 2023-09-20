import sys
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare, PathJoinSubstitution


def get_cfg_file():
    return [
        PathJoinSubstitution([FindPackageShare("laser_copilot"), "launch", "obj_dist.yaml"])
    ]


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name="laser_copilot_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="laser_copilot",
                plugin="laser_copilot::obj_dist",
                name="objdist",
                parameters=get_cfg_file(),
                remappings=[
                    ("sub/lvx", "livox/lidar"),
                    ("out/laser_scan", "mavros/obstacle/send"),
                ],
            )
        ],
        output="screen",
    )

    return launch.LaunchDescription([container])
