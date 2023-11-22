import sys
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def get_composable_node():
    return ComposableNode(
        package="laser_copilot",
        plugin="laser_copilot::safe_fly_controller",
        name="controller",
        parameters=[{
            "use_sim_time": True,
        }],
        remappings=[("/sub/goal", "/move_base_simple/goal")],
    )


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name="laser_copilot_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[get_composable_node()],
        output="screen",
    )

    return launch.LaunchDescription([container])
