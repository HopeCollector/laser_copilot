import sys
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def get_param_declaretions():
    return [
        DeclareLaunchArgument("file_path", description="ros2 bag file path to load")
    ]


def get_composable_node():
    return ComposableNode(
        package="laser_copilot",
        plugin="laser_copilot::traj_replayer",
        name="replayer",
        parameters=[
            {
                "use_sim_time": True,
                "topic": "/fmu/out/vehicle_odometry",
                "file_path": LaunchConfiguration("file_path"),
            }
        ],
        remappings=[
            ("pub/setpoint", "/move_base_simple/goal"),
            ("pub/path", "replayer/pub/path"),
        ],
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

    return launch.LaunchDescription(get_param_declaretions() + [container])
