import sys
import launch
from launch_ros.actions import LoadComposableNodes
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
        package="laser_copilot_applications",
        plugin="laser_copilot_applications::traj_replayer",
        name="replayer",
        parameters=[
            {
                "topic": "/fmu/out/vehicle_odometry",
                "file_path": LaunchConfiguration("file_path"),
            }
        ],
        remappings=[
            ("sub/odom", "/mavros/odometry/out"),
            ("pub/setpoint", "/move_base_simple/goal"),
            ("pub/path", "replayer/pub/path"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def generate_launch_description():
    """Generate launch description with multiple components."""
    return launch.LaunchDescription(
        get_param_declaretions()
        + [
            LoadComposableNodes(
                target_container="laser_copilot/container",
                composable_node_descriptions=[get_composable_node()],
            )
        ]
    )
