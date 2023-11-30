import launch
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def get_composable_node():
    return ComposableNode(
        package="laser_copilot_applications",
        plugin="laser_copilot_applications::safe_fly_controller",
        name="controller",
        parameters=[
            {
                "max_acc": LaunchConfiguration("max_acc"),
                "max_speed": LaunchConfiguration("max_speed"),
                "max_yaw_speed": LaunchConfiguration("max_yaw_speed"),
            }
        ],
        remappings=[
            ("sub/goal", "/move_base_simple/goal"),
            ("sub/objs", "laser_scan/objs"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def generate_launch_description():
    """Generate launch description with multiple components."""
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("max_acc", default_value="0.5"),
            DeclareLaunchArgument("max_speed", default_value="2.0"),
            DeclareLaunchArgument(
                "max_yaw_speed", default_value="30.0", description="degree/s"
            ),
            LoadComposableNodes(
                target_container="laser_copilot/container",
                composable_node_descriptions=[get_composable_node()],
            ),
        ]
    )
