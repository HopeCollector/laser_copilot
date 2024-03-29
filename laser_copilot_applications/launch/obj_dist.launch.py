import launch
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def get_param_declaretions():
    return [
        DeclareLaunchArgument("z_max", default_value="0.25"),
        DeclareLaunchArgument("z_min", default_value="-0.25"),
        DeclareLaunchArgument("cut_distance", default_value="0.2"),
    ]


def get_composable_node():
    return ComposableNode(
        package="laser_copilot_applications",
        plugin="laser_copilot_applications::obj_dist",
        name="objdist",
        parameters=[
            {
                "z_max": LaunchConfiguration("z_max"),
                "z_min": LaunchConfiguration("z_min"),
                "distance": LaunchConfiguration("cut_distance"),
            }
        ],
        remappings=[
            ("sub/lvx", "/livox/lidar"),
            ("sub/pc2", "/x500_lidar/point_cloud"),
            ("out/laser_scan", "laser_scan/objs"),
        ],
        # extra_arguments=[{"use_intra_process_comms": True}],
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
