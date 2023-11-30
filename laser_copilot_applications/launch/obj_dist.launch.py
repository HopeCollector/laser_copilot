import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def get_param_declaretions():
    return [
        DeclareLaunchArgument("z_max", default_value="0.25"),
        DeclareLaunchArgument("z_min", default_value="-0.25"),
        DeclareLaunchArgument("distance", default_value="0.2", description="degree/s"),
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
                "distance": LaunchConfiguration("distance"),
            }
        ],
        remappings=[
            ("sub/lvx", "livox/lidar"),
            ("sub/pc2", "/x500_lidar/point_cloud"),
            ("out/laser_scan", "laser_scan/objs"),
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

    launch_list = []
    launch_list += get_param_declaretions()
    launch_list.append(container)

    return launch.LaunchDescription(launch_list)
