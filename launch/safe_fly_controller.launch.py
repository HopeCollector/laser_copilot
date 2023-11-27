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
        DeclareLaunchArgument("max_acc", default_value="0.5"),
        DeclareLaunchArgument("max_speed", default_value="2.0"),
        DeclareLaunchArgument(
            "max_yaw_speed", default_value="30.0", description="degree/s"
        ),
    ]


def get_composable_node():
    return ComposableNode(
        package="laser_copilot",
        plugin="laser_copilot::safe_fly_controller",
        name="controller",
        parameters=[
            {
                "use_sim_time": True,
                "max_acc": LaunchConfiguration("max_acc"),
                "max_speed": LaunchConfiguration("max_speed"),
                "max_yaw_speed": LaunchConfiguration("max_yaw_speed"),
            }
        ],
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

    launch_list = []
    launch_list += get_param_declaretions()
    launch_list.append(container)

    return launch.LaunchDescription(launch_list)
