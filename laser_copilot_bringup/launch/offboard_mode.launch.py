from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import SetUseSimTime, ComposableNodeContainer, PushRosNamespace


def launch_other_file(pkg_name, file_name):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(pkg_name), "launch", file_name])]
        )
    )


def launch_in_condition(pkg_name, file_name, condition_var):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(pkg_name), "launch", file_name])]
        ),
        condition=IfCondition(LaunchConfiguration(condition_var)),
    )


def declare_param(name, default_var):
    return DeclareLaunchArgument(name, default_value=default_var)


def generate_launch_description():
    pkg_name_app = "laser_copilot_applications"
    return LaunchDescription(
        [
            PushRosNamespace("laser_copilot"),
            declare_param("use_gui", "false"),
            declare_param("use_sim_time", "false"),
            ComposableNodeContainer(
                name="container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                output="screen",
            ),
            SetUseSimTime(
                True, condition=IfCondition(LaunchConfiguration("use_sim_time"))
            ),
            launch_other_file(pkg_name_app, "safe_fly_controller.launch.py"),
            launch_other_file(pkg_name_app, "obj_dist.launch.py"),
            launch_in_condition(pkg_name_app, "visualization.launch.py", "use_gui"),
        ]
    )
