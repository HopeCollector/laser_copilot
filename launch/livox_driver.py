import sys
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# livox ros driver param
xfer_format = 1  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic = 0  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src = 0  # 0-lidar, others-Invalid data src
publish_freq = 20.0  # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type = 0
frame_id = "livox_frame"
lvx_file_path = "/home/livox/livox_test.lvx"
cmdline_bd_code = "livox0000000001"
user_config_path = PathJoinSubstitution(
    [
        FindPackageShare("livox_ros_driver2"),
        "config",
        "mid360_up.json",
    ]
)
# livox ros driver param


def livox_config_file_path():
    user_config_path = PathJoinSubstitution(
        [
            FindPackageShare("livox_ros_driver2"),
            "config",
            "mid360_up.json",
        ]
    )
    if "twd:=up" in sys.argv:
        print("lidar will be set to towards up")
        user_config_path = PathJoinSubstitution(
            [
                FindPackageShare("livox_ros_driver2"),
                "config",
                "mid360_up.json",
            ]
        )
    elif "twd:=down" in sys.argv:
        print("lidar will be set to towards down")
        user_config_path = PathJoinSubstitution(
            [
                FindPackageShare("livox_ros_driver2"),
                "config",
                "mid360_down.json",
            ]
        )
    else:
        print('need "twd:=[up|down]"')
        raise ("need a direction")
    return user_config_path


def generate_launch_description():
    """Generate launch description with multiple components."""
    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": livox_config_file_path()},
        {"cmdline_input_bd_code": cmdline_bd_code},
    ]
    container = ComposableNodeContainer(
        name="laser_copilot_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="livox_ros_driver2",
                plugin="livox_ros::DriverNode",
                name="livox_lidar_publisher",
                parameters=livox_ros2_params,
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription([container])
