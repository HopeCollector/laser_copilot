import launch
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration


def get_param_declaretions():
    return []


def get_composable_node():
    return ComposableNode(
        package="laser_copilot_applications",
        plugin="laser_copilot_applications::sbus_bridge",
        name="sbus_bridge",
        parameters=[
            {
                "device_path": "/dev/ttyUSB0",
                "max_speed": 2.0,
                "max_angular_speed": 30.0,
                "read_time_ms": 20,
                "baudrate": 115200,
                "enable_channel": 5,
                "deadzone": 20,
                "channel_max": 1722,
                "channel_min": 283,
                "channel_mid": 1002,
            }
        ],
        remappings=[
            ("cmd_vel", "sbus_bridge/cmd_vel"),
        ],
        extra_arguments=[{"use_intra_process_comms": False}],
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
