import launch
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def get_composable_node():
    return ComposableNode(
        package="laser_copilot_applications",
        plugin="laser_copilot_applications::visualization_helper",
        name="visualization_helper",
        parameters=[{}],
        remappings=[
            ("sub/debug", "controller/debug"),
            ("sub/gui_setpoint", "/foxglove/gui_setpoint"),
            ("pub/path_stright", "helper/path/stright"),
            ("pub/path_real", "helper/path/real"),
            ("pub/setpoint", "/move_base_simple/goal"),
            ("pub/velocity", "helper/velocity"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def generate_launch_description():
    return launch.LaunchDescription(
        [
            LoadComposableNodes(
                target_container="laser_copilot/container",
                composable_node_descriptions=[get_composable_node()],
            )
        ]
    )
