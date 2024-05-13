import launch
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration


def get_param_declaretions():
    return []


def generate_launch_description():
    """Generate launch description with multiple components."""
    return launch.LaunchDescription(
        get_param_declaretions()
        + [
            Node(
                executable="people_tracker.py",
                package="laser_copilot_applications",
                name="people_tracker",
                remappings=[("out/goal", "/move_base_simple/goal")],
            )
        ]
    )
