import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='laser_copilot_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='laser_copilot',
                    plugin="laser_copilot::obj_dist",
                    name='objdist',
                    remappings=[
                        ('sub/lvx', '/livox/lidar')
                    ])
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])