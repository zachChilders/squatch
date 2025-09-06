from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='squatch_nodes',
            executable='esp_bridge_node',
            name='esp_bridge',
            arguments=['host.docker.internal', '5555'],
            output='screen'
        )
    ])