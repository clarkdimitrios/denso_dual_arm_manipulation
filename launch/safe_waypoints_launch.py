from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manip_facts_lab',
            executable='add_virtual_walls',
            name='add_virtual_walls',
        ),
        Node(
            package='manip_facts_lab',
            executable='waypoints_node',
            name='waypoints_node',
        ),
    ])
