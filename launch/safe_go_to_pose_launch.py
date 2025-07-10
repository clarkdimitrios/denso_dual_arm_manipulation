from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manip_facts_lab',
            executable='go_to_pose_node',
            name='go_to_pose_node',
        ),
        Node(
            package='manip_facts_lab',
            executable='add_virtual_walls',
            name='add_virtual_walls',
        ),
        Node(
            package='manip_facts_lab',
            executable='nullspace_optim_node',
            name='nullspace_optim_node',
        ),
    ])
