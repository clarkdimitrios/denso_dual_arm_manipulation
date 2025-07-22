from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def check_namespace_prefix(context, *args, **kwargs):
    namespace_prefix = LaunchConfiguration('namespace').perform(context)
    actions = []
    if not namespace_prefix:
        actions += [
            LogInfo(msg="\n\n[WARN] 'namespace' is empty — Using default joint names without prefix."),
            LogInfo(msg="If using multiple arms, you probably want to set this as 'left_' or 'right_'.")
        ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Prefix for joint names (e.g., left_ or right_)'
        ),

        OpaqueFunction(function=check_namespace_prefix),

        Node(
            package='manip_facts_lab',
            executable='add_virtual_walls',
            name='add_virtual_walls',
            parameters=[{'namespace': LaunchConfiguration('namespace')}]
        ),

        Node(
            package='manip_facts_lab',
            executable='go_to_pose_node',
            name='go_to_pose_node',
            parameters=[{'namespace': LaunchConfiguration('namespace')}]
        ),
    ])
