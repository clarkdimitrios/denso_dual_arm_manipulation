from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def check_required_arguments(context, *args, **kwargs):
    csv_filename = LaunchConfiguration('csv_filename').perform(context)
    namespace_prefix = LaunchConfiguration('namespace').perform(context)

    actions = []

    if not csv_filename:
        actions += [
            LogInfo(msg="\n\n[ERROR] Missing required launch argument 'csv_filename'."),
            LogInfo(msg="Example usage: ros2 launch manip_facts_lab safe_optim_waypoints_launch.py csv_filename:=waypoints.csv namespace:=left_"),
            Shutdown()
        ]

    if not namespace_prefix:
        actions += [
            LogInfo(msg="\n\n[WARN] 'namespace' is empty — Using default joint names without prefix."),
            LogInfo(msg="If using multiple arms, you probably want to set this as 'left_' or 'right_'.")
        ]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'csv_filename',
            default_value='',
            description='Path to the CSV file with waypoints'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Prefix for joint names (e.g., left_ or right_)'
        ),

        OpaqueFunction(function=check_required_arguments),

        Node(
            package='manip_facts_lab',
            executable='add_virtual_walls',
            name='add_virtual_walls',
            parameters=[{'namespace': LaunchConfiguration('namespace')}]
        ),

        Node(
            package='manip_facts_lab',
            executable='waypoints_optim_node',
            name='waypoints_optim_node',
            parameters=[
                {'csv_filename': LaunchConfiguration('csv_filename')},
                {'namespace': LaunchConfiguration('namespace')}
            ],
        ),
    ])
