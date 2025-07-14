from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def check_csv_filename(context, *args, **kwargs):
    csv_filename = LaunchConfiguration('csv_filename').perform(context)
    if not csv_filename:
        return [
            LogInfo(msg="\n\n[ERROR] Missing required launch argument 'csv_filename'."),
            LogInfo(msg="Example Usage: ros2 launch manip_facts_lab safe_waypoints_launch.py csv_filename:=waypoints_0.csv"),
            Shutdown()
        ]
    return []


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'csv_filename',
            default_value='',
            description='Path to the CSV file with waypoints'
        ),

        OpaqueFunction(function=check_csv_filename),

        Node(
            package='manip_facts_lab',
            executable='add_virtual_walls',
            name='add_virtual_walls',
        ),

        Node(
            package='manip_facts_lab',
            executable='waypoints_node',
            name='waypoints_node',
            parameters=[{'csv_filename': LaunchConfiguration('csv_filename')}],
        ),
    ])
