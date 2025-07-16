from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    denso_bringup_path = get_package_share_directory('denso_robot_bringup')
    denso_bringup_launch = os.path.join(denso_bringup_path, 'launch', 'denso_robot_bringup.launch.py')

    return LaunchDescription([
        # Left arm
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(denso_bringup_launch),
            launch_arguments={
                'model': 'vm60b1',
                'ip_address': '192.168.17.21',
                'sim': 'false',
                'namespace': 'left_',
                'controllers_file': 'left_denso_robot_controllers.yaml',
                'moveit_controllers_file': 'left_moveit_controllers.yaml',
            }.items()
        ),

        # Right arm
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(denso_bringup_launch),
            launch_arguments={
                'model': 'vm60b1',
                'ip_address': '192.168.17.20',
                'sim': 'false',
                'namespace': 'right_',
                'controllers_file': 'right_denso_robot_controllers.yaml',
                'moveit_controllers_file': 'right_moveit_controllers.yaml',
            }.items()
        ),
    ])
