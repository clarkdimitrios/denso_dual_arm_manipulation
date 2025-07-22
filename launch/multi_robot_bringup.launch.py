from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


""" NOT FUNCTIONAL """


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Whether to run in simulation mode (true) or real hardware (false)'
        ),
        DeclareLaunchArgument(
            'launch_rviz_separate',
            default_value='false',
            description='Whether to launch two separate RViz windows (true) or not (false)'
        ),
    ]

    denso_bringup_path = get_package_share_directory('denso_robot_bringup')
    denso_bringup_launch = os.path.join(denso_bringup_path, 'launch', 'denso_robot_bringup.launch.py')

    left_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(denso_bringup_launch),
        launch_arguments={
            'model': 'vm60b1',
            'ip_address': '192.168.17.21',
            'sim': LaunchConfiguration('sim'),
            'namespace': 'left_',
            'launch_rviz': 'true',
            'controllers_file': 'left_denso_robot_controllers.yaml',
            'moveit_controllers_file': 'left_moveit_controllers.yaml',
        }.items()
    )

    right_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(denso_bringup_launch),
        launch_arguments={
            'model': 'vm60b1',
            'ip_address': '192.168.17.20',
            'sim': LaunchConfiguration('sim'),
            'namespace': 'right_',
            'launch_rviz': LaunchConfiguration('launch_rviz_separate'), 
            'controllers_file': 'right_denso_robot_controllers.yaml',
            'moveit_controllers_file': 'right_moveit_controllers.yaml',
        }.items()
    )

    return LaunchDescription(declared_arguments + [left_arm, right_arm])
