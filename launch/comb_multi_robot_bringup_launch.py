from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Whether to run in simulation mode (true) or connect to real hardware (false)'
        )
    ]

    denso_bringup_path = get_package_share_directory('denso_robot_bringup')
    denso_bringup_launch = os.path.join(denso_bringup_path, 'launch', 'denso_robot_bringup.launch.py')

    denso_bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(denso_bringup_launch),
        launch_arguments={
            'model': 'vm60b1',
            # 'ip_address': '',
            'sim': LaunchConfiguration('sim'),
            # 'namespace': '',
            'controllers_file': 'dual_denso_robot_controllers.yaml',
            'moveit_controllers_file': 'dual_moveit_controllers.yaml',
            'description_file': 'dual_denso_robot.urdf.xacro',
            'moveit_config_file': 'dual_denso_robot.srdf.xacro',
            'robot_controller': 'dual_arm_joint_trajectory_controller',
        }.items()
    )

    return LaunchDescription(declared_arguments + [denso_bringup_include])
