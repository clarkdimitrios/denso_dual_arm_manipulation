from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('right_ip',  default_value='192.168.17.20', description='Right RC8 IP'),
        DeclareLaunchArgument('left_ip',  default_value='192.168.17.21', description='Left RC8 IP'),
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Whether to run in simulation mode (true) or connect to real hardware (false)'
        ),
        DeclareLaunchArgument(
            'box',
            default_value='true',
            description='Whether to spawn a box in RViz environment'
        )
    ]

    denso_bringup_path = get_package_share_directory('denso_robot_bringup')
    denso_bringup_launch = os.path.join(denso_bringup_path, 'launch', 'denso_robot_bringup.launch.py')

    denso_bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(denso_bringup_launch),
        launch_arguments={
            'model': 'vm60b1',
            'sim': LaunchConfiguration('sim'),
            'controllers_file': 'dual_denso_robot_controllers.yaml',
            'moveit_controllers_file': 'dual_moveit_controllers.yaml',
            'description_file': 'dual_denso_robot.urdf.xacro',
            'moveit_config_file': 'dual_denso_robot.srdf.xacro',
            'robot_controller': 'dual_arm_joint_trajectory_controller',
            # 'robot_controller': 'right_arm_controller left_arm_controller',
            'right_ip_address':  LaunchConfiguration('right_ip'),
            'left_ip_address':  LaunchConfiguration('left_ip'),
        }.items()
    )

    add_virtual_walls_node = Node(
        package='dual_denso_arm_manipulation',
        executable='add_virtual_walls',
        name='add_virtual_walls',
        parameters=[os.path.join(
            get_package_share_directory('dual_denso_arm_manipulation'),
            'config',
            'virtual_walls.yaml'
        )]
    )

    spawn_lift_box_node = Node(
        package='dual_denso_arm_manipulation',
        executable='spawn_lift_box_rviz',
        name='spawn_lift_box_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('box'))
    )

    tf_broadcast_node = Node(
        package='dual_denso_arm_manipulation',
        executable='box_spawner',
        name='box_spawner',
        output='screen',
        parameters=[{
            'sim': LaunchConfiguration('sim')
        }]
    )

    return LaunchDescription(declared_arguments + [
        denso_bringup_include,
        add_virtual_walls_node,
        # spawn_lift_box_node,
        tf_broadcast_node,
    ])