from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

t2 = 7.0
t3 = t2 + 3.0
t4 = t3 + 2.0

def generate_launch_description():
    # ---------------- Launch arguments ----------------
    declared_arguments = [
        DeclareLaunchArgument(
            'right_ip',
            default_value='192.168.17.20',
            description='Right RC8 IP'
        ),
        DeclareLaunchArgument(
            'left_ip',
            default_value='192.168.17.21',
            description='Left RC8 IP'
        ),
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Whether to run in simulation mode (true) or connect to real hardware (false)',
        ),
        DeclareLaunchArgument(
            'box',
            default_value='true',
            description='Whether to spawn a box in RViz environment',
        ),
    ]

    right_ip = LaunchConfiguration('right_ip')
    left_ip = LaunchConfiguration('left_ip')
    sim = LaunchConfiguration('sim')

    # ---------------- Common bringup launch ----------------
    denso_bringup_path = get_package_share_directory('denso_robot_bringup')
    denso_bringup_launch = os.path.join(
        denso_bringup_path, 'launch', 'denso_robot_bringup.launch.py'
    )

    # ======================================================
    # STEP 1: Right arm HW only (single-URDF, no MoveIt, no RViz)
    # ======================================================
    right_hw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(denso_bringup_launch),
        launch_arguments={
            'model': 'vm60b1',
            'ip_address': right_ip,
            'description_file': 'denso_robot.urdf.xacro',
            'moveit_config_file': 'denso_robot.srdf.xacro',
            'controllers_file': 'dual_denso_robot_controllers.yaml',
            'moveit_controllers_file': 'right_moveit_controllers.yaml',
            'robot_controller': 'right_arm_controller',
            'sim': sim,
            'launch_rviz': 'false',
            'launch_moveit': 'false',
            'launch_hw': 'true',
            'verbose': 'false',
            'namespace': 'right_',
        }.items(),
    )

    # ======================================================
    # STEP 2: Left arm HW only (single-URDF, no MoveIt, no RViz)
    # -> launched with a small delay after step 1
    # ======================================================
    left_hw_raw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(denso_bringup_launch),
        launch_arguments={
            'model': 'vm60b1',
            'ip_address': left_ip,
            'description_file': 'denso_robot.urdf.xacro',
            'moveit_config_file': 'denso_robot.srdf.xacro',
            'controllers_file': 'dual_denso_robot_controllers.yaml',
            'moveit_controllers_file': 'left_moveit_controllers.yaml',
            'robot_controller': 'left_arm_controller',
            'sim': sim,
            'launch_rviz': 'false',
            'launch_moveit': 'false',
            'launch_hw': 'true',
            'verbose': 'false',
            'namespace': 'left_',
        }.items(),
    )

    # Timer for step 2
    left_hw = TimerAction(
        period=t2,   # seconds after launch
        actions=[left_hw_raw],
    )

    # ======================================================
    # STEP 3: Dual-URDF + MoveIt only (no HW, MoveIt+RViz only)
    # -> launched after the two HW bringups
    # ======================================================
    dual_moveit_raw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(denso_bringup_launch),
        launch_arguments={
            'model': 'vm60b1',
            'ip_address': '192.168.17.50',  # dummy / unused for HW=false
            'description_file': 'dual_denso_robot.urdf.xacro',
            'moveit_config_file': 'dual_denso_robot.srdf.xacro',
            'controllers_file': 'dual_denso_robot_controllers.yaml',
            'moveit_controllers_file': 'dual_moveit_controllers.yaml',
            'robot_controller': 'dual_arm_joint_trajectory_controller',
            'sim': sim,
            'launch_rviz': 'true',
            'launch_moveit': 'true',
            'launch_hw': 'false',   # IMPORTANT: do NOT start HW here
            'verbose': 'false',
            'namespace': '',
        }.items(),
    )

    # Timer for step 3 (a bit after step 2)
    dual_moveit = TimerAction(
        period=t3, 
        actions=[dual_moveit_raw],
    )

    # ======================================================
    # EXTRA NODES (after the 3 main steps)
    # (small delay so MoveIt + RViz are ready)
    # ======================================================
    dual_pkg_share = get_package_share_directory('dual_denso_arm_manipulation')

    add_virtual_walls_node = Node(
        package='dual_denso_arm_manipulation',
        executable='add_virtual_walls',
        name='add_virtual_walls',
        parameters=[os.path.join(dual_pkg_share, 'config', 'virtual_walls.yaml')],
        output='screen',
    )

    tf_broadcast_node = Node(
        package='dual_denso_arm_manipulation',
        executable='box_spawner',
        name='box_spawner',
        output='screen',
        parameters=[{'sim': sim}],
    )

    spawn_lift_box_node = Node(
        package='dual_denso_arm_manipulation',
        executable='spawn_lift_box_rviz',
        name='spawn_lift_box_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('box')),
    )

    # If you want the slave streamer here again, you can add it similarly:
    # slv_stream_node = Node(
    #     package='dual_denso_arm_manipulation',
    #     executable='slave_streamer',
    #     name='slave_streamer',
    #     output='screen',
    #     parameters=[{
    #         'period_sec': 0.008,
    #         'controller_trajectory_topic': '/dual_arm_joint_trajectory_controller/joint_trajectory',
    #         'input_topic': '/slave_streamer/trajectory_in',
    #         'change_mode_services': ['/left_vm60b1/ChangeMode','/right_vm60b1/ChangeMode'],
    #         'joint_names': [
    #             'left_joint_1','left_joint_2','left_joint_3',
    #             'left_joint_4','left_joint_5','left_joint_6',
    #             'right_joint_1','right_joint_2','right_joint_3',
    #             'right_joint_4','right_joint_5','right_joint_6',
    #         ],
    #     }],
    # )

    # Small delay so it starts after MoveIt (avoid extra timers where we can)
    extras = TimerAction(
        period=t4,
        actions=[
            add_virtual_walls_node,
            tf_broadcast_node,
            spawn_lift_box_node,
            # slv_stream_node,
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            # Step 1: right HW (no timer)
            right_hw,
            # Step 2: left HW (timer)
            left_hw,
            # Step 3: dual MoveIt (timer)
            dual_moveit,
            # Extra nodes after the main bringup
            extras,
        ]
    )
