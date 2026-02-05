from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

# Base timing (seconds)
t_mux_to_right = 1.0          # give joint_state_mux a moment to come up
t2 = 3.0                      # left HW starts after right HW
t3 = t2 + 3.0                 # dual MoveIt starts after left HW
t_extras = 10.0               # extra delay AFTER dual MoveIt to ensure RViz is ready

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
        # Optional: override the post-RViz extras delay without editing code
        DeclareLaunchArgument(
            'extras_delay',
            default_value=str(t_extras),
            description='Seconds to wait after dual MoveIt/RViz before spawning walls/boxes',
        ),
    ]

    right_ip = LaunchConfiguration('right_ip')
    left_ip = LaunchConfiguration('left_ip')
    sim = LaunchConfiguration('sim')
    extras_delay = LaunchConfiguration('extras_delay')

    # ---------------- Common bringup launch ----------------
    denso_bringup_path = get_package_share_directory('denso_robot_bringup')
    denso_bringup_launch = os.path.join(
        denso_bringup_path, 'launch', 'denso_robot_bringup.launch.py'
    )

    # ======================================================
    # STEP 0: Start joint_state_mux (long-running process)
    # ======================================================
    joint_state_mux_node = Node(
        package='dual_denso_arm_manipulation',
        executable='joint_state_mux',
        name='joint_state_mux',
        output='screen',
    )

    # ======================================================
    # STEP 1: Right arm HW only (single-URDF, no MoveIt, no RViz)
    # ======================================================
    right_hw_raw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(denso_bringup_launch),
        launch_arguments={
            'model': 'vm60b1',
            'ip_address': right_ip,
            'description_file': 'denso_robot.urdf.xacro',
            'moveit_config_file': 'denso_robot.srdf.xacro',
            'controllers_file': 'right_denso_robot_controllers.yaml',
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

    # Delay right HW slightly so mux is alive first
    right_hw = TimerAction(
        period=t_mux_to_right,
        actions=[right_hw_raw],
    )

    # ======================================================
    # STEP 2: Left arm HW only (single-URDF, no MoveIt, no RViz)
    # ======================================================
    left_hw_raw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(denso_bringup_launch),
        launch_arguments={
            'model': 'vm60b1',
            'ip_address': left_ip,
            'description_file': 'denso_robot.urdf.xacro',
            'moveit_config_file': 'denso_robot.srdf.xacro',
            'controllers_file': 'left_denso_robot_controllers.yaml',
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

    # Timer for step 2 (relative to launch start, not relative to step 1)
    # We offset by t_mux_to_right so the spacing stays consistent.
    left_hw = TimerAction(
        period=t_mux_to_right + t2,
        actions=[left_hw_raw],
    )

    # ======================================================
    # STEP 3: Dual-URDF + MoveIt only (no HW)
    # Matches your command exactly (launch_hw:=false)
    # ======================================================
    dual_moveit_raw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(denso_bringup_launch),
        launch_arguments={
            'model': 'vm60b1',
            'ip_address': '192.168.17.50',  # dummy / unused since launch_hw=false
            'description_file': 'dual_denso_robot.urdf.xacro',
            'moveit_config_file': 'dual_denso_robot.srdf.xacro',
            'controllers_file': 'dual_denso_robot_controllers.yaml',
            'moveit_controllers_file': 'dual_moveit_controllers.yaml',
            'robot_controller': 'dual_arm_joint_trajectory_controller',
            'sim': sim,
            'launch_rviz': 'true',
            'launch_moveit': 'true', 
            'launch_hw': 'false',
            'verbose': 'false',
            'namespace': '',
        }.items(),
    )

    dual_moveit = TimerAction(
        period=t_mux_to_right + t3,
        actions=[dual_moveit_raw],
    )

    # ======================================================
    # EXTRA NODES (virtual walls + box spawners)
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

    # Start extras after dual MoveIt + user-controlled delay
    extras = TimerAction(
        period=LaunchConfiguration('extras_delay'),
        actions=[
            add_virtual_walls_node,
            tf_broadcast_node,
            spawn_lift_box_node,
        ],
    )

    # NOTE: extras_delay is absolute time from launch start.
    # We want "extras_delay after dual_moveit starts", so we offset it:
    extras_after_dual = TimerAction(
        period=t_mux_to_right + t3 + float(t_extras),
        actions=[add_virtual_walls_node, tf_broadcast_node, spawn_lift_box_node],
    )

    return LaunchDescription(
        declared_arguments
        + [
            # Step 0
            joint_state_mux_node,

            # Step 1–3
            right_hw,
            left_hw,
            dual_moveit,

            # Extras (safer: hard offset after dual_moveit)
            extras_after_dual,
        ]
    )
