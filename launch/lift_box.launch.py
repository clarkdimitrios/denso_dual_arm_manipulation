#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('dual_denso_arm_manipulation')
    default_temp = os.path.join(pkg_share, 'temp')

    # -------- Launch Configs --------
    lift_height = LaunchConfiguration('lift_height')
    csv_grab = LaunchConfiguration('csv_grab')
    csv_lift = LaunchConfiguration('csv_lift')

    # -------- Executors --------
    dual_exec_grab = Node(
        package='dual_denso_arm_manipulation',
        executable='dual_cartesian_waypoints_node',
        name='dual_exec_grab',
        output='screen',
        parameters=[{
            'csv_filename': csv_grab,
            'shutdown_after_execute': True,
        }],
    )

    dual_exec_lift = Node(
        package='dual_denso_arm_manipulation',
        executable='dual_cartesian_waypoints_node',
        name='dual_exec_lift',
        output='screen',
        parameters=[{
            'csv_filename': csv_lift,
            'shutdown_after_execute': True,
        }],
    )


    # EE-box linker params
    robot_model      = LaunchConfiguration('robot_model')
    box_model        = LaunchConfiguration('box_model')
    left_link        = LaunchConfiguration('left_link')
    right_link       = LaunchConfiguration('right_link')
    box_link         = LaunchConfiguration('box_link')
    attach_left      = LaunchConfiguration('attach_left')
    attach_right     = LaunchConfiguration('attach_right')
    attach_on_state  = LaunchConfiguration('attach_on_state')
    detach_on_state  = LaunchConfiguration('detach_on_state')
    attach_service   = LaunchConfiguration('attach_service')
    detach_service   = LaunchConfiguration('detach_service')

    return LaunchDescription([
        # -------- Arguments --------
        DeclareLaunchArgument('lift_height', default_value='0.8',
                              description='Lift height in the +Z direction'),
        DeclareLaunchArgument('csv_grab', default_value='grab',
                              description='Base CSV filename for left/right grab waypoints'),
        DeclareLaunchArgument('csv_lift', default_value='lift',
                              description='Base CSV filename for left/right lift waypoints'),
        DeclareLaunchArgument('temp_folder', default_value=default_temp,
                              description='Path to the temporary folder for CSV files'),

        # EE-box linker args
        DeclareLaunchArgument('robot_model',     default_value='dual_denso_robot', #'vm60b1',
                              description='Gazebo model name of the robot'),
        DeclareLaunchArgument('box_model',       default_value='lift_box',
                              description='Gazebo model name of the box'),
        DeclareLaunchArgument('left_link',       default_value='left_J6',
                              description='Left end-effector link name (inside robot model)'),
        DeclareLaunchArgument('right_link',      default_value='right_J6',
                              description='Right end-effector link name (inside robot model)'),
        DeclareLaunchArgument('box_link',        default_value='lift_box',
                              description='Box link name (inside box model)'),
        DeclareLaunchArgument('attach_left',     default_value='true',
                              description='Attach left EE to box when triggered'),
        DeclareLaunchArgument('attach_right',    default_value='false',
                              description='Attach right EE to box when triggered'),
        DeclareLaunchArgument('attach_on_state', default_value='lift',
                              description="trajectory_state value that triggers attach"),
        DeclareLaunchArgument('detach_on_state', default_value='release',
                              description="trajectory_state value that triggers detach"),
        DeclareLaunchArgument('attach_service',  default_value='/gazebo/attach',
                              description='Service name for attach()'),
        DeclareLaunchArgument('detach_service',  default_value='/gazebo/detach',
                              description='Service name for detach()'),

        # -------- Clean temp BEFORE nodes start --------
        ExecuteProcess(
            cmd=['/bin/bash', '-lc', f'rm -rf "{default_temp}"; mkdir -p "{default_temp}"'],
            name='cleanup_temp_at_start',
            output='screen'
        ),

        # -------- Perception / pose feeds --------
        Node(
            package='dual_denso_arm_manipulation',
            executable='box_pose_sub',
            name='box_pose_sub',
            output='screen',
        ),
        Node(
            package='dual_denso_arm_manipulation',
            executable='end_effector_pose',
            name='end_effector_pose',
            output='screen',
        ),

        # -------- Collision manager (applies scene change and acks 'lift_ready') --------
        # Node(
        #     package='dual_denso_arm_manipulation',
        #     executable='collision_manager', 
        #     name='collision_manager',
        #     output='screen',
        # ),

        # -------- EE-Box Linker --------
        Node(
            package='dual_denso_arm_manipulation',
            executable='ee_box_linker',
            name='ee_box_linker',
            output='screen',
            parameters=[{
                'robot_model': 'vm60b1',
                'box_model': 'lift_box',
                'left_link': 'left_J6',
                'right_link': 'right_J6',
                'box_link': 'lift_box',
                'attach_left': True,
                'attach_right': False,
                'attach_on_state': 'attach',
                'detach_on_state': 'detach',
            }],
        ),

        # -------- Trajectory generator (publishes 'grab', waits for ack, then 'lift') --------
        Node(
            package='dual_denso_arm_manipulation',
            executable='lift_traj_generator',
            name='lift_traj_generator',
            output='screen',
            parameters=[{'lift_height': lift_height}],
        ),

        # -------- CSV writer (records poses into temp/{grab|lift}_{left|right}.csv) --------
        Node(
            package='dual_denso_arm_manipulation',
            executable='traj_to_csv',
            name='traj_to_csv',
            output='screen',
        ),

        # -------- Executors (sequential; no overlap) --------
        # Start GRAB executor immediately; it will poll temp/ for CSVs and run once, then exit.
        dual_exec_grab,

        # Sequential, so need to fix unnecessary delays...
        # When GRAB exits, launch LIFT executor (runs once, then exits).
        RegisterEventHandler(
            OnProcessExit(
                target_action=dual_exec_grab,
                on_exit=[dual_exec_lift],
            )
        ),

        LogInfo(msg="Launch complete"),
    ])
