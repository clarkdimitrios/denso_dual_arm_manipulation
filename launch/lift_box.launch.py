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

    return LaunchDescription([
        # -------- Arguments --------
        DeclareLaunchArgument('lift_height', default_value='0.5',
                              description='Lift height in the +Z direction'),
        DeclareLaunchArgument('csv_grab', default_value='grab',
                              description='Base CSV filename for left/right grab waypoints'),
        DeclareLaunchArgument('csv_lift', default_value='lift',
                              description='Base CSV filename for left/right lift waypoints'),
        DeclareLaunchArgument('temp_folder', default_value=default_temp,
                              description='Path to the temporary folder for CSV files'),

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
