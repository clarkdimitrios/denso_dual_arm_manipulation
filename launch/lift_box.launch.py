#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            'csv_filename', 
            default_value='lift',  # Base CSV filename for left and right waypoints
            description='The base CSV filename for left and right waypoints'
        ),
        DeclareLaunchArgument(
            'temp_folder', 
            default_value=os.path.join(
                get_package_share_directory('dual_denso_arm_manipulation'), 'temp'
            ),  # Temporary folder to store generated lift CSV files
            description='Path to the temporary folder to store CSV files'
        ),

        # Launch the box pose subscriber node
        Node(
            package='dual_denso_arm_manipulation', 
            executable='box_pose_sub',  
            name='box_pose_sub',
            output='screen',
        ),

        # Launch the end effector pose subscriber node
        Node(
            package='dual_denso_arm_manipulation', 
            executable='end_effector_pose',  
            name='end_effector_pose',
            output='screen',
        ),

        # Launch the trajectory generator node
        Node(
            package='dual_denso_arm_manipulation',  
            executable='lift_traj_generator', 
            name='lift_traj_generator',
            output='screen',
        ),

        # Launch the trajectory to CSV node
        Node(
            package='dual_denso_arm_manipulation',  
            executable='traj_to_csv', 
            name='traj_to_csv',
            output='screen',
        ),

        TimerAction(
            period=10.0,  # seconds to wait before launching the node
            actions=[
                Node(
                    package='dual_denso_arm_manipulation', 
                    executable='dual_cartesian_waypoints_node', 
                    name='dual_cartesian_waypoints_node',
                    output='screen',
                    parameters=[{
                        'csv_filename': LaunchConfiguration('csv_filename')
                    }],
                )
            ]
        ),

        # Delete the temporary folder after launch
        ExecuteProcess(
            cmd=['rm', '-rf', LaunchConfiguration('temp_folder')], 
            name='delete_temp_folder',
            output='screen',
        ),

        LogInfo(
            msg="Launching process completed."
        ),
    ])
