from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup_nodes(context, *args, **kwargs):
    dual = LaunchConfiguration('dual').perform(context).lower() == 'true'
    ns = LaunchConfiguration('namespace').perform(context)
    actions = []

    if dual:
        actions.append(LogInfo(msg="\n[INFO] Dual arm mode enabled — Launching go_to_pose_node for 'left_' and 'right_'."))
        for prefix in ['left_', 'right_']:
            actions.append(
                Node(
                    package='dual_denso_arm_manipulation',
                    executable='go_to_pose_node',
                    name=f'{prefix}go_to_pose_node',
                    parameters=[{'namespace': prefix}]
                )
            )
    else:
        if not ns:
            actions += [
                LogInfo(msg="\n[WARN] 'namespace' is empty — Using default joint names without prefix."),
                LogInfo(msg="If using multiple arms, you probably want to set this as 'left_' or 'right_'.")
            ]
        actions += [
            Node(
                package='dual_denso_arm_manipulation',
                executable='add_virtual_walls',
                name='add_virtual_walls',
                parameters=[{'namespace': ns}]
            ),
            Node(
                package='dual_denso_arm_manipulation',
                executable='go_to_pose_node',
                name='go_to_pose_node',
                parameters=[{'namespace': ns}]
            ),
        ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Prefix for joint names (e.g., left_ or right_) — ignored if dual mode is enabled'
        ),
        DeclareLaunchArgument(
            'dual',
            default_value='true',
            description='Launch in dual-arm mode (spawns go_to_pose_node for left_ and right_)'
        ),
        OpaqueFunction(function=setup_nodes)
    ])
