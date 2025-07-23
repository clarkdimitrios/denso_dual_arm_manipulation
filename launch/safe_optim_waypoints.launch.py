from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def setup_nodes(context, *args, **kwargs):
    dual = LaunchConfiguration('dual').perform(context).lower() == 'true'

    # Normalize filename (remove .csv if included)
    raw_csv_input = LaunchConfiguration('csv_filename').perform(context)
    csv_prefix = raw_csv_input[:-4] if raw_csv_input.endswith('.csv') else raw_csv_input

    ns = LaunchConfiguration('namespace').perform(context)

    actions = []

    if not csv_prefix:
        actions += [
            LogInfo(msg="\n\n[ERROR] Missing required launch argument 'csv_filename'."),
            LogInfo(msg="Example usage: ros2 launch manip_facts_lab safe_optim_waypoints_launch.py csv_filename:=waypoints namespace:=left_"),
            Shutdown()
        ]
        return actions

    if dual:
        left_csv = csv_prefix + '_left.csv'
        right_csv = csv_prefix + '_right.csv'
        fallback_csv = csv_prefix + '.csv'

        left_exists = os.path.exists(left_csv)
        right_exists = os.path.exists(right_csv)

        if left_exists and right_exists:
            actions.append(LogInfo(msg=f"\n[INFO] Dual mode — Using: {left_csv} and {right_csv}"))
            csv_files = {'left_': left_csv, 'right_': right_csv}
        else:
            actions.append(LogInfo(
                msg=f"\n[WARN] One or both of {left_csv}, {right_csv} not found — falling back to: {fallback_csv} for both"))
            csv_files = {'left_': fallback_csv, 'right_': fallback_csv}

        for prefix in ['left_', 'right_']:
            actions.append(
                Node(
                    package='manip_facts_lab',
                    executable='waypoints_optim_node',
                    name=f'{prefix}waypoints_optim_node',
                    parameters=[
                        {'namespace': prefix},
                        {'csv_filename': csv_files[prefix]}
                    ],
                )
            )

    else:
        if not ns:
            actions += [
                LogInfo(msg="\n\n[WARN] 'namespace' is empty — Using default joint names without prefix."),
                LogInfo(msg="If using multiple arms, you probably want to set this as 'left_' or 'right_'.")
            ]
        actions += [
            Node(
                package='manip_facts_lab',
                executable='add_virtual_walls',
                name='add_virtual_walls',
                parameters=[{'namespace': ns}]
            ),
            Node(
                package='manip_facts_lab',
                executable='waypoints_optim_node',
                name='waypoints_optim_node',
                parameters=[
                    {'namespace': ns},
                    {'csv_filename': csv_prefix + '.csv'}
                ],
            ),
        ]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'csv_filename',
            default_value='',
            description="Base name for CSV files. Accepts 'name' or 'name.csv'."
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Joint name prefix (e.g., left_ or right_) — used only in single-arm mode'
        ),
        DeclareLaunchArgument(
            'dual',
            default_value='true',
            description='If true, launches waypoints_optim_node for both arms'
        ),
        OpaqueFunction(function=setup_nodes)
    ])
