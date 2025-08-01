from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def setup_nodes(context, *args, **kwargs):
    raw_csv_input = LaunchConfiguration('csv_filename').perform(context)
    csv_prefix = raw_csv_input[:-4] if raw_csv_input.endswith('.csv') else raw_csv_input
    cartesian_mode = LaunchConfiguration('cartesian').perform(context).lower() in ['true', '1', 'yes']

    actions = []

    if not csv_prefix:
        actions += [
            LogInfo(msg="\n\n[ERROR] Missing required launch argument 'csv_filename'."),
            LogInfo(msg="Example: ros2 launch dual_denso_arm_manipulation dual_waypoints.launch.py csv_filename:=waypoints_0"),
            Shutdown()
        ]
        return actions

    pkg_path = get_package_share_directory('dual_denso_arm_manipulation')
    waypoints_dir = os.path.join(pkg_path, 'waypoints')

    left_csv = os.path.join(waypoints_dir, f"{csv_prefix}_left.csv")
    right_csv = os.path.join(waypoints_dir, f"{csv_prefix}_right.csv")
    fallback_csv = os.path.join(waypoints_dir, f"{csv_prefix}.csv")

    left_exists = os.path.exists(left_csv)
    right_exists = os.path.exists(right_csv)

    if left_exists and right_exists:
        actions.append(LogInfo(msg=f"\n[INFO] Dual-arm mode — Using: {left_csv} and {right_csv}"))
    else:
        actions.append(LogInfo(
            msg=f"\n[WARN] One or both of {left_csv}, {right_csv} not found — falling back to: {fallback_csv} for both"
        ))

    # Select executable based on cartesian_mode flag
    exec_name = 'dual_cartesian_waypoints_node' if cartesian_mode else 'dual_arm_waypoints_node'
    mode_label = "Cartesian" if cartesian_mode else "Joint Space"
    actions.append(LogInfo(msg=f"\n[INFO] Launching in {mode_label} mode using executable: {exec_name}"))

    actions.append(
        Node(
            package='dual_denso_arm_manipulation',
            executable=exec_name,
            name=exec_name,
            parameters=[{'csv_filename': csv_prefix}],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'csv_filename',
            default_value='',
            description="Base name for CSV files. Uses <base>_left.csv and <base>_right.csv if both exist, otherwise falls back to <base>.csv for both arms"
        ),
        DeclareLaunchArgument(
            'cartesian',
            default_value='false',
            description="If true, use Cartesian mode (dual_cartesian_waypoints_node) instead of joint space"
        ),
        OpaqueFunction(function=setup_nodes)
    ])
