from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import tempfile
import rclpy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from dual_denso_arm_manipulation.utils import get_xacro_properties, euler_to_quat

# def get_box_pose():
#     # Get the box pose from the scene_config.xacro
#     scene_path = os.path.join(
#         get_package_share_directory('dual_denso_arm_manipulation'),
#         'config',
#         'scene_config.xacro' 
#     )
#     props = get_xacro_properties(scene_path)
#     return {
#         "x": float(props.get("lift_box_x", "0.0")),
#         "y": float(props.get("lift_box_y", "0.0")),
#         "z": float(props.get("lift_box_z", "0.0")),
#         "R": float(props.get("lift_box_roll", "0.0")),
#         "P": float(props.get("lift_box_pitch", "0.0")),
#         "Y": float(props.get("lift_box_yaw", "0.0")),
#     }

# def spawn_box_fn(context, *args, **kwargs):
    # lift_box_xacro = os.path.join(
    #     get_package_share_directory('dual_denso_arm_manipulation'),
    #     'config',
    #     'lift_box.xacro'
    # )

    # urdf_doc = xacro.process_file(lift_box_xacro)
    # urdf_string = urdf_doc.toprettyxml()

    # pose = get_box_pose()

    # tmpfile = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
    # tmpfile.write(urdf_string)
    # tmpfile.close()

    # broadcaster = TransformBroadcaster(context)

    # transform = TransformStamped()
    # transform.header.stamp = rclpy.time.Time().to_msg()
    # transform.header.frame_id = "world" 
    # transform.child_frame_id = "lift_box" 

    # orientation = euler_to_quat(pose["R"], pose["P"], pose["Y"])

    # transform.transform.translation.x = pose["x"]
    # transform.transform.translation.y = pose["y"]
    # transform.transform.translation.z = pose["z"]
    # transform.transform.rotation.x = orientation[1] 
    # transform.transform.rotation.y = orientation[2]
    # transform.transform.rotation.z = orientation[3]
    # transform.transform.rotation.w = orientation[0]

    # broadcaster.sendTransform(transform)

    # return [
    #     Node(
    #         package='gazebo_ros',
    #         executable='spawn_entity.py',
    #         name='spawn_lift_box',
    #         output='screen',
    #         arguments=[
    #             '-entity', 'lift_box',
    #             '-file', tmpfile.name,
    #             '-x', str(pose["x"]),
    #             '-y', str(pose["y"]),
    #             '-z', str(pose["z"]),
    #             '-R', str(pose["R"]),
    #             '-P', str(pose["P"]),
    #             '-Y', str(pose["Y"]),
    #         ]
    #     )
    # ]

def generate_launch_description():
    declared_arguments = [
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
        spawn_lift_box_node,
        tf_broadcast_node,
        # OpaqueFunction(
        #     function=spawn_box_fn,
        #     condition=IfCondition(LaunchConfiguration('sim'))
        # )
    ])