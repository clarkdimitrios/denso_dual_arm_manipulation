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

from dual_denso_arm_manipulation.utils import get_xacro_properties 

def get_box_pose():
    scene_path = os.path.join(
        get_package_share_directory('dual_denso_arm_manipulation'),
        'config',
        'scene_config.xacro' 
    )
    props = get_xacro_properties(scene_path)
    return {
        "x": props.get("lift_box_x", "0.0"),
        "y": props.get("lift_box_y", "0.0"),
        "z": props.get("lift_box_z", "0.0"),
        "R": props.get("lift_box_roll", "0.0"),
        "P": props.get("lift_box_pitch", "0.0"),
        "Y": props.get("lift_box_yaw", "0.0"),
    }

def spawn_box_fn(context, *args, **kwargs):
    lift_box_xacro = os.path.join(
        get_package_share_directory('dual_denso_arm_manipulation'),
        'config',
        'lift_box.xacro'
    )

    urdf_doc = xacro.process_file(lift_box_xacro)
    urdf_string = urdf_doc.toprettyxml()

    pose = get_box_pose()

    tmpfile = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
    tmpfile.write(urdf_string)
    tmpfile.close()

    return [
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_lift_box',
            output='screen',
            arguments=[
                '-entity', 'lift_box',
                '-file', tmpfile.name,
                '-x', pose["x"],
                '-y', pose["y"],
                '-z', pose["z"],
                '-R', pose["R"],
                '-P', pose["P"],
                '-Y', pose["Y"],
            ]
        )
    ]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Whether to run in simulation mode (true) or connect to real hardware (false)'
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
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        denso_bringup_include,
        add_virtual_walls_node,
        spawn_lift_box_node,
        OpaqueFunction(
            function=spawn_box_fn,
            condition=IfCondition(LaunchConfiguration('sim'))
        )
    ])