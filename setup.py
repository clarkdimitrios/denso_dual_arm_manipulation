from glob import glob
from setuptools import find_packages, setup

package_name = 'dual_denso_arm_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/waypoints', glob('waypoints/*.csv')),
        (f'share/{package_name}/urdf', glob('urdf/*.xacro')),
        (f'share/{package_name}/config', glob('config/*.*')),
        (f'share/{package_name}/worlds', glob('worlds/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clark',
    maintainer_email='cabourjeily3@gatech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go_to_pose_node = dual_denso_arm_manipulation.go_to_pose:main',
            'add_virtual_walls = dual_denso_arm_manipulation.add_virtual_wall:main',
            'waypoints_node = dual_denso_arm_manipulation.follow_waypoints:main',
            'dual_arm_waypoints_node = dual_denso_arm_manipulation.dual_follow_waypoints:main',
            'dual_cartesian_waypoints_node = dual_denso_arm_manipulation.dual_cartesian_waypoints:main',
            'waypoints_optim_node = dual_denso_arm_manipulation.follow_waypoints_optim:main',
            'robust_waypoints_node = dual_denso_arm_manipulation.robust_waypoints_optim:main',
            'controller_auto_switcher = dual_denso_arm_manipulation.controller_auto_switcher:main',
            'spawn_lift_box_rviz = dual_denso_arm_manipulation.spawn_lift_box_rviz:main',
            'box_spawner = dual_denso_arm_manipulation.box_spawner:main',
            'lift_traj_generator = dual_denso_arm_manipulation.lift_traj_generator:main',
            'box_pose_sub = dual_denso_arm_manipulation.box_pose_sub:main',
            'end_effector_pose = dual_denso_arm_manipulation.end_effector_pose:main',
            'traj_to_csv = dual_denso_arm_manipulation.traj_to_csv:main',
            'collision_manager = dual_denso_arm_manipulation.collision_manager:main',
            'ee_box_linker = dual_denso_arm_manipulation.ee_box_linker:main',
        ],
    },
)
