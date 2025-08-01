from glob import glob
from setuptools import find_packages, setup

package_name = 'manip_facts_lab'

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
            'go_to_pose_node = manip_facts_lab.go_to_pose:main',
            'add_virtual_walls = manip_facts_lab.add_virtual_wall:main',
            'waypoints_node = manip_facts_lab.follow_waypoints:main',
            'dual_arm_waypoints_node = manip_facts_lab.dual_follow_waypoints:main',
            'waypoints_optim_node = manip_facts_lab.follow_waypoints_optim:main',
            'robust_waypoints_node = manip_facts_lab.robust_waypoints_optim:main',
        ],
    },
)
