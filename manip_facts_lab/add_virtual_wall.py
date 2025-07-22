#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Pose
import transforms3d
import yaml


class WallPublisher(Node):
    def __init__(self):
        super().__init__("add_virtual_wall")

        # Declare parameters with correct default types
        self.declare_parameter('namespace', '')
        self.declare_parameter('walls', [''])  # Must be a list of strings by default

        ns = self.get_parameter('namespace').value
        walls_param = self.get_parameter('walls').value  # List[str]

        joint_names = [f"{ns}joint_{i}" for i in range(1, 7)]

        if not ns:
            self.get_logger().warn("Parameter 'namespace' is empty — using default joint names without prefix.")

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name=f"{ns}base_link",
            end_effector_name=f"{ns}J6",
            group_name=f"{ns}arm",
        )

        valid_walls = [w for w in walls_param if w.strip()]
        if not valid_walls:
            self.get_logger().warn("No walls specified — nothing will be added.")
        else:
            self.add_walls(valid_walls)

        self.create_timer(1.0, lambda: rclpy.shutdown())

    def add_walls(self, walls_list):
        for i, wall_string in enumerate(walls_list):
            wall_dict = self._parse_wall_param(wall_string)
            if not wall_dict:
                continue

            pose = Pose()
            pose.position.x = wall_dict.get('pose', [0, 0, 0, 0, 0, 0])[0]
            pose.position.y = wall_dict.get('pose', [0, 0, 0, 0, 0, 0])[1]
            pose.position.z = wall_dict.get('pose', [0, 0, 0, 0, 0, 0])[2]

            roll, pitch, yaw = wall_dict.get('pose', [0, 0, 0, 0, 0, 0])[3:6]
            q = transforms3d.euler.euler2quat(roll, pitch, yaw, axes='sxyz')
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            size = wall_dict.get('size', [1.0, 1.0, 1.0])

            self.get_logger().info(f"Adding wall '{wall_dict.get('id', f'wall_{i}')}' at {pose.position}")
            self.moveit2.add_collision_box(
                id=wall_dict.get('id', f'wall_{i}'),
                size=size,
                pose=pose
            )

    def _parse_wall_param(self, wall_string):
        try:
            return yaml.safe_load(wall_string)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse wall param: {e}")
            return None


def main():
    rclpy.init()
    node = WallPublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
