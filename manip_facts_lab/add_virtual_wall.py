#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Pose
import transforms3d


class WallPublisher(Node):
    def __init__(self):
        super().__init__("add_virtual_wall")

        self.declare_parameter('namespace', '') #expected: left_, right_
        ns = self.get_parameter('namespace').get_parameter_value().string_value
        joint_names = [f"{ns}joint_{i}" for i in range(1,7)]

        if not ns:
            self.get_logger().warn(
                "\n\nParameter 'namespace_prefix' is empty — "
                "using default joint names without prefix.\n"
                "If you are running with a namespaced robot (e.g., left_ or right_),\n"
                "make sure to set this parameter with:\n"
                "  ros2 run manip_facts_lab waypoints_optim_node --ros-args -p namespace:=<prefix>\n"
            )


        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names, #["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name=f"{ns}base_link", #"base_link",
            end_effector_name=f"{ns}J6", #"J6",
            group_name=f"{ns}arm", #"arm"
        )

        self.get_logger().info(f"Adding rear wall to {ns}arm planning scene...")
        self.add_rear_wall()
        self.get_logger().info("Wall added.")

        # Allow time to publish before shutdown
        self.create_timer(1.0, lambda: rclpy.shutdown())

    def add_rear_wall(self):
        pose = Pose()
        pose.position.y = 0.455  # centers a 1cm wall at x = 0.35 (y in rviz)
        pose.position.x = 0.0
        pose.position.z = 1.0     # center of 2m tall wall
        
        
        q = transforms3d.euler.euler2quat(1.5708, 0.0, 0.0, axes='sxyz')
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        self.moveit2.add_collision_box(
            id="rear_virtual_wall",
            size=[0.01, 2.0, 2.0],  # [depth (x), width (y), height (z)]
            pose=pose
        )

def main():
    rclpy.init()
    node = WallPublisher()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
