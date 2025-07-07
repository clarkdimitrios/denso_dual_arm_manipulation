#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class WallPublisher(Node):
    def __init__(self):
        super().__init__("add_virtual_wall")

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3",
                         "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="J6", 
            group_name="arm"
        )

        self.get_logger().info("Adding rear wall to planning scene...")
        self.add_rear_wall()
        self.get_logger().info("Wall added.")
        rclpy.shutdown()

    def add_rear_wall(self):
        wall = CollisionObject()
        wall.id = "rear_virtual_wall"
        wall.header.frame_id = "base_link"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 1.0, 2.0]  # thickness, width, height

        pose = Pose()
        pose.position.x = -0.355  # flush with x = -0.35
        pose.position.y = 0.0
        pose.position.z = 0.25
        pose.orientation.w = 1.0

        wall.primitives.append(primitive)
        wall.primitive_poses.append(pose)
        wall.operation = CollisionObject.ADD

        self.moveit2._planning_scene_interface.apply_collision_object(wall)

def main():
    rclpy.init()
    node = WallPublisher()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
