#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2
import transforms3d


class GoToPoseNode(Node):
    def __init__(self):
        super().__init__("go_to_pose_node")

        # Initialize MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
            ],
            base_link_name="base_link",
            end_effector_name="J6",
            group_name="arm",
        )

        # Prompt for velocity/acceleration scaling
        self.set_motion_scaling_factors()

        self.get_logger().info("Ready to input a pick pose")
        self.move_to_user_pose()

    def set_motion_scaling_factors(self):
        def prompt_float(prompt_text, default):
            try:
                value = input(f"{prompt_text} [default {default}]: ").strip()
                return float(value) if value else default
            except ValueError:
                self.get_logger().warn(f"Invalid input, using default: {default}")
                return default

        velocity = prompt_float("Velocity scaling factor (0.0-1.0)", 0.2)
        acceleration = prompt_float("Acceleration scaling factor (0.0-1.0)", 0.2)

        velocity = max(0.0, min(1.0, velocity))
        acceleration = max(0.0, min(1.0, acceleration))

        self.moveit2.max_velocity_scaling_factor = velocity
        self.moveit2.max_acceleration_scaling_factor = acceleration

    def move_to_user_pose(self):
        # Prompt for user input
        x = float(input("x (m): "))
        y = float(input("y (m): "))
        z = float(input("z (m): "))

        def get_input_deg(prompt, default):
            s = input(f"{prompt} [default {default}°]: ")
            return float(s) if s.strip() else default

        roll_deg = get_input_deg("roll (deg)", 0.0)
        pitch_deg = get_input_deg("pitch (deg)", 0.0)
        yaw_deg = get_input_deg("yaw (deg)", 0.0)

        # Convert to quaternion
        quat = transforms3d.euler.euler2quat(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
            axes='sxyz'
        )

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]
        pose.orientation.w = quat[0]

        self.get_logger().info("Planning to target pose...")
        self.moveit2.move_to_pose(pose)

        self.get_logger().info("Motion complete")


def main():
    rclpy.init()
    node = GoToPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
