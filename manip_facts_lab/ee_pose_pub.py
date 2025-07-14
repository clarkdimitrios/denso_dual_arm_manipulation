#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2


class EndEffectorPosePublisher(Node):
    def __init__(self):
        super().__init__('ee_pose_pub')

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "joint_1", "joint_2", "joint_3",
                "joint_4", "joint_5", "joint_6"
            ],
            base_link_name="base_link",
            end_effector_name="J6",
            group_name="arm"
        )

        self.ee_pose_pub = self.create_publisher(PoseStamped, 'ee_pose', 10)
        self.timer = self.create_timer(0.5, self.publish_ee_pose)

    def publish_ee_pose(self):
        if not self.moveit2.new_joint_state_available:
            self.get_logger().warn("Waiting for joint states...")
            return

        fk_pose = self.moveit2.compute_fk()

        if fk_pose is None:
            self.get_logger().warn("Failed to compute FK for EE pose.")
            return

        self.ee_pose_pub.publish(fk_pose)
        self.get_logger().info("Published EE pose.")

def main():
    rclpy.init()
    node = EndEffectorPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
