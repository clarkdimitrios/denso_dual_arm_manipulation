#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2
import math
import transforms3d
import csv
from ament_index_python.packages import get_package_share_directory
import os


class WaypointPlanner(Node):
    def __init__(self):
        super().__init__('waypoints_node')

        self.declare_parameter('namespace', '')  # expected: left_, right_
        ns = self.get_parameter('namespace').get_parameter_value().string_value
        joint_names = [f"{ns}joint_{i}" for i in range(1, 7)]

        if not ns:
            self.get_logger().warn(
                "\n\nParameter 'namespace_prefix' is empty — "
                "using default joint names without prefix.\n"
                "If you are running with a namespaced robot (e.g., left_ or right_),\n"
                "make sure to set this parameter with:\n"
                "  ros2 run dual_denso_arm_manipulation waypoints_optim_node --ros-args -p namespace:=<prefix>\n"
            )

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name=f"{ns}base_link",
            end_effector_name=f"{ns}J6",
            group_name=f"{ns}arm",
        )

        self.get_logger().info(f"Initialized MoveIt2 with joints: {joint_names}")

        self.declare_parameter("csv_filename", "")
        csv_filename = self.get_parameter("csv_filename").get_parameter_value().string_value

        if not csv_filename:
            self.get_logger().error(
                "\n\nMissing required parameter 'csv_filename'.\n"
                "Please run with:\n"
                "  ros2 run dual_denso_arm_manipulation waypoints_node --ros-args -p csv_filename:=waypoints_0.csv\n"
            )
            rclpy.shutdown()
            return

        # Add .csv extension if missing
        if not csv_filename.endswith(".csv"):
            csv_filename += ".csv"

        pkg_path = get_package_share_directory('dual_denso_arm_manipulation')
        csv_path = os.path.join(pkg_path, 'waypoints', csv_filename)
        self.get_logger().info(f"Loading waypoints from: {csv_path}")
        self.execute_waypoints_from_csv(csv_path)

    def pose_from_row(self, row):
        x, y, z = float(row[0]), float(row[1]), float(row[2])
        roll_deg, pitch_deg, yaw_deg = float(row[3]), float(row[4]), float(row[5])

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

        return pose

    def execute_waypoints_from_csv(self, filename):
        try:
            with open(filename, newline='') as csvfile:
                reader = csv.reader(csvfile)
                for i, row in enumerate(reader):
                    if len(row) == 3:
                        row += ['0', '0', '0']
                    if len(row) != 6:
                        self.get_logger().warn(f"Skipping invalid row {i + 1}: {row}")
                        continue
                    pose = self.pose_from_row(row)
                    self.get_logger().info(f"Moving to waypoint {i + 1}...")
                    self.moveit2.move_to_pose(pose)
                    self.get_logger().info(f"Sending pose command using joints: {self.moveit2.joint_names}")
                    success = self.moveit2.wait_until_executed()
                    if success:
                        self.get_logger().info(f"Arrived at waypoint {i + 1}")
                    else:
                        self.get_logger().warn(f"Failed to reach waypoint {i + 1}")
        except FileNotFoundError:
            self.get_logger().error(f"CSV file '{filename}' not found.")


def main():
    rclpy.init()
    node = WaypointPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
