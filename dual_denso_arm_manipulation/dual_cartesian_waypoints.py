#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
from math import radians
import transforms3d
from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2
from ament_index_python.packages import get_package_share_directory


class DualArmCartesianPlanner(Node):
    def __init__(self):
        super().__init__('dual_cartesian_waypoints_node')

        # Left arm MoveIt interface
        self.left_moveit = MoveIt2(
            node=self,
            joint_names=[f"left_joint_{i}" for i in range(1, 7)],
            base_link_name="left_base_link",
            end_effector_name="left_J6",
            group_name="left_arm"
        )

        # Right arm MoveIt interface
        self.right_moveit = MoveIt2(
            node=self,
            joint_names=[f"right_joint_{i}" for i in range(1, 7)],
            base_link_name="right_base_link",
            end_effector_name="right_J6",
            group_name="right_arm"
        )

        # Load CSV base name from parameter
        self.declare_parameter("csv_filename", "")
        raw_csv = self.get_parameter("csv_filename").get_parameter_value().string_value.strip()
        if not raw_csv:
            self.get_logger().error("No CSV base name provided")
            return

        # Remove extension if provided
        if raw_csv.lower().endswith(".csv"):
            csv_base = raw_csv[:-4]
        else:
            csv_base = raw_csv

        # Find package waypoints dir
        pkg_path = get_package_share_directory('dual_denso_arm_manipulation')
        waypoints_dir = os.path.join(pkg_path, 'waypoints')

        # Build file paths
        left_file = os.path.join(waypoints_dir, f"{csv_base}_left.csv")
        right_file = os.path.join(waypoints_dir, f"{csv_base}_right.csv")
        fallback_file = os.path.join(waypoints_dir, f"{csv_base}.csv")

        # Check if both _left and _right exist
        if os.path.exists(left_file) and os.path.exists(right_file):
            self.get_logger().info(f"[INFO] Using separate files:\n  Left:  {left_file}\n  Right: {right_file}")
        else:
            self.get_logger().warn(
                f"[WARN] One or both of _left/_right CSV files not found. "
                f"Falling back to: {fallback_file} for BOTH arms."
            )
            left_file = fallback_file
            right_file = fallback_file

        # Run execution
        self.execute_cartesian_waypoints(left_file, right_file)

    def execute_cartesian_waypoints(self, left_file, right_file):
        left_rows = self.load_csv_with_padding(left_file)
        right_rows = self.load_csv_with_padding(right_file)

        n_steps = min(len(left_rows), len(right_rows))
        if n_steps == 0:
            self.get_logger().error("No valid waypoints found")
            return

        for i in range(n_steps):
            lx, ly, lz, lrx, lry, lrz = left_rows[i]
            rx, ry, rz, rrx, rry, rrz = right_rows[i]

            left_pose = self.xyzrpy_to_pose(lx, ly, lz, lrx, lry, lrz)
            right_pose = self.xyzrpy_to_pose(rx, ry, rz, rrx, rry, rrz)

            self.get_logger().info(f"[Step {i+1}/{n_steps}] Moving both arms")
            self.left_moveit.move_to_pose(left_pose)
            self.right_moveit.move_to_pose(right_pose)

    def load_csv_with_padding(self, filename):
        """Load CSV and ensure each row has XYZRPY (6 values)."""
        rows = []
        try:
            with open(filename, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if not row:
                        continue
                    values = list(map(float, row))
                    if len(values) == 3:
                        values.extend([0.0, 0.0, 0.0])  # Pad with RPY=0
                    elif len(values) != 6:
                        self.get_logger().warn(f"Skipping invalid row in {filename}: {row}")
                        continue
                    rows.append(values)
        except FileNotFoundError:
            self.get_logger().error(f"File not found: {filename}")
        return rows

    def xyzrpy_to_pose(self, x, y, z, roll, pitch, yaw):
        quat = transforms3d.euler.euler2quat(
            radians(roll),
            radians(pitch),
            radians(yaw),
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


def main(args=None):
    rclpy.init(args=args)
    node = DualArmCartesianPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
