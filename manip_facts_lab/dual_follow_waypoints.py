#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
from ament_index_python.packages import get_package_share_directory
from pymoveit2 import MoveIt2

# TODO: Fix so that it uses poses not joint angles, probably requires per-arm controllers, not a single one...


class DualArmWaypointPlanner(Node):
    def __init__(self):
        super().__init__('dual_arm_waypoints_node')

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                f"left_joint_{i}" for i in range(1, 7)
            ] + [
                f"right_joint_{i}" for i in range(1, 7)
            ],
            base_link_name="world",
            end_effector_name="right_J6",  # Arbitrary, planning group is dual_arm
            group_name="dual_arm",
        )

        self.get_logger().info("Initialized MoveIt2 with dual_arm joint group")

        # Load parameter
        self.declare_parameter("csv_filename", "")
        base_filename = self.get_parameter("csv_filename").get_parameter_value().string_value

        if not base_filename:
            self.get_logger().error(
                "\n\nMissing required parameter 'csv_filename'.\n"
                "Run with:\n"
                "  ros2 run manip_facts_lab dual_arm_waypoints_node --ros-args -p csv_filename:=waypoints_0\n"
            )
            rclpy.shutdown()
            return

        # Remove .csv extension if included
        if base_filename.endswith(".csv"):
            base_filename = base_filename[:-4]

        pkg_path = get_package_share_directory('manip_facts_lab')
        waypoints_dir = os.path.join(pkg_path, 'waypoints')

        left_file = os.path.join(waypoints_dir, f"{base_filename}_left.csv")
        right_file = os.path.join(waypoints_dir, f"{base_filename}_right.csv")
        fallback_file = os.path.join(waypoints_dir, f"{base_filename}.csv")

        if os.path.exists(left_file) and os.path.exists(right_file):
            self.get_logger().info(f"Found left: {left_file}")
            self.get_logger().info(f"Found right: {right_file}")
        else:
            self.get_logger().warn(
                f"\n[WARN] One or both of {base_filename}_left.csv, {base_filename}_right.csv not found — "
                f"falling back to: {base_filename}.csv for both arms\n"
            )
            left_file = fallback_file
            right_file = fallback_file

        self.load_and_execute_waypoints(left_file, right_file)

    def parse_row(self, row):
        try:
            values = [float(v) for v in row]
            if len(values) == 3:
                values += [0.0, 0.0, 0.0]
            elif len(values) != 6:
                return None
            return values
        except ValueError:
            return None

    def load_and_execute_waypoints(self, left_path, right_path):
        try:
            with open(left_path, newline='') as left_csv, open(right_path, newline='') as right_csv:
                left_reader = list(csv.reader(left_csv))
                right_reader = list(csv.reader(right_csv))

                n = min(len(left_reader), len(right_reader))
                for i in range(n):
                    left_row = self.parse_row(left_reader[i])
                    right_row = self.parse_row(right_reader[i])
                    if left_row is None or right_row is None:
                        self.get_logger().warn(f"Skipping invalid row {i + 1}")
                        continue

                    full_joints = left_row + right_row
                    self.get_logger().info(f"Moving to dual-arm waypoint {i + 1}...")
                    self.moveit2.move_to_configuration(full_joints)
                    success = self.moveit2.wait_until_executed()
                    if success:
                        self.get_logger().info(f"Arrived at waypoint {i + 1}")
                    else:
                        self.get_logger().warn(f"Failed to reach waypoint {i + 1}")
        except FileNotFoundError as e:
            self.get_logger().error(f"File not found: {e.filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to load or execute waypoints: {str(e)}")


def main():
    rclpy.init()
    node = DualArmWaypointPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
