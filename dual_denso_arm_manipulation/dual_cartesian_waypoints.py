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
import tf2_ros
from builtin_interfaces.msg import Time

# NOTE: Cartesian mode fails easily because any waypoint can be loaded, make sure it's kinematically feasible, or IK computation fails.

class DualArmCartesianPlanner(Node):
    def __init__(self):
        super().__init__('dual_cartesian_waypoints_node')

        # TF Buffer + Listener for coordinates transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

        # Dual-arm MoveIt interface (for sync execution)
        self.dual_moveit = MoveIt2(
            node=self,
            joint_names=[f"left_joint_{i}" for i in range(1, 7)] +
                        [f"right_joint_{i}" for i in range(1, 7)],
            base_link_name="world",
            end_effector_name="right_J6", # unused, arbitrary
            group_name="dual_arm"
        )

        self.declare_parameter("csv_filename", "")
        raw_csv = self.get_parameter("csv_filename").get_parameter_value().string_value.strip()
        if not raw_csv:
            self.get_logger().error("No CSV base name provided")
            return

        csv_base = raw_csv[:-4] if raw_csv.lower().endswith(".csv") else raw_csv

        pkg_path = get_package_share_directory('dual_denso_arm_manipulation')
        waypoints_dir = os.path.join(pkg_path, 'waypoints')

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

        # Get right arm offset from TF
        self.right_offset = self.get_right_arm_offset()

        self.execute_cartesian_waypoints(left_file, right_file)

    def get_right_arm_offset(self):
        """Get translation from world -> right_base_link from TF."""
        self.get_logger().info("Waiting for TF transform: world -> right_base_link")
        for i in range(50):  # Wait up to ~5 seconds
            try:
                trans = self.tf_buffer.lookup_transform(
                    "world", "right_base_link", Time(), timeout=rclpy.duration.Duration(seconds=0.1)
                )
                t = trans.transform.translation
                self.get_logger().info(f"[TF] Right arm offset from TF: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")
                return (t.x, t.y, t.z)
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn("[TF] Could not get TF for right_base_link — assuming (0,0,0)")
        return (0.0, 0.0, 0.0)

    def execute_cartesian_waypoints(self, left_file, right_file):
        left_rows = self.load_csv_with_padding(left_file)
        right_rows = self.load_csv_with_padding(right_file)

        n_steps = min(len(left_rows), len(right_rows))
        if n_steps == 0:
            self.get_logger().error("[ERROR] No valid waypoints found")
            return

        for i in range(n_steps):
            lx, ly, lz, lrx, lry, lrz = left_rows[i]
            rx, ry, rz, rrx, rry, rrz = right_rows[i]

            self.get_logger().info(f"\n===== Step {i+1}/{n_steps} =====")
            self.get_logger().info(f"[Left CSV Local]  XYZRPY=({lx:.3f}, {ly:.3f}, {lz:.3f}, {lrx:.1f}, {lry:.1f}, {lrz:.1f})")
            self.get_logger().info(f"[Right CSV Local] XYZRPY=({rx:.3f}, {ry:.3f}, {rz:.3f}, {rrx:.1f}, {rry:.1f}, {rrz:.1f})")

            # Apply TF offset to right arm (only for logs)
            rx_world = rx + self.right_offset[0]
            ry_world = ry + self.right_offset[1]
            rz_world = rz + self.right_offset[2]
            self.get_logger().info(f"[Right Offset Applied] XYZ=({rx_world:.3f}, {ry_world:.3f}, {rz_world:.3f})")

            left_pose = self.xyzrpy_to_pose(lx, ly, lz, lrx, lry, lrz)
            # right_pose = self.xyzrpy_to_pose(rx_world, ry_world, rz_world, rrx, rry, rrz)
            right_pose = self.xyzrpy_to_pose(rx, ry, rz, rrx, rry, rrz) #moveit actually uses local coords

            # Compute IK (returns all joints anyway... but separated for clarity)
            left_joints = self.left_moveit.compute_ik(
                position=(left_pose.position.x, left_pose.position.y, left_pose.position.z),
                quat_xyzw=(left_pose.orientation.x, left_pose.orientation.y,
                        left_pose.orientation.z, left_pose.orientation.w)
            )
            right_joints = self.right_moveit.compute_ik(
                position=(right_pose.position.x, right_pose.position.y, right_pose.position.z),
                quat_xyzw=(right_pose.orientation.x, right_pose.orientation.y,
                        right_pose.orientation.z, right_pose.orientation.w)
            )

            # Log IK results
            if left_joints is None:
                self.get_logger().warn("[IK] Left arm: NO SOLUTION")
            else:
                self.get_logger().info(f"[IK] Left arm joints: {list(left_joints.position[:6])}")

            if right_joints is None:
                self.get_logger().warn("[IK] Right arm: NO SOLUTION")
            else:
                self.get_logger().info(f"[IK] Right arm joints: {list(right_joints.position[6:])}")

            if left_joints is None or right_joints is None:
                self.get_logger().warn(f"[SKIP] Step {i+1}: IK failed for one or both arms")
                continue

            # Build combined dual-arm goal (now in joint space aka angles)
            dual_goal = list(left_joints.position[:6]) + list(right_joints.position[6:])

            self.get_logger().info("[Move] Executing sync move for both arms")

            # Execute and wait for completion
            self.dual_moveit.move_to_configuration(dual_goal)
            self.dual_moveit.wait_until_executed()



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
                        self.get_logger().warn(f"[CSV] Skipping invalid row in {filename}: {row}")
                        continue
                    rows.append(values)
        except FileNotFoundError:
            self.get_logger().error(f"[CSV] File not found: {filename}")
        return rows

    def xyzrpy_to_pose(self, x, y, z, roll, pitch, yaw):
        quat = transforms3d.euler.euler2quat(
            radians(roll), radians(pitch), radians(yaw), axes='sxyz'
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
