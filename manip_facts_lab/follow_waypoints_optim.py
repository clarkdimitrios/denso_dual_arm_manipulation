#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2
import math
import transforms3d
import csv
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os


class WaypointOptimizer:
    def __init__(self, moveit2, node, num_samples=10):
        self.moveit2 = moveit2
        self.node = node
        self.num_samples = num_samples

    def sample_ik_solutions(self, pose, i=0):
        ik_solutions = []
        for _ in range(self.num_samples):
            q_init = np.random.uniform(-1, 1, len(self.moveit2.joint_names)).tolist()
            q_sol = self.moveit2.compute_ik(
                position=pose.position,
                quat_xyzw=(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                ),
                start_joint_state=q_init,
            )
            if q_sol is not None:
                ik_solutions.append(np.array(q_sol.position))

        if ik_solutions:
            self.node.get_logger().info(f"Waypoint {i+1}: Found {len(ik_solutions)} IK solutions out of {self.num_samples} samples.")
        else:
            self.node.get_logger().warn(f"Waypoint {i+1}: No IK solutions found after {self.num_samples} attempts for this waypoint!")

        return ik_solutions


    def optimize_sequence(self, waypoints):
        ik_samples = [self.sample_ik_solutions(p, i) for i, p in enumerate(waypoints)]
        if any(len(s) == 0 for s in ik_samples):
            self.node.get_logger().error("One or more waypoints have no IK solutions. Aborting.")
            return []

        cost_table = [{} for _ in ik_samples]
        for idx, q in enumerate(ik_samples[0]):
            cost_table[0][idx] = {'cost': 0.0, 'prev': None, 'q': q}

        for i in range(1, len(ik_samples)):
            for idx_curr, q_curr in enumerate(ik_samples[i]):
                min_cost = float('inf')
                min_prev = None
                for idx_prev, entry in cost_table[i - 1].items():
                    cost = entry['cost'] + np.linalg.norm(q_curr - entry['q'])
                    if cost < min_cost:
                        min_cost = cost
                        min_prev = idx_prev
                cost_table[i][idx_curr] = {'cost': min_cost, 'prev': min_prev, 'q': q_curr}

        last_idx = min(cost_table[-1], key=lambda idx: cost_table[-1][idx]['cost'])
        path = []
        for i in reversed(range(len(ik_samples))):
            path.append(cost_table[i][last_idx]['q'])
            last_idx = cost_table[i][last_idx]['prev']
        path.reverse()
        return path


class WaypointOptimizerNode(Node):
    def __init__(self):
        super().__init__('waypoint_optim_node')

        self.declare_parameter('csv_filename', '')
        csv_filename = self.get_parameter('csv_filename').get_parameter_value().string_value
        pkg_path = get_package_share_directory('manip_facts_lab')
        csv_path = os.path.join(pkg_path, 'waypoints', csv_filename)

        if not csv_filename:
            self.get_logger().error(
                "\n\nMissing required parameter 'csv_filename'.\n"
                "Please run with:\n"
                "  ros2 run manip_facts_lab waypoints_optim_node --ros-args -p csv_filename:=<filename>\n"
            )
            rclpy.shutdown()
            return
        
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

        optimizer = WaypointOptimizer(self.moveit2, self)
        waypoints = self.load_waypoints(csv_path)
        if waypoints:
            q_sequence = optimizer.optimize_sequence(waypoints)
            for idx, q in enumerate(q_sequence):
                self.get_logger().info(f"Moving to optimized waypoint {idx + 1}...")
                self.moveit2.move_to_configuration(q.tolist())
                success = self.moveit2.wait_until_executed()
                if success:
                    self.get_logger().info(f"Arrived at optimized waypoint {idx + 1}")
                else:
                    self.get_logger().warn(f"Failed to reach waypoint {idx + 1}")

    def load_waypoints(self, filename):
        waypoints = []
        try:
            with open(filename, newline='') as csvfile:
                reader = csv.reader(csvfile)
                for i, row in enumerate(reader):
                    if len(row) not in [3, 6]:
                        self.get_logger().warn(f"Skipping invalid row {i + 1}: {row}")
                        continue
                    while len(row) < 6:
                        row.append('0')
                    waypoints.append(self.row_to_pose(row))
            return waypoints
        except FileNotFoundError:
            self.get_logger().error(f"CSV file '{filename}' not found.")
            return []

    def row_to_pose(self, row):
        x, y, z = float(row[0]), float(row[1]), float(row[2])
        roll, pitch, yaw = map(math.radians, map(float, row[3:6]))
        quat = transforms3d.euler.euler2quat(roll, pitch, yaw, axes='sxyz')
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]
        pose.orientation.w = quat[0]
        return pose


def main():
    rclpy.init()
    node = WaypointOptimizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
