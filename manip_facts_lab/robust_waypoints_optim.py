#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2
import math
import transforms3d
import csv
import numpy as np
from stlpy.STL import STLMonitor
from ament_index_python.packages import get_package_share_directory
import os

from scripts.robust_specs import load_specs


class RobustWaypointOptimizer(Node):
    def __init__(self, moveit2, num_samples_per_waypoint=5):
        super().__init__('robust_waypoints_node')
        self.moveit2 = moveit2
        self.num_samples = num_samples_per_waypoint

    def sample_ik_solutions(self, pose):
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
        return ik_solutions

    def evaluate_robustness(self, trajectory, spec):
        monitor = STLMonitor(spec)
        trace = {'t': list(range(len(trajectory))), 'x': [q[0] for q in trajectory]}
        return monitor.robustness(trace)

    def optimize_waypoint_sequence(self, waypoints):
        ik_samples_per_waypoint = [self.sample_ik_solutions(pose) for pose in waypoints]
        num_waypoints = len(ik_samples_per_waypoint)
        cost_table = [{} for _ in range(num_waypoints)]

        for idx, q in enumerate(ik_samples_per_waypoint[0]):
            cost_table[0][idx] = {'cost': 0.0, 'prev': None, 'q': q}

        for i in range(1, num_waypoints):
            for idx_curr, q_curr in enumerate(ik_samples_per_waypoint[i]):
                min_cost = float('inf')
                min_prev_idx = None
                for idx_prev, data_prev in cost_table[i - 1].items():
                    q_prev = data_prev['q']
                    cost = data_prev['cost'] + np.linalg.norm(q_curr - q_prev)
                    if cost < min_cost:
                        min_cost = cost
                        min_prev_idx = idx_prev
                cost_table[i][idx_curr] = {'cost': min_cost, 'prev': min_prev_idx, 'q': q_curr}

        final_idx = min(cost_table[-1], key=lambda idx: cost_table[-1][idx]['cost'])
        optimal_path = []
        for i in reversed(range(num_waypoints)):
            q = cost_table[i][final_idx]['q']
            optimal_path.append(q)
            final_idx = cost_table[i][final_idx]['prev']

        optimal_path.reverse()
        return optimal_path


class RobustWaypointPlanner(Node):
    def __init__(self):
        super().__init__('robust_waypoint_planner')
        self.declare_parameter('csv_filename', '')
        csv_filename = self.get_parameter('csv_filename').get_parameter_value().string_value
        pkg_path = get_package_share_directory('manip_facts_lab')
        csv_path = os.path.join(pkg_path, 'waypoints', csv_filename)

        if not csv_filename:
            self.get_logger().error(
                "Missing required parameter 'csv_filename'. Run with:\n"
                "ros2 run manip_facts_lab robust_waypoints_node --ros-args -p csv_filename:=waypoints_0.csv"
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

        self.optimizer = RobustWaypointOptimizer(self.moveit2)

        self.get_logger().info(f"Loading waypoints from {csv_filename}...")
        waypoints = self.load_waypoints_from_csv(csv_filename)

        if waypoints:
            specs = load_specs(len(waypoints))
            joint_path = self.optimizer.optimize_waypoint_sequence(waypoints)

            for idx, spec in enumerate(specs):
                robustness = self.optimizer.evaluate_robustness(joint_path, spec)
                self.get_logger().info(f"[Spec {idx + 1}] Robustness: {robustness}")

            for idx, q in enumerate(joint_path):
                self.get_logger().info(f"Moving to optimized waypoint {idx + 1}...")
                self.moveit2.move_to_configuration(q.tolist())
                self.moveit2.wait_until_executed()
                self.get_logger().info(f"Arrived at waypoint {idx + 1}")

    def load_waypoints_from_csv(self, filename):
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
                    waypoints.append(self.pose_from_row(row))
            return waypoints
        except FileNotFoundError:
            self.get_logger().error(f"CSV file '{filename}' not found.")
            return []

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


def main():
    rclpy.init()
    node = RobustWaypointPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
