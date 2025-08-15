#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration

from geometry_msgs.msg import Pose
from std_msgs.msg import String
import csv
from dual_denso_arm_manipulation.utils import quat_to_euler_deg
from ament_index_python.packages import get_package_share_directory
import os
import tf2_ros
from copy import deepcopy

class TrajectoryToCSV(Node):
    def __init__(self):
        super().__init__('traj_to_csv')

        self.left_ee_traj_sub = self.create_subscription(Pose, 'left_ee_trajectory', self.left_ee_callback, 10)
        self.right_ee_traj_sub = self.create_subscription(Pose, 'right_ee_trajectory', self.right_ee_callback, 10)
        self.state_sub = self.create_subscription(String, 'trajectory_state', self.state_callback, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Buffers
        self.left_trajectory = []
        self.right_trajectory = []

        self.prev_state = None
        self.state = None
        self.saved_current_state = False
        self.last_msg_time = None

        self.declare_parameter('flush_timeout_sec', 1.0)  # inactivity to auto-save current state
        self.declare_parameter('require_both_arms', True) # set False to save if only one arm sent data

        self.right_offset = self.get_right_arm_offset()

        # Inactivity watchdog
        self.create_timer(0.2, self._maybe_auto_save)

        self.get_logger().info("traj_to_csv ready. Publish 'trajectory_state' = 'grab' or 'lift' BEFORE streaming poses.")

    # ---------- Callbacks ----------
    def state_callback(self, state_msg: String):
        new_state = state_msg.data.strip().lower()
        if new_state not in ('grab', 'lift'):
            self.get_logger().warn(f"Ignoring unknown state '{new_state}'. Use 'grab' or 'lift'.")
            return

        self.get_logger().info(f"[STATE] {self.state} -> {new_state}")

        # Save the state we’re leaving (if it had data)
        if self.state in ('grab', 'lift') and (self.left_trajectory or self.right_trajectory):
            self._save_current_state(self.state)

        # Start new state
        self.left_trajectory.clear()
        self.right_trajectory.clear()
        self.prev_state = self.state
        self.state = new_state
        self.saved_current_state = False
        self.last_msg_time = None

    def left_ee_callback(self, pose_msg: Pose):
        if self.state not in ('grab', 'lift'):
            return
        self.left_trajectory.append(self.pose_to_list(pose_msg))
        self.last_msg_time = self.get_clock().now()

    def right_ee_callback(self, pose_msg: Pose):
        if self.state not in ('grab', 'lift'):
            return
        pose = deepcopy(pose_msg)
        pose.position.x -= self.right_offset[0]
        pose.position.y -= self.right_offset[1]
        pose.position.z -= self.right_offset[2]
        self.right_trajectory.append(self.pose_to_list(pose))
        self.last_msg_time = self.get_clock().now()

    # ---------- Helpers ----------
    def pose_to_list(self, pose: Pose):
        roll, pitch, yaw = quat_to_euler_deg(pose.orientation)  # degrees
        return [pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw]
        # return [pose.position.x, pose.position.y, pose.position.z, 0.0, 0.0, 0.0] # debugging

    def get_right_arm_offset(self):
        self.get_logger().info("Waiting for TF: world -> right_base_link (up to 5s)")
        deadline = self.get_clock().now() + Duration(seconds=5.0)
        while self.get_clock().now() < deadline:
            try:
                trans = self.tf_buffer.lookup_transform(
                    'world', 'right_base_link', RclpyTime(), timeout=Duration(seconds=0.1)
                )
                t = trans.transform.translation
                self.get_logger().info(f"[TF] Right offset: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")
                return (t.x, t.y, t.z)
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().warn("[TF] Could not get TF; assuming (0,0,0)")
        return (0.0, 0.0, 0.0)

    def _save_current_state(self, state_name: str):
        if state_name not in ('grab', 'lift'):
            return

        require_both = bool(self.get_parameter('require_both_arms').value)
        if require_both and (not self.left_trajectory or not self.right_trajectory):
            self.get_logger().warn(f"[SAVE] Skipped {state_name}: waiting for both arms (set require_both_arms:=false to override).")
            return

        tmp_path = os.path.join(get_package_share_directory('dual_denso_arm_manipulation'), 'temp')
        os.makedirs(tmp_path, exist_ok=True)

        left_file = os.path.join(tmp_path, f'{state_name}_left.csv')
        right_file = os.path.join(tmp_path, f'{state_name}_right.csv')

        with open(left_file, 'w', newline='') as f:
            w = csv.writer(f)
            for r in self.left_trajectory:  w.writerow(r)

        with open(right_file, 'w', newline='') as f:
            w = csv.writer(f)
            for r in self.right_trajectory: w.writerow(r)

        self.get_logger().info(f"[SAVE] {state_name}: left -> {left_file}")
        self.get_logger().info(f"[SAVE] {state_name}: right -> {right_file}")
        self.saved_current_state = True

    def _maybe_auto_save(self):
        """Auto-save the current state after inactivity (no poses) for flush_timeout_sec."""
        if self.state not in ('grab', 'lift') or self.saved_current_state:
            return
        if self.last_msg_time is None:
            return
        timeout = float(self.get_parameter('flush_timeout_sec').value)
        if (self.get_clock().now() - self.last_msg_time) > Duration(seconds=timeout):
            self.get_logger().info(f"[AUTO-SAVE] No new poses for {timeout:.2f}s; saving '{self.state}'.")
            self._save_current_state(self.state)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToCSV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort save on shutdown if current state has data and wasn't saved yet.
        try:
            if node.state in ('grab','lift') and not node.saved_current_state and (node.left_trajectory or node.right_trajectory):
                node._save_current_state(node.state)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
