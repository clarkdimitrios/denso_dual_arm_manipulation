#!/usr/bin/env python3
import os
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion

from ament_index_python.packages import get_package_share_directory
from dual_denso_arm_manipulation.utils import (
    get_xacro_properties, 
    rot_to_quat_xyzw, 
    quat_xyzw_to_rot, 
    rotate_vec_by_quat,
)


class LiftTrajGen(Node):
    def __init__(self):
        super().__init__('lift_traj_generator')

        # ---------------- Params ----------------
        self.declare_parameter('lift_height', 0.5)
        self.declare_parameter('box_frame', 'lift_box')
        self.declare_parameter('vertical_tol', 0.2)      # |n·z| <= tol → vertical face
        self.declare_parameter('approach_offset', 0.05)   # stand-off along face normal (meters)

        self.lift_height     = float(self.get_parameter('lift_height').value)
        self.box_frame       = str(self.get_parameter('box_frame').value)
        self.vertical_tol    = float(self.get_parameter('vertical_tol').value)
        self.approach_offset = float(self.get_parameter('approach_offset').value)

        # Topics
        topic_name = f'pose_{self.box_frame}'

        # ---------------- IO ----------------
        self.obj_sub     = self.create_subscription(Pose, topic_name,        self.obj_callback,    10)
        self.left_ee_sub = self.create_subscription(Pose, 'left_ee_pose',    self.left_ee_callback, 10)
        self.right_ee_sub= self.create_subscription(Pose, 'right_ee_pose',   self.right_ee_callback,10)

        self.left_ee_trajectory_pub  = self.create_publisher(Pose, 'left_ee_trajectory',  10)
        self.right_ee_trajectory_pub = self.create_publisher(Pose, 'right_ee_trajectory', 10)
        self.state_pub               = self.create_publisher(String, 'trajectory_state',  10)

        # Planning-scene readiness (ACM/attach) ACK
        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
        ready_qos.history     = HistoryPolicy.KEEP_LAST

        self.lift_ready = False
        self.ready_sub  = self.create_subscription(String, 'planning_scene_ready',
                                                   self._ready_callback, ready_qos)

        # ---------------- State ----------------
        self.obj_pose     = None
        self.left_ee_pose = None
        self.right_ee_pose= None

        self.get_logger().info("LiftTrajGen ready: waiting for box & EE poses.")

    # ========== Callbacks ==========
    def obj_callback(self, pose_msg: Pose):
        if self.obj_pose is None:
            self.obj_pose = pose_msg
            self.get_logger().info("Received Box Pose")

        if self.obj_pose and self.left_ee_pose and self.right_ee_pose:
            self.get_logger().info("All poses received, generating trajectories.")

            size_xyz = self.get_box_size()
            faces = self._box_faces_world(self.obj_pose, size_xyz)

            # Build grabs: nearest vertical face to each EE, center of that face
            left_grab_pose  = self.closest_vertical_side_center_pose(self.left_ee_pose,  faces)
            right_grab_pose = self.closest_vertical_side_center_pose(self.right_ee_pose, faces)

            # Optional stand-off
            if self.approach_offset != 0.0:
                left_grab_pose  = self._offset_along_y(left_grab_pose,  self.approach_offset)   # back off along -Y (ee frame)
                right_grab_pose = self._offset_along_y(right_grab_pose, self.approach_offset)

            # Lifts: add Z in world
            left_lift_pose  = self.create_lift_pose(left_grab_pose)
            right_lift_pose = self.create_lift_pose(right_grab_pose)

            self.create_and_publish_trajectory(left_grab_pose, right_grab_pose,
                                               left_lift_pose, right_lift_pose)

    def left_ee_callback(self, pose_msg: Pose):
        if self.left_ee_pose is None:
            self.left_ee_pose = pose_msg
            self.get_logger().info("Received Left EE Pose")

    def right_ee_callback(self, pose_msg: Pose):
        if self.right_ee_pose is None:
            self.right_ee_pose = pose_msg
            self.get_logger().info("Received Right EE Pose")

    def _ready_callback(self, msg: String):
        if msg.data.strip().lower() == 'lift_ready':
            self.lift_ready = True

    # ========== Helpers ==========
    def wait_for_lift_ready(self, timeout_sec=5.0) -> bool:
        self.lift_ready = False
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.lift_ready:
                return True
        return False

    def create_and_publish_trajectory(self, left_grab_pose: Pose, right_grab_pose: Pose,
                                      left_lift_pose: Pose, right_lift_pose: Pose):
        # ---- Phase 1: GRAB (both arms under 'grab') ----
        self.state_pub.publish(String(data='grab'))
        time.sleep(0.05)
        self.publish_trajectory(left_grab_pose,  'left')
        self.publish_trajectory(right_grab_pose, 'right')

        # ---- Phase 2: LIFT (both arms under 'lift'; wait for ACM/attach ack) ----
        self.state_pub.publish(String(data='lift'))
        time.sleep(0.05)
        if not self.wait_for_lift_ready(timeout_sec=5.0):
            self.get_logger().warn("Timeout waiting for planning scene ready; proceeding anyway.")
        for i in range(2):
            self.publish_trajectory(left_lift_pose[i],  'left')
            self.publish_trajectory(right_lift_pose[i], 'right')

    def publish_trajectory(self, goal_pose: Pose, arm_side: str):
        if arm_side == 'left':
            self.left_ee_trajectory_pub.publish(goal_pose)
            self.get_logger().info("Published Left EE Trajectory")
        else:
            self.right_ee_trajectory_pub.publish(goal_pose)
            self.get_logger().info("Published Right EE Trajectory")

    def create_lift_pose(self, grab_pose: Pose, contact_depth=0.0):
        # Hold box
        p1 = Pose()
        p1 = self._offset_along_y(grab_pose, -self.approach_offset * (1 + contact_depth)) # revert offset + depth

        # Lift box
        p2 = Pose()
        p2.position.x = p1.position.x
        p2.position.y = p1.position.y
        p2.position.z = p1.position.z + self.lift_height
        p2.orientation = p1.orientation
        return p1, p2

    def get_box_size(self):
        """Read box size from scene_config.xacro; assumes link origin at box center."""
        config_path = os.path.join(
            get_package_share_directory('dual_denso_arm_manipulation'),
            'config', 'scene_config.xacro'
        )
        try:
            props = get_xacro_properties(config_path)
        except Exception as e:
            self.get_logger().error(f"Failed to read scene_config.xacro: {e}")
            return (0.5, 0.5, 0.5)

        lx = float(props.get(f'{self.box_frame}_size_x', '0.5'))
        ly = float(props.get(f'{self.box_frame}_size_y', '0.5'))
        lz = float(props.get(f'{self.box_frame}_size_z', '0.5'))
        return (lx, ly, lz)

    # ========== Geometry (world-frame faces & orientations) ==========
    def closest_vertical_side_center_pose(self, ee_pose: Pose, faces: list) -> Pose:
        ee = np.array([ee_pose.position.x, ee_pose.position.y, ee_pose.position.z])
        face = self._pick_vertical_face(faces, ee, tol=self.vertical_tol)
        p = Pose()
        p.position.x, p.position.y, p.position.z = face['center'].tolist()
        p.orientation = self._quat_align_y_to(face['normal'])
        return p

    def _box_faces_world(self, obj_pose: Pose, size_xyz):
        """Return lateral faces (center, outward normal) in WORLD frame."""
        lx, ly, lz = size_xyz
        # Local corners around origin (centered box)
        pts_local = np.array([
            [-lx/2,-ly/2,-lz/2],
            [ lx/2,-ly/2,-lz/2],
            [ lx/2, ly/2,-lz/2],
            [-lx/2, ly/2,-lz/2],
            [-lx/2,-ly/2, lz/2],
            [ lx/2,-ly/2, lz/2],
            [ lx/2, ly/2, lz/2],
            [-lx/2, ly/2, lz/2],
        ])
        R = quat_xyzw_to_rot(obj_pose.orientation)
        t = np.array([obj_pose.position.x, obj_pose.position.y, obj_pose.position.z])
        t = t + R @ np.array([0.0, 0.0, lz/2.0]) # Since the box is not through the ground
        pts_world = (R @ pts_local.T).T + t

        # Four lateral faces (consistent winding for outward normal)
        faces_idx = [
            [0,1,5,4],  # +X
            [1,2,6,5],  # +Y
            [2,3,7,6],  # -X
            [3,0,4,7],  # -Y
        ]
        faces = []
        for idx in faces_idx:
            P = pts_world[idx]
            # normal via cross of two edges
            n = np.cross(P[1]-P[0], P[2]-P[1])
            n_norm = np.linalg.norm(n)
            if n_norm < 1e-12:
                continue
            n = n / n_norm
            c = P.mean(axis=0)
            # ensure outward wrt center t
            if np.dot(n, c - t) < 0:
                n = -n
            faces.append({'center': c, 'normal': n})
        return faces

    def _pick_vertical_face(self, faces: list, ee_pos_world: np.ndarray,
                            z_hat=np.array([0,0,1]), tol=0.2):
        """Pick a vertical face: |n·z| small; among candidates choose nearest to EE.
           If none within tol, pick the most vertical one."""
        if not faces:
            # Should not happen; fallback
            return {'center': np.zeros(3), 'normal': np.array([1,0,0])}

        scores = [abs(np.dot(f['normal'], z_hat)) for f in faces]
        candidates = [f for f, s in zip(faces, scores) if s <= tol]
        if not candidates:
            candidates = [faces[int(np.argmin(scores))]]
        face = min(candidates, key=lambda f: np.linalg.norm(f['center'] - ee_pos_world))
        return face


    def _offset_along_y(self, pose: Pose, dist: float) -> Pose:
        """Move pose along its local +Y axis by 'dist' (meters)."""
        y_axis_world = rotate_vec_by_quat(np.array([0.0, 1.0, 0.0]), pose.orientation)
        p = Pose()
        p.position.x = pose.position.x + dist * y_axis_world[0]
        p.position.y = pose.position.y + dist * y_axis_world[1]
        p.position.z = pose.position.z + dist * y_axis_world[2]
        p.orientation = pose.orientation
        return p

    def _quat_align_y_to(self, normal: np.ndarray, up_hint=np.array([0,0,1])) -> Quaternion:
        """Build rotation with Y-axis along 'normal' and Z-axis close to 'up_hint'."""
        y = normal / (np.linalg.norm(normal) + 1e-12)
        z = up_hint - np.dot(up_hint, y) * y
        if np.linalg.norm(z) < 1e-6:
            up_hint = np.array([1,0,0])
            z = up_hint - np.dot(up_hint, y) * y
        z /= (np.linalg.norm(z) + 1e-12)
        x = np.cross(y, z)
        x /= (np.linalg.norm(x) + 1e-12)
        R = np.column_stack((x, y, z))  # columns are axes in world
        return rot_to_quat_xyzw(R)


def main(args=None):
    rclpy.init(args=args)
    node = LiftTrajGen()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
