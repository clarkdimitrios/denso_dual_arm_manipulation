#!/usr/bin/env python3

import os
import csv
import time
from math import radians, degrees
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient

import tf2_ros
import transforms3d
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pymoveit2 import MoveIt2


class DualArmCartesianPlanner(Node):
    def __init__(self):
        super().__init__('dual_cartesian_waypoints_node')

        # Parameters / paths
        self.declare_parameter("shutdown_after_execute", True)
        self.declare_parameter("csv_filename", "")
        self.declare_parameter("allow_non_cart_lift", False) 
        self.pkg_path = get_package_share_directory('dual_denso_arm_manipulation')
        self.default_waypoints_dir = os.path.join(self.pkg_path, 'waypoints')

        self.allow_non_cart_lift = self.get_parameter("allow_non_cart_lift") \
                                       .get_parameter_value().bool_value
        
        self.declare_parameter("cartesian_required_fraction", 0.99)
        self.declare_parameter("cartesian_step_schedule", [0.010, 0.012, 0.015, 0.020, 0.025])
        self.declare_parameter("cartesian_jump_schedule", [0.0, 5.0, 10.0, 20.0])
        self.declare_parameter("cartesian_z_nudges_m", [0.0, 0.002, -0.002, 0.004, -0.004])

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # MoveIt interfaces
        self.left_moveit = MoveIt2(
            node=self,
            joint_names=[f"left_joint_{i}" for i in range(1, 7)],
            base_link_name="left_base_link",
            end_effector_name="left_J6",
            group_name="left_arm",
        )
        self.right_moveit = MoveIt2(
            node=self,
            joint_names=[f"right_joint_{i}" for i in range(1, 7)],
            base_link_name="right_base_link",
            end_effector_name="right_J6",
            group_name="right_arm",
        )
        self.dual_moveit = MoveIt2(
            node=self,
            joint_names=[f"left_joint_{i}" for i in range(1, 7)] +
                        [f"right_joint_{i}" for i in range(1, 7)],
            base_link_name="world",
            end_effector_name="right_J6",  # unused, arbitrary
            group_name="dual_arm",
        )

        # Cartesian path service client (same service for both groups)
        self.cart_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')

        # ExecuteTrajectory action client (for executing merged dual trajectories)
        self.exec_action = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

        # Joint state monitor (ensure robot state settles after motion)
        self._js = {}
        self.create_subscription(JointState, 'joint_states', self._js_cb, 50)

        self.linker_cmd_pub = self.create_publisher(String, 'ee_linker_cmd', 10)
        self.attach_before_segment = self.declare_parameter('attach_before_segment', 2).value

        # internal flag so we only attach once
        self._linker_attached = False

        self.right_offset = None 
        self._is_lift_mode = False

        self.get_logger().info(f"DualArmCartesianPlanner ready. allow_non_cart_lift={self.allow_non_cart_lift}")

    # ---------- Blocking runner ----------
    def run_once_blocking(self):
        raw_csv = self.get_parameter("csv_filename").get_parameter_value().string_value.strip()
        if not raw_csv:
            self.get_logger().error("No CSV base name provided (param 'csv_filename').")
            return

        csv_base = raw_csv[:-4] if raw_csv.lower().endswith(".csv") else raw_csv

        waypoints_dir = self.default_waypoints_dir
        if csv_base in ('grab', 'lift'):
            waypoints_dir = os.path.join(self.pkg_path, 'temp')

        left_file = os.path.join(waypoints_dir, f"{csv_base}_left.csv")
        right_file = os.path.join(waypoints_dir, f"{csv_base}_right.csv")
        fallback_file = os.path.join(waypoints_dir, f"{csv_base}.csv")

        # Wait for CSVs to appear (ROS executor spins in background)
        while rclpy.ok():
            if os.path.exists(left_file) and os.path.exists(right_file):
                self.get_logger().info(f"[INFO] Using separate files:\n  Left:  {left_file}\n  Right: {right_file}")
                break
            elif os.path.exists(fallback_file) and csv_base not in ('grab', 'lift'):
                self.get_logger().warn(
                    f"[WARN] One or both _left/_right missing. Falling back to: {fallback_file} for BOTH arms."
                )
                left_file = fallback_file
                right_file = fallback_file
                break
            else:
                self.get_logger().info(
                    f"Waiting for CSVs under: {waypoints_dir}\n"
                    f"Expected either both of:\n  {csv_base}_left.csv\n  {csv_base}_right.csv\n"
                    f"or fallback:\n  {csv_base}.csv"
                )
                time.sleep(5.0)

        # Mark lift mode (enables Cartesian straight-line vertical motion)
        self._is_lift_mode = (os.path.basename(left_file).startswith('lift') or csv_base == 'lift')
        self.get_logger().info(f"[Mode] Lift mode = {self._is_lift_mode}")

        # TF offset
        self.right_offset = self.get_right_arm_offset()

        # Execute
        self.execute_cartesian_waypoints(left_file, right_file)

    # ---------- Helpers ----------
    def _js_cb(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self._js[n] = p

    def _wait_for_robot(self, names, target, tol=0.01, timeout=3.0):
        deadline = self.get_clock().now() + Duration(seconds=timeout)
        while self.get_clock().now() < deadline:
            if self._js and all(abs(self._js.get(n, 1e9) - v) <= tol for n, v in zip(names, target)):
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def get_right_arm_offset(self):
        """Get translation from world -> right_base_link from TF."""
        self.get_logger().info("Waiting for TF transform: world -> right_base_link (up to 5s)")
        deadline = self.get_clock().now() + Duration(seconds=5.0)
        while self.get_clock().now() < deadline:
            try:
                trans = self.tf_buffer.lookup_transform(
                    "world", "right_base_link", RclpyTime(), timeout=Duration(seconds=0.1)
                )
                t = trans.transform.translation
                self.get_logger().info(f"[TF] Right arm offset: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")
                return (t.x, t.y, t.z)
            except Exception:
                time.sleep(0.05)
        self.get_logger().warn("[TF] Could not get TF for right_base_link — assuming (0,0,0)")
        return (0.0, 0.0, 0.0)

    def _positions_in_order(self, joint_state, expected_names):
        """Return positions ordered to match expected_names. None if any expected name missing."""
        name_to_pos = {n: p for n, p in zip(joint_state.name, joint_state.position)}
        ordered = []
        for n in expected_names:
            if n not in name_to_pos:
                self.get_logger().error(f"[IK] Expected joint '{n}' not in JointState: {joint_state.name}")
                return None
            ordered.append(name_to_pos[n])
        return ordered

    def load_csv_with_padding(self, filename):
        """Load CSV and ensure each row has XYZRPY (6 values)."""
        rows = []
        try:
            with open(filename, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if not row:
                        continue
                    # Allow comments / headers
                    if isinstance(row[0], str) and row[0].strip().startswith('#'):
                        continue
                    try:
                        values = list(map(float, row))
                    except ValueError:
                        self.get_logger().warn(f"[CSV] Skipping non-numeric row in {filename}: {row}")
                        continue
                    if len(values) == 3:
                        values.extend([0.0, 0.0, 0.0])  # Pad with RPY=0
                    elif len(values) != 6:
                        self.get_logger().warn(f"[CSV] Skipping invalid row in {filename}: {row}")
                        continue
                    rows.append(values)
        except FileNotFoundError:
            self.get_logger().warn(f"[CSV] File not found (yet): {filename}")
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

    def _fmt_pose(self, p: Pose):
        """Nice XYZRPY(deg) string for logs."""
        wxyz = (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
        r, pt, yw = transforms3d.euler.quat2euler(wxyz, axes='sxyz')
        return (f"XYZ=({p.position.x:.3f},{p.position.y:.3f},{p.position.z:.3f}) "
                f"RPY(deg)=({degrees(r):.1f},{degrees(pt):.1f},{degrees(yw):.1f})")
    
    def _linker_cmd(self, cmd: str, sleep_sec: float = 0.1):
        self.get_logger().info(f"[LinkerCmd] -> {cmd}")
        self.linker_cmd_pub.publish(String(data=cmd))
        if sleep_sec > 0:
            time.sleep(sleep_sec)


    # ---------- Core execution ----------
    def execute_cartesian_waypoints(self, left_file, right_file):
        left_rows = self.load_csv_with_padding(left_file)
        right_rows = self.load_csv_with_padding(right_file)

        n_steps = min(len(left_rows), len(right_rows))
        if n_steps == 0:
            self.get_logger().error("[ERROR] No valid waypoints found")
            return

        left_order  = [f"left_joint_{k}"  for k in range(1, 7)]
        right_order = [f"right_joint_{k}" for k in range(1, 7)]

        # --- LIFT mode: try Cartesian for ALL rows, sequentially ---
        start_i = 0
        if getattr(self, "_is_lift_mode", False) and n_steps >= 1:
            self.get_logger().info(f"[Cartesian] Lift mode active → attempting straight-line segments for {n_steps} waypoint(s).")

            for i in range(n_steps):
                lx, ly, lz, lrx, lry, lrz = left_rows[i]
                rx, ry, rz, rrx, rry, rrz = right_rows[i]
                left_target  = self.xyzrpy_to_pose(lx, ly, lz, lrx, lry, lrz)
                right_target = self.xyzrpy_to_pose(rx, ry, rz, rrx, rry, rrz)

                self.get_logger().info(f"[Cartesian] Segment {i+1}/{n_steps}")
                self.get_logger().info(f"[Cartesian]   Left  target: {self._fmt_pose(left_target)}")
                self.get_logger().info(f"[Cartesian]   Right target: {self._fmt_pose(right_target)}")

                ok = self._execute_dual_cartesian_lift(left_target, right_target, left_order, right_order)
                if not ok:
                    if not getattr(self, "allow_non_cart_lift", True):
                        self.get_logger().error(
                            f"[Cartesian] Segment {i+1} failed and allow_non_cart_lift:=false → aborting without IK fallback."
                        )
                        return
                    self.get_logger().warn(
                        f"[Cartesian] Segment {i+1} failed; falling back to step-wise IK from this segment onward."
                    )
                    start_i = i  # fallback IK will start from the failing row
                    break

                if (i + 2) == int(self.attach_before_segment) and not self._linker_attached and n_steps > (i + 1):
                    self.get_logger().info("[Linker] Attaching box now (before next LIFT segment)...")
                    # publish to ee_box_linker, which calls /ATTACHLINK internally
                    self._linker_cmd('attach', sleep_sec=0.1)
                    self._linker_attached = True
            else:
                # All Cartesian segments succeeded
                self.get_logger().info("[Cartesian] All lift segments completed with Cartesian paths. Done.")
                return

        # --- IK path (normal mode OR fallback after partial Cartesian) ---
        left_order  = [f"left_joint_{k}"  for k in range(1, 7)]
        right_order = [f"right_joint_{k}" for k in range(1, 7)]

        for i in range(start_i, n_steps):
            lx, ly, lz, lrx, lry, lrz = left_rows[i]
            rx, ry, rz, rrx, rry, rrz = right_rows[i]

            self.get_logger().info(f"\n===== Step {i+1}/{n_steps} =====")
            self.get_logger().info(f"[Left CSV Local]  XYZRPY=({lx:.3f}, {ly:.3f}, {lz:.3f}, {lrx:.1f}, {lry:.1f}, {lrz:.1f})")
            self.get_logger().info(f"[Right CSV Local] XYZRPY=({rx:.3f}, {ry:.3f}, {rz:.3f}, {rrx:.1f}, {rry:.1f}, {rrz:.1f})")

            # For reference logs only (planning uses local coords)
            rx_world = rx + self.right_offset[0]
            ry_world = ry + self.right_offset[1]
            rz_world = rz + self.right_offset[2]
            self.get_logger().info(f"[Right Offset Applied] WORLD XYZ=({rx_world:.3f}, {ry_world:.3f}, {rz_world:.3f})")

            left_pose  = self.xyzrpy_to_pose(lx, ly, lz, lrx, lry, lrz)
            right_pose = self.xyzrpy_to_pose(rx, ry, rz, rrx, rry, rrz)  # right is local to right_base_link

            # Keep trying until both IKs succeed
            self.get_logger().info("[IK] Searching for a valid pair of IK solutions (will keep trying)...")
            left_pos, right_pos = self._solve_both_arms_until_success(left_pose, right_pose, left_order, right_order)

            if left_pos is None or right_pos is None:
                self.get_logger().error("[IK] Aborted due to shutdown while searching for IK.")
                return

            self.get_logger().info(f"[IK] Left  arm joints: {left_pos}")
            self.get_logger().info(f"[IK] Right arm joints: {right_pos}")

            dual_goal = left_pos + right_pos

            self.get_logger().info("[Move] Executing sync move for both arms")
            try:
                self.dual_moveit.move_to_configuration(dual_goal)
                self.dual_moveit.wait_until_executed()
            except Exception as e:
                self.get_logger().error(f"[Move] Execution failed: {e}")
                # Optional: retry execution here if you want

    # ---------- Cartesian lift (dual, merged) ----------
    def _execute_dual_cartesian_lift(self, left_target: Pose, right_target: Pose,
                                    left_order, right_order) -> bool:
        """
        Try hard to get a vertical, straight-line Cartesian lift for BOTH arms.
        It retries with (1) tiny Z nudges, (2) a step-size schedule, (3) jump-threshold schedule.
        Succeeds only when BOTH arms achieve the required fraction (~1.0).
        """
        self.get_logger().info("[Cartesian] Planning straight-line lift for both arms...")

        # Service/action availability
        if not self.cart_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("[Cartesian] compute_cartesian_path service not available.")
            return False
        if not self.exec_action.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("[Cartesian] execute_trajectory action server not available.")
            return False

        # Current EE poses via TF (start waypoints)
        left_start  = self._current_tip_pose('left_base_link', 'left_J6')
        right_start = self._current_tip_pose('right_base_link', 'right_J6')
        if left_start is None or right_start is None:
            self.get_logger().warn("[Cartesian] Could not get current EE poses; aborting Cartesian mode.")
            return False

        self.get_logger().info(f"[Cartesian] Left  start: {self._fmt_pose(left_start.pose)}")
        self.get_logger().info(f"[Cartesian] Right start: {self._fmt_pose(right_start.pose)}")
        self.get_logger().info(f"[Cartesian] Left  target: {self._fmt_pose(left_target)}")
        self.get_logger().info(f"[Cartesian] Right target: {self._fmt_pose(right_target)}")

        # --- Retry strategy ---
        required_fraction = getattr(self, "cartesian_required_fraction", 0.99)
        step_schedule     = getattr(self, "cartesian_step_schedule", [0.010, 0.012, 0.015, 0.020, 0.025])
        jump_schedule     = getattr(self, "cartesian_jump_schedule", [0.0, 5.0, 10.0, 20.0])
        z_nudges          = getattr(self, "cartesian_z_nudges_m",  [0.0, +0.002, -0.002, +0.004, -0.004])

        # local helper: return a copy of pose with z nudged
        def _with_z(p: Pose, dz: float) -> Pose:
            out = Pose()
            out.position.x = p.position.x
            out.position.y = p.position.y
            out.position.z = p.position.z + float(dz)
            out.orientation = p.orientation
            return out

        attempt = 0
        best = {"left_frac": 0.0, "right_frac": 0.0, "combo": 0.0, "desc": ""}

        for dz in z_nudges:
            lt = _with_z(left_target, dz)
            rt = _with_z(right_target, dz)

            for step in step_schedule:
                for jump in jump_schedule:
                    attempt += 1
                    avoid_collisions = True  # we still want to avoid OTHER collisions

                    self.get_logger().info(
                        f"[Cartesian] Attempt {attempt}: dz={dz:+.3f} m, step={step:.3f} m, jump={jump:.1f}, "
                        f"avoid_collisions={avoid_collisions}"
                    )

                    # Left arm path
                    l_traj, l_fraction = self._compute_cartesian_path(
                        group='left_arm', base_link='left_base_link', tip_link='left_J6',
                        waypoints=[left_start.pose, lt],
                        max_step=step, jump_threshold=jump, avoid_collisions=avoid_collisions
                    )

                    # Right arm path
                    r_traj, r_fraction = self._compute_cartesian_path(
                        group='right_arm', base_link='right_base_link', tip_link='right_J6',
                        waypoints=[right_start.pose, rt],
                        max_step=step, jump_threshold=jump, avoid_collisions=avoid_collisions
                    )

                    # If either call failed outright
                    if l_traj is None or r_traj is None:
                        self.get_logger().warn("[Cartesian] compute_cartesian_path returned no trajectory for one or both arms.")
                        continue

                    nl = len(l_traj.joint_trajectory.points)
                    nr = len(r_traj.joint_trajectory.points)
                    combo = min(l_fraction, r_fraction)
                    self.get_logger().info(
                        f"[Cartesian] Fractions: left={l_fraction:.3f} (points={nl}), "
                        f"right={r_fraction:.3f} (points={nr})"
                    )

                    # Track best (for debugging)
                    if combo > best["combo"]:
                        best.update(left_frac=l_fraction, right_frac=r_fraction, combo=combo,
                                    desc=f"dz={dz:+.3f}, step={step:.3f}, jump={jump:.1f}, pts=({nl},{nr})")

                    # Accept only if both are near perfect
                    if l_fraction >= required_fraction and r_fraction >= required_fraction:
                        self.get_logger().info("[Cartesian] Found acceptable paths for BOTH arms. Merging...")

                        # Merge into dual trajectory
                        dual_traj = self._merge_joint_trajectories(
                            l_traj.joint_trajectory, left_order,
                            r_traj.joint_trajectory, right_order
                        )
                        if dual_traj is None or len(dual_traj.points) == 0:
                            self.get_logger().warn("[Cartesian] Failed to merge trajectories; continuing retries.")
                            continue

                        total_pts = len(dual_traj.points)
                        last_t = dual_traj.points[-1].time_from_start.sec \
                                + dual_traj.points[-1].time_from_start.nanosec * 1e-9
                        self.get_logger().info(
                            f"[Cartesian] Merged dual trajectory: points={total_pts}, duration={last_t:.3f}s"
                        )

                        # Execute via ExecuteTrajectory
                        robot_traj = RobotTrajectory()
                        robot_traj.joint_trajectory = dual_traj

                        self.get_logger().info("[Cartesian] Sending ExecuteTrajectory goal...")
                        goal = ExecuteTrajectory.Goal()
                        goal.trajectory = robot_traj

                        send_future = self.exec_action.send_goal_async(goal)
                        while rclpy.ok() and not send_future.done():
                            time.sleep(0.01)
                        goal_handle = send_future.result()
                        if not goal_handle or not goal_handle.accepted:
                            self.get_logger().error("[Cartesian] ExecuteTrajectory goal NOT accepted; continuing retries.")
                            continue

                        self.get_logger().info("[Cartesian] Goal accepted; waiting for result...")
                        result_future = goal_handle.get_result_async()
                        while rclpy.ok() and not result_future.done():
                            time.sleep(0.02)

                        result = result_future.result()
                        if result and result.result.error_code.val == 1:  # SUCCESS
                            self.get_logger().info("[Cartesian] Execution completed: SUCCEEDED")
                            # Wait for settle
                            names = left_order + right_order
                            last = dual_traj.points[-1].positions
                            if not self._wait_for_robot(names, last, tol=0.01, timeout=2.0):
                                self.get_logger().warn("[State] Joint state monitor not fully settled after Cartesian; brief pause.")
                                time.sleep(0.3)
                            return True
                        else:
                            code = None
                            if result and result.result and result.result.error_code:
                                code = result.result.error_code.val
                            self.get_logger().error(f"[Cartesian] Execution finished with non-success error code: {code}; continuing retries.")
                            # Continue retries

                    # brief pause so we don't hammer services
                    time.sleep(0.03)

        # If we reach here, no acceptable Cartesian plan was found
        self.get_logger().warn(
            "[Cartesian] Exhausted retries without achieving required fractions. "
            f"Best seen: left={best['left_frac']:.3f}, right={best['right_frac']:.3f} "
            f"({best['desc']})"
        )
        return False


    def _current_tip_pose(self, base_link: str, tip_link: str) -> PoseStamped:
        try:
            tf = self.tf_buffer.lookup_transform(base_link, tip_link, RclpyTime(), timeout=Duration(seconds=0.2))
            ps = PoseStamped()
            ps.header.frame_id = base_link
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = tf.transform.translation.x
            ps.pose.position.y = tf.transform.translation.y
            ps.pose.position.z = tf.transform.translation.z
            ps.pose.orientation = tf.transform.rotation
            return ps
        except Exception as e:
            self.get_logger().warn(f"[TF] current tip pose failed for {tip_link}: {e}")
            return None

    def _compute_cartesian_path(self, group: str, base_link: str, tip_link: str,
                                waypoints, max_step=0.01, jump_threshold=0.0, avoid_collisions=True):
        req = GetCartesianPath.Request()
        req.group_name = group
        req.link_name = tip_link
        req.header.frame_id = base_link
        req.waypoints = waypoints
        req.max_step = float(max_step)
        req.jump_threshold = float(jump_threshold)
        req.avoid_collisions = bool(avoid_collisions)

        self.get_logger().info(f"[Cartesian] Requesting path: group={group}, base={base_link}, tip={tip_link}, "
                               f"waypoints={len(waypoints)}, max_step={max_step:.3f}")

        future = self.cart_cli.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        resp = future.result()
        if resp is None:
            self.get_logger().warn("[Cartesian] Service call returned None.")
            return None, 0.0

        if resp.solution and resp.solution.joint_trajectory.points:
            end_pt = resp.solution.joint_trajectory.points[-1]
            self.get_logger().info(f"[Cartesian] {group} path points={len(resp.solution.joint_trajectory.points)}, "
                                   f"fraction={resp.fraction:.3f}, last_time={end_pt.time_from_start.sec + end_pt.time_from_start.nanosec*1e-9:.3f}s")
        else:
            self.get_logger().warn(f"[Cartesian] {group} path empty; fraction={resp.fraction:.3f}")
        return resp.solution, resp.fraction

    def _merge_joint_trajectories(self, jt_left: JointTrajectory, left_order,
                                  jt_right: JointTrajectory, right_order) -> JointTrajectory:
        # Collect unique time stamps
        def times(jt):
            return [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in jt.points]
        tL = times(jt_left)
        tR = times(jt_right)
        t_all = sorted(set(tL + tR))
        if not t_all:
            return None

        self.get_logger().info(f"[Cartesian] Merge: left_times={len(tL)}, right_times={len(tR)}, merged_times={len(t_all)}")

        # Helper: get last-known positions up to time t
        def positions_at(jt, order, t):
            idx = 0
            for i, p in enumerate(jt.points):
                tt = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                if tt <= t:
                    idx = i
                else:
                    break
            names = jt.joint_names
            point = jt.points[idx]
            name_to_pos = {n: v for n, v in zip(names, point.positions)}
            return [name_to_pos[n] for n in order]

        # Build dual trajectory
        dual = JointTrajectory()
        dual.joint_names = left_order + right_order

        for t in t_all:
            lp = positions_at(jt_left, left_order, t)
            rp = positions_at(jt_right, right_order, t)
            pt = JointTrajectoryPoint()
            pt.positions = lp + rp
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            pt.time_from_start.sec = sec
            pt.time_from_start.nanosec = nsec
            dual.points.append(pt)

        return dual

    # ---------- IK fallback ----------
    def _pose_with_adjustments(self, pose_in: Pose, dx=0.0, dy=0.0, dz=0.0, dyaw_deg=0.0) -> Pose:
        p = deepcopy(pose_in)
        p.position.x += dx
        p.position.y += dy
        p.position.z += dz
        q = (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)  # wxyz
        r, pitch, yaw = transforms3d.euler.quat2euler(q, axes='sxyz')
        yaw += radians(dyaw_deg)
        qw, qx, qy, qz = transforms3d.euler.euler2quat(r, pitch, yaw, axes='sxyz')
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = qw, qx, qy, qz
        return p

    def _solve_both_arms_until_success(self, left_pose: Pose, right_pose: Pose, left_order, right_order):
        yaw_variants_deg = [0.0, 180.0, -180.0, 5.0, -5.0]
        nudges = [
            (0.0, 0.0, 0.0),
            (0.00, 0.00,  0.01), (0.00, 0.00, -0.01),
            (0.01, 0.00,  0.00), (-0.01, 0.00, 0.00),
            (0.00, 0.01,  0.00), (0.00, -0.01, 0.00),
            (0.02, 0.00,  0.00), (-0.02, 0.00, 0.00),
            (0.00, 0.02,  0.00), (0.00, -0.02, 0.00),
            (0.00, 0.00,  0.02), (0.00, 0.00, -0.02),
        ]

        attempt = 0
        last_log = time.time()
        while rclpy.ok():
            for dyaw in yaw_variants_deg:
                for dx, dy, dz in nudges:
                    attempt += 1
                    lp = self._pose_with_adjustments(left_pose,  dx, dy, dz, dyaw)
                    rp = self._pose_with_adjustments(right_pose, dx, dy, dz, dyaw)

                    left_js = self.left_moveit.compute_ik(
                        position=(lp.position.x, lp.position.y, lp.position.z),
                        quat_xyzw=(lp.orientation.x, lp.orientation.y, lp.orientation.z, lp.orientation.w)
                    )
                    right_js = self.right_moveit.compute_ik(
                        position=(rp.position.x, rp.position.y, rp.position.z),
                        quat_xyzw=(rp.orientation.x, rp.orientation.y, rp.orientation.z, rp.orientation.w)
                    )

                    if left_js is not None and right_js is not None:
                        left_pos  = self._positions_in_order(left_js,  left_order)
                        right_pos = self._positions_in_order(right_js, right_order)
                        if left_pos is not None and right_pos is not None:
                            self.get_logger().info(f"[IK] Success after {attempt} attempts "
                                                   f"(yaw+={dyaw}°, nudge=({dx:.3f},{dy:.3f},{dz:.3f})).")
                            return left_pos, right_pos

                    now = time.time()
                    if now - last_log > 2.0:
                        self.get_logger().warn(f"[IK] Still searching... attempts={attempt}")
                        last_log = now
                    time.sleep(0.02)
        return None, None


def main(args=None):
    import threading

    rclpy.init(args=args)
    node = DualArmCartesianPlanner()

    shutdown_after_execute = node.get_parameter('shutdown_after_execute') \
                                 .get_parameter_value().bool_value

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_once_blocking()

        if shutdown_after_execute:
            node.get_logger().info("shutdown_after_execute:=true → shutting down after one run.")
        else:
            node.get_logger().info("shutdown_after_execute:=false → keeping node alive.")
            try:
                spin_thread.join()
                return
            except KeyboardInterrupt:
                pass

    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        try:
            spin_thread.join(timeout=1.0)
        except Exception:
            pass


if __name__ == '__main__':
    main()
