#!/usr/bin/env python3
import time
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String

from moveit_msgs.msg import (
    PlanningScene,
    AllowedCollisionMatrix,
    AllowedCollisionEntry,
    PlanningSceneComponents,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetPlanningScene,
)


class CollisionManager(Node):
    """
    Robust ACM manager:
      - Listens to 'trajectory_state' ("grab" / "lift")
      - On 'lift': allows collisions between {box_id} and {left_tip,right_tip}
        via ApplyPlanningScene; falls back to publishing on /planning_scene if needed
      - Verifies result and then publishes 'planning_scene_ready' := 'lift_ready'
    """

    def __init__(self):
        super().__init__("collision_manager")

        # ---------------- Params ----------------
        self.declare_parameter("box_id", "lift_box")
        self.declare_parameter("left_tip", "left_J6")
        self.declare_parameter("right_tip", "right_J6")

        # Service timeouts / retries
        self.declare_parameter("service_wait_timeout_sec", 10.0)     # wait for service to exist
        self.declare_parameter("call_timeout_sec", 2.0)              # per RPC call timeout
        self.declare_parameter("pub_fallback_retries", 3)            # /planning_scene fallback publish count
        self.declare_parameter("verify_after_apply", True)           # verify ACM after applying

        self.box_id    = str(self.get_parameter("box_id").value)
        self.left_tip  = str(self.get_parameter("left_tip").value)
        self.right_tip = str(self.get_parameter("right_tip").value)

        self.service_wait_timeout_sec = float(self.get_parameter("service_wait_timeout_sec").value)
        self.call_timeout_sec         = float(self.get_parameter("call_timeout_sec").value)
        self.pub_fallback_retries     = int(self.get_parameter("pub_fallback_retries").value)
        self.verify_after_apply       = bool(self.get_parameter("verify_after_apply").value)

        # State in from trajectory generator
        self.state_sub = self.create_subscription(
            String, "trajectory_state", self._state_cb, 10
        )

        # Planning scene diff publisher (topic fallback)
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        # Latched ACK for lift readiness
        ack_qos = QoSProfile(depth=1)
        ack_qos.reliability = ReliabilityPolicy.RELIABLE
        ack_qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
        ack_qos.history     = HistoryPolicy.KEEP_LAST
        self.ready_pub = self.create_publisher(String, "planning_scene_ready", ack_qos)

        # MoveIt service clients
        self.apply_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get_cli   = self.create_client(GetPlanningScene, "/get_planning_scene")

        self._wait_for_service(self.get_cli,   "/get_planning_scene",   self.service_wait_timeout_sec)
        self._wait_for_service(self.apply_cli, "/apply_planning_scene", self.service_wait_timeout_sec)

        self.get_logger().info(
            f"[ACM] Ready. box_id='{self.box_id}', "
            f"gripper_links=['{self.left_tip}', '{self.right_tip}']"
        )

    # ---------------- Callbacks ----------------
    def _state_cb(self, msg: String):
        state = msg.data.strip().lower()
        if state not in ("grab", "lift"):
            self.get_logger().warn(f"[ACM] Ignoring unknown state '{state}' (use 'grab' or 'lift').")
            return

        if state == "grab":
            # Default scene usually already disallows touching the box; nothing to do.
            self.get_logger().info("[ACM] 'grab' received → keeping default collisions (no touching box).")
            return

        # state == "lift" → allow touching box
        self.get_logger().info("[ACM] 'lift' received → allowing collisions with the box for both tips.")
        ok = self._allow_box_touch([self.left_tip, self.right_tip], self.box_id)
        if ok:
            self._publish_lift_ready()
        else:
            self.get_logger().error("[ACM] Failed to apply ACM for lift; NOT publishing lift_ready.")

    # ---------------- Core ACM logic ----------------
    def _allow_box_touch(self, tip_links: List[str], box_id: str) -> bool:
        # Fetch current ACM (may be empty the first time)
        scene = self._fetch_scene()
        if scene is None:
            self.get_logger().error("[ACM] GetPlanningScene timed out / no result.")
            # Continue with an empty ACM so we can still try to apply a diff
            acm = AllowedCollisionMatrix()
        else:
            acm = scene.allowed_collision_matrix

        # Build new ACM that allows collisions between box and the tip links
        new_acm = self._acm_allow_pairs(
            acm_in=acm,
            pairs=[(box_id, tl) for tl in tip_links]
        )

        # Apply via service first
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.allowed_collision_matrix = new_acm

        if self._apply_scene(scene_msg):
            self.get_logger().info("[ACM] ApplyPlanningScene success.")
            applied = True
        else:
            self.get_logger().warn("[ACM] ApplyPlanningScene failed; falling back to /planning_scene topic.")
            applied = self._publish_scene_diff(scene_msg, retries=self.pub_fallback_retries)

        # Optional verification
        if applied and self.verify_after_apply:
            ver = self._verify_pairs_allowed(box_id, tip_links)
            if not ver:
                self.get_logger().error("[ACM] Verification failed; ACM change not reflected in scene.")
                return False

        return applied

    # ---------------- Utilities: Build/Apply/Verify ----------------
    def _acm_allow_pairs(self, acm_in: AllowedCollisionMatrix,
                         pairs: List[tuple]) -> AllowedCollisionMatrix:
        """
        Return a new AllowedCollisionMatrix with the union of existing entries +
        any names in 'pairs', where each pair (a,b) is set to allowed both ways.
        """
        # Current names → index
        names: List[str] = list(acm_in.entry_names) if acm_in.entry_names else []
        idx: Dict[str, int] = {n: i for i, n in enumerate(names)}

        # Expand with any new names from pairs
        for a, b in pairs:
            if a not in idx:
                idx[a] = len(names)
                names.append(a)
            if b not in idx:
                idx[b] = len(names)
                names.append(b)

        N = len(names)

        # Start with a dense False matrix
        mat = [[False]*N for _ in range(N)]

        # Copy over existing flags where indices/names still exist
        if acm_in.entry_names and acm_in.entry_values:
            old_idx = {n: i for i, n in enumerate(acm_in.entry_names)}
            for old_row_name, old_row in zip(acm_in.entry_names, acm_in.entry_values):
                if old_row_name not in idx:
                    continue
                i_new = idx[old_row_name]
                # old_row.enabled aligns with old entry_names order
                for old_j, old_col_name in enumerate(acm_in.entry_names):
                    if old_col_name not in idx:
                        continue
                    j_new = idx[old_col_name]
                    val = bool(old_row.enabled[old_j]) if old_j < len(old_row.enabled) else False
                    mat[i_new][j_new] = val

        # Enforce symmetry & diagonal False
        for i in range(N):
            mat[i][i] = False

        # Enable new allowed pairs (both directions)
        for a, b in pairs:
            ia, ib = idx[a], idx[b]
            mat[ia][ib] = True
            mat[ib][ia] = True

        # Pack back into AllowedCollisionMatrix
        out = AllowedCollisionMatrix()
        out.entry_names = names
        out.entry_values = []
        for i in range(N):
            row = AllowedCollisionEntry()
            row.enabled = mat[i]
            out.entry_values.append(row)
        return out

    def _apply_scene(self, scene_msg: PlanningScene) -> bool:
        if not self.apply_cli.service_is_ready():
            self.get_logger().warn("[ACM] /apply_planning_scene service not ready.")
            return False

        req = ApplyPlanningScene.Request()
        req.scene = scene_msg
        future = self.apply_cli.call_async(req)

        ok = self._spin_until_future(future, self.call_timeout_sec)
        if not ok:
            return False

        res = future.result()
        return bool(res and getattr(res, "success", True))

    def _publish_scene_diff(self, scene_msg: PlanningScene, retries: int = 3) -> bool:
        scene_msg.is_diff = True
        for i in range(max(1, retries)):
            self.scene_pub.publish(scene_msg)
            time.sleep(0.1)
        return True

    def _fetch_scene(self) -> PlanningScene:
        if not self.get_cli.service_is_ready():
            return None
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.get_cli.call_async(req)
        ok = self._spin_until_future(future, self.call_timeout_sec)
        if not ok:
            return None
        res = future.result()
        return getattr(res, "scene", None)

    def _verify_pairs_allowed(self, box_id: str, tip_links: List[str]) -> bool:
        scene = self._fetch_scene()
        if scene is None:
            self.get_logger().warn("[ACM] Could not verify: GetPlanningScene returned no scene.")
            return False

        names = list(scene.allowed_collision_matrix.entry_names or [])
        values = list(scene.allowed_collision_matrix.entry_values or [])
        if not names or not values:
            self.get_logger().warn("[ACM] Could not verify: ACM empty.")
            return False

        def allowed(a: str, b: str) -> bool:
            if a not in names or b not in names:
                return False
            ia, ib = names.index(a), names.index(b)
            row_a = values[ia].enabled
            row_b = values[ib].enabled
            return (ib < len(row_a) and bool(row_a[ib])) and (ia < len(row_b) and bool(row_b[ia]))

        ok = True
        for tl in tip_links:
            if allowed(box_id, tl):
                self.get_logger().info(f"[ACM] Verified: '{box_id}' allowed with '{tl}'.")
            else:
                self.get_logger().error(f"[ACM] NOT allowed after apply: '{box_id}' with '{tl}'.")
                ok = False
        return ok

    def _publish_lift_ready(self):
        msg = String()
        msg.data = "lift_ready"
        self.ready_pub.publish(msg)
        self.get_logger().info("[ACM] Published 'lift_ready' (latched).")

    # ---------------- Helpers ----------------
    def _wait_for_service(self, client, name: str, timeout_sec: float):
        self.get_logger().info(f"[ACM] Waiting for service {name} (up to {timeout_sec:.1f}s)...")
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and self.get_clock().now() < deadline:
            if client.wait_for_service(timeout_sec=0.2):
                self.get_logger().info(f"[ACM] Service {name} is ready.")
                return
        self.get_logger().warn(f"[ACM] Service {name} not ready; will use topic fallback if needed.")

    def _spin_until_future(self, future, timeout_sec: float) -> bool:
        # Non-blocking-ish: step the executor to process the RPC, but do not hang forever
        deadline = time.time() + max(0.01, timeout_sec)
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                return True
        return future.done()


def main(args=None):
    rclpy.init(args=args)
    node = CollisionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
