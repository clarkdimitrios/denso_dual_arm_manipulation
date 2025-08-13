import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import csv
from builtin_interfaces.msg import Time
from dual_denso_arm_manipulation.utils import quat_to_euler_deg
from ament_index_python.packages import get_package_share_directory
import os
import tf2_ros

class TrajectoryToCSV(Node):
    def __init__(self):
        super().__init__('traj_to_csv')

        # Subscribers for left and right end effector trajectories
        self.left_ee_sub = self.create_subscription(Pose, 'left_ee_trajectory', self.left_ee_callback, 10)
        self.right_ee_sub = self.create_subscription(Pose, 'right_ee_trajectory', self.right_ee_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Temp storage for the trajectories
        self.left_trajectory = []
        self.right_trajectory = []
        self.right_offset = self.get_right_arm_offset()


    def left_ee_callback(self, pose_msg):
        """Callback to store the left end effector trajectory"""
        self.left_trajectory.append(self.pose_to_list(pose_msg))

    def right_ee_callback(self, pose_msg):
        """Callback to store the right end effector trajectory"""
        pose = pose_msg
        # Subtract the offset to get the local position of the right arm
        pose.position.x -= self.right_offset[0]
        pose.position.y -= self.right_offset[1]
        pose.position.z -= self.right_offset[2]
        self.right_trajectory.append(self.pose_to_list(pose))

    def pose_to_list(self, pose):
        """Convert Pose to a list of position and orientation (in degrees)"""
        roll, pitch, yaw = quat_to_euler_deg(pose.orientation)
        return [pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw]


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

    def save_to_csv(self):
        """Save the trajectories to CSV files as 'lift_left.csv' and 'lift_right.csv'"""
        tmp_path = os.path.join(
            get_package_share_directory('dual_denso_arm_manipulation'),
            'temp',
        )

        os.makedirs(tmp_path, exist_ok=True)

        left_file_path = os.path.join(tmp_path, 'lift_left.csv')
        right_file_path = os.path.join(tmp_path, 'lift_right.csv')

        with open(left_file_path, 'w', newline='') as left_file:
            writer = csv.writer(left_file)
            # writer.writerow(['x', 'y', 'z', 'roll', 'pitch', 'yaw']) 
            for row in self.left_trajectory:
                writer.writerow(row)
        self.get_logger().info(f"Saved left trajectory to {left_file_path}")

        with open(right_file_path, 'w', newline='') as right_file:
            writer = csv.writer(right_file)
            # writer.writerow(['x', 'y', 'z', 'roll', 'pitch', 'yaw']) 
            for row in self.right_trajectory:
                writer.writerow(row)
        self.get_logger().info(f"Saved right trajectory to {right_file_path}")

        return left_file_path, right_file_path

    def process_trajectories(self):
        """Process the received trajectories"""
        if len(self.left_trajectory) > 0 and len(self.right_trajectory) > 0:

            left_file, right_file = self.save_to_csv()

            self.get_logger().info(f"Processing completed. Left trajectory file: {left_file}, Right trajectory file: {right_file}")
        else:
            self.get_logger().warn("Waiting for both trajectories to be received.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToCSV()

    def check_and_save():
        if len(node.left_trajectory) > 0 and len(node.right_trajectory) > 0:
            node.save_to_csv()
            node.get_logger().info("Trajectory files saved, shutting down...")
            rclpy.shutdown()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            check_and_save()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
if __name__ == '__main__':
    main()
