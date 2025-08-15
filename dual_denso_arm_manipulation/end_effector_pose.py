import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time
import tf2_ros

class EndEffectorPoseFetcher(Node):
    def __init__(self):
        super().__init__('end_effector_pose')

        # Initialize MoveIt2 for Left and Right Arm
        self.left_moveit = MoveIt2(
            node=self,
            joint_names=[f"left_joint_{i}" for i in range(1, 7)],
            base_link_name="left_base_link",
            end_effector_name="left_J6",
            group_name="left_arm"
        )

        self.right_moveit = MoveIt2(
            node=self,
            joint_names=[f"right_joint_{i}" for i in range(1, 7)],
            base_link_name="right_base_link",
            end_effector_name="right_J6", 
            group_name="right_arm"
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.right_offset = self.get_right_arm_offset()

        self.left_ee_pose_pub = self.create_publisher(Pose, 'left_ee_pose', 10)
        self.right_ee_pose_pub = self.create_publisher(Pose, 'right_ee_pose', 10)

        # Timer to periodically fetch and publish end effector poses
        # self.create_timer(1.0, self.get_and_publish_end_effector_poses)

        self.get_and_publish_end_effector_poses()
        

    def get_and_publish_end_effector_poses(self):
        # Compute FK for left and right arm end effectors
        left_fk_pose = self.left_moveit.compute_fk(fk_link_names=["left_J6"])  
        right_fk_pose = self.right_moveit.compute_fk(fk_link_names=["right_J6"])  

        if left_fk_pose:
            self.get_logger().info(f"[FK] Found Left EE Pose")
        else:
            self.get_logger().info(f"[FK] Left EE Pose not computed yet...")
        if right_fk_pose:
            self.get_logger().info(f"[FK] Found Right EE Pose")
        else:
            self.get_logger().info(f"[FK] Right EE Pose not computed yet...")


        if left_fk_pose and right_fk_pose:
            left_pose_msg = Pose()
            left_pose_msg = left_fk_pose[0].pose

            right_pose_msg = Pose()
            right_pose_msg = right_fk_pose[0].pose
            right_pose_msg.position.x += self.right_offset[0]
            right_pose_msg.position.y += self.right_offset[1]
            right_pose_msg.position.z += self.right_offset[2]

            # Publish the *global* end effector poses
            self.left_ee_pose_pub.publish(left_pose_msg)
            self.right_ee_pose_pub.publish(right_pose_msg)

            # Log the published poses
            self.get_logger().info(f"Published Left EE Pose")
            self.get_logger().info(f"Published Right EE Pose")
        else:
            self.get_logger().warn("Could not compute FK for one or both arms.")

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


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorPoseFetcher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
