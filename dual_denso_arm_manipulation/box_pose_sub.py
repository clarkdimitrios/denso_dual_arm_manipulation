import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Pose, Point
from tf2_ros import TransformException
from dual_denso_arm_manipulation.utils import quat_to_euler_deg

class BoxPoseSubscriber(Node):
    def __init__(self):
        super().__init__('box_pose_sub')

        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('box_frame', 'lift_box') # should make this work for multiple objects later

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Dictionary to store publishers for each detected box frame
        self.box_publishers = {}

        self.create_timer(1.0, self.get_and_publish_poses)

    def get_and_publish_poses(self):
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        box_frame = self.get_parameter('box_frame').get_parameter_value().string_value
        
        try:
            transform = self.tf_buffer.lookup_transform(base_frame, box_frame, rclpy.time.Time())

            pose_msg = Pose()
            pose_msg.position = pose_msg.position = Point(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                z=transform.transform.translation.z
            )
            pose_msg.orientation = transform.transform.rotation

            topic_name = f'pose_{box_frame}'
            if box_frame not in self.box_publishers:
                self.box_publishers[box_frame] = self.create_publisher(Pose, topic_name, 10)

            self.box_publishers[box_frame].publish(pose_msg)

            rpy = quat_to_euler_deg(pose_msg.orientation)
            self.get_logger().info(f"Published {box_frame} Pose: \nPosition = x:{pose_msg.position.x} y:{pose_msg.position.y} z:{pose_msg.position.z}, "
                                   f"\nOrientation = roll:{rpy[0]} pitch:{rpy[1]} yaw:{rpy[2]}"
                                   )

        except TransformException as ex:
            self.get_logger().warn(f"Could not get transform for {box_frame}: {ex}")


def main(args=None):
    rclpy.init(args=args)
    node = BoxPoseSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
