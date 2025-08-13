import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from dual_denso_arm_manipulation.utils import get_xacro_properties, euler_to_quat
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np
from math import degrees

class LiftTrajGen(Node):
    def __init__(self):
        super().__init__('lift_traj_generator')

        self.declare_parameter('lift_height', 2.0)
        self.lift_height = self.get_parameter('lift_height').get_parameter_value().double_value

        self.declare_parameter('box_frame', 'lift_box')
        self.box_frame = self.get_parameter('box_frame').get_parameter_value().string_value

        topic_name = f'pose_{self.box_frame}'

        self.obj_sub = self.create_subscription(Pose, topic_name, self.obj_callback, 10)
        self.left_ee_sub = self.create_subscription(Pose, 'left_ee_pose', self.left_ee_callback, 10)
        self.right_ee_sub = self.create_subscription(Pose, 'right_ee_pose', self.right_ee_callback, 10)

        self.left_ee_trajectory_pub = self.create_publisher(Pose, 'left_ee_trajectory', 10)
        self.right_ee_trajectory_pub = self.create_publisher(Pose, 'right_ee_trajectory', 10)

        # Initial poses, stored once and used for trajectory calculation
        self.obj_pose = None
        self.left_ee_pose = None
        self.right_ee_pose = None

    def obj_callback(self, pose_msg):
        """Callback to handle box pose (Only published once after receiving all poses)"""
        if not self.obj_pose: # Only store once
            self.obj_pose = pose_msg
            self.get_logger().info(f"Received Box Pose")

        # Ensure both end effector poses are received
        if self.obj_pose and self.left_ee_pose and self.right_ee_pose:
            self.get_logger().info("All poses received, generating trajectories.")
            # self.get_logger().info(f"obj_pose: {self.obj_pose}")
            # self.get_logger().info(f"left_ee_pose: {self.left_ee_pose}")
            # self.get_logger().info(f"right_ee_pose: {self.right_ee_pose}")

            obj_size = self.get_box_size()

            # Get box corners in local coordinates
            box_corners = self.get_box_corners(*obj_size)

            # Generate the end effector goal poses for grabbing the box
            left_grab_pose = self.get_closest_side_center(self.left_ee_pose, box_corners)
            right_grab_pose = self.get_closest_side_center(self.right_ee_pose, box_corners)

            left_lift_pose = self.create_lift_pose(left_grab_pose)
            right_lift_pose = self.create_lift_pose(right_grab_pose)

            # Create a trajectory that first grabs and then lifts the box
            self.create_and_publish_trajectory(left_grab_pose, left_lift_pose, 'left')
            self.create_and_publish_trajectory(right_grab_pose, right_lift_pose, 'right')

    def left_ee_callback(self, pose_msg):
        """Callback to handle left end effector pose (Only updates once)"""
        if not self.left_ee_pose:  # Only store once
            self.left_ee_pose = pose_msg
            self.get_logger().info(f"Received Left EE Pose")

    def right_ee_callback(self, pose_msg):
        """Callback to handle right end effector pose (Only updates once)"""
        if not self.right_ee_pose:  # Only store once
            self.right_ee_pose = pose_msg
            self.get_logger().info(f"Received Right EE Pose")

    def get_box_corners(self, length, width, height):
        """Calculate the corners of the box based on its size"""
        corners = [
            (-length / 2, -width / 2, -height / 2),
            (length / 2, -width / 2, -height / 2),
            (length / 2, width / 2, -height / 2),
            (-length / 2, width / 2, -height / 2),
            (-length / 2, -width / 2, height / 2),
            (length / 2, -width / 2, height / 2),
            (length / 2, width / 2, height / 2),
            (-length / 2, width / 2, height / 2),
        ]
        return corners

    def get_closest_side_center(self, ee_pose, box_corners):
        """Find the closest side to the end effector and return its center, with correct orientation for y-axis alignment"""
        min_distance = float('inf')
        closest_side_center = None
        closest_side = None

        # Define the sides of the box
        sides = [
            [box_corners[0], box_corners[1], box_corners[4], box_corners[5]],  # front
            [box_corners[1], box_corners[2], box_corners[5], box_corners[6]],  # right
            [box_corners[2], box_corners[3], box_corners[6], box_corners[7]],  # back
            [box_corners[3], box_corners[0], box_corners[7], box_corners[4]],  # left
        ]

        for side in sides:
            side_center = np.mean(np.array(side), axis=0)

            # Calculate the distance from the end effector to this side's center
            distance = np.linalg.norm(np.array([ee_pose.position.x, ee_pose.position.y, ee_pose.position.z]) - side_center)

            if distance < min_distance:
                min_distance = distance
                closest_side_center = side_center
                closest_side = side

        goal_pose = Pose()
        goal_pose.position.x = closest_side_center[0]
        goal_pose.position.y = closest_side_center[1]
        goal_pose.position.z = closest_side_center[2]

        # Calculate the orientation (align y-axis with the normal of the side)
        orientation = self.calculate_grab_orientation(closest_side)

        goal_pose.orientation = orientation

        return goal_pose

    def calculate_grab_orientation(self, side):
        """Calculate the orientation of the end effector for grabbing the box from the side and aligning the y-axis with the normal"""

        # The goal is to align the y-axis of the end effector (rotation axis) with the normal of the side

        # Calculate two vectors in the plane of the side
        v1 = np.array(side[1]) - np.array(side[0]) 
        v2 = np.array(side[3]) - np.array(side[0]) 

        # Normal vector of the side
        normal = np.cross(v1, v2) 
        normal = normal / np.linalg.norm(normal)

        roll, pitch, yaw = self.vector_to_rpy(normal)
        quat = euler_to_quat(degrees(roll), degrees(pitch), degrees(yaw))
        quat = Quaternion(
            x=quat[1], y=quat[2], z=quat[3], w=quat[0]
        )

        return quat

    def vector_to_rpy(self, normal):
        """Convert a normal vector to roll, pitch, and yaw angles"""
        pitch = np.arcsin(-normal[2])  # Sin of pitch = -z-component of the normal
        roll = np.arctan2(normal[1], normal[2])  # Roll = arctan(y/z)
        yaw = np.arctan2(normal[0], normal[2])  # Yaw = arctan(x/z)
        return roll, pitch, yaw


    def create_lift_pose(self, grab_pose):
        """Create a lifting motion pose by increasing the Z position"""
        lift_pose = Pose()
        lift_pose.position.x = grab_pose.position.x
        lift_pose.position.y = grab_pose.position.y
        lift_pose.position.z = grab_pose.position.z + self.lift_height  # Add the lift height
        return lift_pose

    def create_and_publish_trajectory(self, grab_pose, lift_pose, arm_side):
        """Create a trajectory from grab to lift and publish it"""
        self.publish_trajectory(grab_pose, arm_side)
        self.publish_trajectory(lift_pose, arm_side)

    def publish_trajectory(self, goal_pose, arm_side):
        """Publish the trajectory to the specified arm"""
        traj_msg = Pose()
        traj_msg = goal_pose

        if arm_side == 'left':
            self.left_ee_trajectory_pub.publish(traj_msg)
            self.get_logger().info(f"Published Left EE Trajectory")
        else:
            self.right_ee_trajectory_pub.publish(traj_msg)
            self.get_logger().info(f"Published Right EE Trajectory")

    def get_box_size(self):
        """Return the box size from the Xacro file"""
        config_path = os.path.join(
            get_package_share_directory('dual_denso_arm_manipulation'),
            'config',
            'scene_config.xacro'
        )
        try:
            props = get_xacro_properties(config_path)
        except Exception as e:
            self.get_logger().error(f"Failed to read scene_config.xacro: {e}")
            return (0.5, 0.5, 0.5)  # Default size in case of error
        
        lx = float(props.get(f'{self.box_frame}_size_x', '0.5'))
        ly = float(props.get(f'{self.box_frame}_size_y', '0.5'))
        lz = float(props.get(f'{self.box_frame}_size_z', '0.5'))
        
        return lx, ly, lz

def main(args=None):
    rclpy.init(args=args)
    node = LiftTrajGen()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
