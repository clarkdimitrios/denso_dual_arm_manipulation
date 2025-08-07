#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

import os
from ament_index_python.packages import get_package_share_directory
from dual_denso_arm_manipulation.utils import get_xacro_properties, pose_euler_to_quat


class RVizBoxSpawner(Node):
    def __init__(self):
        super().__init__('spawn_lift_box_rviz')

        self.collision_pub = self.create_publisher(CollisionObject, 'collision_object', 10)

        self.has_published = False
        self.timer = self.create_timer(1.0, self.publish_once)

    def publish_once(self):
        if self.has_published:
            return

        config_path = os.path.join(
            get_package_share_directory('dual_denso_arm_manipulation'),
            'config',
            'scene_config.xacro'
        )
        try:
            props = get_xacro_properties(config_path)
        except Exception as e:
            self.get_logger().error(f"Failed to read scene_config.xacro: {e}")
            return

        # Pose
        x = float(props.get('lift_box_x', '0.0'))
        y = float(props.get('lift_box_y', '0.0'))
        z = float(props.get('lift_box_z', '0.0'))
        roll = float(props.get('lift_box_roll', '0.0'))
        pitch = float(props.get('lift_box_pitch', '0.0'))
        yaw = float(props.get('lift_box_yaw', '0.0'))

        # Size
        lx = float(props.get('lift_box_size_x', '0.5'))
        ly = float(props.get('lift_box_size_y', '0.5'))
        lz = float(props.get('lift_box_size_z', '0.5'))

        adj_z = z + lz / 2.0
        pose = pose_euler_to_quat(x, y, adj_z, roll, pitch, yaw)

        # Collision object
        collision_obj = CollisionObject()
        collision_obj.header = Header(frame_id="world")
        collision_obj.id = "lift_box"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [lx, ly, lz]

        collision_obj.primitives.append(primitive)
        collision_obj.primitive_poses.append(pose)
        collision_obj.operation = CollisionObject.ADD

        self.collision_pub.publish(collision_obj)

        self.get_logger().info(
            f"Spawned lift box at ({x}, {y}, {z}) with size ({lx}, {ly}, {lz})"
        )

        self.has_published = True


def main(args=None):
    rclpy.init(args=args)
    node = RVizBoxSpawner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
