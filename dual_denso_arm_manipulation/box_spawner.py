from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import rclpy
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
import xacro
import tempfile
from dual_denso_arm_manipulation.utils import get_xacro_properties, euler_to_quat

class BoxSpawner(Node):
    def __init__(self):
        super().__init__('box_spawner')

        self.pose = self.get_box_pose()

        self.broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.broadcast_transform) 

        # If in simulation mode, spawn the box in Gazebo
        self.declare_parameter('sim', False)
        if self.get_parameter('sim').get_parameter_value().bool_value:
            self.spawn_box()

    def get_box_pose(self):
        """Get the box pose from scene_config.xacro."""
        scene_path = os.path.join(
            get_package_share_directory('dual_denso_arm_manipulation'),
            'config',
            'scene_config.xacro' 
        )
        props = get_xacro_properties(scene_path)
        return {
            "x": float(props.get("lift_box_x", "0.0")),
            "y": float(props.get("lift_box_y", "0.0")),
            "z": float(props.get("lift_box_z", "0.0")),
            "R": float(props.get("lift_box_roll", "0.0")),
            "P": float(props.get("lift_box_pitch", "0.0")),
            "Y": float(props.get("lift_box_yaw", "0.0")),
        }

    def spawn_box(self):
        """Spawn the box in Gazebo (only if in simulation mode)."""
        lift_box_xacro = os.path.join(
            get_package_share_directory('dual_denso_arm_manipulation'),
            'config',
            'lift_box.xacro'
        )

        urdf_doc = xacro.process_file(lift_box_xacro)
        urdf_string = urdf_doc.toprettyxml()

        tmpfile = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
        tmpfile.write(urdf_string)
        tmpfile.close()

        self.get_logger().info("Spawning lift box in Gazebo")
        subprocess.Popen([
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'lift_box', '-file', tmpfile.name,
            '-x', str(self.pose['x']), '-y', str(self.pose['y']), '-z', str(self.pose['z']),
            '-R', str(self.pose['R']), '-P', str(self.pose['P']), '-Y', str(self.pose['Y']),
        ])

    def broadcast_transform(self):
        """Broadcast the transform of the box periodically."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "lift_box"

        orientation = euler_to_quat(self.pose["R"], self.pose["P"], self.pose["Y"])

        transform.transform.translation.x = self.pose["x"]
        transform.transform.translation.y = self.pose["y"]
        transform.transform.translation.z = self.pose["z"]
        transform.transform.rotation.x = orientation[1] 
        transform.transform.rotation.y = orientation[2]
        transform.transform.rotation.z = orientation[3]
        transform.transform.rotation.w = orientation[0]

        self.broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = BoxSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
