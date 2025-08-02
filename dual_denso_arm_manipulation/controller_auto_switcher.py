#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
from moveit_msgs.msg import DisplayTrajectory

""" NOT USED ANYMORE """

class ControllerAutoSwitcher(Node):
    def __init__(self):
        super().__init__('controller_auto_switcher')

        # Declare and read controllers parameter
        self.declare_parameter('controllers', [
            'dual_arm_joint_trajectory_controller',
            'left_arm_controller',
            'right_arm_controller'
        ])
        self.controllers = self.get_parameter(
            'controllers'
        ).get_parameter_value().string_array_value

        if len(self.controllers) != 3:
            self.get_logger().error("Expected 3 controllers: [dual, left, right]")
            return

        self.dual_controller = self.controllers[0]
        self.left_controller = self.controllers[1]
        self.right_controller = self.controllers[2]
        self.active_controller = None

        # Subscribe to planned trajectory from MoveIt
        topic = "/move_group/display_planned_path"
        self.get_logger().info(f"Listening for planned paths on: {topic}")
        self.create_subscription(
            DisplayTrajectory,
            topic,
            self.planned_path_callback,
            10
        )

    def planned_path_callback(self, msg: DisplayTrajectory):
        """Triggered when a new plan is created in MoveIt."""
        if not msg.trajectory:
            return

        # Get joint names from the first trajectory
        joint_names = msg.trajectory[0].joint_trajectory.joint_names
        if not joint_names:
            return

        # Decide which controller should be active
        if self.is_dual(joint_names):
            target_controller = self.dual_controller
        elif self.is_left(joint_names):
            target_controller = self.left_controller
        elif self.is_right(joint_names):
            target_controller = self.right_controller
        else:
            self.get_logger().warn(f"Unknown joint set in planned path: {joint_names}")
            return

        if target_controller != self.active_controller:
            self.switch_controllers(target_controller)

    def is_dual(self, joint_names):
        return any(name.startswith("left_joint") for name in joint_names) and \
               any(name.startswith("right_joint") for name in joint_names)

    def is_left(self, joint_names):
        return all(name.startswith("left_joint") for name in joint_names)

    def is_right(self, joint_names):
        return all(name.startswith("right_joint") for name in joint_names)

    def switch_controllers(self, target):
        """Stop other controllers and activate the target one."""
        stop_controllers = [c for c in self.controllers if c != target]
        self.get_logger().info(f"Switching to {target} (stopping {stop_controllers})")

        cmd = [
            "ros2", "control", "switch_controllers",
            "--start-controllers", target,
            "--stop-controllers"
        ] + stop_controllers + [
            "--strict", "--activate-asap"
        ]

        try:
            subprocess.run(cmd, check=True)
            self.active_controller = target
            self.get_logger().info(f"Successfully switched to {target}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to switch controllers: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerAutoSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
