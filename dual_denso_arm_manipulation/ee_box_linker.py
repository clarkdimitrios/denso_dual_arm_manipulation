#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from linkattacher_msgs.srv import AttachLink, DetachLink

class EEBoxLinkerIFRA(Node):
    def __init__(self):
        super().__init__('ee_box_linker')

        # Params
        self.declare_parameter('robot_model', 'vm60b1')
        self.declare_parameter('box_model',   'lift_box')
        self.declare_parameter('left_link',   'left_J6')
        self.declare_parameter('right_link',  'right_J6')
        self.declare_parameter('box_link',    'lift_box')
        self.declare_parameter('attach_left',  True)
        self.declare_parameter('attach_right', False)
        self.declare_parameter('attach_on_state', 'attach')
        self.declare_parameter('detach_on_state', 'detach')

        self.robot_model  = self.get_parameter('robot_model').get_parameter_value().string_value
        self.box_model    = self.get_parameter('box_model').get_parameter_value().string_value
        self.left_link    = self.get_parameter('left_link').get_parameter_value().string_value
        self.right_link   = self.get_parameter('right_link').get_parameter_value().string_value
        self.box_link     = self.get_parameter('box_link').get_parameter_value().string_value
        self.attach_left  = self.get_parameter('attach_left').get_parameter_value().bool_value
        self.attach_right = self.get_parameter('attach_right').get_parameter_value().bool_value
        self.attach_on    = self.get_parameter('attach_on_state').get_parameter_value().string_value
        self.detach_on    = self.get_parameter('detach_on_state').get_parameter_value().string_value

        # IFRA services
        self.attach_cli = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_cli = self.create_client(DetachLink, '/DETACHLINK')

        self.create_subscription(String, 'ee_linker_cmd', self._link_cmd_cb, 10)

        self.get_logger().info(
            f"EEBoxLinkerIFRA ready. Box='{self.box_model}'. "
            f"Attach on '{self.attach_on}', detach on '{self.detach_on}'. "
            f"Sides: left={self.attach_left}, right={self.attach_right}"
        )

    def _wait(self, client, name, timeout=5.0):
        ok = client.wait_for_service(timeout_sec=timeout)
        if not ok:
            self.get_logger().warn(f"[EE-Box] Service '{name}' not available yet.")
        return ok

    def _attach(self, link_name):
        if not self._wait(self.attach_cli, '/ATTACHLINK', timeout=5.0):
            return False
        req = AttachLink.Request()
        req.model1_name = self.robot_model
        req.link1_name  = link_name
        req.model2_name = self.box_model
        req.link2_name  = self.box_link
        future = self.attach_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        if res and res.success:
            self.get_logger().info(f"[EE-Box] ATTACH ok: {self.robot_model}:{link_name} ↔ {self.box_model}:{self.box_link}")
            return True
        self.get_logger().error(f"[EE-Box] ATTACH failed: {res.message if res else 'no response'}")
        return False

    def _detach(self, link_name):
        if not self._wait(self.detach_cli, '/DETACHLINK', timeout=5.0):
            return False
        req = DetachLink.Request()
        req.model1_name = self.robot_model
        req.link1_name  = link_name
        req.model2_name = self.box_model
        req.link2_name  = self.box_link
        future = self.detach_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        if res and res.success:
            self.get_logger().info(f"[EE-Box] DETACH ok: {self.robot_model}:{link_name} ↔ {self.box_model}:{self.box_link}")
            return True
        self.get_logger().error(f"[EE-Box] DETACH failed: {res.message if res else 'no response'}")
        return False

    def _link_cmd_cb(self, msg: String):
        s = msg.data.strip().lower()
        self.get_logger().info(f"[EE-Box] link_state='{s}'")
        if s == self.attach_on:
            if self.attach_left:
                self._attach(self.left_link)
            if self.attach_right:
                self._attach(self.right_link)
        elif s == self.detach_on:
            if self.attach_left:
                self._detach(self.left_link)
            if self.attach_right:
                self._detach(self.right_link)

def main():
    rclpy.init()
    node = EEBoxLinkerIFRA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
