#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMux(Node):
    def __init__(self):
        super().__init__('joint_state_mux')

        self.left_topic  = self.declare_parameter('left_topic',  '/left/joint_states').value
        self.right_topic = self.declare_parameter('right_topic', '/right/joint_states').value
        self.out_topic   = self.declare_parameter('out_topic',   '/joint_states').value

        self.pub = self.create_publisher(JointState, self.out_topic, 10)

        # latest caches
        self.left = None
        self.right = None

        self.create_subscription(JointState, self.left_topic,  self._left_cb,  10)
        self.create_subscription(JointState, self.right_topic, self._right_cb, 10)

        # publish at a steady rate (100 Hz is fine)
        self.timer = self.create_timer(0.01, self._tick)

        self.get_logger().info(f"Muxing {self.left_topic} + {self.right_topic} -> {self.out_topic}")

    def _left_cb(self, msg: JointState):
        self.left = msg

    def _right_cb(self, msg: JointState):
        self.right = msg

    def _tick(self):
        if self.left is None and self.right is None:
            return

        out = JointState()
        # Use latest stamp available
        if self.left is not None and self.right is not None:
            out.header.stamp = self.left.header.stamp if self.left.header.stamp.sec >= self.right.header.stamp.sec else self.right.header.stamp
        elif self.left is not None:
            out.header.stamp = self.left.header.stamp
        else:
            out.header.stamp = self.right.header.stamp

        name = []
        position = []
        velocity = []
        effort = []

        def extend(js: JointState):
            nonlocal name, position, velocity, effort
            if js is None:
                return
            name.extend(list(js.name))
            position.extend(list(js.position) if js.position else [])
            velocity.extend(list(js.velocity) if js.velocity else [])
            effort.extend(list(js.effort) if js.effort else [])

        extend(self.left)
        extend(self.right)

        out.name = name
        if position: out.position = position
        if velocity: out.velocity = velocity
        if effort:   out.effort = effort

        self.pub.publish(out)

def main():
    rclpy.init()
    node = JointStateMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
