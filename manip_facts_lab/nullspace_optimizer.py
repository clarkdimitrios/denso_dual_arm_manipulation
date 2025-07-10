#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from pymoveit2 import MoveIt2
import PyKDL
from kdl_parser_py.urdf import treeFromParam

class NullspaceOptimizer(Node):
    def __init__(self):
        super().__init__('nullspace_optim_node')

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "joint_1", "joint_2", "joint_3",
                "joint_4", "joint_5", "joint_6"
            ],
            base_link_name="base_link",
            end_effector_name="J6",
            group_name="arm"
        )

        self.q_min = np.array([-2.967060, -1.570796, -1.396263,
                               -3.228859, -2.094395, -6.283185])
        self.q_max = np.array([ 2.967060,  2.356194,  2.932153,
                                3.228859,  2.094395,  6.283185])

        _, self.kdl_tree = treeFromParam('robot_description')
        self.kdl_chain = self.kdl_tree.getChain("base_link", "J6")

        self.joint_names = [
            seg.getJoint().getName()
            for seg in self.kdl_chain.segments
            if seg.getJoint().getType() != PyKDL.Joint.NoneJoint
        ]
        self.kdl_fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kdl_chain)
        self.kdl_jac_solver = PyKDL.ChainJntToJacSolver(self.kdl_chain)

        # Timer to trigger optimization periodically
        self.timer = self.create_timer(1.0, self.optimize)

    def optimize(self):
        # Check that joint state is available
        q = np.array(self.moveit2.joint_positions)
        if q.size == 0:
            self.get_logger().warn("Joint states not available yet!")
            return

        J = self.get_jacobian(q)
        if J is None:
            self.get_logger().warn("Jacobian not available!")
            return

        if J.shape[0] < 6 or np.linalg.matrix_rank(J) < 6:
            self.get_logger().warn("Jacobian not full rank yet!")
            return

        J_pinv = np.linalg.pinv(J)
        null_proj = np.eye(len(q)) - J_pinv @ J

        q_center = (self.q_min + self.q_max) / 2
        grad = q_center - q

        dq = null_proj @ grad * 0.05
        q_new = q + dq

        self.get_logger().info("Optimizing toward center of joint range...")
        self.moveit2.move_to_configuration(q_new.tolist())

    def get_jacobian(self, q):
        num_joints = len(q)
        q_kdl = PyKDL.JntArray(num_joints)
        for i in range(num_joints):
            q_kdl[i] = q[i]

        jacobian = PyKDL.Jacobian(num_joints)
        if self.kdl_jac_solver.JntToJac(q_kdl, jacobian) < 0:
            self.get_logger().error("Failed to compute Jacobian")
            return None

        return np.array([[jacobian[i, j] for j in range(num_joints)] for i in range(6)])

def main():
    rclpy.init()
    node = NullspaceOptimizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
