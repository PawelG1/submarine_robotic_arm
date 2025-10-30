#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'install', 'sub_arm', 'lib', 'python3.12', 'site-packages'))
from sub_arm.inverse_kinematics_model import inverse_kinematics, forward_kinematics

class ArmStatePublisher(Node):
    def __init__(self):
        super().__init__('arm_state_publisher')
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.pose_sub = self.create_subscription(Pose, 'end_effector_target_pos', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # Publikuj co 0.1s
        self.get_logger().info('ArmStatePublisher has been started')

        # Lista nazw joint'ów (z URDF, tylko revolute)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Przykładowe kąty (można zmienić na podstawie IK)
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # W radianach

    def pose_callback(self, msg):
        end_effector_pose = np.array([msg.position.x, msg.position.y, msg.position.z])
        theta_solution, success = inverse_kinematics(end_effector_pose, self.joint_positions)
        
        if success:
            self.joint_positions = theta_solution.tolist()
            # Oblicz pozycję z FK dla weryfikacji
            _, fk_pos, _ = forward_kinematics(theta_solution)
            self.get_logger().info(f'IK success: target={end_effector_pose}, fk_pos={fk_pos}, error={np.linalg.norm(end_effector_pose - fk_pos)}, theta={theta_solution}')
        else:
            self.get_logger().warn(f'IK failed: target={end_effector_pose} out of reach')
        
        self.get_logger().info(f'Received target pose: x={msg.position.x}, y={msg.position.y}, z={msg.position.z}')

    def set_joint_positions(self, positions):
        """Ustaw kąty joint'ów (lista 6 wartości w radianach)"""
        if len(positions) == 6:
            self.joint_positions = positions
        else:
            self.get_logger().error('Joint positions must be a list of 6 floats')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        # velocity i effort opcjonalne, zostaw puste

        self.joint_state_pub.publish(msg)
        self.get_logger().debug(f'Published joint states: {self.joint_positions}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

