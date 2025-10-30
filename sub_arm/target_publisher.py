#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, TransformStamped
from tf2_ros import TransformBroadcaster
from .simple_kinematics import forward_kinematics

class ForwardKinematics:
    def __init__(self):
        pass

class EndEffectorPosition:

    def __init__(self):
        pass

    def get_position(self):
        self.end_effector_pos
        theta = [0, 0, 0, 0, 0, 0]  #przykladowe katy joints
        points, pos, T = forward_kinematics(theta)
        return pos


class TargetPublisher(Node):

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        super().__init__('target_publisher')
        self.end_effector_pos = self.create_subscription(Pose, 'end_effector_target_pos', self.set_end_effector_target_pos, 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.5, lambda: self.publish_target(self.x, self.y, self.z))
        self.get_logger().info('TargetPublisher has been started')

    def set_end_effector_target_pos(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z
        msg
        self.get_logger().info(f'Received target position: x={self.x}, y={self.y}, z={self.z}')

    def publish_target(self, x=0.0, y=0.0, z=0.0):
        x = self.x
        y = self.y
        z = self.z
        self.get_logger().info(f'Publishing marker at x={x:.3f} y={y:.3f} z={z:.3f}')
        # Definiowanie markera celu
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_namespace"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=y, y=x, z=z)  #pozycja celu, ZAMIENIIONO X I Y !!
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        #publikacja markera celu
        self.marker_pub.publish(marker)

        #publikuj transformacje TF: map -> base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published target marker and TF')


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()