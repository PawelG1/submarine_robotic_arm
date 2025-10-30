#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import TransformBroadcaster

class MyVisuNode(Node):
    

    def __init__(self):
        super().__init__('my_visu_node')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.25, self.timer_callback)
        self.get_logger().info('MyVisuNode has been started')
        self.count = 0.0  #var do animacji

    def timer_callback(self):

        #zdefiniuj marker
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        #marker.pose.position = Point(x=0.0, y=0.0, z=0.0)  # Centrum układu
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        #dodaje animacje krecenia sie kulki
        self.count += 0.1
        marker.pose.position = Point(x=math.sin(self.count), y=math.cos(self.count), z=math.sin(self.count)/2)
        #publish markera
        self.publisher_.publish(marker)

        # Publikuj transformację TF: map -> base_link
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
        self.get_logger().info('Published marker and TF')

def main(args=None):
    rclpy.init(args=args)
    node = MyVisuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()