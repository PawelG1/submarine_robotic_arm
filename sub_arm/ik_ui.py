#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

class IKUI(Node):
    def __init__(self):
        super().__init__('ik_ui')
        self.pose_pub = self.create_publisher(Pose, 'end_effector_target_pos', 10)
        self.get_logger().info('IK UI has been started')

        # Początkowa pozycja
        self.x = 0.0
        self.y = 0.0
        self.z = 0.3  # Wysokość end-effectora

        # UI z matplotlib
        self.setup_ui()

    def setup_ui(self):
        fig, ax = plt.subplots()
        plt.subplots_adjust(bottom=0.4)

        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(-0.5, 0.5)
        ax.set_title('End-Effector Position Control')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')

        # Punkt reprezentujący pozycję
        self.point, = ax.plot([self.x], [self.y], 'ro', markersize=10)

        # Suwaki
        axcolor = 'lightgoldenrodyellow'
        ax_x = plt.axes([0.15, 0.3, 0.7, 0.02], facecolor=axcolor)
        ax_y = plt.axes([0.15, 0.25, 0.7, 0.02], facecolor=axcolor)
        ax_z = plt.axes([0.15, 0.2, 0.7, 0.02], facecolor=axcolor)

        self.slider_x = Slider(ax_x, 'X', -0.3, 0.3, valinit=self.x)
        self.slider_y = Slider(ax_y, 'Y', -0.3, 0.3, valinit=self.y)
        self.slider_z = Slider(ax_z, 'Z', 0.0, 0.35, valinit=self.z)

        self.slider_x.on_changed(self.update)
        self.slider_y.on_changed(self.update)
        self.slider_z.on_changed(self.update)

        plt.show()

    def update(self, val):
        self.x = self.slider_x.val
        self.y = self.slider_y.val
        self.z = self.slider_z.val

        self.point.set_data([self.x], [self.y])

        # Publikuj Pose
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z
        pose.orientation.w = 1.0  # Brak orientacji na razie

        self.pose_pub.publish(pose)
        self.get_logger().info(f'Published target pose: x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = IKUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()