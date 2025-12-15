#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np

class RobotVisualizer(Node):
    def __init__(self):
        super().__init__('robot_visualizer')

        # Subscribe to robot pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10)

        # Publisher for robot marker
        self.marker_pub = self.create_publisher(Marker, '/robot_marker', 10)

        self.get_logger().info("Robot Visualizer Started")

    def pose_callback(self, msg):
        """Visualize robot as an arrow marker"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Position from pose
        marker.pose.position.x = msg.pose.position.x
        marker.pose.position.y = msg.pose.position.y
        marker.pose.position.z = 0.1  # Slightly above ground

        # Orientation from pose
        marker.pose.orientation = msg.pose.orientation

        # Scale (arrow size) - MUCH LARGER for visibility
        marker.scale.x = 1.5  # Length (3x larger)
        marker.scale.y = 0.3  # Width (3x larger)
        marker.scale.z = 0.3  # Height (3x larger)

        # Color (bright blue robot)
        marker.color.r = 0.0
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0  # Persistent

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RobotVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
