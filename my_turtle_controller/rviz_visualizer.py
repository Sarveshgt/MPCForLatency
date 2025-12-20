#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class RVizVisualizer(Node):
    def __init__(self):
        super().__init__('rviz_visualizer')

        # Subscribe to predicted goal
        self.goal_sub = self.create_subscription(
            Point,
            '/predicted_goal',
            self.goal_callback,
            10)

        # Publisher for markers
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_markers', 10)

        # Obstacle definitions (matching obstacle_visualizer.py)
        self.obstacles = [
            {'x': 3.0, 'y': 3.0, 'radius': 0.5},
            {'x': 8.0, 'y': 3.0, 'radius': 0.5},
            {'x': 3.0, 'y': 8.0, 'radius': 0.5},
            {'x': 8.0, 'y': 8.0, 'radius': 0.5}
        ]

        # Publish obstacles periodically
        self.timer = self.create_timer(1.0, self.publish_obstacles)

        self.get_logger().info("RViz Visualizer Started")

    def publish_obstacles(self):
        """Publish obstacle markers"""
        marker_array = MarkerArray()

        for i, obs in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = obs['x']
            marker.pose.position.y = obs['y']
            marker.pose.position.z = 0.25  # Half the height

            # Orientation (identity quaternion)
            marker.pose.orientation.w = 1.0

            # Scale (diameter = 2 * radius, height = 1.0) - TALLER
            marker.scale.x = obs['radius'] * 2.0
            marker.scale.y = obs['radius'] * 2.0
            marker.scale.z = 1.0  # Taller cylinders for better visibility

            # Color (bright red, more opaque)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8  # More opaque

            marker.lifetime.sec = 0  # Persistent

            marker_array.markers.append(marker)

        # Publish obstacle markers
        self.marker_pub.publish(marker_array)

    def goal_callback(self, msg):
        """Visualize predicted goal as a green sphere"""
        marker_array = MarkerArray()

        # Goal marker
        goal_marker = Marker()
        goal_marker.header.frame_id = 'map'
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = 'predicted_goal'
        goal_marker.id = 100
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD

        # Position
        goal_marker.pose.position.x = msg.x
        goal_marker.pose.position.y = msg.y
        goal_marker.pose.position.z = 0.0

        # Orientation
        goal_marker.pose.orientation.w = 1.0

        # Scale - LARGER goal marker
        goal_marker.scale.x = 0.6  # 2x larger
        goal_marker.scale.y = 0.6
        goal_marker.scale.z = 0.6

        # Color (bright glowing green)
        goal_marker.color.r = 0.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0

        goal_marker.lifetime.sec = 5  # Disappears after 5 seconds

        marker_array.markers.append(goal_marker)

        # Add text label above the goal
        text_marker = Marker()
        text_marker.header.frame_id = 'map'
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = 'goal_label'
        text_marker.id = 101
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        text_marker.pose.position.x = msg.x
        text_marker.pose.position.y = msg.y
        text_marker.pose.position.z = 0.5

        text_marker.text = f"GOAL ({msg.x:.1f}, {msg.y:.1f})"

        text_marker.scale.z = 0.5  # Larger text

        text_marker.color.r = 0.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0

        text_marker.lifetime.sec = 5

        marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

        self.get_logger().info(f"Visualizing goal at ({msg.x:.2f}, {msg.y:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = RVizVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
