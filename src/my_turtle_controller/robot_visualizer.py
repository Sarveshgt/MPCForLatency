#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np

class RobotVisualizer(Node):
    def __init__(self):
        super().__init__('robot_visualizer')

        # Subscribe to Real Robot (Blue)
        self.create_subscription(PoseStamped, '/current_pose', self.real_pose_callback, 10)
        
        # Subscribe to Ghost Robot (Red)
        self.create_subscription(PoseStamped, '/ghost_pose', self.ghost_pose_callback, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/robot_markers', 10)
        
        # History for Trails
        self.real_path = []
        self.ghost_path = []
        self.max_path_length = 200  # Keep last 200 points

        self.get_logger().info("Visualizer with Trails Started")

    def real_pose_callback(self, msg):
        self.real_path.append(msg.pose.position)
        if len(self.real_path) > self.max_path_length:
            self.real_path.pop(0)
            
        # Blue, Solid Car, ID offset 0
        self.publish_composite(msg, (0.0, 0.5, 1.0, 1.0), 0, self.real_path, "real_trail")

    def ghost_pose_callback(self, msg):
        self.ghost_path.append(msg.pose.position)
        if len(self.ghost_path) > self.max_path_length:
            self.ghost_path.pop(0)

        # Red, Transparent Car, ID offset 10
        self.publish_composite(msg, (1.0, 0.0, 0.0, 0.5), 10, self.ghost_path, "ghost_trail")

    def publish_composite(self, msg, color_rgba, id_offset, path_points, ns_trail):
        marker_array = MarkerArray()
        r, g, b, a = color_rgba
        
        # --- 1. Draw The Car (Same as before) ---
        rot_matrix = self.quaternion_to_matrix(msg.pose.orientation)
        base_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        # Chassis
        chassis = self.create_marker(msg, id_offset + 0, Marker.CUBE, r, g, b, a)
        chassis.scale.x, chassis.scale.y, chassis.scale.z = (2.0, 1.0, 0.5)
        chassis.pose.position.z += 0.25
        marker_array.markers.append(chassis)

        # Cabin
        cabin_offset = np.array([-0.2, 0.0, 0.5]) 
        cabin_pos = base_pos + rot_matrix @ cabin_offset
        cabin = self.create_marker(msg, id_offset + 1, Marker.CUBE, r, g, b, a)
        cabin.scale.x, cabin.scale.y, cabin.scale.z = (1.0, 0.9, 0.4)
        cabin.pose.position = Point(x=cabin_pos[0], y=cabin_pos[1], z=cabin_pos[2] + 0.25)
        marker_array.markers.append(cabin)

        # Wheels
        wheel_offsets = [(0.6, 0.6, 0), (0.6, -0.6, 0), (-0.6, 0.6, 0), (-0.6, -0.6, 0)]
        q_rot_wheel = [0.707106, 0.0, 0.0, 0.707106] 
        q_robot = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        q_final = self.multiply_quaternions(q_robot, q_rot_wheel)

        for i, offset in enumerate(wheel_offsets):
            local_pos = np.array(offset)
            global_pos = base_pos + rot_matrix @ local_pos
            wheel = self.create_marker(msg, id_offset + 2 + i, Marker.CYLINDER, 0.1, 0.1, 0.1, a)
            wheel.pose.position = Point(x=global_pos[0], y=global_pos[1], z=0.15)
            wheel.pose.orientation.x, wheel.pose.orientation.y = q_final[0], q_final[1]
            wheel.pose.orientation.z, wheel.pose.orientation.w = q_final[2], q_final[3]
            wheel.scale.x, wheel.scale.y, wheel.scale.z = (0.4, 0.4, 0.2)
            marker_array.markers.append(wheel)

        # --- 2. Draw The Trail (Ribbon) ---
        trail = Marker()
        trail.header.frame_id = 'map'
        trail.header.stamp = self.get_clock().now().to_msg()
        trail.ns = ns_trail
        trail.id = id_offset + 100
        trail.type = Marker.LINE_STRIP
        trail.action = Marker.ADD
        trail.scale.x = 0.1  # Line width
        trail.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.6)
        trail.points = path_points
        marker_array.markers.append(trail)

        self.marker_pub.publish(marker_array)

    def create_marker(self, source_msg, mid, mtype, r, g, b, a):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_car'
        marker.id = mid
        marker.type = mtype
        marker.action = Marker.ADD
        marker.pose.orientation = source_msg.pose.orientation
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (r, g, b, a)
        return marker

    def quaternion_to_matrix(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2*y*y - 2*z*z,  2*x*y - 2*z*w,      2*x*z + 2*y*w],
            [2*x*y + 2*z*w,      1 - 2*x*x - 2*z*z,  2*y*z - 2*x*w],
            [2*x*z - 2*y*w,      2*y*z + 2*x*w,      1 - 2*x*x - 2*y*y]
        ])

    def multiply_quaternions(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return [
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ]

def main(args=None):
    rclpy.init(args=args)
    node = RobotVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()