#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np

class GhostController(Node):
    def __init__(self):
        super().__init__('ghost_controller')

        # Subscribe to the same DELAYED input as the real robot
        self.cmd_sub = self.create_subscription(
            Twist,
            '/turtle_controller_input',
            self.cmd_callback,
            10)

        # Publish ghost pose to a different topic
        self.pose_pub = self.create_publisher(PoseStamped, '/ghost_pose', 10)

        # State
        self.x = 5.5
        self.y = 5.5  # Start slightly offset so they don't clip perfectly? No, same spot is better for comparison.
        self.theta = 0.0
        self.dt = 0.1
        
        # Naive Control State
        self.current_v = 0.0
        self.current_w = 0.0
        self.last_cmd_time = self.get_clock().now()
        
        # Physics timer
        self.create_timer(self.dt, self.update_physics)

    def cmd_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self.current_v = msg.linear.x
        self.current_w = msg.angular.z

    def update_physics(self):
        # --- NAIVE LOGIC ---
        # If no command received for 0.5s, STOP.
        # This robot has NO prediction. It lives moment-to-moment.
        time_since = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since > 0.5:
            self.current_v = 0.0
            self.current_w = 0.0

        # Simple Unicycle Physics
        self.x += self.current_v * np.cos(self.theta) * self.dt
        self.y += self.current_v * np.sin(self.theta) * self.dt
        self.theta += self.current_w * self.dt

        # Publish Pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.orientation.z = np.sin(self.theta/2)
        msg.pose.orientation.w = np.cos(self.theta/2)
        
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GhostController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()