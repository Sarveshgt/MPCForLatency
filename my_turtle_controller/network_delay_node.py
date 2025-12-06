import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from collections import deque
import time
import random

class NetworkDelayNode(Node):
    def __init__(self):
        super().__init__('network_delay_node')
        
        # --- SETTINGS ---
        self.latency_ms = 500.0   # 500ms Delay
        self.drop_probability = 0.0 # 0% Packet Loss (will change to test later)
        
        # 1. Taking "Clean" input (from MPC)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_input',
            self.listener_callback,
            10)
            
        # 2. Publish to the "Real" robot (Turtlesim)
        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel', 
            10)
            
        self.msg_buffer = deque()
        self.timer = self.create_timer(0.01, self.process_buffer)
        self.get_logger().info(f"Network Node Started: {self.latency_ms}ms delay")

    def listener_callback(self, msg):
        # Store message with arrival time
        self.msg_buffer.append((msg, time.time()))

    def process_buffer(self):
        if not self.msg_buffer:
            return

        current_time = time.time()
        msg, arrival_time = self.msg_buffer[0]
        
        # If enough time has passed...
        if (current_time - arrival_time) * 1000 >= self.latency_ms:
            self.msg_buffer.popleft()
            
            # Send it (unless dropped)
            if random.random() > self.drop_probability:
                self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NetworkDelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()