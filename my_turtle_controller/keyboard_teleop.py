#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Publisher to /cmd_input
        self.publisher = self.create_publisher(Twist, '/cmd_input', 10)

        # Velocity settings
        self.linear_speed = 2.0
        self.angular_speed = 2.0

        self.get_logger().info("Keyboard Teleop Started")
        self.get_logger().info("Use arrow keys to move, 'q' to quit")
        self.get_logger().info("Up/Down: Forward/Backward")
        self.get_logger().info("Left/Right: Turn")

        # Get terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        # Start reading keys
        self.read_keys()

    def get_key(self):
        """Read a single keypress"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def read_keys(self):
        """Main loop to read keyboard input"""
        try:
            while True:
                key = self.get_key()

                twist = Twist()

                if key == '\x1b':  # ESC sequence for arrow keys
                    key = self.get_key()
                    if key == '[':
                        key = self.get_key()
                        if key == 'A':  # Up arrow
                            twist.linear.x = self.linear_speed
                            self.get_logger().info("Forward")
                        elif key == 'B':  # Down arrow
                            twist.linear.x = -self.linear_speed
                            self.get_logger().info("Backward")
                        elif key == 'C':  # Right arrow
                            twist.angular.z = -self.angular_speed
                            self.get_logger().info("Turn Right")
                        elif key == 'D':  # Left arrow
                            twist.angular.z = self.angular_speed
                            self.get_logger().info("Turn Left")

                elif key == 'q' or key == '\x03':  # 'q' or Ctrl+C
                    self.get_logger().info("Quitting...")
                    break

                elif key == ' ':  # Space to stop
                    twist = Twist()
                    self.get_logger().info("Stop")

                # Publish command
                self.publisher.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
