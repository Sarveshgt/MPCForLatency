import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import sys
import numpy as np
import math

class TurtleWaypointController(Node):
    def __init__(self):
        super().__init__('turtle1_waypoint_controller')
        
        # Defining paraters
        self.pose = None
        self.waypoints = [
            (2.0, 2.0),
            (2.0, 9.0),
            (9.0, 9.0),
            (9.0, 2.0),
        ]
        self.current_waypoint_idx = 0
        self.goal_tolerance = 0.05
        self.max_linear_vel = 2.0
        self.max_angular_vel = 4.0
        self.old_x_error = 0
        self.old_yaw_error = 0

        # P-controller (x, theta)
        self.Kp = np.diag([1.5, 4.0])
        self.Kd = np.diag([1.0, 1.0])

        # Interaction with turtlesim
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Running controller
        self.timer = self.create_timer(0.05, self.waypoints_control)

    def pose_callback(self, msg):
        self.pose = msg

    def waypoints_control(self):
        if self.pose is None:
            return

        goal_x, goal_y = self.waypoints[self.current_waypoint_idx]

        dx = goal_x - self.pose.x
        dy = goal_y - self.pose.y

        dist = math.sqrt(dx ** 2 + dy ** 2)
        
        if dist < self.goal_tolerance:
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
            self.get_logger().info(f"Reached waypoint. Onto {self.waypoints[self.current_waypoint_idx]}")

        x_error = dist
        yaw_error = self.normalize_angle(math.atan2(dy, dx) - self.pose.theta)

        dx_error = self.old_x_error - x_error
        dyaw_error = self.old_yaw_error - yaw_error

        v = self.Kp[0, 0] * x_error + self.Kd[0, 0] * dx_error
        omega = self.Kp[1, 1] * yaw_error + self.Kd[1, 1] * dyaw_error

        self.get_logger().info(f"x_error: {x_error}, yaw_error: {yaw_error}, ")

        # v = max(-self.max_linear_vel, min(self.max_linear_vel, v))
        # omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))

        self.old_x_error = x_error
        self.old_yaw_error = yaw_error

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self.pub.publish(twist)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = TurtleWaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

