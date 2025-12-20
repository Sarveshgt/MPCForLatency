#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Path
import numpy as np
from scipy.optimize import minimize
from collections import deque

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller_node')

        # Publishers
        self.predicted_goal_pub = self.create_publisher(Point, '/predicted_goal', 10)
        self.mpc_trajectory_pub = self.create_publisher(Path, '/mpc_trajectory', 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist,
            '/turtle_controller_input',
            self.cmd_callback,
            10)

        self.delay_status_sub = self.create_subscription(
            Bool,
            '/network_delay_active',
            self.delay_status_callback,
            10)

        # MPC Parameters
        self.horizon = 10
        self.dt = 0.1

        # Internal robot state
        self.x = 5.5
        self.y = 5.5
        self.theta = 0.0
        self.pose_received = True
        self.delay_active = False

        # Store current command
        self.current_cmd = Twist()
        
        # --- NEW: Watchdog Timer variable ---
        # Initialize with current time to prevent immediate timeout error on startup
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # Stop if no command for 0.5 seconds

        # Command history
        self.command_history = deque(maxlen=10)

        # State update timer
        self.state_update_timer = self.create_timer(self.dt, self.update_state)

        # Predicted goal state
        self.predicted_goal_x = None
        self.predicted_goal_y = None

        # MPC control loop timer
        self.mpc_timer = None

        # Obstacles
        self.obstacles = [
            {'x': 3.0, 'y': 3.0, 'radius': 0.5},
            {'x': 8.0, 'y': 3.0, 'radius': 0.5},
            {'x': 3.0, 'y': 8.0, 'radius': 0.5},
            {'x': 8.0, 'y': 8.0, 'radius': 0.5}
        ]

        self.safety_margin = 0.3

        self.get_logger().info("Turtle Controller Started with Safety Timeout")

    def update_state(self):
        """Update internal robot state by integrating velocity commands"""
        
        # --- NEW: TIMEOUT CHECK ---
        # Calculate time since last command was received
        time_since_last_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        # If timeout exceeded and we aren't already stopped, stop the robot
        if time_since_last_cmd > self.cmd_timeout:
            if self.current_cmd.linear.x != 0.0 or self.current_cmd.angular.z != 0.0:
                self.current_cmd = Twist()  # Force stop
                # Optional: Log once when stopping to avoid spamming
                # self.get_logger().info("Timeout: Stopping robot")

        # Integrate current velocity command
        v = self.current_cmd.linear.x
        w = self.current_cmd.angular.z

        # Update state using unicycle model
        self.x += v * np.cos(self.theta) * self.dt
        self.y += v * np.sin(self.theta) * self.dt
        self.theta += w * self.dt

        # Normalize theta to [-pi, pi]
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        # Keep within bounds
        self.x = np.clip(self.x, 0.0, 11.0)
        self.y = np.clip(self.y, 0.0, 11.0)

        # Publish current pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.z = np.sin(self.theta / 2.0)
        pose_msg.pose.orientation.w = np.cos(self.theta / 2.0)
        self.current_pose_pub.publish(pose_msg)

    def cmd_callback(self, msg):
        """Receive commands and update timestamp"""
        # --- NEW: Update the last command timestamp ---
        self.last_cmd_time = self.get_clock().now()

        # Add command to history
        self.command_history.append({
            'linear': msg.linear.x,
            'angular': msg.angular.z
        })

        if not self.delay_active:
            if self.is_safe_command(msg):
                self.current_cmd = msg
                self.get_logger().debug(f"Applying command: v={msg.linear.x:.2f}")
            else:
                self.get_logger().warn("⚠️ Command blocked - obstacle!")
                self.stop_robot()

    def is_safe_command(self, cmd):
        """Check if command hits obstacle"""
        predicted_x = self.x + cmd.linear.x * np.cos(self.theta) * self.dt
        predicted_y = self.y + cmd.linear.x * np.sin(self.theta) * self.dt

        for obs in self.obstacles:
            dist_to_obs = np.sqrt(
                (predicted_x - obs['x'])**2 +
                (predicted_y - obs['y'])**2
            )
            if dist_to_obs < (obs['radius'] + self.safety_margin):
                return False
        return True

    def delay_status_callback(self, msg):
        self.delay_active = msg.data
        if self.delay_active:
            self.get_logger().warn("⚠️ Network delay detected! Activating MPC...")
            self.predict_goal_from_history()
            if self.mpc_timer is None:
                self.mpc_timer = self.create_timer(self.dt, self.mpc_control_loop)
        else:
            self.get_logger().info("✓ Network delay resolved.")
            if self.mpc_timer is not None:
                self.mpc_timer.cancel()
                self.mpc_timer = None
            self.predicted_goal_x = None
            self.predicted_goal_y = None

    def predict_goal_from_history(self):
        if len(self.command_history) == 0:
            return
        
        # Start prediction from current state
        pred_x, pred_y, pred_th = self.x, self.y, self.theta
        
        for cmd in self.command_history:
            v, w = cmd['linear'], cmd['angular']
            pred_x += v * np.cos(pred_th) * self.dt
            pred_y += v * np.sin(pred_th) * self.dt
            pred_th += w * self.dt

        self.predicted_goal_x = pred_x
        self.predicted_goal_y = pred_y

        goal_msg = Point()
        goal_msg.x = self.predicted_goal_x
        goal_msg.y = self.predicted_goal_y
        self.predicted_goal_pub.publish(goal_msg)

    def mpc_control_loop(self):
        if self.predicted_goal_x is None:
            return

        dist_to_goal = np.sqrt((self.predicted_goal_x - self.x)**2 + (self.predicted_goal_y - self.y)**2)
        if dist_to_goal < 0.1:
            self.stop_robot()
            return

        # Optimization setup (simplified for brevity - logic same as before)
        initial_guess = np.zeros(self.horizon * 2)
        initial_guess[0::2] = 0.1
        bounds = [(0.0, 2.0), (-2.0, 2.0)] * self.horizon
        
        result = minimize(self.cost_function, initial_guess, bounds=bounds, method='SLSQP')
        optimal_controls = result.x.reshape((self.horizon, 2))
        
        self.publish_mpc_trajectory(optimal_controls)
        
        # Apply MPC Output
        # Note: MPC output is self-driven, so we update timestamp to prevent timeout during MPC
        self.last_cmd_time = self.get_clock().now() 
        self.current_cmd.linear.x = float(optimal_controls[0, 0])
        self.current_cmd.angular.z = float(optimal_controls[0, 1])

    def model_unicycle(self, state, control):
        x, y, theta = state
        v, w = control
        return [x + v * np.cos(theta) * self.dt, 
                y + v * np.sin(theta) * self.dt, 
                theta + w * self.dt]

    def cost_function(self, u_flat):
        controls = u_flat.reshape((self.horizon, 2))
        cost = 0.0
        curr = [self.x, self.y, self.theta]

        for i in range(self.horizon):
            curr = self.model_unicycle(curr, controls[i])
            
            # Distance to goal
            dist_sq = (self.predicted_goal_x - curr[0])**2 + (self.predicted_goal_y - curr[1])**2
            cost += dist_sq
            
            # Obstacle Avoidance
            for obs in self.obstacles:
                d_obs = np.sqrt((curr[0]-obs['x'])**2 + (curr[1]-obs['y'])**2)
                safe_dist = obs['radius'] + self.safety_margin
                if d_obs < safe_dist:
                    cost += 1000.0 * np.exp(5.0 * (safe_dist - d_obs))
                elif d_obs < safe_dist + 1.0:
                    cost += 10.0 / (d_obs - safe_dist + 0.1)
        return cost

    def publish_mpc_trajectory(self, optimal_controls):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        curr = [self.x, self.y, self.theta]
        
        for i in range(self.horizon):
            curr = self.model_unicycle(curr, optimal_controls[i])
            pose = PoseStamped()
            pose.pose.position.x = curr[0]
            pose.pose.position.y = curr[1]
            path.poses.append(pose)
        self.mpc_trajectory_pub.publish(path)

    def stop_robot(self):
        self.current_cmd = Twist()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()