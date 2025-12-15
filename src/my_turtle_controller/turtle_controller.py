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
        # RViz2 Visualization Publishers
        self.predicted_goal_pub = self.create_publisher(Point, '/predicted_goal', 10)
        self.mpc_trajectory_pub = self.create_publisher(Path, '/mpc_trajectory', 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)

        # Subscribers
        # 1. Subscribe to commands from network_delay_node
        self.cmd_sub = self.create_subscription(
            Twist,
            '/turtle_controller_input',
            self.cmd_callback,
            10)

        # 2. Subscribe to delay status from network_delay_node
        self.delay_status_sub = self.create_subscription(
            Bool,
            '/network_delay_active',
            self.delay_status_callback,
            10)

        # MPC Parameters (define BEFORE using self.dt!)
        self.horizon = 10
        self.dt = 0.1

        # Internal robot state (no turtlesim dependency)
        self.x = 5.5  # Start in center of 11x11 space
        self.y = 5.5
        self.theta = 0.0
        self.pose_received = True  # Always true since we track internally
        self.delay_active = False

        # Store current command for integration
        self.current_cmd = Twist()

        # Command history: store last 10 commands from teleop
        self.command_history = deque(maxlen=10)

        # State update timer (integrate velocity commands) - create AFTER defining dt
        self.state_update_timer = self.create_timer(self.dt, self.update_state)

        # Predicted goal state (when delay is active)
        self.predicted_goal_x = None
        self.predicted_goal_y = None

        # MPC control loop timer (only active during delay)
        self.mpc_timer = None

        # Obstacle definitions (matching obstacle_visualizer.py and rviz_visualizer.py)
        self.obstacles = [
            {'x': 3.0, 'y': 3.0, 'radius': 0.5},
            {'x': 8.0, 'y': 3.0, 'radius': 0.5},
            {'x': 3.0, 'y': 8.0, 'radius': 0.5},
            {'x': 8.0, 'y': 8.0, 'radius': 0.5}
        ]

        # Safety margin added to obstacle radius
        self.safety_margin = 0.3

        self.get_logger().info("Turtle Controller Started")
        self.get_logger().info(f"Loaded {len(self.obstacles)} obstacles with safety margin {self.safety_margin}")

    def update_state(self):
        """Update internal robot state by integrating velocity commands"""
        # Integrate current velocity command
        v = self.current_cmd.linear.x
        w = self.current_cmd.angular.z

        # Update state using unicycle model
        self.x += v * np.cos(self.theta) * self.dt
        self.y += v * np.sin(self.theta) * self.dt
        self.theta += w * self.dt

        # Normalize theta to [-pi, pi]
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        # Keep within bounds (0-11 range for turtlesim compatibility)
        self.x = np.clip(self.x, 0.0, 11.0)
        self.y = np.clip(self.y, 0.0, 11.0)

        # Publish current pose for RViz2 visualization
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0
        # Convert theta to quaternion (simple 2D rotation around z-axis)
        pose_msg.pose.orientation.z = np.sin(self.theta / 2.0)
        pose_msg.pose.orientation.w = np.cos(self.theta / 2.0)
        self.current_pose_pub.publish(pose_msg)

    def cmd_callback(self, msg):
        """
        Receive commands from network_delay_node.
        Store in history and forward to turtle (if no delay).
        """
        # Add command to history
        self.command_history.append({
            'linear': msg.linear.x,
            'angular': msg.angular.z
        })

        # If no delay is active, check for obstacles and apply the command
        if not self.delay_active:
            # Check if command would lead us into an obstacle
            if self.is_safe_command(msg):
                self.current_cmd = msg  # Apply command to internal state
                self.get_logger().debug(f"Applying command: v={msg.linear.x:.2f}, w={msg.angular.z:.2f}")
            else:
                # Stop if command would hit obstacle
                self.get_logger().warn("⚠️  Command blocked - would hit obstacle!")
                self.stop_robot()

    def is_safe_command(self, cmd):
        """
        Check if a command would lead the turtle into an obstacle.
        Simulates one step ahead to check for collisions.
        """
        # Simulate where we'd be after applying this command for dt seconds
        predicted_x = self.x + cmd.linear.x * np.cos(self.theta) * self.dt
        predicted_y = self.y + cmd.linear.x * np.sin(self.theta) * self.dt

        # Check distance to all obstacles
        for obs in self.obstacles:
            dist_to_obs = np.sqrt(
                (predicted_x - obs['x'])**2 +
                (predicted_y - obs['y'])**2
            )

            safe_distance = obs['radius'] + self.safety_margin

            # If too close, command is not safe
            if dist_to_obs < safe_distance:
                return False

        return True

    def delay_status_callback(self, msg):
        """
        Handle delay status changes from network_delay_node.
        When delay becomes active (True), predict goal and start MPC.
        When delay ends (False), stop MPC and resume normal operation.
        """
        self.delay_active = msg.data

        if self.delay_active:
            self.get_logger().warn("⚠️  Network delay detected! Activating MPC prediction...")

            # Predict goal state based on last 10 commands
            self.predict_goal_from_history()

            # Start MPC control loop
            if self.mpc_timer is None:
                self.mpc_timer = self.create_timer(self.dt, self.mpc_control_loop)
        else:
            self.get_logger().info("✓ Network delay resolved. Resuming normal operation.")

            # Stop MPC control loop
            if self.mpc_timer is not None:
                self.mpc_timer.cancel()
                self.mpc_timer = None

            # Reset predicted goal
            self.predicted_goal_x = None
            self.predicted_goal_y = None

    def predict_goal_from_history(self):
        """
        Predict where the turtle would be if the last 10 commands were applied.
        This gives us a goal state for the MPC to navigate to during delay.
        """
        if len(self.command_history) == 0:
            self.get_logger().warn("Cannot predict goal: insufficient data")
            return

        # Start from current pose
        predicted_x = self.x
        predicted_y = self.y
        predicted_theta = self.theta

        # Simulate forward using the command history
        for cmd in self.command_history:
            v = cmd['linear']
            w = cmd['angular']

            # Simple kinematic model (same as MPC)
            predicted_x += v * np.cos(predicted_theta) * self.dt
            predicted_y += v * np.sin(predicted_theta) * self.dt
            predicted_theta += w * self.dt

        self.predicted_goal_x = predicted_x
        self.predicted_goal_y = predicted_y

        # Publish predicted goal for RViz2 visualization
        goal_msg = Point()
        goal_msg.x = self.predicted_goal_x
        goal_msg.y = self.predicted_goal_y
        goal_msg.z = 0.0
        self.predicted_goal_pub.publish(goal_msg)

        self.get_logger().info(
            f"📍 Predicted goal: ({self.predicted_goal_x:.2f}, {self.predicted_goal_y:.2f}) "
            f"from current ({self.x:.2f}, {self.y:.2f})"
        )

    def mpc_control_loop(self):
        """
        MPC control loop - only runs during network delay.
        Navigates turtle toward predicted goal state.
        """
        if self.predicted_goal_x is None:
            return

        # Check if we've reached the predicted goal
        dist_to_goal = np.sqrt(
            (self.predicted_goal_x - self.x)**2 +
            (self.predicted_goal_y - self.y)**2
        )

        if dist_to_goal < 0.1:
            self.get_logger().info("Predicted goal reached! Waiting for delay to resolve...")
            self.stop_robot()
            return

        # Initial guess for optimizer
        initial_guess = np.zeros(self.horizon * 2)
        initial_guess[0::2] = 0.1  # Small forward velocity

        # Constraints on velocity
        bounds = []
        for _ in range(self.horizon):
            bounds.append((0.0, 2.0))    # v bounds
            bounds.append((-2.0, 2.0))   # w bounds

        # Run optimization
        result = minimize(
            self.cost_function,
            initial_guess,
            bounds=bounds,
            method='SLSQP'
        )

        # Apply first optimal control
        optimal_controls = result.x.reshape((self.horizon, 2))
        v_opt = optimal_controls[0, 0]
        w_opt = optimal_controls[0, 1]

        # Publish MPC trajectory for RViz2 visualization
        self.publish_mpc_trajectory(optimal_controls)

        # Apply MPC command to internal state
        self.current_cmd.linear.x = float(v_opt)
        self.current_cmd.angular.z = float(w_opt)

        self.get_logger().debug(f"MPC control: v={v_opt:.2f}, w={w_opt:.2f}, dist={dist_to_goal:.2f}")

    def model_unicycle(self, state, control):
        """
        Kinematic model of the turtle.
        State: [x, y, theta]
        Control: [v, w]
        """
        x, y, theta = state
        v, w = control

        x_new = x + v * np.cos(theta) * self.dt
        y_new = y + v * np.sin(theta) * self.dt
        theta_new = theta + w * self.dt

        return [x_new, y_new, theta_new]

    def cost_function(self, u_flat):
        """
        MPC cost function - minimize distance to predicted goal while avoiding obstacles.
        """
        controls = u_flat.reshape((self.horizon, 2))

        cost = 0.0
        current_state = [self.x, self.y, self.theta]

        for i in range(self.horizon):
            v = controls[i, 0]
            w = controls[i, 1]

            # Simulate one step forward
            current_state = self.model_unicycle(current_state, [v, w])

            # 1. Distance cost to predicted goal
            dx = self.predicted_goal_x - current_state[0]
            dy = self.predicted_goal_y - current_state[1]
            dist_sq = dx**2 + dy**2
            cost += dist_sq

            # 2. Heading cost (point toward goal)
            target_angle = np.arctan2(dy, dx)
            angle_error = target_angle - current_state[2]
            angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
            cost += 2.0 * (angle_error**2)

            # 3. Control effort cost (smooth movement)
            cost += 0.01 * (v**2 + w**2)

            # 4. OBSTACLE AVOIDANCE COST
            # Add large penalty if predicted path gets too close to any obstacle
            for obs in self.obstacles:
                # Distance from predicted position to obstacle center
                dist_to_obs = np.sqrt(
                    (current_state[0] - obs['x'])**2 +
                    (current_state[1] - obs['y'])**2
                )

                # Safe distance = obstacle radius + safety margin
                safe_distance = obs['radius'] + self.safety_margin

                # If we're too close to obstacle, add large penalty
                if dist_to_obs < safe_distance:
                    # Exponential penalty - gets very large as we get closer
                    # This creates a "repulsive force" from obstacles
                    penetration = safe_distance - dist_to_obs
                    cost += 1000.0 * np.exp(5.0 * penetration)

                # Even outside safe distance, add mild repulsive cost
                # This helps the path "bend around" obstacles smoothly
                elif dist_to_obs < safe_distance + 1.0:
                    avoidance_weight = 10.0 / (dist_to_obs - safe_distance + 0.1)
                    cost += avoidance_weight

        return cost

    def publish_mpc_trajectory(self, optimal_controls):
        """Publish the predicted MPC trajectory for RViz2 visualization"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        # Start from current pose
        current_state = [self.x, self.y, self.theta]

        # Simulate trajectory using optimal controls
        for i in range(self.horizon):
            v = optimal_controls[i, 0]
            w = optimal_controls[i, 1]

            # Simulate one step
            current_state = self.model_unicycle(current_state, [v, w])

            # Add to path
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = current_state[0]
            pose.pose.position.y = current_state[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = np.sin(current_state[2] / 2.0)
            pose.pose.orientation.w = np.cos(current_state[2] / 2.0)
            path.poses.append(pose)

        self.mpc_trajectory_pub.publish(path)

    def stop_robot(self):
        """Stop the turtle"""
        self.current_cmd = Twist()  # Zero velocity

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
