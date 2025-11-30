#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
from scipy.optimize import minimize

class TurtleMPC(Node):
    def __init__(self):
        super().__init__('turtle_mpc_node')
        
        # Initialize Publisher and Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.pose = Pose()
        self.pose_received = False

        # MPC Parameters
        self.horizon = 10   # Look 10 steps into the future
        self.dt = 0.1       # Time step duration (seconds)
        
        # Get Goal from Arguments
        if len(sys.argv) < 3:
            self.get_logger().error("Usage: ros2 run <pkg> <node> x y")
            sys.exit(1)
            
        self.goal_x = float(sys.argv[1])
        self.goal_y = float(sys.argv[2])

        # Control Loop
        self.timer = self.create_timer(self.dt, self.control_loop)

    def pose_callback(self, data):
        self.pose = data
        self.pose_received = True

    def model_unicycle(self, state, control):
        """
        Kinematic model of the turtle.
        State: [x, y, theta]
        Control: [v, w]
        """
        x, y, theta = state
        v, w = control
        
        # Update state based on physics
        x_new = x + v * np.cos(theta) * self.dt
        y_new = y + v * np.sin(theta) * self.dt
        theta_new = theta + w * self.dt
        
        return [x_new, y_new, theta_new]

    def cost_function(self, u_flat):
        """
        The Optimizer calls this function thousands of times.
        It asks: "If I use these inputs (u), how bad is the result (cost)?"
        """
        # Reshape the flat input array back into pairs of [v, w]
        controls = u_flat.reshape((self.horizon, 2))
        
        cost = 0.0
        # Start simulation from current actual position
        current_state = [self.pose.x, self.pose.y, self.pose.theta]
        
        for i in range(self.horizon):
            v = controls[i, 0]
            w = controls[i, 1]
            
            # Simulate one step forward
            current_state = self.model_unicycle(current_state, [v, w])
            
            # --- COST CALCULATION ---
            
            # 1. Distance Cost (Get closer to goal)
            dx = self.goal_x - current_state[0]
            dy = self.goal_y - current_state[1]
            dist_sq = dx**2 + dy**2
            cost += dist_sq
            
            # 2. Heading Cost (The Fix!)
            # Encourage robot to point at goal, even if stationary.
            target_angle = np.arctan2(dy, dx)
            angle_error = target_angle - current_state[2]
            
            # Normalize angle to -pi to pi
            angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
            
            # Add to cost (Weight: 2.0 to make turning high priority)
            cost += 2.0 * (angle_error**2)
            
            # 3. Control Effort Cost (Save energy, smooth movement)
            cost += 0.01 * (v**2 + w**2)

        return cost

    def control_loop(self):
        if not self.pose_received:
            return

        # Check if we are close enough to stop
        dist_to_goal = np.sqrt((self.goal_x - self.pose.x)**2 + (self.goal_y - self.pose.y)**2)
        if dist_to_goal < 0.1:
            self.stop_robot()
            self.get_logger().info("Goal Reached!")
            sys.exit()

        # Initial guess for the optimizer
        # TRICK: Initialize with small forward velocity (0.1) instead of zeros.
        # This helps the math "gradients" flow so the solver doesn't get stuck.
        initial_guess = np.zeros(self.horizon * 2)
        initial_guess[0::2] = 0.1 

        # Constraints: 
        # Linear velocity (v) between 0 and 2.0
        # Angular velocity (w) between -2.0 and 2.0
        bounds = []
        for _ in range(self.horizon):
            bounds.append((0.0, 2.0))   # v bounds
            bounds.append((-2.0, 2.0)) # w bounds

        # Run the Optimization
        result = minimize(self.cost_function, initial_guess, bounds=bounds, method='SLSQP')
        
        # The result contains the optimal sequence for the next 10 steps.
        # We only take the FIRST step [v0, w0] and apply it.
        optimal_controls = result.x.reshape((self.horizon, 2))
        v_opt = optimal_controls[0, 0]
        w_opt = optimal_controls[0, 1]

        # Publish the optimal control
        msg = Twist()
        msg.linear.x = float(v_opt)
        msg.angular.z = float(w_opt)
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()