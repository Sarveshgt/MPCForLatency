#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import numpy as np
import threading

class PerformancePlotter(Node):
    def __init__(self):
        super().__init__('performance_plotter')
        
        self.time_history = []
        self.real_vel_history = []
        self.ghost_vel_history = []
        
        # New: Track delay zones
        self.delay_active = False
        self.delay_zones = [] # List of (start_time, end_time)
        self.current_delay_start = None
        
        self.last_real_pos = None
        self.last_ghost_pos = None
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.create_subscription(PoseStamped, '/current_pose', self.real_cb, 10)
        self.create_subscription(PoseStamped, '/ghost_pose', self.ghost_cb, 10)
        
        # Subscribe to delay status for the background shading
        self.create_subscription(Bool, '/network_delay_active', self.delay_cb, 10)

    def delay_cb(self, msg):
        now = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        if msg.data and not self.delay_active:
            # Delay just started
            self.delay_active = True
            self.current_delay_start = now
        elif not msg.data and self.delay_active:
            # Delay just ended
            self.delay_active = False
            if self.current_delay_start is not None:
                self.delay_zones.append((self.current_delay_start, now))
                self.current_delay_start = None

    def real_cb(self, msg):
        self.update_data(msg, is_ghost=False)

    def ghost_cb(self, msg):
        self.update_data(msg, is_ghost=True)

    def update_data(self, msg, is_ghost):
        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        curr_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        
        if is_ghost:
            if self.last_ghost_pos is not None:
                dist = np.linalg.norm(curr_pos - self.last_ghost_pos)
                vel = dist / 0.1 
                if len(self.time_history) > len(self.ghost_vel_history):
                    self.ghost_vel_history.append(vel)
            self.last_ghost_pos = curr_pos
        else:
            if self.last_real_pos is not None:
                dist = np.linalg.norm(curr_pos - self.last_real_pos)
                vel = dist / 0.1
                self.time_history.append(t)
                self.real_vel_history.append(vel)
                while len(self.ghost_vel_history) < len(self.real_vel_history):
                     self.ghost_vel_history.append(0)
            self.last_real_pos = curr_pos

def plot_thread(node):
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Styling
    real_line, = ax.plot([], [], 'b-', linewidth=2.5, label='MPC Robot (Optimized)')
    ghost_line, = ax.plot([], [], 'r--', linewidth=2.0, alpha=0.7, label='Ghost (Naive/Lagged)')
    
    ax.set_ylim(-0.2, 3.0)
    ax.set_title("Robot Velocity vs Network Delay")
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.legend(loc='upper left')
    ax.grid(True, linestyle=':', alpha=0.6)
    
    # Text annotation for delay
    delay_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, 
                         color='red', fontweight='bold', fontsize=12)
    
    while rclpy.ok():
        if len(node.time_history) > 10:
            x_data = node.time_history[-150:] # Show last ~15s
            y_real = node.real_vel_history[-150:]
            y_ghost = node.ghost_vel_history[-150:]
            min_len = min(len(x_data), len(y_real), len(y_ghost))
            
            real_line.set_data(x_data[:min_len], y_real[:min_len])
            ghost_line.set_data(x_data[:min_len], y_ghost[:min_len])
            
            # Highlight Delay Zones
            # Clear previous spans to avoid clutter (inefficient but simple for demo)
            [p.remove() for p in ax.patches] 
            
            # Draw completed delay zones
            for start, end in node.delay_zones:
                # Only draw if visible in current window
                if end > x_data[0]:
                    ax.axvspan(start, end, color='red', alpha=0.15)
            
            # Draw active delay zone
            if node.delay_active and node.current_delay_start is not None:
                ax.axvspan(node.current_delay_start, x_data[-1], color='red', alpha=0.15)
                delay_text.set_text("⚠️ NETWORK LAG DETECTED")
            else:
                delay_text.set_text("")

            ax.set_xlim(x_data[0], x_data[-1] + 0.5)
            fig.canvas.draw()
            fig.canvas.flush_events()
        
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = PerformancePlotter()
    t = threading.Thread(target=rclpy.spin, args=(node,))
    t.start()
    plot_thread(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()