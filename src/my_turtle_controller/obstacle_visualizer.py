#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ObstacleVisualizer(Node):
    def __init__(self):
        super().__init__('obstacle_visualizer')

        # Obstacle definitions are now only in rviz_visualizer.py
        # This node is kept for backward compatibility but does nothing
        # (obstacles are visualized directly in RViz2)

        self.get_logger().info("Obstacle Visualizer Started (RViz2-only mode)")
        self.get_logger().info("Obstacles are visualized in RViz2 via rviz_visualizer node")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
