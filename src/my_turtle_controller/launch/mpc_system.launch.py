#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('my_turtle_controller')
    rviz_config = os.path.join(pkg_share, 'rviz', 'mpc_visualization.rviz')

    return LaunchDescription([
        # Launch network delay node
        Node(
            package='my_turtle_controller',
            executable='network_delay_node',
            name='network_delay_node',
            output='screen'
        ),

        # Launch turtle controller (with internal state tracking - no turtlesim needed!)
        Node(
            package='my_turtle_controller',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen'
        ),

        # Launch robot visualizer (blue arrow marker)
        Node(
            package='my_turtle_controller',
            executable='robot_visualizer',
            name='robot_visualizer',
            output='screen'
        ),

        # Launch RViz visualizer (obstacles and goal markers)
        Node(
            package='my_turtle_controller',
            executable='rviz_visualizer',
            name='rviz_visualizer',
            output='screen'
        ),

        # Launch RViz2 with config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        # Launch keyboard teleop (no turtlesim dependency!)
        Node(
            package='my_turtle_controller',
            executable='keyboard_teleop',
            name='teleop',
            output='screen',
            prefix='gnome-terminal --'  # Run in separate terminal
        ),
    ])
