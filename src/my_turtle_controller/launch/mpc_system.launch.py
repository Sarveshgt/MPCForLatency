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
        # 1. Network Delay Node
        Node(
            package='my_turtle_controller',
            executable='network_delay_node',
            name='network_delay_node',
            output='screen'
        ),

        # 2. Real Turtle Controller (MPC)
        Node(
            package='my_turtle_controller',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen'
        ),

        # 3. NEW: Ghost Controller (The delayed/choppy robot)
        Node(
            package='my_turtle_controller',
            executable='ghost_controller',
            name='ghost_controller',
            output='screen'
        ),

        # 4. Robot Visualizer (Now handles BOTH cars)
        Node(
            package='my_turtle_controller',
            executable='robot_visualizer',
            name='robot_visualizer',
            output='screen'
        ),

        # 5. RViz Visualizer (Obstacles/Goal)
        Node(
            package='my_turtle_controller',
            executable='rviz_visualizer',
            name='rviz_visualizer',
            output='screen'
        ),

        # 6. NEW: Performance Plotter (The Graph)
        Node(
            package='my_turtle_controller',
            executable='performance_plotter',
            name='performance_plotter',
            output='screen'
        ),

        # 7. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        # 8. Keyboard Teleop
        Node(
            package='my_turtle_controller',
            executable='keyboard_teleop',
            name='teleop',
            output='screen',
            prefix='gnome-terminal --'
        ),
    ])
