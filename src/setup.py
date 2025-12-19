from setuptools import find_packages, setup

package_name = 'my_turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz', ['my_turtle_controller/rviz/mpc_visualization.rviz']),
        ('share/' + package_name + '/launch', ['my_turtle_controller/launch/mpc_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sarveshv',
    maintainer_email='sarveshv@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle_controller = my_turtle_controller.turtle_controller:main',
            'network_delay_node = my_turtle_controller.network_delay_node:main',
            'jitter_node = my_turtle_controller.jitter_node:main',
            'tunnel_node = my_turtle_controller.tunnel_node:main',
            'mpc_supervisor_node = my_turtle_controller.mpc_supervisor_node:main',
            'obstacle_visualizer = my_turtle_controller.obstacle_visualizer:main',
            'rviz_visualizer = my_turtle_controller.rviz_visualizer:main',
            'robot_visualizer = my_turtle_controller.robot_visualizer:main',
            'keyboard_teleop = my_turtle_controller.keyboard_teleop:main',
            'demo_sequence = my_turtle_controller.demo_sequence:main',
            'ghost_controller = my_turtle_controller.ghost_controller:main',     # <--- NEW
            'performance_plotter = my_turtle_controller.performance_plotter:main', # <--- NEW
        ],
    },
)
