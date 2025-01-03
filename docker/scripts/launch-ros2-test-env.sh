#!/bin/bash

# Make the script executable and fail on errors
set -e

# Switch to ros user if we're root
if [ "$(id -u)" = "0" ]; then
    exec sudo -u ros "$0" "$@"
fi

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Add Jazzy-specific handling if needed
if [ "$ROS_DISTRO" = "jazzy" ]; then
    source /home/ros/.venv/bin/activate
fi

# Create and build sample workspace if it doesn't exist
SAMPLE_WS="/sample_ws"
if [ ! -d "$SAMPLE_WS/src" ]; then
    echo "Creating sample ROS2 workspace..."
    mkdir -p $SAMPLE_WS/src
    cd $SAMPLE_WS/src
    
    # Create a custom demo package
    ros2 pkg create --build-type ament_python ros2_demos \
        --dependencies rclpy geometry_msgs turtlesim
    
    # Create a simple publisher node in the demo package
    mkdir -p ros2_demos/ros2_demos
    cat > ros2_demos/ros2_demos/turtle_circle.py << 'EOL'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleCircle(Node):
    def __init__(self):
        super().__init__('turtle_circle')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TurtleCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOL

    # Update package setup files
    cat > ros2_demos/setup.py << 'EOL'
from setuptools import setup

package_name = 'ros2_demos'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2_user',
    maintainer_email='user@todo.todo',
    description='ROS2 demo package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_circle = ros2_demos.turtle_circle:main',
        ],
    },
)
EOL

    # Create launch directory and launch file
    mkdir -p ros2_demos/launch
    cat > ros2_demos/launch/demo.launch.py << 'EOL'
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='ros2_demos',
            executable='turtle_circle',
            name='turtle_circle',
            output='screen'
        )
    ])
EOL

    # Create the resource directory and marker file
    mkdir -p ros2_demos/resource
    touch ros2_demos/resource/ros2_demos

    # Make the Python script executable
    chmod +x ros2_demos/ros2_demos/turtle_circle.py

    cd $SAMPLE_WS
    
    # Source ROS environment again to ensure rosdep can find ROS packages
    source /opt/ros/${ROS_DISTRO}/setup.bash
    
    # Update rosdep as non-root user
    rosdep update
    
    echo "Installing dependencies with rosdep..."
    sudo -E rosdep install --from-paths src -y --ignore-src
    
    # Build the workspace
    colcon build
    
    echo "Sample workspace created successfully!"
fi

# Source the workspace
source $SAMPLE_WS/install/setup.bash

# Print available demo commands
echo "
ROS2 Demo Commands Available:
----------------------------
ros2 launch ros2_demos demo.launch.py    # Launch turtlesim with circle motion
ros2 run turtlesim turtlesim_node       # Run turtlesim alone
ros2 run ros2_demos turtle_circle       # Run circle motion node
"

# Execute the command passed to the script, or start an interactive shell
if [ $# -eq 0 ]; then
    exec /bin/bash
else
    exec "$@"
fi 