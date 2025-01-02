#!/bin/bash

# Make the script executable and fail on errors
set -e

# Source ROS1 environment
source /opt/ros/noetic/setup.bash

# Create and build sample workspace if it doesn't exist
SAMPLE_WS="/sample_ws"
if [ ! -d "$SAMPLE_WS/src" ]; then
    echo "Creating sample ROS1 workspace..."
    mkdir -p $SAMPLE_WS/src
    cd $SAMPLE_WS/src
    
    # Initialize workspace with catkin
    catkin_init_workspace
    
    # Create a custom demo package
    catkin_create_pkg ros1_demos std_msgs rospy geometry_msgs turtlesim
    
    # Create a simple publisher node in the demo package
    mkdir -p ros1_demos/scripts
    cat > ros1_demos/scripts/turtle_circle.py << 'EOL'
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_turtle_in_circle():
    rospy.init_node('turtle_circle', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    vel_msg = Twist()
    vel_msg.linear.x = 1.0
    vel_msg.angular.z = 1.0
    
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_turtle_in_circle()
    except rospy.ROSInterruptException:
        pass
EOL

    # Make the script executable
    chmod +x ros1_demos/scripts/turtle_circle.py
    
    # Create a launch file
    mkdir -p ros1_demos/launch
    cat > ros1_demos/launch/demo.launch << 'EOL'
<launch>
    <!-- Start turtlesim node -->
    <node name="turtlesim" pkg="turtlesim" type="turtlesim_node" />
    
    <!-- Start our circle node -->
    <node name="turtle_circle" pkg="ros1_demos" type="turtle_circle.py" output="screen" />
</launch>
EOL

    # Update package.xml with additional dependencies
    cd ros1_demos
    sed -i '/<\/package>/i \  <exec_depend>turtlesim</exec_depend>' package.xml
    
    # Build the workspace
    cd $SAMPLE_WS
    catkin_make
    
    echo "Sample workspace created successfully!"
fi

# Source the workspace
source $SAMPLE_WS/devel/setup.bash

# Print available demo commands
echo "
ROS1 Demo Commands Available:
----------------------------
roslaunch ros1_demos demo.launch    # Launch turtlesim with circle motion
rosrun turtlesim turtlesim_node    # Run turtlesim alone
rosrun ros1_demos turtle_circle.py  # Run circle motion node
"

# Execute the command passed to the script, or start an interactive shell
if [ $# -eq 0 ]; then
    exec /bin/bash
else
    exec "$@"
fi 