#!/usr/bin/env python3

import os
import subprocess
from typing import Optional
from ros_to_markdown.core.ros_detector import ROSDetector, ROSVersion

################################################################################
# 1. PACKAGE CONFIGURATION
################################################################################

# ROS1 package definitions (package_name -> dependencies)
ROS1_PACKAGES = {
    "sensor_data": "std_msgs rospy geometry_msgs",
    "data_processing": "std_msgs rospy",
    "environment_integration": "std_msgs rospy geometry_msgs",
    "robot_control": "std_msgs rospy geometry_msgs",
    "action_server": "std_msgs rospy geometry_msgs actionlib actionlib_msgs",
    "tricky_scenarios": "std_msgs rospy geometry_msgs std_srvs",  # added std_srvs for delayed_response
    "visualization": "std_msgs rospy",
    "user_interface": "std_msgs rospy geometry_msgs"
}

# ROS2 package definitions (package_name -> dependencies)
ROS2_PACKAGES = {
    "sensor_data": "rclpy std_msgs geometry_msgs",
    "data_processing": "rclpy std_msgs geometry_msgs",
    "environment_integration": "rclpy std_msgs geometry_msgs",
    "robot_control": "rclpy std_msgs geometry_msgs",
    "action_server": "rclpy std_msgs geometry_msgs action_msgs actionlib_msgs",
    "tricky_scenarios": "rclpy std_msgs geometry_msgs std_srvs",  # for delayed_response
    "visualization": "rclpy std_msgs geometry_msgs",
    "user_interface": "rclpy std_msgs geometry_msgs"
}

################################################################################
# 2. CUSTOM INTERFACE FILES (MSG, SRV, ACTION)
################################################################################

CUSTOM_FILES = {
    "data_processing": {
        "msg": {
            "FilteredData.msg": """float64 temperature
float64 humidity
bool    valid
string  timestamp
"""
        }
    },
    "environment_integration": {
        "msg": {
            "EnvironmentData.msg": """# A placeholder for environment data
# e.g., a simple map or contextual info
float64[] map_data
string frame_id
"""
        }
    },
    "action_server": {
        "action": {
            "MoveToGoal.action": """# Goal
geometry_msgs/PoseStamped target_pose

---
# Result
bool success

---
# Feedback
string current_status
"""
        }
    },
    "tricky_scenarios": {
        "srv": {
            "GetLatestReadings.srv": """# Request
# (Intentionally empty; all we need is to retrieve stored data)

---
# Response
float64 temperature
float64 humidity
bool    valid
string  timestamp
"""
        }
    }
}

################################################################################
# 3. ROS1 NODE SCRIPTS (Python)
################################################################################
ROS1_SCRIPTS = {
    "sensor_data": {
        "scripts": {
            "temperature_publisher.py": r"""#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import random

def main():
    rospy.init_node('temperature_publisher', anonymous=True)
    pub = rospy.Publisher('/sensor/temperature', Float64, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        temp_value = 20.0 + random.uniform(-1.0, 1.0)
        rospy.loginfo("Publishing temperature: %.2f", temp_value)
        pub.publish(temp_value)
        rate.sleep()

if __name__ == '__main__':
    main()
""",
            "humidity_publisher.py": r"""#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import random

def main():
    rospy.init_node('humidity_publisher', anonymous=True)
    pub = rospy.Publisher('/sensor/humidity', Float64, queue_size=10)
    rate = rospy.Rate(2)  # 2Hz

    while not rospy.is_shutdown():
        humidity_value = 50.0 + random.uniform(-5.0, 5.0)
        rospy.loginfo("Publishing humidity: %.2f", humidity_value)
        pub.publish(humidity_value)
        rate.sleep()

if __name__ == '__main__':
    main()
"""
        }
    },
    "data_processing": {
        "scripts": {
            "data_filter.py": r"""#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from data_processing.msg import FilteredData

class DataFilterNode:
    def __init__(self):
        rospy.init_node('data_filter', anonymous=True)
        self.pub_filtered = rospy.Publisher('/data/filtered', FilteredData, queue_size=10)

        rospy.Subscriber('/sensor/temperature', Float64, self.temp_callback)
        rospy.Subscriber('/sensor/humidity', Float64, self.hum_callback)

        self.temperature = None
        self.humidity = None

    def temp_callback(self, msg):
        self.temperature = msg.data
        self.publish_filtered()

    def hum_callback(self, msg):
        self.humidity = msg.data
        self.publish_filtered()

    def publish_filtered(self):
        if self.temperature is not None and self.humidity is not None:
            filtered = FilteredData()
            filtered.temperature = self.temperature
            filtered.humidity = self.humidity
            filtered.valid = True
            filtered.timestamp = str(rospy.Time.now())
            self.pub_filtered.publish(filtered)
            rospy.loginfo("DataFilter: Published FilteredData")

def main():
    node = DataFilterNode()
    rospy.spin()

if __name__ == '__main__':
    main()
""",
            "data_processor.py": r"""#!/usr/bin/env python3
import rospy
from data_processing.msg import FilteredData
from std_msgs.msg import String

class DataProcessorNode:
    def __init__(self):
        rospy.init_node('data_processor', anonymous=True)
        self.pub_processed = rospy.Publisher('/data/processed', String, queue_size=10)
        rospy.Subscriber('/data/filtered', FilteredData, self.filtered_callback)

    def filtered_callback(self, msg):
        text_out = "Processed => temp: {:.2f}, hum: {:.2f}, valid: {}".format(
            msg.temperature, msg.humidity, msg.valid)
        rospy.loginfo("DataProcessor: %s", text_out)
        self.pub_processed.publish(text_out)

def main():
    node = DataProcessorNode()
    rospy.spin()

if __name__ == '__main__':
    main()
"""
        }
    },
    "environment_integration": {
        "scripts": {
            "environment_builder.py": r"""#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from environment_integration.msg import EnvironmentData

class EnvironmentBuilderNode:
    def __init__(self):
        rospy.init_node('environment_builder', anonymous=True)
        self.pub_env = rospy.Publisher('/environment/data', EnvironmentData, queue_size=10)
        rospy.Subscriber('/data/processed', String, self.processed_callback)

    def processed_callback(self, msg):
        env_msg = EnvironmentData()
        env_msg.frame_id = "map"
        # Convert message string into a dummy array of ASCII values
        env_msg.map_data = [ord(c) for c in msg.data]
        self.pub_env.publish(env_msg)
        rospy.loginfo("EnvironmentBuilder: Published EnvironmentData")

def main():
    node = EnvironmentBuilderNode()
    rospy.spin()

if __name__ == '__main__':
    main()
"""
        }
    },
    "robot_control": {
        "scripts": {
            "move_robot.py": r"""#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class MoveRobotNode:
    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.pub_status = rospy.Publisher('/robot/status', String, queue_size=10)

    def cmd_vel_callback(self, cmd):
        rospy.loginfo("move_robot: Received cmd_vel linear=%.2f, angular=%.2f",
                      cmd.linear.x, cmd.angular.z)
        status_msg = "MOVING with linear=%.2f, angular=%.2f" % (cmd.linear.x, cmd.angular.z)
        self.pub_status.publish(status_msg)

def main():
    node = MoveRobotNode()
    rospy.spin()

if __name__ == '__main__':
    main()
"""
        }
    },
    "action_server": {
        "scripts": {
            "move_to_goal.py": r"""#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Twist
from environment_integration.msg import EnvironmentData
from action_server.msg import MoveToGoalAction, MoveToGoalFeedback, MoveToGoalResult

class MoveToGoalServer:
    def __init__(self):
        rospy.init_node('move_to_goal_server', anonymous=True)
        self.server = actionlib.SimpleActionServer(
            '/move_to_goal',
            MoveToGoalAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()

        rospy.Subscriber('/environment/data', EnvironmentData, self.env_callback)
        self.environment_data = None

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def env_callback(self, msg):
        self.environment_data = msg

    def execute_cb(self, goal):
        rospy.loginfo("MoveToGoal: Received goal => target_pose=(%.2f, %.2f)",
                      goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

        feedback = MoveToGoalFeedback()
        result = MoveToGoalResult()

        rate = rospy.Rate(2)
        for i in range(5):
            if self.server.is_preempt_requested():
                rospy.loginfo("MoveToGoal: Preempted!")
                self.server.set_preempted()
                return

            twist = Twist()
            twist.linear.x = 0.1
            self.cmd_vel_pub.publish(twist)

            feedback.current_status = "Moving... step {}/5".format(i+1)
            self.server.publish_feedback(feedback)
            rate.sleep()

        # Stop
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        result.success = True
        self.server.set_succeeded(result, "Goal Reached")

def main():
    node = MoveToGoalServer()
    rospy.spin()

if __name__ == '__main__':
    main()
"""
        }
    },
    "tricky_scenarios": {
        "scripts": {
            "conflicting_publishers.py": r"""#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import random

def main():
    rospy.init_node('conflicting_temperature_publisher', anonymous=True)
    pub = rospy.Publisher('/sensor/temperature', Float64, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        temp_value = 100.0 + random.uniform(-5.0, 5.0)
        rospy.logwarn("Conflicting publisher: publishing temperature: %.2f", temp_value)
        pub.publish(temp_value)
        rate.sleep()

if __name__ == '__main__':
    main()
""",
            "delayed_response.py": r"""#!/usr/bin/env python3
import rospy
import time
from std_srvs.srv import Trigger, TriggerResponse

def handle_delayed_request(req):
    rospy.loginfo("DelayedResponse: Received request. Sleeping 5 seconds...")
    time.sleep(5)
    return TriggerResponse(
        success=True,
        message="DelayedResponse done after 5s"
    )

def main():
    rospy.init_node('delayed_response_server', anonymous=True)
    s = rospy.Service('/delayed_response', Trigger, handle_delayed_request)
    rospy.loginfo("DelayedResponse: service /delayed_response is ready.")
    rospy.spin()

if __name__ == '__main__':
    main()
"""
        }
    },
    "visualization": {
        "scripts": {
            "visualize_data.py": r"""#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def data_callback(msg):
    rospy.loginfo("VISUALIZE: Received processed data => %s", msg.data)

def main():
    rospy.init_node('visualize_data', anonymous=True)
    rospy.Subscriber('/data/processed', String, data_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
"""
        }
    },
    "user_interface": {
        "scripts": {
            "command_listener.py": r"""#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('command_listener', anonymous=True)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    forward = True

    while not rospy.is_shutdown():
        twist = Twist()
        if forward:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.5

        rospy.loginfo("UserInterface: Publishing Twist forward=%s", forward)
        pub_cmd_vel.publish(twist)

        forward = not forward
        rate.sleep()

if __name__ == '__main__':
    main()
"""
        }
    }
}


################################################################################
# 4. ROS2 NODE SCRIPTS (Python)
################################################################################

ROS2_SCRIPTS = {
    "sensor_data": {
        # We'll place them in sensor_data/sensor_data/ for ament_python structure
        "scripts": {
            "temperature_publisher.py": r"""#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.pub = self.create_publisher(Float64, '/sensor/temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_temp)

    def publish_temp(self):
        temp_value = 20.0 + random.uniform(-1.0, 1.0)
        self.get_logger().info(f"Publishing temperature: {temp_value:.2f}")
        msg = Float64()
        msg.data = temp_value
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
""",
            "humidity_publisher.py": r"""#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class HumidityPublisher(Node):
    def __init__(self):
        super().__init__('humidity_publisher')
        self.pub = self.create_publisher(Float64, '/sensor/humidity', 10)
        self.timer = self.create_timer(0.5, self.publish_humidity)  # 2Hz

    def publish_humidity(self):
        humidity_value = 50.0 + random.uniform(-5.0, 5.0)
        self.get_logger().info(f"Publishing humidity: {humidity_value:.2f}")
        msg = Float64()
        msg.data = humidity_value
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumidityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
        }
    },
    "data_processing": {
        "scripts": {
            "data_filter.py": r"""#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from data_processing.msg import FilteredData

class DataFilter(Node):
    def __init__(self):
        super().__init__('data_filter')
        self.temp_sub = self.create_subscription(Float64, '/sensor/temperature', self.temp_callback, 10)
        self.hum_sub = self.create_subscription(Float64, '/sensor/humidity', self.hum_callback, 10)
        self.pub_filtered = self.create_publisher(FilteredData, '/data/filtered', 10)

        self.temperature = None
        self.humidity = None

    def temp_callback(self, msg):
        self.temperature = msg.data
        self.publish_filtered()

    def hum_callback(self, msg):
        self.humidity = msg.data
        self.publish_filtered()

    def publish_filtered(self):
        if self.temperature is not None and self.humidity is not None:
            fmsg = FilteredData()
            fmsg.temperature = float(self.temperature)
            fmsg.humidity = float(self.humidity)
            fmsg.valid = True
            fmsg.timestamp = str(self.get_clock().now().to_msg())
            self.pub_filtered.publish(fmsg)
            self.get_logger().info("Published FilteredData")

def main(args=None):
    rclpy.init(args=args)
    node = DataFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
""",
            "data_processor.py": r"""#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from data_processing.msg import FilteredData
from std_msgs.msg import String

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')
        self.pub_processed = self.create_publisher(String, '/data/processed', 10)
        self.sub_filtered = self.create_subscription(FilteredData, '/data/filtered', self.filtered_callback, 10)

    def filtered_callback(self, msg):
        text_out = f"Processed => temp: {msg.temperature:.2f}, hum: {msg.humidity:.2f}, valid: {msg.valid}"
        self.get_logger().info(text_out)
        self.pub_processed.publish(String(data=text_out))

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
        }
    },
    # ... Similarly for environment_integration, robot_control, action_server, etc.
    # For brevity, we'll show one or two more as examples:
    "environment_integration": {
        "scripts": {
            "environment_builder.py": r"""#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from environment_integration.msg import EnvironmentData

class EnvironmentBuilder(Node):
    def __init__(self):
        super().__init__('environment_builder')
        self.pub_env = self.create_publisher(EnvironmentData, '/environment/data', 10)
        self.sub_processed = self.create_subscription(String, '/data/processed', self.processed_callback, 10)

    def processed_callback(self, msg):
        env_msg = EnvironmentData()
        env_msg.frame_id = "map"
        env_msg.map_data = [float(ord(c)) for c in msg.data]
        self.pub_env.publish(env_msg)
        self.get_logger().info("Published EnvironmentData")

def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
        }
    }
    # In a real solution, you'd define the remaining scripts for ROS2:
    #   robot_control/move_robot.py
    #   action_server/move_to_goal.py
    #   tricky_scenarios/conflicting_publishers.py, etc.
    #   user_interface/command_listener.py
    #   visualization/visualize_data.py
    # etc. 
    # But we'll keep it concise for demonstration.
}

################################################################################
# WORKSPACE CREATION (AS BEFORE) + NODE SCRIPT CREATION
################################################################################

def create_ros1_workspace(base_path: str) -> None:
    """Create the ROS1 workspace structure and packages, including custom files & node scripts."""
    workspace = os.path.join(base_path, "ros1_workspace")
    src_dir = os.path.join(workspace, "src")

    # Create base workspace folders
    os.makedirs(src_dir, exist_ok=True)
    os.makedirs(os.path.join(workspace, "build"), exist_ok=True)
    os.makedirs(os.path.join(workspace, "devel"), exist_ok=True)

    # Initialize catkin workspace
    subprocess.run(["catkin_init_workspace"], cwd=src_dir)

    # Create packages
    for pkg, deps in ROS1_PACKAGES.items():
        create_ros1_package(pkg, deps, workspace)

    # Create interface files
    for pkg_name, interfaces in CUSTOM_FILES.items():
        pkg_path = os.path.join(src_dir, pkg_name)
        if os.path.isdir(pkg_path):
            create_interface_files(pkg_path, interfaces)
            update_ros1_cmakelists_and_packagexml(pkg_path, interfaces)
        else:
            print(f"[ROS1] Package {pkg_name} not found; skipping interface creation.")

    # Create node scripts
    for pkg_name, script_data in ROS1_SCRIPTS.items():
        pkg_path = os.path.join(src_dir, pkg_name)
        if os.path.isdir(pkg_path):
            write_ros1_scripts(pkg_path, script_data)
        else:
            print(f"[ROS1] Package {pkg_name} not found; skipping script creation.")


def create_ros2_workspace(base_path: str) -> None:
    """Create the ROS2 workspace structure and packages, including custom files & node scripts."""
    workspace = os.path.join(base_path, "ros2_workspace")
    src_dir = os.path.join(workspace, "src")

    # Create base workspace folders
    os.makedirs(src_dir, exist_ok=True)
    os.makedirs(os.path.join(workspace, "build"), exist_ok=True)
    os.makedirs(os.path.join(workspace, "install"), exist_ok=True)

    # Create packages
    for pkg, deps in ROS2_PACKAGES.items():
        create_ros2_package(pkg, deps, workspace)

    # Create interface files
    for pkg_name, interfaces in CUSTOM_FILES.items():
        pkg_path = os.path.join(src_dir, pkg_name)
        if os.path.isdir(pkg_path):
            create_interface_files(pkg_path, interfaces)
            update_ros2_package_files(pkg_path, interfaces)
        else:
            print(f"[ROS2] Package {pkg_name} not found; skipping interface creation.")

    # Create node scripts
    for pkg_name, script_data in ROS2_SCRIPTS.items():
        pkg_path = os.path.join(src_dir, pkg_name)
        if os.path.isdir(pkg_path):
            write_ros2_scripts(pkg_path, script_data)
        else:
            print(f"[ROS2] Package {pkg_name} not found; skipping script creation.")


################################################################################
# HELPERS FOR PACKAGE CREATION
################################################################################

def create_ros1_package(package_name: str, dependencies: str, workspace: str) -> None:
    src_dir = os.path.join(workspace, "src")
    print(f"[ROS1] Creating package: {package_name} with deps: {dependencies}")
    subprocess.run(["catkin_create_pkg", package_name] + dependencies.split(), cwd=src_dir)

def create_ros2_package(package_name: str, dependencies: str, workspace: str) -> None:
    src_dir = os.path.join(workspace, "src")
    print(f"[ROS2] Creating package: {package_name} with deps: {dependencies}")
    subprocess.run([
        "ros2", "pkg", "create", package_name,
        "--build-type", "ament_python",
        "--dependencies"
    ] + dependencies.split(), cwd=src_dir)


################################################################################
# HELPERS FOR INTERFACE FILE CREATION
################################################################################

def create_interface_files(package_path: str, interfaces: dict) -> None:
    """Create directories (msg/, srv/, action/) and write placeholder interface files."""
    for interface_type in ["msg", "srv", "action"]:
        if interface_type in interfaces:
            dir_path = os.path.join(package_path, interface_type)
            os.makedirs(dir_path, exist_ok=True)
            for filename, content in interfaces[interface_type].items():
                file_path = os.path.join(dir_path, filename)
                if not os.path.exists(file_path):
                    print(f"Creating interface file: {file_path}")
                    with open(file_path, "w") as f:
                        f.write(content)


################################################################################
# ROS1 BUILD FILE UPDATES (CMakeLists, package.xml)
################################################################################

def update_ros1_cmakelists_and_packagexml(package_path: str, interfaces: dict) -> None:
    cmakelists_path = os.path.join(package_path, "CMakeLists.txt")
    packagexml_path = os.path.join(package_path, "package.xml")

    # 1) CMakeLists.txt
    if os.path.isfile(cmakelists_path):
        with open(cmakelists_path, "r") as f:
            data = f.read()

        # Insert find_package(... message_generation ...)
        if "find_package(catkin REQUIRED" not in data or "message_generation" not in data:
            find_package_block = """find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)\n"""
            data = find_package_block + data

        # Add message/service/action
        add_msg = ""
        if "msg" in interfaces:
            add_msg += "add_message_files(\n  FILES\n"
            for m in interfaces["msg"]:
                add_msg += f"    {m}\n"
            add_msg += ")\n\n"

        add_srv = ""
        if "srv" in interfaces:
            add_srv += "add_service_files(\n  FILES\n"
            for s in interfaces["srv"]:
                add_srv += f"    {s}\n"
            add_srv += ")\n\n"

        add_action = ""
        if "action" in interfaces:
            add_action += "add_action_files(\n  FILES\n"
            for a in interfaces["action"]:
                add_action += f"    {a}\n"
            add_action += ")\n\n"

        if (add_msg or add_srv or add_action) and "generate_messages(" not in data:
            generation_block = add_msg + add_srv + add_action
            generation_block += "generate_messages(\n  DEPENDENCIES std_msgs geometry_msgs\n)\n"
            data += "\n" + generation_block
            data += "\ncatkin_package(CATKIN_DEPENDS message_runtime)\n"

        with open(cmakelists_path, "w") as f:
            f.write(data)

    # 2) package.xml
    if os.path.isfile(packagexml_path):
        with open(packagexml_path, "r") as f:
            pkgxml = f.read()

        if "<build_depend>message_generation</build_depend>" not in pkgxml:
            idx = pkgxml.find("</package>")
            pkgxml = pkgxml[:idx] + "  <build_depend>message_generation</build_depend>\n" + pkgxml[idx:]
        if "<exec_depend>message_runtime</exec_depend>" not in pkgxml:
            idx = pkgxml.find("</package>")
            pkgxml = pkgxml[:idx] + "  <exec_depend>message_runtime</exec_depend>\n" + pkgxml[idx:]

        with open(packagexml_path, "w") as f:
            f.write(pkgxml)


################################################################################
# ROS2 BUILD FILE UPDATES (package.xml, setup.py)
################################################################################

def update_ros2_package_files(package_path: str, interfaces: dict) -> None:
    packagexml_path = os.path.join(package_path, "package.xml")
    if os.path.isfile(packagexml_path):
        with open(packagexml_path, "r") as f:
            pkgxml = f.read()

        needed_deps = [
            "rosidl_default_generators",
            "rosidl_default_runtime",
            "builtin_interfaces",
        ]
        for dep in needed_deps:
            if f"<build_depend>{dep}</build_depend>" not in pkgxml:
                idx = pkgxml.find("</package>")
                insert_str = f"  <build_depend>{dep}</build_depend>\n  <exec_depend>{dep}</exec_depend>\n"
                pkgxml = pkgxml[:idx] + insert_str + pkgxml[idx:]

        with open(packagexml_path, "w") as f:
            f.write(pkgxml)

    # If there's no setup.py, create a basic one
    setup_py_path = os.path.join(package_path, "setup.py")
    if not os.path.exists(setup_py_path):
        pkg_name = os.path.basename(package_path)
        content = f"""from setuptools import setup

package_name = '{pkg_name}'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auto_generated',
    maintainer_email='user@todo.todo',
    description='ROS2 Package: {pkg_name}',
    license='TODO',
    tests_require=['pytest'],
    entry_points={{
        'console_scripts': [
        ],
    }},
)
"""
        python_dir = os.path.join(package_path, pkg_name)
        os.makedirs(python_dir, exist_ok=True)
        open(os.path.join(python_dir, "__init__.py"), "a").close()

        with open(setup_py_path, "w") as f:
            f.write(content)


################################################################################
# SCRIPT WRITING HELPERS (ROS1 & ROS2)
################################################################################

def write_ros1_scripts(package_path: str, script_data: dict) -> None:
    """
    Create a 'scripts' folder in the ROS1 package and write each script file, 
    then chmod +x them.
    """
    scripts_dir = os.path.join(package_path, "scripts")
    os.makedirs(scripts_dir, exist_ok=True)

    if "scripts" in script_data:
        for filename, content in script_data["scripts"].items():
            file_path = os.path.join(scripts_dir, filename)
            print(f"[ROS1] Creating script {file_path}")
            with open(file_path, "w") as f:
                f.write(content)
            os.chmod(file_path, 0o755)

def write_ros2_scripts(package_path: str, script_data: dict) -> None:
    """
    In ament_python, we typically place Python modules in 
    'package_name/package_name/*.py'. We'll do that here for consistency.
    Then we can add them to entry_points in setup.py if desired.
    """
    pkg_name = os.path.basename(package_path)  # e.g. "sensor_data"
    python_dir = os.path.join(package_path, pkg_name)
    os.makedirs(python_dir, exist_ok=True)

    if "scripts" in script_data:
        for filename, content in script_data["scripts"].items():
            file_path = os.path.join(python_dir, filename)
            print(f"[ROS2] Creating script {file_path}")
            with open(file_path, "w") as f:
                f.write(content)
            os.chmod(file_path, 0o755)


################################################################################
# MAIN ENTRY POINT
################################################################################

def main() -> None:
    """Main function to create the ROS1 and ROS2 workspaces based on the new system design."""
    # You can change this if you want to create the workspace in a different location

    base_path = "/"  # Update this path as needed
    
    print(f"Creating workspaces under: {base_path}")
    os.makedirs(base_path, exist_ok=True)

    ros_version = ROSDetector.detect_ros_version()

    # Create workspaces
    if ros_version == ROSVersion.ROS1:
        create_ros1_workspace(base_path)
    elif ros_version == ROSVersion.ROS2:
        create_ros2_workspace(base_path)

if __name__ == "__main__":
    main()
