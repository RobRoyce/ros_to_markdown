#!/usr/bin/env python3
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
