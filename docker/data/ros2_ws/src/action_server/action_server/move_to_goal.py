#!/usr/bin/env python3

# You'd also have a generated "MoveToGoal" class from your .action:
# from action_server.action._move_to_goal import MoveToGoal, MoveToGoal_Result, MoveToGoal_Feedback
# or something similar once it's compiled.
# We'll stub these for demonstration:

from typing import Any, Optional

from action_server.action._move_to_goal import MoveToGoal_Feedback, MoveToGoal_Result

from environment_integration.msg import EnvironmentData
from geometry_msgs.msg import Twist
import rclpy

# In ROS2, you'd typically import the auto-generated Python module for your action.
# This might look like:
# from action_server.action import MoveToGoal
# But for brevity, we show a minimal approach.
# We can use the "rclpy.action" interfaces:
from rclpy.action import CancelResponse, GoalResponse
from rclpy.node import Node


class MoveToGoalServer(Node):
    def __init__(self) -> None:
        super().__init__("move_to_goal_server")

        # This is where we'd create an ActionServer if the action is properly generated
        # e.g.:
        # self._action_server = ActionServer(
        #     self,
        #     MoveToGoal,
        #     'move_to_goal',
        #     execute_callback=self.execute_callback,
        #     goal_callback=self.goal_callback,
        #     cancel_callback=self.cancel_callback
        # )

        # For demonstration, let's assume the environment data is still being subscribed
        self.env_sub = self.create_subscription(
            EnvironmentData, "/environment/data", self.env_callback, 10
        )
        self.environment_data = None

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("MoveToGoal action server (stub) started.")

    def env_callback(self, msg: Any) -> None:
        self.environment_data = msg

    # Action logic stubs:
    def goal_callback(self, goal_request: Any) -> Any:
        # Decide if we accept or reject
        self.get_logger().info("Received goal request.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: Any) -> Any:
        self.get_logger().info("Received cancel request.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: Any) -> Any:
        self.get_logger().info("Executing goal...")

        # Very simple loop
        feedback_msg = MoveToGoal_Feedback()
        for i in range(5):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled.")
                goal_handle.canceled()
                return MoveToGoal_Result(success=False)

            twist = Twist()
            twist.linear.x = 0.1
            self.cmd_vel_pub.publish(twist)

            feedback_msg.current_status = f"Moving... step {i+1}/5"
            goal_handle.publish_feedback(feedback_msg)
            await self._sleep(1.0)

        twist = Twist()
        self.cmd_vel_pub.publish(twist)  # stop

        result = MoveToGoal_Result()
        result.success = True
        goal_handle.succeed()
        self.get_logger().info("Goal reached!")
        return result

    async def _sleep(self, secs: float) -> None:
        # For a real async approach, you'd use 'rclpy.executors' or 'asyncio'
        # but let's do a naive approach:
        import asyncio

        await asyncio.sleep(secs)


def main(args: Optional[Any] = None) -> None:
    rclpy.init(args=args)
    node = MoveToGoalServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
