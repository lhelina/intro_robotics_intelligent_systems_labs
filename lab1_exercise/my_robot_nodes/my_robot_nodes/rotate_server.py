#!/usr/bin/env python3

"""Action server that rotates the robot by a requested angle using odometry."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import Rotate


def yaw_from_quaternion(q):
    """Convert an orientation quaternion into yaw in radians."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def angle_difference(target, current):
    """Calculate the shortest angular difference, handling angle wrapping."""
    return math.atan2(
        math.sin(target - current),
        math.cos(target - current)
    )


class RotateServer(Node):

    def __init__(self):
        super().__init__("rotate_server")

        self.declare_parameter("turn_speed", 0.5)

        self.yaw = None

        # Allows odometry callbacks to run while the action is executing.
        group = ReentrantCallbackGroup()

        self.cmd_vel = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.on_odom,
            10,
            callback_group=group
        )

        self.action_server = ActionServer(
            self,
            Rotate,
            "/rotate",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=group
        )

        self.get_logger().info("Rotate action server is ready.")

    def on_odom(self, msg):
        """Update the robot's current yaw from odometry."""
        self.yaw = yaw_from_quaternion(
            msg.pose.pose.orientation
        )

    def on_goal(self, goal_request):
        """Accept non-zero rotation goals when odometry is available."""
        if abs(goal_request.angle) < 0.001:
            self.get_logger().warning("Rejected zero rotation goal.")
            return GoalResponse.REJECT

        if self.yaw is None:
            self.get_logger().warning("Rejected goal: waiting for odometry.")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        """Allow the client to cancel a rotation."""
        self.get_logger().info("Rotation cancellation requested.")
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        """Rotate until the requested angle has been reached."""

        goal = goal_handle.request.angle
        start_yaw = self.yaw
        last_yaw = start_yaw

        turned = 0.0

        turn_speed = abs(
            float(self.get_parameter("turn_speed").value)
        )

        # Positive angle means left; negative means right.
        direction = 1.0 if goal > 0.0 else -1.0

        twist = Twist()
        twist.angular.z = direction * turn_speed

        feedback = Rotate.Feedback()
        result = Rotate.Result()

        self.get_logger().info(
            f"Rotating {goal:.2f} radians."
        )

        while rclpy.ok():

            current_yaw = self.yaw

            if current_yaw is None:
                time.sleep(0.1)
                continue

            # Calculate the small change since the previous odometry update.
            # This handles crossing from +pi to -pi or -pi to +pi.
            change = angle_difference(current_yaw, last_yaw)

            turned += change
            last_yaw = current_yaw

            feedback.turned = float(turned)
            result.turned = float(turned)

            if goal_handle.is_cancel_requested:
                self.cmd_vel.publish(Twist())
                goal_handle.canceled()

                self.get_logger().info(
                    f"Rotation canceled after {turned:.2f} radians."
                )
                return result

            if abs(turned) >= abs(goal):
                break

            self.cmd_vel.publish(twist)
            goal_handle.publish_feedback(feedback)

            time.sleep(0.1)

        # Always stop the robot after completing the rotation.
        self.cmd_vel.publish(Twist())

        result.turned = float(turned)

        goal_handle.succeed()

        self.get_logger().info(
            f"Rotation completed. Turned {turned:.2f} radians."
        )

        return result


def main(args=None):
    rclpy.init(args=args)

    node = RotateServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cmd_vel.publish(Twist())
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
