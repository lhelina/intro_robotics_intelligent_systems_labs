#!/usr/bin/env python3
"""Action server that drives the robot straight forward for a given distance,
measured with wheel odometry, and aborts in front of an obstacle.
It holds the heading the robot had at the start.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from lab_interfaces.msg import ObstacleInfo


def yaw_from_quaternion(q):
    """Return rotation about the vertical axis in radians."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def angle_difference(target, current):
    """Return the wrapped difference between two angles."""
    return math.atan2(
        math.sin(target - current),
        math.cos(target - current)
    )


class DriveDistanceServer(Node):

    def __init__(self):
        super().__init__("drive_distance_server")

        self.declare_parameter("speed", 0.3)
        self.declare_parameter("heading_gain", 1.0)

        self.position = None
        self.yaw = None
        self.too_close = False

        group = ReentrantCallbackGroup()

        self.cmd_vel = self.create_publisher(
            Twist, "cmd_vel", 10
        )

        self.create_subscription(
            Odometry,
            "odom",
            self.on_odom,
            10,
            callback_group=group
        )

        self.create_subscription(
            ObstacleInfo,
            "obstacle",
            self.on_obstacle,
            10,
            callback_group=group
        )

        self.action_server = ActionServer(
            self,
            DriveDistance,
            "drive_distance",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=group,
        )

        self.get_logger().info(
            "Drive distance action server is ready."
        )

    def on_odom(self, msg):
        self.position = msg.pose.pose.position
        self.yaw = yaw_from_quaternion(
            msg.pose.pose.orientation
        )

    def on_obstacle(self, msg):
        self.too_close = msg.too_close

    def on_goal(self, goal):
        if goal.distance <= 0.0:
            self.get_logger().warning(
                "Rejected goal: distance must be greater than zero."
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def execute(self, goal_handle):

        goal = goal_handle.request.distance
        result = DriveDistance.Result()

        # Wait until the first odometry message arrives.
        self.get_logger().info("Waiting for odometry...")

        start_time = time.time()

        while rclpy.ok() and (
            self.position is None or self.yaw is None
        ):

            if goal_handle.is_cancel_requested:
                self.cmd_vel.publish(Twist())
                goal_handle.canceled()
                return result

            if time.time() - start_time > 10.0:
                self.get_logger().error(
                    "Odometry did not arrive within 10 seconds."
                )
                self.cmd_vel.publish(Twist())
                goal_handle.abort()
                return result

            time.sleep(0.1)

        if not rclpy.ok():
            self.cmd_vel.publish(Twist())
            goal_handle.abort()
            return result

        # Record the starting position and heading.
        start = self.position
        start_yaw = self.yaw

        self.get_logger().info(
            f"Driving {goal:.2f} m"
        )

        twist = Twist()
        twist.linear.x = self.get_parameter("speed").value

        heading_gain = self.get_parameter(
            "heading_gain"
        ).value

        feedback = DriveDistance.Feedback()

        while rclpy.ok():

            travelled = math.hypot(
                self.position.x - start.x,
                self.position.y - start.y
            )

            result.travelled = travelled

            # Handle cancellation.
            if goal_handle.is_cancel_requested:
                self.cmd_vel.publish(Twist())
                goal_handle.canceled()

                self.get_logger().info(
                    f"Canceled after {travelled:.2f} m"
                )

                return result

            # Stop if an obstacle is too close.
            if self.too_close:
                self.cmd_vel.publish(Twist())
                goal_handle.abort()

                self.get_logger().info(
                    f"Obstacle ahead, aborted after {travelled:.2f} m"
                )

                return result

            # Finish when the requested distance is reached.
            if travelled >= goal:
                break

            # Correct the heading to keep the robot straight.
            twist.angular.z = heading_gain * angle_difference(
                start_yaw,
                self.yaw
            )

            self.cmd_vel.publish(twist)

            feedback.travelled = travelled
            goal_handle.publish_feedback(feedback)

            time.sleep(0.1)

        # Stop the robot after reaching the distance.
        self.cmd_vel.publish(Twist())

        goal_handle.succeed()

        result.reached = True

        self.get_logger().info(
            f"Reached, travelled {result.travelled:.2f} m"
        )

        return result

def main():
    rclpy.init()
    node = DriveDistanceServer()

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()