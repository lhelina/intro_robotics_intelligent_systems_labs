#!/usr/bin/env python3

"""Action client that drives the robot in a square."""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate
from action_msgs.msg import GoalStatus


class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")

        self.declare_parameter("side_length", 0.5)

        self.drive_client = ActionClient(
            self,
            DriveDistance,
            "/drive_distance"
        )

        self.rotate_client = ActionClient(
            self,
            Rotate,
            "/rotate"
        )

        self.cmd_vel = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

    def stop_robot(self):
        """Publish zero velocity."""
        self.cmd_vel.publish(Twist())

    def execute_goal(self, client, goal):
        """Send an action goal and wait for its result."""

        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Action server was not available."
            )
            return None

        goal_future = client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            goal_future
        )

        goal_handle = goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            return None

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        return result_future.result()

    def run_square(self):
        """Drive four sides and rotate left after each side."""

        side_length = float(
            self.get_parameter("side_length").value
        )

        if side_length <= 0.0 or side_length > 1.0:
            self.get_logger().error(
                "side_length must be greater than 0 and at most 1 meter."
            )
            return

        quarter_turn = math.pi / 2.0

        for side in range(1, 5):

            self.get_logger().info(
                f"Driving side {side}/4: {side_length} meters."
            )

            # Drive forward.
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = side_length

            drive_result = self.execute_goal(
                self.drive_client,
                drive_goal
            )

            if (
                drive_result is None
                or drive_result.status != GoalStatus.STATUS_SUCCEEDED
                or not drive_result.result.reached
            ):
                self.stop_robot()
                self.get_logger().error(
                    f"Failed to complete side {side}."
                )
                return

            self.get_logger().info(
                f"Side {side} completed. Turning left."
            )

            # Rotate left by 90 degrees.
            rotate_goal = Rotate.Goal()
            rotate_goal.angle = quarter_turn

            rotate_result = self.execute_goal(
                self.rotate_client,
                rotate_goal
            )

            if (
                rotate_result is None
                or rotate_result.status != GoalStatus.STATUS_SUCCEEDED
            ):
                self.stop_robot()
                self.get_logger().error(
                    f"Failed to rotate after side {side}."
                )
                return

        self.stop_robot()

        self.get_logger().info(
            "Square completed successfully!"
        )


def main(args=None):
    rclpy.init(args=args)

    node = SquareDriver()

    try:
        node.run_square()
    except KeyboardInterrupt:
        node.get_logger().info("Square movement interrupted.")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
