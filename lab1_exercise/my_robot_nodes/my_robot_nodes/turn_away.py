import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed
from std_srvs.srv import SetBool


class TurnAway(Node):

    def __init__(self):
        super().__init__('turn_away')

        # Parameters
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('max_turn_speed', 1.5)

        # The robot starts disabled
        self.enabled = False

        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.status_pub = self.create_publisher(
            RobotStatus, '/robot_status', 10
        )

        # Subscriber
        self.obstacle_sub = self.create_subscription(
            ObstacleInfo,
            '/obstacle',
            self.obstacle_callback,
            10
        )

        # Services
        self.enable_service = self.create_service(
            SetBool,
            '/turn_away_enable',
            self.enable_callback
        )

        self.speed_service = self.create_service(
            SetTurnSpeed,
            '/set_turn_speed',
            self.speed_callback
        )

        self.get_logger().info('Turn away node started. Currently disabled.')

    def obstacle_callback(self, msg):

        status = RobotStatus()
        status.obstacle_distance = msg.distance

        # Do nothing while disabled
        if not self.enabled:
            status.state = 'stopped'
            self.status_pub.publish(status)
            return

        velocity = Twist()

        forward_speed = self.get_parameter('forward_speed').value
        turn_speed = self.get_parameter('turn_speed').value

        if msg.too_close:

            velocity.linear.x = 0.0

            # Obstacle on the left: turn right
            if msg.angle > 0:
                velocity.angular.z = -turn_speed
            else:
                velocity.angular.z = turn_speed

            status.state = 'turning'

        else:

            velocity.linear.x = forward_speed
            velocity.angular.z = 0.0

            status.state = 'forward'

        self.cmd_pub.publish(velocity)
        self.status_pub.publish(status)

    def enable_callback(self, request, response):

        was_enabled = self.enabled
        self.enabled = request.data

        if was_enabled and not self.enabled:
            # Publish one zero velocity when switching off
            self.cmd_pub.publish(Twist())

        response.success = True

        if self.enabled:
            response.message = 'Turn away behavior enabled.'
        else:
            response.message = 'Turn away behavior disabled.'

        self.get_logger().info(response.message)

        return response

    def speed_callback(self, request, response):

        max_speed = self.get_parameter('max_turn_speed').value

        if request.turn_speed <= 0 or request.turn_speed > max_speed:

            response.success = False
            response.message = (
                f'Turn speed must be greater than 0 '
                f'and at most {max_speed}.'
            )

        else:

            self.set_parameters([
                Parameter(
                    'turn_speed',
                    value=float(request.turn_speed)
                )
            ])

            response.success = True
            response.message = (
                f'Turn speed set to {request.turn_speed}.'
            )

        return response


def main(args=None):

    rclpy.init(args=args)

    node = TurnAway()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
