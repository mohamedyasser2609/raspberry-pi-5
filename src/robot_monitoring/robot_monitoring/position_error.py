import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

import math


class PositionError(Node):

    def __init__(self):
        super().__init__('position_error')

        self.current_x = 0.0
        self.current_y = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0

        self.goal_received = False

        # robot pose
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # goal pose
        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # publish error
        self.error_pub = self.create_publisher(
            Float32,
            '/position_error',
            10
        )

        self.timer = self.create_timer(0.1, self.compute_error)

    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_received = True

    def compute_error(self):

        if not self.goal_received:
            return

        error = math.sqrt(
            (self.goal_x - self.current_x) ** 2 +
            (self.goal_y - self.current_y) ** 2
        )

        msg = Float32()
        msg.data = error

        self.error_pub.publish(msg)

        self.get_logger().info(f'Position Error: {error:.3f}')


def main(args=None):
    rclpy.init(args=args)

    node = PositionError()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
