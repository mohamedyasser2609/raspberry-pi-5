#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped

import math
from tf_transformations import quaternion_from_euler
class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')

        # ==========================
        # Subscriber
        # ==========================
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/encoder_ticks',
            self.encoder_callback,
            10
        )

        # ==========================
        # Publisher
        # ==========================
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)


        # ==========================
        # Robot parameters
        # ==========================
        self.ticks_per_rev = 2048
        self.wheel_radius = 0.05   # meters
        self.wheel_base = 0.30     # meters

        # ==========================
        # State
        # ==========================
        self.prev_left = None
        self.prev_right = None
        self.prev_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    # ==========================
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # ==========================
    def encoder_callback(self, msg):

        current_time = self.get_clock().now()

        left_ticks = msg.data[0]
        right_ticks = msg.data[1]

        # أول قراءة
        if self.prev_left is None:
            self.prev_left = left_ticks
            self.prev_right = right_ticks
            self.prev_time = current_time
            return

        # ==========================
        # dt
        # ==========================
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            return

        self.prev_time = current_time

        # ==========================
        # Delta ticks
        # ==========================
        delta_left = left_ticks - self.prev_left
        delta_right = right_ticks - self.prev_right

        self.prev_left = left_ticks
        self.prev_right = right_ticks

        # ==========================
        # Wheel distance
        # ==========================
        wheel_circ = 2 * math.pi * self.wheel_radius

        d_left = (delta_left / self.ticks_per_rev) * wheel_circ
        d_right = (delta_right / self.ticks_per_rev) * wheel_circ

        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        # ==========================
        # Pose update
        # ==========================
        if abs(d_theta) > 1e-6:
            r = d_center / d_theta
            self.x += r * (math.sin(self.theta + d_theta) - math.sin(self.theta))
            self.y -= r * (math.cos(self.theta + d_theta) - math.cos(self.theta))
        else:
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)

        self.theta += d_theta
        self.theta = self.normalize_angle(self.theta)

        # ==========================
        # Velocity
        # ==========================
        v = d_center / dt
        w = d_theta / dt

        # ==========================
        # Quaternion
        # ==========================
        q = quaternion_from_euler(0, 0, self.theta)

        # ==========================
        # Odometry message
        # ==========================
        odom = Odometry()

        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3]
        )

        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        odom.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]

        # ==========================
        # Publish
        # ==========================
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
