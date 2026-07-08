#!/usr/bin/env python3

import math
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler


class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')

        # ==========================================
        # Subscriber
        # ==========================================
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/encoder_ticks',
            self.encoder_callback,
            10
        )

        # ==========================================
        # Publisher
        # ==========================================
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom_raw',
            10
        )

        # ==========================================
        # Robot parameters
        # ==========================================
        self.ticks_per_rev = 2048
        self.wheel_radius = 0.05
        self.wheel_base = 0.60

        # Reverse encoder direction if needed
        self.reverse_direction = False

        # ==========================================
        # Moving Average Filter Setup
        # ==========================================
        # نافذة الفلتر (6 قراءات مناسبة جداً لمعدل 50Hz لمنع الـ Jitter بدون حدوث Lag)
        self.filter_window_size = 6
        self.v_buffer = deque(maxlen=self.filter_window_size)
        self.w_buffer = deque(maxlen=self.filter_window_size)

        # ==========================================
        # State variables
        # ==========================================
        self.prev_left = None
        self.prev_right = None
        self.prev_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    # ==========================================
    # Normalize angle between -pi and +pi
    # ==========================================
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # ==========================================
    # Encoder callback
    # ==========================================
    def encoder_callback(self, msg):

        # ==========================================
        # Safety check for encoder message
        # ==========================================
        if len(msg.data) < 2:
            return

        current_time = self.get_clock().now()

        left_ticks = msg.data[0]
        right_ticks = msg.data[1]

        # ==========================================
        # First encoder reading initialization
        # ==========================================
        if self.prev_left is None:
            self.prev_left = left_ticks
            self.prev_right = right_ticks
            self.prev_time = current_time
            return

        # ==========================================
        # Calculate dt in seconds
        # ==========================================
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        # Reject invalid dt values
        if dt <= 0.001 or dt > 0.2:
            self.prev_time = current_time
            return

        # ==========================================
        # Calculate encoder delta
        # ==========================================
        delta_left = left_ticks - self.prev_left
        delta_right = right_ticks - self.prev_right

        # ==========================================
        # Encoder glitch protection
        # ==========================================
        if abs(delta_left) > 10000 or abs(delta_right) > 10000:
            self.get_logger().warn("Encoder jump detected, skipping...")
            self.prev_left = left_ticks
            self.prev_right = right_ticks
            self.prev_time = current_time
            # تفريغ الفلتر عند حدوث قفزة غريبة لضمان عدم تلوث البيانات النظيفة
            self.v_buffer.clear()
            self.w_buffer.clear()
            return

        # ==========================================
        # Reverse direction if motors are inverted
        # ==========================================
        if self.reverse_direction:
            delta_left = -delta_left
            delta_right = -delta_right

        # ==========================================
        # Convert encoder ticks to wheel distance
        # ==========================================
        wheel_circ = 2.0 * math.pi * self.wheel_radius
        d_left = (delta_left / self.ticks_per_rev) * wheel_circ
        d_right = (delta_right / self.ticks_per_rev) * wheel_circ

        # ==========================================
        # Robot linear and angular displacement
        # ==========================================
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        # ==========================================
        # Pose integration
        # ==========================================
        if abs(d_theta) > 1e-6:
            # Arc motion integration
            r = d_center / d_theta
            self.x += r * (math.sin(self.theta + d_theta) - math.sin(self.theta))
            self.y -= r * (math.cos(self.theta + d_theta) - math.cos(self.theta))
        else:
            # Straight line approximation
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)

        # ==========================================
        # Update robot heading
        # ==========================================
        self.theta += d_theta
        self.theta = self.normalize_angle(self.theta)

        # ==========================================
        # Compute RAW velocities
        # ==========================================
        v_raw = d_center / dt
        w_raw = d_theta / dt

        # ==========================================
        # Reject unrealistic velocity spikes
        # ==========================================
        if abs(v_raw) > 2.0 or abs(w_raw) > 5.0:
            self.get_logger().warn("Velocity spike detected, skipping...")
            self.prev_left = left_ticks
            self.prev_right = right_ticks
            self.prev_time = current_time
            return

        # ==========================================
        # Apply Moving Average Filter
        # ==========================================
        self.v_buffer.append(v_raw)
        self.w_buffer.append(w_raw)

        v_filtered = sum(self.v_buffer) / len(self.v_buffer)
        w_filtered = sum(self.w_buffer) / len(self.w_buffer)

        # ==========================================
        # Quaternion conversion
        # ==========================================
        q = quaternion_from_euler(0, 0, self.theta)

        # ==========================================
        # Create odometry message
        # ==========================================
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        odom.pose.pose.orientation = Quaternion(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3]
        )

        # Linear and angular velocity (استخدام القيم المفلترة الآن النظيفة)
        odom.twist.twist.linear.x = v_filtered
        odom.twist.twist.angular.z = w_filtered

        # ==========================================
        # Pose covariance
        # ==========================================
        odom.pose.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    99999,0,    0,    0,
            0,    0,    0,    99999,0,    0,
            0,    0,    0,    0,    99999,0,
            0,    0,    0,    0,    0,    0.1
        ]

        # ==========================================
        # Twist covariance
        # ==========================================
        odom.twist.covariance = [
            0.05, 0,    0,    0,    0,    0,
            0,    0.05, 0,    0,    0,    0,
            0,    0,    99999,0,    0,    0,
            0,    0,    0,    99999,0,    0,
            0,    0,    0,    0,    99999,0,
            0,    0,    0,    0,    0,    0.1
        ]

        # ==========================================
        # Debug logging
        # ==========================================
        self.get_logger().debug(
            f"x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}, "
            f"v_filt={v_filtered:.2f}, w_filt={w_filtered:.2f}"
        )

        # ==========================================
        # Publish odometry
        # ==========================================
        self.odom_pub.publish(odom)

        # ==========================================
        # Update previous values
        # ==========================================
        self.prev_left = left_ticks
        self.prev_right = right_ticks
        self.prev_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
