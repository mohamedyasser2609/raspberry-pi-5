
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped

import math
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster


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
        # TF Broadcaster
        # ==========================
        self.tf_broadcaster = TransformBroadcaster(self)

        # ==========================
        # Robot parameters
        # ==========================
        self.ticks_per_rev = 2048
        self.wheel_radius = 0.065
        self.wheel_base = 0.50

        # ⚠️ IMPORTANT: غيري ده لو الاتجاه غلط
        self.reverse_direction = False

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

        # ==========================
        # Safety check
        # ==========================
        if len(msg.data) < 2:
            return

        current_time = self.get_clock().now()

        left_ticks = msg.data[0]
        right_ticks = msg.data[1]

        # ==========================
        # First reading
        # ==========================
        if self.prev_left is None:
            self.prev_left = left_ticks
            self.prev_right = right_ticks
            self.prev_time = current_time
            return

        # ==========================
        # dt
        # ==========================
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        if dt <= 0 or dt > 1.0:
            return

        self.prev_time = current_time

        # ==========================
        # Delta ticks
        # ==========================
        delta_left = left_ticks - self.prev_left
        delta_right = right_ticks - self.prev_right

        # ==========================
        # Glitch protection
        # ==========================
        if abs(delta_left) > 10000 or abs(delta_right) > 10000:
            self.get_logger().warn("Encoder jump detected, skipping...")
            return

        self.prev_left = left_ticks
        self.prev_right = right_ticks

        # ==========================
        # Direction fix (NEW)
        # ==========================
        if self.reverse_direction:
            delta_left, delta_right = -delta_left, -delta_right

        # ==========================
        # Wheel distance
        # ==========================
        wheel_circ = 2 * math.pi * self.wheel_radius

        d_left = (delta_left / self.ticks_per_rev) * wheel_circ
        d_right = (delta_right / self.ticks_per_rev) * wheel_circ

        d_center = (d_left + d_right) / 2.0

        # ⚠️ اتجاه الدوران
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
        # Velocity clamp (NEW)
        # ==========================
        if abs(v) > 2.0:
            v = 0.0
        if abs(w) > 5.0:
            w = 0.0

        # ==========================
        # Quaternion
        # ==========================
        q = quaternion_from_euler(0, 0, self.theta)

        # ==========================
        # TF publish
        # ==========================
        t = TransformStamped()

        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation = Quaternion(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3]
        )

       # self.tf_broadcaster.sendTransform(t)

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

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # Debug logging
        self.get_logger().debug(
            f"x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}, v={v:.2f}"
        )

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
