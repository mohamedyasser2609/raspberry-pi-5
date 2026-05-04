#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import struct
import threading
import time

# ==========================
# PROTOCOL
# ==========================
START_BYTE       = 0xAA
END_BYTE         = 0x55

CMD_PING         = 0x01
CMD_ACK          = 0x02
CMD_NACK         = 0x03

CMD_MOTOR_STOP   = 0x11
CMD_TWIST_CMD    = 0x12   # ⭐ الجديد

CMD_IMU_DATA     = 0x22
CMD_ENCODER_DATA = 0x23


class TM4CBridgeNode(Node):

    def __init__(self):
        super().__init__('tm4c_bridge')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        # Serial
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f'Serial opened: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial: {e}')
            raise

        # Publishers
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoder_ticks', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Stats
        self.rx_count = 0
        self.tx_count = 0
        self.error_count = 0

        # Timeout safety
        self.last_cmd_time = 0.0
        self.cmd_timeout = 0.5

        # Threads
        self.running = True
        self.rx_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.rx_thread.start()

        # Timers
        self.create_timer(2.0, self.send_ping)
        self.create_timer(5.0, self.print_stats)
        self.create_timer(0.1, self.check_cmd_timeout)

        self.get_logger().info('TM4C Bridge (TWIST MODE) started')

    # ==========================
    # PACKET
    # ==========================
    def calculate_checksum(self, command, length, data):
        cs = command ^ length
        for b in data:
            cs ^= b
        return cs & 0xFF

    def send_packet(self, command, data=b''):
        length = len(data)
        checksum = self.calculate_checksum(command, length, data)
        packet = bytes([START_BYTE, command, length]) + data + bytes([checksum, END_BYTE])

        try:
            self.ser.write(packet)
            self.tx_count += 1
        except serial.SerialException as e:
            self.get_logger().error(f'TX error: {e}')

    # ==========================
    # CMD VEL (TWIST)
    # ==========================
    def cmd_vel_callback(self, msg):
        # scale (m/s → mm/s)
        linear_mmps = int(msg.linear.x * 1000.0)
        angular_mrads = int(msg.angular.z * 1000.0)

        # clamp
        linear_mmps = max(-32768, min(32767, linear_mmps))
        angular_mrads = max(-32768, min(32767, angular_mrads))

        data = struct.pack('<hh', linear_mmps, angular_mrads)
        self.send_packet(CMD_TWIST_CMD, data)

        self.last_cmd_time = time.monotonic()

    def send_ping(self):
        self.send_packet(CMD_PING)

    def check_cmd_timeout(self):
        if self.last_cmd_time == 0.0:
            return

        if time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            # stop robot
            data = struct.pack('<hh', 0, 0)
            self.send_packet(CMD_TWIST_CMD, data)
            self.last_cmd_time = 0.0

    # ==========================
    # RECEIVE LOOP
    # ==========================
    def receive_loop(self):
        while self.running:
            try:
                byte = self.ser.read(1)

                if len(byte) == 0 or byte[0] != START_BYTE:
                    continue

                header = self.ser.read(2)
                if len(header) < 2:
                    self.error_count += 1
                    continue

                command, length = header

                if length > 120:
                    self.error_count += 1
                    continue

                data = self.ser.read(length) if length > 0 else b''
                if len(data) < length:
                    self.error_count += 1
                    continue

                footer = self.ser.read(2)
                if len(footer) < 2:
                    self.error_count += 1
                    continue

                checksum, end_byte = footer

                if end_byte != END_BYTE or checksum != self.calculate_checksum(command, length, data):
                    self.error_count += 1
                    continue

                self.rx_count += 1
                self.handle_packet(command, data)

            except Exception as e:
                self.get_logger().error(f'RX error: {e}')
                time.sleep(0.01)

    # ==========================
    # HANDLE PACKETS
    # ==========================
    def handle_packet(self, command, data):

        if command == CMD_ENCODER_DATA and len(data) >= 12:
            self.publish_encoder(data)

        elif command == CMD_IMU_DATA and len(data) >= 12:
            self.publish_imu(data)

        elif command == CMD_PING:
            self.send_packet(CMD_ACK)

    # ==========================
    # ENCODER
    # ==========================
    def publish_encoder(self, data):
        left_ticks, right_ticks, left_vel, right_vel = struct.unpack('<iihh', data[:12])

        msg = Int32MultiArray()
        msg.data = [left_ticks, right_ticks]
        self.encoder_pub.publish(msg)

    # ==========================
    # IMU
    # ==========================
    def publish_imu(self, data):
        ax, ay, az, gx, gy, gz = struct.unpack('<hhhhhh', data[:12])

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.linear_acceleration.x = ax / 100.0
        msg.linear_acceleration.y = ay / 100.0
        msg.linear_acceleration.z = az / 100.0

        msg.angular_velocity.x = gx / 100.0
        msg.angular_velocity.y = gy / 100.0
        msg.angular_velocity.z = gz / 100.0

        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01

        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01

        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01

        self.imu_pub.publish(msg)

    # ==========================
    # STATS
    # ==========================
    def print_stats(self):
        self.get_logger().info(
            f'Bridge Stats | RX: {self.rx_count} | TX: {self.tx_count} | Errors: {self.error_count}'
        )

    # ==========================
    # SHUTDOWN
    # ==========================
    def destroy_node(self):
        self.running = False

        if self.ser.is_open:
            self.send_packet(CMD_MOTOR_STOP)
            time.sleep(0.05)
            self.ser.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TM4CBridgeNode()

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
