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
# PROTOCOL CONSTANTS
# ==========================
START_BYTE          = 0xAA
END_BYTE            = 0x55

CMD_PING            = 0x01
CMD_ACK             = 0x02
CMD_TIME_SYNC       = 0x05  # New: Time Synchronization
CMD_TWIST_CMD       = 0x12
CMD_MOTOR_STOP      = 0x11
CMD_IMU_DATA        = 0x22
CMD_ENCODER_DATA    = 0x23
CMD_STATUS          = 0x30

class TM4CBridgeNode(Node):

    def __init__(self):
        super().__init__('tm4c_bridge')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        # Serial Connection
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Serial opened: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial: {e}')
            raise

        # ROS Publishers & Subscribers
        self.encoder_pub = self.create_publisher(Int32MultiArray, '/encoder_ticks', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Stats counters
        self.rx_count = 0
        self.tx_count = 0
        self.error_count = 0

        # Safety Heartbeat
        self.last_cmd_time = 0.0
        self.cmd_timeout = 0.5

        # Background Receiver Thread
        self.running = True
        self.rx_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.rx_thread.start()

        # Timers
        self.create_timer(1.0, self.send_time_sync) # Sync time every 1s
        self.create_timer(5.0, self.print_stats)
        self.create_timer(0.1, self.check_cmd_timeout)

        self.get_logger().info('TM4C Bridge (SYNC MODE) started')

    # ==========================
    # PROTOCOL HELPERS
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
        except Exception as e:
            self.get_logger().error(f'TX Error: {e}')

    # ==========================
    # OUTGOING: TIME SYNC
    # ==========================
    def send_time_sync(self):
        """Sends current RPi Wall Time to Tiva to synchronize clocks."""
        now = self.get_clock().now()
        sec, nsec = now.seconds_nanoseconds()
        data = struct.pack('<II', sec, nsec)
        self.send_packet(CMD_TIME_SYNC, data)

    # ==========================
    # OUTGOING: VELOCITY
    # ==========================
    def cmd_vel_callback(self, msg):
        # Convert m/s to mm/s and rad/s to mrad/s
        v_mmps = int(msg.linear.x * 1000.0)
        w_mrads = int(msg.angular.z * 1000.0)

        # Pack into 4 bytes (little-endian sint16, sint16)
        data = struct.pack('<hh', 
                           max(-32768, min(32767, v_mmps)), 
                           max(-32768, min(32767, w_mrads)))
        self.send_packet(CMD_TWIST_CMD, data)
        self.last_cmd_time = time.monotonic()

    def check_cmd_timeout(self):
        if self.last_cmd_time > 0 and (time.monotonic() - self.last_cmd_time > self.cmd_timeout):
            self.send_packet(CMD_TWIST_CMD, struct.pack('<hh', 0, 0))
            self.last_cmd_time = 0.0

    # ==========================
    # INCOMING: RECEIVE LOOP
    # ==========================
    def receive_loop(self):
        while self.running:
            try:
                # Find Start Byte
                if self.ser.read(1) != bytes([START_BYTE]):
                    continue

                # Read Header (CMD + LEN)
                header = self.ser.read(2)
                if len(header) < 2: continue
                cmd, length = header

                # Read Data
                data = self.ser.read(length) if length > 0 else b''
                if len(data) < length: continue

                # Read Footer (Checksum + END)
                footer = self.ser.read(2)
                if len(footer) < 2: continue
                checksum, end_byte = footer

                # Validation
                if end_byte == END_BYTE and checksum == self.calculate_checksum(cmd, length, data):
                    self.rx_count += 1
                    self.handle_packet(cmd, data)
                else:
                    self.error_count += 1

            except Exception as e:
                self.get_logger().error(f'RX loop error: {e}')
                time.sleep(0.01)

    def handle_packet(self, command, data):
        """Dispatches received packets to publishers."""
        if command == CMD_ENCODER_DATA:
            self.publish_encoder(data)
        elif command == CMD_IMU_DATA:
            self.publish_imu(data)
        elif command == CMD_ACK:
            pass # Tiva confirmed a command

    # ==========================
    # PUBLISHERS
    # ==========================
    def publish_encoder(self, data):
        # Format: <IIiihh (TimestampSec, TimestampNsec, L_Ticks, R_Ticks, L_Vel, R_Vel)
        if len(data) < 20: return
        try:
            sec, nsec, l_ticks, r_ticks, l_rpm, r_rpm = struct.unpack('<IIiihh', data[:20])
            
            msg = Int32MultiArray()
            msg.data = [l_ticks, r_ticks]
            self.encoder_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Encoder unpack error: {e}")

    def publish_imu(self, data):
        # Format: <IIhhhhhh (TimestampSec, TimestampNsec, Ax, Ay, Az, Gx, Gy, Gz)
        if len(data) < 20: return
        try:
            sec, nsec, ax, ay, az, gx, gy, gz = struct.unpack('<IIhhhhhh', data[:20])

            msg = Imu()
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nsec
            msg.header.frame_id = 'imu_link'

            # Convert scaled ints back to real values (x100)
            msg.linear_acceleration.x = ax / 100.0
            msg.linear_acceleration.y = ay / 100.0
            msg.linear_acceleration.z = az / 100.0
            msg.angular_velocity.x = gx / 100.0
            msg.angular_velocity.y = gy / 100.0
            msg.angular_velocity.z = gz / 100.0

            self.imu_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"IMU unpack error: {e}")

    def print_stats(self):
        self.get_logger().info(f'Bridge Stats | RX: {self.rx_count} | TX: {self.tx_count} | Errors: {self.error_count}')

    def destroy_node(self):
        self.running = False
        if self.ser.is_open:
            self.send_packet(CMD_MOTOR_STOP)
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
