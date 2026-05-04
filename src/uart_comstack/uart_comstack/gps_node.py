import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)

        try:

            self.ser = serial.Serial('/dev/ttyAMA4', baudrate=9600, timeout=1)
            self.get_logger().info('GPS Serial Port Opened Successfully (AMA0)')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')

        self.timer = self.create_timer(0.1, self.read_gps_data)

    def read_gps_data(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('ascii', errors='replace').strip()
                
     
                if 'GGA' in line:
                    try:
                        msg = pynmea2.parse(line)
                        
                       
                        if msg.latitude == 0.0 and msg.longitude == 0.0:
                            self.get_logger().warn('Waiting for GPS satellite lock... (Please go outdoors)')
                            return

                        ros_msg = NavSatFix()
                        ros_msg.header.stamp = self.get_clock().now().to_msg()
                        ros_msg.header.frame_id = "gps_link"

                        ros_msg.latitude = msg.latitude
                        ros_msg.longitude = msg.longitude
                        ros_msg.altitude = float(msg.altitude) if msg.altitude else 0.0
                        
                        
                        ros_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                        self.publisher_.publish(ros_msg)
                        self.get_logger().info(f'SUCCESS! Lat: {msg.latitude}, Lon: {msg.longitude}')

                    except pynmea2.ParseError:
                        pass 
                    except AttributeError:
                        pass
        except Exception as e:
            self.get_logger().error(f'Serial Read Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
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
