import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('local_fusion'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        # ================= TM4C UART Bridge =================
        Node(
            package='uart_comstack',
            executable='tm4c_bridge',
            name='tm4c_bridge',
            output='screen'
        ),

        # ================= Odometry Node =================
        Node(
            package='uart_comstack',
            executable='odometry_node',
            name='odometry_node',
            output='screen'
        ),

        # ================= EKF Node =================
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],
            output='screen'
        ),

        # ================= LiDAR Node =================
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',    
                'frame_id': 'laser'
            }]
        ),

        # ================= Static Transforms =================
        # base_link to laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0.1', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'laser'],
            output='screen'
        ),

        # base_link to imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'imu_link'],
            output='screen'
        )
    ])
