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

        # ================= ODOM =================
        Node(
            package='uart_comstack',
            executable='odometry_node',
            name='odom_node',
            output='screen'
        ),

        # ================= EKF =================
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='local_ekf',
            parameters=[ekf_config],
            output='screen'
        ),

        # ================= RPLIDAR =================
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar',
            output='screen'
        ),
    ])
