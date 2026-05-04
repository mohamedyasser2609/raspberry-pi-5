from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ================= CONFIG FILES =================
    slam_config = os.path.join(
        get_package_share_directory('uart_comstack'),
        'config',
        'mapper_params_online_async.yaml'
    )

    ekf_config = os.path.join(
        get_package_share_directory('local_fusion'),
        'config',
        'ekf.yaml'
    )

    global_ekf_config = os.path.join(
        get_package_share_directory('local_fusion'),
        'config',
        'global_ekf.yaml'
    )

    nav2_config = os.path.join(
        '/home/raspberrypi/project_ws/src/nav2_config/config/nav2_params.yaml'
    )

    # ================= NAV2 INCLUDE =================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_config'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_config
        }.items()
    )

    return LaunchDescription([

        # ================= TM4C BRIDGE =================
        Node(
            package='uart_comstack',
            executable='tm4c_bridge',
            name='tm4c_bridge',
            output='screen'
        ),

        # ================= GPS =================
        Node(
            package='uart_comstack',
            executable='gps_node',
            name='gps_node',
            output='screen'
        ),

        # ================= RAW ODOM =================
        Node(
            package='uart_comstack',
            executable='odometry_node',
            name='odometry_node',
            output='screen'
        ),

        # ================= LOCAL EKF =================
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='local_ekf',
            output='screen',
            parameters=[ekf_config]
        ),

        # ================= NAVSAT TRANSFORM =================
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'magnetic_declination_radians': 0.0,
                'yaw_offset': 0.0,
                'zero_altitude': True,
                'broadcast_utm_transform': True,
                'publish_filtered_gps': True,
                'use_odometry_yaw': True
            }],
            remappings=[
                ('imu', '/imu/data_raw'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odometry/filtered')
            ]
        ),

        # ================= GLOBAL EKF =================
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='global_ekf',
            output='screen',
            parameters=[global_ekf_config],
            remappings=[
                ('odometry/filtered', '/odometry/filtered_global')
            ]
        ),

        # ================= LIDAR =================
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            respawn=True,
            respawn_delay=2.0
        ),

        # ================= TF BASE → LASER =================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            output='screen',
            arguments=[
                '0', '0', '0.1',
                '0', '0', '0',
                'base_link',
                'laser'
            ]
        ),

        # ================= SLAM TOOLBOX =================
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {
                    'use_sim_time': False,
                    'base_frame': 'base_link',
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'odom_topic': '/odometry/filtered',
                    'publish_tf': True
                }
            ]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['slam_toolbox']}]
        ),

        # ================= NAV2 =================
        nav2_launch
    ])
