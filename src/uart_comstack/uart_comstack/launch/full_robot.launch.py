import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    tm4c_port = LaunchConfiguration('tm4c_port')
    rplidar_port = LaunchConfiguration('rplidar_port')
    rplidar_baudrate = LaunchConfiguration('rplidar_baudrate')

    # ================= 1. URDF & ROBOT STATE PUBLISHER =================
    xacro_file = os.path.join(get_package_share_directory('nav2_config'), 'urdf', 'tankette.urdf.xacro')
    
    #  Xacro  URDF
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    #  Robot State Publisher  TFs  URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': False
        }]
    )

    # ================= 2. CONFIG FILES PATHS =================
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

    nav2_config = os.path.join(
        get_package_share_directory('nav2_config'),
        'config',
        'nav2_params.yaml'
    )

    # Global EKF  GPS 
    # global_ekf_config = os.path.join(
    #     get_package_share_directory('local_fusion'),
    #     'config',
    #     'global_ekf.yaml'
    # )

    # ================= 3. NAV2 INCLUDE =================
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

    # ================= 4. ALL NODES AGGREGATION =================
    return LaunchDescription([
 #       DeclareLaunchArgument(
 #           'tm4c_port',
#            default_value='/dev/serial/by-path/platform-xhci-hcd.1-usb-0:1:1.0-port0',
 #           description='Stable serial port for the TM4C bridge'
 #       ),
 #       DeclareLaunchArgument(
 #           'rplidar_port',
 #           default_value='/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0-port0',
 #           description='Stable serial port for the RPLidar'
 #       ),
 #       DeclareLaunchArgument(
 #           'rplidar_baudrate',
 #           default_value='115200',
 #           description='RPLidar serial baud rate'
 #       ),
#
        #  URDF  TFs
#        robot_state_publisher_node,

        # ================= TM4C BRIDGE =================
#        Node(
#            package='uart_comstack',
#            executable='tm4c_bridge',
#            name='tm4c_bridge',
#            output='screen',
#            parameters=[{
#                'serial_port': tm4c_port,
#                'baud_rate': 115200,
#            }]
#        ),
#
        # ================= RAW ODOM =================
#        Node(
#            package='uart_comstack',
#            executable='odometry_node',
#            name='odometry_node',
#            output='screen'
#        ),
#
        # ================= LOCAL EKF =================
#        Node(
#            package='robot_localization',
#            executable='ekf_node',
#            name='local_ekf',
#            output='screen',
#            parameters=[ekf_config]
#        ),
#
        # ================= NAVSAT TRANSFORM (OUTDOOR GPS ONLY) =================
        # Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': False,
        #         'magnetic_declination_radians': 0.0,
        #         'yaw_offset': 0.0,
        #         'zero_altitude': True,
        #         'broadcast_utm_transform': True,
        #         'publish_filtered_gps': True,
        #         'use_odometry_yaw': True
        #     }],
        #     remappings=[
        #         ('imu', '/imu/data_raw'),
        #         ('gps/fix', '/gps/fix'),
        #         ('odometry/filtered', '/odometry/filtered')
        #     ]
        # ),

        # ================= GLOBAL EKF (OUTDOOR GPS ONLY) =================
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='global_ekf',
        #     output='screen',
        #     parameters=[global_ekf_config],
        #     remappings=[
        #         ('odometry/filtered', '/odometry/filtered_global')
        #     ]
        # ),

        # ================= LIDAR =================
 #       Node(
 #           package='rplidar_ros',
 #           executable='rplidar_node',
 #           name='rplidar_node',
 #           output='screen',
 #           parameters=[{
 #               'serial_port': rplidar_port,
 #               'serial_baudrate': ParameterValue(rplidar_baudrate, value_type=int),
 #               'frame_id': 'laser',
 #               'inverted': False,
 #               'angle_compensate': True,
 #               'scan_mode': 'Standard'
 #           }],
 #           respawn=True,
 #           respawn_delay=2.0
 #       ),

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
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['slam_toolbox'],
                'bond_timeout': 0.0
            }]
        ),

        # ================= NAV2 STACK =================
        TimerAction(
            period=3.0,
            actions=[nav2_launch]
        )
    ])

