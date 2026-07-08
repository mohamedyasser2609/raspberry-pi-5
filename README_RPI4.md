# 📡 Raspberry Pi 4 (8 GB) — ROS2 Setup Guide

This guide outlines the configuration, code modifications, and system setup required to run the **Hardware Interface Layer** on the **Raspberry Pi 4 (8 GB)**. 

The RPi4 handles the physical interface to sensors and actuators (LiDAR, TM4C serial bridge, Odometry calculation, and local EKF sensor fusion) while leaving heavy computing (SLAM and Nav2) to the RPi5.

---

## 1. Role Allocation & Flow

```
[Sensors/Hardware] ➔ [RPi4: Hardware Nodes] ➔ (Ethernet/DDS) ➔ [RPi5: SLAM/Nav2]
```

On the RPi4, you will run:
1. **`robot_state_publisher`** (URDF publisher)
2. **`tm4c_bridge`** (Serial connection to the Tiva-C microcontroller)
3. **`odometry_node`** (Encoder ticks conversion to raw odometry)
4. **`local_ekf`** (IMU + Encoder sensor fusion)
5. **`rplidar_node`** (LiDAR sensor driver)

---

## 2. Code Changes for the RPi4 Workspace

Since you are copying this workspace to the RPi4, make the following modifications **only inside the RPi4's local copy of the workspace**:

### 2.1 The Reversed Commenting in `full_robot.launch.py`
In [full_robot.launch.py](file:///home/mohamed-yasser/Development/raspberry-pi-5/src/uart_comstack/uart_comstack/launch/full_robot.launch.py), you must **reverse the commenting** to activate the hardware nodes and deactivate the SLAM/Nav2 nodes.

Modify the node execution block inside the returned `LaunchDescription` list:

```python
    return LaunchDescription([
        # 1. UNCOMMENT - Launch parameters
        DeclareLaunchArgument(
            'tm4c_port',
            default_value='/dev/serial/by-path/platform-xhci-hcd.1-usb-0:1:1.0-port0',
            description='Stable serial port for the TM4C bridge'
        ),
        DeclareLaunchArgument(
            'rplidar_port',
            default_value='/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0-port0',
            description='Stable serial port for the RPLidar'
        ),
        DeclareLaunchArgument(
            'rplidar_baudrate',
            default_value='115200',
            description='RPLidar serial baud rate'
        ),

        # 2. UNCOMMENT - URDF static TFs
        robot_state_publisher_node,

        # 3. UNCOMMENT - TM4C Hardware Bridge
        Node(
            package='uart_comstack',
            executable='tm4c_bridge',
            name='tm4c_bridge',
            output='screen',
            parameters=[{
                'serial_port': tm4c_port,
                'baud_rate': 115200,
            }]
        ),

        # 4. UNCOMMENT - Encoder Odometry node
        Node(
            package='uart_comstack',
            executable='odometry_node',
            name='odometry_node',
            output='screen'
        ),

        # 5. UNCOMMENT - Local EKF Node (Sensory Fusion)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='local_ekf',
            output='screen',
            parameters=[ekf_config]
        ),

        # 6. UNCOMMENT - RPLiDAR Laser Driver
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': rplidar_port,
                'serial_baudrate': ParameterValue(rplidar_baudrate, value_type=int),
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            respawn=True,
            respawn_delay=2.0
        ),

        # 7. COMMENT OUT - SLAM Toolbox (Runs on RPi5)
        # Node(
        #     package='slam_toolbox',
        #     ...
        # ),

        # 8. COMMENT OUT - SLAM Lifecycle Manager (Runs on RPi5)
        # Node(
        #     package='nav2_lifecycle_manager',
        #     ...
        # ),

        # 9. COMMENT OUT - Nav2 Stack Launch (Runs on RPi5)
        # TimerAction(
        #     period=3.0,
        #     actions=[nav2_launch]
        # )
    ])
```

---

## 3. RPi4 System Dependencies & Hardware Rules

### 3.1 Install RPLiDAR Driver Package
The workspace `src/rplidar_ros` directory is empty. On the RPi4, install the ROS2 driver directly:
```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-rplidar-ros
```

### 3.2 Install Robot Localization Package
Make sure the EKF filter dependency is installed:
```bash
sudo apt install ros-${ROS_DISTRO}-robot-localization
```

### 3.3 Create Persistent USB Serial Port Symlinks (Rules)
Since TM4C and RPLidar connect via USB, their ports can swap (e.g. `/dev/ttyUSB0` ↔ `/dev/ttyUSB1`) upon reboot. 

Create a custom udev rule file:
```bash
sudo nano /etc/udev/rules.d/99-robot-serial.rules
```
Add the following configuration (verify Vendor IDs `idVendor` and Product IDs `idProduct` using `lsusb`):
```udev
# TM4C123 Board
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1cbe", ATTRS{idProduct}=="00fd", MODE="0666", SYMLINK+="sensors/tm4c"

# RPLidar (using CP210x USB-to-UART Converter)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="sensors/rplidar"
```
Reload rules:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Now update your default ports in the launch file to:
- `tm4c_port` ➔ `/dev/sensors/tm4c`
- `rplidar_port` ➔ `/dev/sensors/rplidar`

---

## 4. Multi-Machine Networking (Ethernet Connection)

Because the two Pis are connected directly via Ethernet, they need stable networking.

### 4.1 Interface configuration (Static IP)
In your RPi4 network settings, assign a static IP address to the Ethernet interface (`eth0`):
- **RPi4 IP**: `10.0.0.4`
- **RPi4 Subnet Mask**: `255.255.255.0`

*(Ensure RPi5 is configured with IP `10.0.0.5`)*

### 4.2 ROS2 Domain ID and Discovery
Add the following to the end of `~/.bashrc` on the RPi4:
```bash
export ROS_DOMAIN_ID=42
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds_profile.xml
```

Create `/etc/fastdds_profile.xml` to restrict ROS2 DDS traffic to only flow over the Ethernet adapter, preventing loopbacks or multi-interface issues:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>UDPv4Transport</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
          <address>10.0.0.4</address>
        </interfaceWhiteList>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="default_participant" is_default_profile="true">
      <rtps>
        <userTransports>
          <transport_id>UDPv4Transport</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
```

### 4.3 Setup Chrony NTP Time Sync (Essential for TF)
To ensure TFs don't fail due to clock skew between the two boards, configure RPi4 as the time server.

1. Install chrony:
   ```bash
   sudo apt install chrony
   ```
2. Configure it:
   ```bash
   sudo nano /etc/chrony/chrony.conf
   ```
   Add the following lines at the end of the file:
   ```chrony
   # Allow connection from local network
   allow 10.0.0.0/24
   # Serve time even if not synchronized to a system clock source
   local stratum 10
   ```
3. Restart the service:
   ```bash
   sudo systemctl restart chrony
   ```

---

## 5. Building and Running

1. Source ROS2:
   ```bash
   source /opt/ros/${ROS_DISTRO}/setup.bash
   ```
2. Build the workspace on RPi4:
   ```bash
   cd ~/project_ws
   colcon build --symlink-install
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```
4. Run the launch file:
   ```bash
   ros2 launch uart_comstack full_robot.launch.py
   ```
