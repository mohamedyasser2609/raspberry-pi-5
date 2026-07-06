#!/usr/bin/env python3
"""
Nav2 Tankette Diagnostics
Checks if Nav2 is properly connected and publishes velocity commands to /cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import time


class Nav2Diagnostics(Node):
    """Diagnose Nav2 and verify connections"""
    
    def __init__(self):
        super().__init__('nav2_diagnostics')
        
        # Subscribe to Nav2 outputs
        self.plan_sub = self.create_subscription(
            Path, '/plan', self.plan_callback, 10)
        self.local_plan_sub = self.create_subscription(
            Path, '/local_plan', self.local_plan_callback, 10)
        
        # Subscribe to velocity topics to monitor what's being published
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_vel_nav_sub = self.create_subscription(
            Twist, '/cmd_vel_nav', self.cmd_vel_nav_callback, 10)
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_vel_raw_callback, 10)
        
        # Publisher to ensure /cmd_vel gets commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State tracking
        self.last_plan = None
        self.last_local_plan = None
        self.last_cmd_vel = None
        self.last_cmd_vel_nav = None
        self.last_cmd_vel_raw = None
        self.plan_received = False
        self.cmd_vel_received = False
        self.cmd_vel_nav_received = False
        
        # Create timer for diagnostics
        self.timer = self.create_timer(1.0, self.run_diagnostics)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Nav2 Tankette Diagnostics Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Monitoring topics:')
        self.get_logger().info('  ✓ /plan (global plan)')
        self.get_logger().info('  ✓ /local_plan (local plan)')
        self.get_logger().info('  ✓ /cmd_vel (velocity commands)')
        self.get_logger().info('  ✓ /cmd_vel_nav (Nav2 raw output)')
        self.get_logger().info('  ✓ /cmd_vel_raw (raw output)')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Steps to test:')
        self.get_logger().info('1. Open RViz2')
        self.get_logger().info('2. Click "Nav2 Goal" button')
        self.get_logger().info('3. Click on map to set goal')
        self.get_logger().info('4. Watch diagnostics output below')
        self.get_logger().info('=' * 60)
    
    def plan_callback(self, msg: Path):
        """Monitor global plan"""
        self.last_plan = msg
        self.plan_received = True
    
    def local_plan_callback(self, msg: Path):
        """Monitor local plan"""
        self.last_local_plan = msg
    
    def cmd_vel_callback(self, msg: Twist):
        """Monitor /cmd_vel"""
        self.last_cmd_vel = msg
        self.cmd_vel_received = True
    
    def cmd_vel_nav_callback(self, msg: Twist):
        """Monitor /cmd_vel_nav"""
        self.last_cmd_vel_nav = msg
        self.cmd_vel_nav_received = True
    
    def cmd_vel_raw_callback(self, msg: Twist):
        """Monitor /cmd_vel_raw"""
        self.last_cmd_vel_raw = msg
    
    def run_diagnostics(self):
        """Run diagnostics and display status"""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('📊 NAV2 DIAGNOSTICS')
        self.get_logger().info('=' * 60)
        
        # Check planning
        if self.plan_received and self.last_plan and len(self.last_plan.poses) > 0:
            self.get_logger().info(f'✅ Global Plan: {len(self.last_plan.poses)} waypoints')
        else:
            self.get_logger().warn('⚠️  Global Plan: No plan received (set a Nav2 Goal)')
        
        # Check local plan
        if self.last_local_plan and len(self.last_local_plan.poses) > 0:
            self.get_logger().info(f'✅ Local Plan: {len(self.last_local_plan.poses)} waypoints')
        else:
            self.get_logger().warn('⚠️  Local Plan: Waiting for controller...')
        
        # Check velocity commands
        self.get_logger().info('')
        self.get_logger().info('Velocity Commands:')
        
        if self.cmd_vel_received and self.last_cmd_vel:
            vx = self.last_cmd_vel.linear.x
            vz = self.last_cmd_vel.angular.z
            self.get_logger().info(f'  ✅ /cmd_vel:     vx={vx:.3f} m/s, vz={vz:.3f} rad/s')
        else:
            self.get_logger().warn('  ❌ /cmd_vel:     No commands (should receive from Nav2)')
        
        if self.cmd_vel_nav_received and self.last_cmd_vel_nav:
            vx = self.last_cmd_vel_nav.linear.x
            vz = self.last_cmd_vel_nav.angular.z
            self.get_logger().info(f'  ℹ️  /cmd_vel_nav: vx={vx:.3f} m/s, vz={vz:.3f} rad/s')
        else:
            self.get_logger().warn('  ℹ️  /cmd_vel_nav: Not publishing')
        
        if self.last_cmd_vel_raw:
            vx = self.last_cmd_vel_raw.linear.x
            vz = self.last_cmd_vel_raw.angular.z
            self.get_logger().info(f'  ℹ️  /cmd_vel_raw: vx={vx:.3f} m/s, vz={vz:.3f} rad/s')
        else:
            self.get_logger().warn('  ℹ️  /cmd_vel_raw: Not publishing')
        
        # Status summary
        self.get_logger().info('')
        self.get_logger().info('Status Summary:')
        
        if self.plan_received:
            self.get_logger().info('✅ Nav2 is planning (global planner working)')
        else:
            self.get_logger().warn('❌ No plan yet - click "Nav2 Goal" in RViz2')
        
        if self.cmd_vel_received:
            self.get_logger().info('✅ Velocity commands on /cmd_vel (robot should move)')
        else:
            self.get_logger().warn('❌ No /cmd_vel commands (check Nav2 stack)')
        
        if self.plan_received and self.cmd_vel_received:
            self.get_logger().info('✅ Navigation pipeline is working!')
        else:
            self.get_logger().warn('⚠️  Navigation may not be working properly')
        
        self.get_logger().info('=' * 60)


def main():
    rclpy.init()
    diagnostics = Nav2Diagnostics()
    
    try:
        rclpy.spin(diagnostics)
    except KeyboardInterrupt:
        diagnostics.get_logger().info('Diagnostics stopped')
    finally:
        diagnostics.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
