#!/usr/bin/env python3
"""
Custom Pure Pursuit Path-Following Controller Node for ROS 2
============================================================
Designed to replace or act as a diagnostic alternative to the C++ controller_server.
Supports differential drive chassis kinematics.

Modes of Operation:
1. Action Mode: Hosts 'FollowPath' action server (compatible with Nav2 BT).
2. Topic Mode: Listens directly to '/plan' path topic (standalone debugging).

Features:
- Lookahead tracking with Pure Pursuit curvature calculation.
- Large heading error protection: Rotates in place if the path deviates significantly.
- Adaptive velocity profiling: Slows down on curves and decelerates near the goal.
- Goal orientation matching: Aligns yaw at the final pose.
- Resilient imports: Falls back to Topic Mode if nav2_msgs are missing.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry

import tf2_ros
from tf2_ros import TransformException

# Fallback for systems lacking Nav2 messages
try:
    from nav2_msgs.action import FollowPath
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    ACTION_MODE_AVAILABLE = True
except ImportError:
    ACTION_MODE_AVAILABLE = False


def yaw_from_quaternion(q):
    """Extract yaw (heading) from geometry_msgs/Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # ----------------- PARAMETERS -----------------
        self.declare_parameter('lookahead_dist', 0.45)        # meters
        self.declare_parameter('max_linear_vel', 0.22)        # m/s
        self.declare_parameter('min_linear_vel', 0.04)        # m/s
        self.declare_parameter('max_angular_vel', 0.8)        # rad/s
        self.declare_parameter('min_angular_vel_turn', 0.25)  # rad/s (overcomes static friction)
        
        self.declare_parameter('goal_tolerance', 0.15)        # meters
        self.declare_parameter('yaw_tolerance', 0.10)         # rad (~5.7 degrees)
        self.declare_parameter('deceleration_dist', 0.6)      # meters (start slowing down)
        
        self.declare_parameter('k_angular', 2.0)              # P gain for path curvature control
        self.declare_parameter('k_yaw', 2.2)                  # P gain for in-place turning
        self.declare_parameter('max_heading_error', 0.785)    # rad (~45 degrees, threshold for in-place turn)
        
        self.declare_parameter('robot_frame', 'base_link')    # robot local frame
        self.declare_parameter('use_action_server', True)     # Enable Nav2 Action interface
        self.declare_parameter('plan_topic', '/plan')         # Topic fallback
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')   # Output command topic
        self.declare_parameter('controller_rate', 10.0)       # Hz

        # Load values
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.min_linear_vel = self.get_parameter('min_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_angular_vel_turn = self.get_parameter('min_angular_vel_turn').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.deceleration_dist = self.get_parameter('deceleration_dist').value
        self.k_angular = self.get_parameter('k_angular').value
        self.k_yaw = self.get_parameter('k_yaw').value
        self.max_heading_error = self.get_parameter('max_heading_error').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.use_action_server = self.get_parameter('use_action_server').value
        self.plan_topic = self.get_parameter('plan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.controller_rate = self.get_parameter('controller_rate').value

        # Log parameters
        self.get_logger().info(f"Loaded config: L={self.lookahead_dist}m, V_max={self.max_linear_vel}m/s, W_max={self.max_angular_vel}rad/s")

        # ----------------- TF SETUP -----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ----------------- PUBLISHERS -----------------
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ----------------- MODE DECISION -----------------
        self.active_path = None
        
        if self.use_action_server and ACTION_MODE_AVAILABLE:
            self.get_logger().info("Initializing FollowPath Action Server mode...")
            self.action_server = ActionServer(
                self,
                FollowPath,
                'follow_path',
                execute_callback=self.action_execute_callback,
                goal_callback=self.action_goal_callback,
                cancel_callback=self.action_cancel_callback
            )
        else:
            if self.use_action_server and not ACTION_MODE_AVAILABLE:
                self.get_logger().warn("Action server mode requested, but nav2_msgs not found. Falling back to Topic Mode.")
            
            self.get_logger().info(f"Initializing Topic Mode (subscribing to {self.plan_topic})...")
            self.plan_sub = self.create_subscription(Path, self.plan_topic, self.plan_callback, 10)
            
            # Control timer for Topic Mode
            self.control_timer = self.create_timer(1.0 / self.controller_rate, self.topic_control_loop)

        self.get_logger().info("Pure Pursuit Controller Node Initialized.")

    # -------------------------------------------------------------
    # CONTROL LOGIC (SHARED BETWEEN TOPIC AND ACTION MODES)
    # -------------------------------------------------------------

    def control_step(self, path: Path):
        """
        Executes one control iteration.
        Returns: (success, linear_vel, angular_vel, distance_to_goal)
        """
        if not path or len(path.poses) == 0:
            return False, 0.0, 0.0, 0.0

        path_frame = path.header.frame_id

        # 1. Look up robot's current pose in path frame
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                path_frame,
                self.robot_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
            ryaw = yaw_from_quaternion(transform.transform.rotation)
        except TransformException as e:
            self.get_logger().warn(f"Could not transform {self.robot_frame} to {path_frame}: {e}")
            return False, 0.0, 0.0, 999.0

        # 2. Extract goal pose (last point on path)
        goal_pose = path.poses[-1].pose
        gx = goal_pose.position.x
        gy = goal_pose.position.y
        gyaw = yaw_from_quaternion(goal_pose.orientation)

        # 3. Calculate distance and yaw error to goal
        dx_goal = gx - rx
        dy_goal = gy - ry
        dist_to_goal = math.sqrt(dx_goal**2 + dy_goal**2)
        yaw_to_goal = normalize_angle(gyaw - ryaw)

        # 4. Check if robot has reached goal position
        if dist_to_goal < self.goal_tolerance:
            # We are at the position. Now align the yaw (heading).
            if abs(yaw_to_goal) < self.yaw_tolerance:
                self.get_logger().info("Goal position and yaw tolerance reached!")
                return True, 0.0, 0.0, dist_to_goal
            else:
                # Rotate in place to match goal orientation
                omega = self.k_yaw * yaw_to_goal
                omega = self.clamp_angular(omega)
                omega = self.apply_min_turn(omega)
                self.get_logger().info(f"Goal position reached. Aligning yaw: err={yaw_to_goal:.3f} rad, cmd_w={omega:.3f}", once=True)
                return False, 0.0, omega, dist_to_goal

        # 5. Goal not reached: Find lookahead point
        lookahead_pt = self.find_lookahead_point(path, rx, ry)
        lx = lookahead_pt.pose.position.x
        ly = lookahead_pt.pose.position.y

        # 6. Transform lookahead point to robot's local coordinate frame
        dx = lx - rx
        dy = ly - ry
        local_x = math.cos(ryaw) * dx + math.sin(ryaw) * dy
        local_y = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
        lookahead_dist_act = math.sqrt(local_x**2 + local_y**2)

        # 7. Compute heading error to lookahead point
        heading_error = math.atan2(local_y, local_x)

        # 8. Check if rotation in place is required (heading error exceeds threshold)
        if abs(heading_error) > self.max_heading_error:
            omega = self.k_yaw * heading_error
            omega = self.clamp_angular(omega)
            omega = self.apply_min_turn(omega)
            self.get_logger().info(f"Large heading error ({heading_error:.3f} rad). Rotating in place...", once=True)
            return False, 0.0, omega, dist_to_goal

        # 9. Normal Pure Pursuit tracking (Drive forward + Steer)
        # Curvature formula: k = 2 * local_y / L^2
        if lookahead_dist_act > 0.05:
            curvature = (2.0 * local_y) / (lookahead_dist_act**2)
        else:
            curvature = 0.0

        # Adaptive linear velocity
        # A: slow down when heading error is larger
        v = self.max_linear_vel * math.cos(heading_error)
        v = max(0.0, v)

        # B: slow down as we approach the goal (deceleration profile)
        if dist_to_goal < self.deceleration_dist:
            v_scale = dist_to_goal / self.deceleration_dist
            v = v * v_scale

        # Clamp linear velocity to limits
        v = max(self.min_linear_vel, min(self.max_linear_vel, v))

        # Compute angular velocity: w = v * curvature
        omega = v * curvature * self.k_angular
        omega = self.clamp_angular(omega)

        return False, v, omega, dist_to_goal

    # -------------------------------------------------------------
    # HELPER UTILITIES
    # -------------------------------------------------------------

    def find_lookahead_point(self, path: Path, rx: float, ry: float) -> PoseStamped:
        """Finds the point on the path closest to the target lookahead distance."""
        # Find path index closest to the robot
        min_dist = float('inf')
        closest_idx = 0
        for i, pose_stamped in enumerate(path.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            dist = math.sqrt((px - rx)**2 + (py - ry)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Search forward along the path for first point >= lookahead distance
        for i in range(closest_idx, len(path.poses)):
            px = path.poses[i].pose.position.x
            py = path.poses[i].pose.position.y
            dist = math.sqrt((px - rx)**2 + (py - ry)**2)
            if dist >= self.lookahead_dist:
                return path.poses[i]

        # Default to the end of the path (goal) if lookahead distance is not met
        return path.poses[-1]

    def clamp_angular(self, omega: float) -> float:
        """Clamp angular velocity to parameter limit."""
        return max(-self.max_angular_vel, min(self.max_angular_vel, omega))

    def apply_min_turn(self, omega: float) -> float:
        """Ensures turn commands are strong enough to overcome friction when rotating in place."""
        if abs(omega) < self.min_angular_vel_turn:
            if omega >= 0:
                return self.min_angular_vel_turn
            else:
                return -self.min_angular_vel_turn
        return omega

    def stop_robot(self):
        """Publish zero velocity to safely halt the robot."""
        msg = Twist()
        try:
            if rclpy.ok():
                self.cmd_vel_pub.publish(msg)
        except Exception as e:
            # Context might be invalid during shutdown, which is expected
            pass

    def publish_cmd(self, v, w):
        """Publish a velocity twist command."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

    # -------------------------------------------------------------
    # TOPIC MODE IMPLEMENTATION
    # -------------------------------------------------------------

    def plan_callback(self, msg: Path):
        """Receives new paths in Topic Mode."""
        self.get_logger().info(f"Received new path via topic ({len(msg.poses)} points)")
        self.active_path = msg

    def topic_control_loop(self):
        """Timer loop execution for Topic Mode."""
        if self.active_path is None:
            return

        success, v, w, dist = self.control_step(self.active_path)

        if success:
            self.stop_robot()
            self.get_logger().info("Finished tracking path successfully.")
            self.active_path = None
        else:
            self.publish_cmd(v, w)

    # -------------------------------------------------------------
    # ACTION MODE IMPLEMENTATION (Only registered if nav2_msgs is available)
    # -------------------------------------------------------------

    if ACTION_MODE_AVAILABLE:
        def action_goal_callback(self, goal_request):
            """Accept or reject incoming FollowPath goals."""
            self.get_logger().info("Received new path request via Action Server")
            return GoalResponse.ACCEPT

        def action_cancel_callback(self, goal_handle):
            """Accept goal cancellations."""
            self.get_logger().info("Cancel request received for active goal")
            return CancelResponse.ACCEPT

        def action_execute_callback(self, goal_handle):
            """Core execution loop for tracking action goals."""
            self.get_logger().info("Executing FollowPath action...")
            
            path = goal_handle.request.path
            feedback = FollowPath.Feedback()
            result = FollowPath.Result()
            
            rate = self.create_rate(self.controller_rate)
            
            while rclpy.ok():
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.stop_robot()
                    self.get_logger().info("FollowPath action cancelled.")
                    return result

                # Compute control values
                success, v, w, dist = self.control_step(path)

                if success:
                    self.stop_robot()
                    goal_handle.succeed()
                    self.get_logger().info("FollowPath action succeeded!")
                    return result
                
                # Publish velocities to the robot
                self.publish_cmd(v, w)

                # Publish feedback (Note: FollowPath feedback uses float32 speed, not Twist)
                feedback.distance_to_goal = float(dist)
                feedback.speed = float(v)
                goal_handle.publish_feedback(feedback)

                rate.sleep()

            # Safeguard if loop breaks unexpectedly
            self.stop_robot()
            return result


# -------------------------------------------------------------
# RUN ENTRYPOINT
# -------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    
    # Use MultiThreadedExecutor to support concurrent action server call and TF queries
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Pure Pursuit controller...")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
