#!/usr/bin/env python3
"""
 ===========================================================================
   NAVIGATION NODE - Waypoint Following with PID Control
 ===========================================================================

  High-level navigation node that:
  1. Maintains a list of waypoints (x, y, theta goals)
  2. Uses PID controller to track path
  3. Publishes velocity commands to /cmd_vel
  4. Uses ultrasonic sensors for obstacle detection and speed modulation

  Waypoints are received from task_manager or can be set via service.

  PID CONTROLLER:
   - Linear velocity PID: controls forward speed to reach waypoint
   - Angular velocity PID: controls turn rate to align with waypoint

  SUBSCRIBERS:
   - /robot_pose (geometry_msgs/msg/Pose) - current robot position
   - /waypoint (geometry_msgs/msg/Point) - target waypoint (x, y, 0=theta)
   - /ultrasonic_data (geometry_msgs/msg/Point) - ultrasonic distances

  PUBLISHERS:
   - /cmd_vel (geometry_msgs/msg/Twist) - velocity commands to ESP32
   - /navigation_status (geometry_msgs/msg/Point) - status updates

  SERVICES:
   - /emergency_stop (std_srvs/srv/Empty) - immediate stop

  PARAMETERS:
   - max_linear_speed: max forward speed m/s (default: 0.2)
   - max_angular_speed: max turn rate rad/s (default: 1.5)
   - pid_linear_kp, ki, kd: linear velocity PID gains
   - pid_angular_kp, ki, kd: angular velocity PID gains
   - obstacle_threshold_near: stop distance (m, default: 0.20)
   - obstacle_threshold_far: slowdown distance (m, default: 0.50)
   - obstacle_slowdown_factor: speed multiplier when warning (default: 0.5)
   - use_sensor_confidence: use pose confidence for navigation (default: True)

  ===========================================================================
"""

import json
import math
import os
import time

import numpy as np
import rclpy
import rclpy.parameter
from geometry_msgs.msg import Point, PoseStamped, Twist
from rclpy.node import Node
from std_srvs.srv import Empty


class PIDController:
    """Simple PID controller with anti-windup"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.prev_error = 0.0
        self.integral = 0.0
        self._last_ts = time.perf_counter()

    def compute(self, current, target):
        now = time.perf_counter()
        dt = now - self._last_ts
        self._last_ts = now
        if dt <= 0 or dt > 0.5:
            dt = 0.05
        error = target - current
        self.integral += error * dt
        windup_limit = min(5.0, self.output_limit / max(self.ki, 1e-6))
        self.integral = max(-windup_limit, min(windup_limit, self.integral))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        """Reset integrator and derivative state (call when stopping)"""
        self.integral = 0.0
        self.prev_error = 0.0
        self._last_ts = time.perf_counter()


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Parameters
        self.declare_parameter('max_linear_speed', 0.20)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('pid_linear_kp', 2.0)
        self.declare_parameter('pid_linear_ki', 0.1)
        self.declare_parameter('pid_linear_kd', 0.5)
        self.declare_parameter('pid_angular_kp', 3.0)
        self.declare_parameter('pid_angular_ki', 0.2)
        self.declare_parameter('pid_angular_kd', 1.0)
        self.declare_parameter('waypoint_threshold', 0.15)  # meters to consider waypoint reached
        self.declare_parameter('angle_threshold', 0.2)  # radians
        self.declare_parameter('obstacle_threshold_near', 0.20)   # meters - stop!
        self.declare_parameter('obstacle_threshold_far', 0.50)    # meters - slow down
        self.declare_parameter('obstacle_slowdown_factor', 0.5)   # multiply speed by this
        self.declare_parameter('use_sensor_confidence', True)

        # H1: override paramètres depuis robot_config.json si disponible
        self._apply_robot_config()

        self.max_lin = self.get_parameter('max_linear_speed').value
        self.max_ang = self.get_parameter('max_angular_speed').value
        self.obstacle_threshold_near = self.get_parameter('obstacle_threshold_near').value
        self.obstacle_threshold_far = self.get_parameter('obstacle_threshold_far').value
        self.obstacle_slowdown_factor = self.get_parameter('obstacle_slowdown_factor').value
        self.use_sensor_confidence = self.get_parameter('use_sensor_confidence').value

        # PID controllers
        self.pid_linear = PIDController(
            self.get_parameter('pid_linear_kp').value,
            self.get_parameter('pid_linear_ki').value,
            self.get_parameter('pid_linear_kd').value,
            self.max_lin
        )
        self.pid_angular = PIDController(
            self.get_parameter('pid_angular_kp').value,
            self.get_parameter('pid_angular_ki').value,
            self.get_parameter('pid_angular_kd').value,
            self.max_ang
        )

        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        self.angle_threshold = self.get_parameter('angle_threshold').value

        self.get_logger().info('Navigation Node started')


        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Sensor confidence tracking
        self.last_confident_pose = None
        self.pose_confidence_threshold = 0.3

        # Ultrasonic obstacle detection state
        self.us_front = -1.0
        self.us_back = -1.0
        self.us_left = -1.0
        self.us_right = -1.0
        self.front_obstacle = False
        self.front_obstacle_warning = False

        # Target waypoint
        self.target_x = None
        self.target_y = None
        self.target_theta = None
        self.waypoint_active = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Point, '/navigation_status', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10)
        self.waypoint_sub = self.create_subscription(
            Point, '/waypoint', self.waypoint_callback, 10)
        self.ultrasonic_sub = self.create_subscription(
            Point, '/ultrasonic_data', self.ultrasonic_callback, 10)

        # Services
        self.e_stop_srv = self.create_service(
            Empty, '/emergency_stop', self.emergency_stop_callback)

        # Timer for control loop at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        # Stop robot initially
        self._publish_vel(0.0, 0.0)

    def pose_callback(self, msg: PoseStamped):
        """Update current robot position with confidence tracking"""
        # Use header stamp age as a rough confidence indicator
        confidence = 1.0
        try:
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            now_sec = self.get_clock().now().nanoseconds / 1e9
            stamp_age = now_sec - stamp_sec
            confidence = max(0.0, min(1.0, 1.0 - stamp_age))
        except Exception:
            pass

        if confidence > self.pose_confidence_threshold:
            self.last_confident_pose = (msg.pose.position.x, msg.pose.position.y)

        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

        # Extract yaw from quaternion
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, \
                         msg.pose.orientation.z, msg.pose.orientation.w
        self.current_yaw = self._quaternion_to_yaw(qx, qy, qz, qw)

    def waypoint_callback(self, msg: Point):
        """Receive new waypoint from task_manager"""
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_theta = msg.z if msg.z != 0 else None
        self.waypoint_active = True
        self.get_logger().info(f'New waypoint: ({self.target_x:.2f}, {self.target_y:.2f})')
        # Reset PID state
        self.pid_linear.reset()
        self.pid_angular.reset()

    def ultrasonic_callback(self, msg: Point):
        """Process ultrasonic data for obstacle detection"""
        # msg.x = front, msg.y = back, msg.z = left, msg.w = right (stored in z for legacy)
        self.us_front = msg.x / 100.0 if msg.x > 0 else -1.0  # cm to meters
        self.us_back = msg.y / 100.0 if msg.y > 0 else -1.0
        self.us_left = msg.z / 100.0 if msg.z > 0 else -1.0
        self.us_right = msg.w / 100.0 if hasattr(msg, 'w') and msg.w > 0 else -1.0

        # Detect obstacles
        self.front_obstacle = self.us_front > 0 and self.us_front < self.obstacle_threshold_near
        self.front_obstacle_warning = self.us_front > 0 and self.us_front < self.obstacle_threshold_far

        self.get_logger().debug(f"US: front={self.us_front:.2f}m back={self.us_back:.2f}m")

    def emergency_stop_callback(self, request, response):
        """Immediate stop - all velocities zero"""
        self.get_logger().warn("EMERGENCY STOP!")
        self.waypoint_active = False
        self._publish_vel(0.0, 0.0)
        return response

    def control_loop(self):
        """Main control loop - compute and publish velocity commands"""
        if not self.waypoint_active or self.target_x is None:
            # No active waypoint, stop
            self._publish_vel(0.0, 0.0)
            return

        # Check for obstacle in path
        if self.front_obstacle:
            # STOP - obstacle too close; reset PID to avoid integral windup during halt
            self.pid_linear.reset()
            self.pid_angular.reset()
            self.get_logger().warn(f"OBSTACLE DETECTED! front={self.us_front:.2f}m - STOPPING")
            self._publish_vel(0.0, 0.0)

            # Publish obstacle status
            status = Point()
            status.x = self.current_x
            status.y = self.current_y
            status.z = 2.0  # 2 = obstacle stop
            self.status_pub.publish(status)
            return

        # Compute distance and angle to waypoint
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dist_to_waypoint = math.sqrt(dx*dx + dy*dy)

        # Angle to waypoint in world frame
        angle_to_waypoint = math.atan2(dy, dx)

        # Relative angle: angle between robot heading and waypoint direction
        relative_angle = self._normalize_angle(angle_to_waypoint - self.current_yaw)

        # Check if waypoint reached
        if dist_to_waypoint < self.waypoint_threshold:
            self.get_logger().info(f'Waypoint reached! ({self.target_x:.2f}, {self.target_y:.2f})')
            if self.target_theta is not None:
                # Now rotate to target orientation
                angle_error = self._normalize_angle(self.target_theta - self.current_yaw)
                if abs(angle_error) > self.angle_threshold:
                    angular_vel = self.pid_angular.compute(0, angle_error)
                    self._publish_vel(0.0, angular_vel)
                    return

            # Waypoint fully completed
            self.waypoint_active = False
            self._publish_vel(0.0, 0.0)

            # Publish status
            status = Point()
            status.x = self.current_x
            status.y = self.current_y
            status.z = 1.0  # 1 = waypoint reached
            self.status_pub.publish(status)
            return

        # PID control for angular velocity (turn towards waypoint)
        angular_vel = self.pid_angular.compute(0, relative_angle)

        # Apply speed modulation based on front distance
        speed_modifier = 1.0
        if self.front_obstacle_warning:
            # Slow down when approaching obstacle
            speed_modifier = self.obstacle_slowdown_factor
            self.get_logger().debug(f"Obstacle warning - reducing speed to {speed_modifier*100:.0f}%")

        # PID control for linear velocity (move towards waypoint)
        # Slow down when angle error is large
        angle_ok = abs(relative_angle) < math.radians(30)
        if angle_ok:
            linear_vel = self.pid_linear.compute(0, dist_to_waypoint)
            linear_vel *= speed_modifier  # Apply obstacle slowdown
        else:
            linear_vel = 0.0  # Don't move forward if not facing waypoint

        self._publish_vel(linear_vel, angular_vel)

        # Debug
        self.get_logger().debug(
            f'dist={dist_to_waypoint:.2f} angle={math.degrees(relative_angle):.1f}deg '
            f'lin={linear_vel:.2f} ang={angular_vel:.2f}'
        )

    def _publish_vel(self, linear_x, angular_z):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = max(-self.max_lin, min(self.max_lin, linear_x))
        msg.angular.z = max(-self.max_ang, min(self.max_ang, angular_z))
        self.cmd_vel_pub.publish(msg)

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _quaternion_to_yaw(self, x, y, z, w):
        """Extract yaw from quaternion"""
        # Yaw (rotation around Z)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _apply_robot_config(self):
        """Charge robot_config.json et override les paramètres ROS2 si présents."""
        cfg_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            'data', 'robot_config.json'
        )
        try:
            with open(cfg_path, 'r') as f:
                cfg = json.load(f)
        except Exception:
            return
        nav = cfg.get('navigation', {})
        overrides = [
            ('max_linear_speed',       nav.get('max_linear_speed')),
            ('max_angular_speed',      nav.get('max_angular_speed')),
            ('pid_linear_kp',          nav.get('pid_linear_kp')),
            ('pid_linear_ki',          nav.get('pid_linear_ki')),
            ('pid_linear_kd',          nav.get('pid_linear_kd')),
            ('pid_angular_kp',         nav.get('pid_angular_kp')),
            ('pid_angular_ki',         nav.get('pid_angular_ki')),
            ('pid_angular_kd',         nav.get('pid_angular_kd')),
            ('obstacle_threshold_near',nav.get('obstacle_threshold_near')),
            ('obstacle_threshold_far', nav.get('obstacle_threshold_far')),
        ]
        for param, val in overrides:
            if val is not None:
                self.set_parameters([rclpy.parameter.Parameter(param,
                    rclpy.parameter.Parameter.Type.DOUBLE, float(val))])
        self.get_logger().info(f'[CFG] robot_config.json nav section loaded from {cfg_path}')

    def destroy_node(self):
        self._publish_vel(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()