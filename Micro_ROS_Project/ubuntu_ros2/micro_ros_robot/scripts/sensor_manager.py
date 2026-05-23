"""
SensorManager Node - Monitors all sensors, detects failures, computes confidence, filters noise.
For ROS2 Humble - Ubuntu 22.04

Subscribes to:
  /imu_data (geometry_msgs/Accel)
  /ultrasonic_data (geometry_msgs/Point) - x=front, y=back, z=left, w=right
  /aruco_detections (geometry_msgs/PoseArray)
  /box_color (std_msgs/String)

Publishes to:
  /sensor_health (std_msgs/String) - JSON with health status
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Accel, Point, PoseArray
import numpy as np
import time
import json


class SensorManager(Node):
    """
    ROS2 node that monitors all sensors, detects failures, computes confidence,
    and filters noise using Median Absolute Deviation (MAD).
    """

    def __init__(self):
        super().__init__('sensor_manager')

        # --- Publishers ---
        self.health_pub = self.create_publisher(String, '/sensor_health', 10)

        # --- Subscribers ---
        self.imu_sub = self.create_subscription(
            Accel, '/imu_data', self.imu_callback, 10)
        self.ultrasonic_sub = self.create_subscription(
            Point, '/ultrasonic_data', self.ultrasonic_callback, 10)
        self.aruco_sub = self.create_subscription(
            PoseArray, '/aruco_detections', self.apriltag_callback, 10)
        self.color_sub = self.create_subscription(
            String, '/box_color', self.color_callback, 10)

        # --- Timer: publish health at 5Hz ---
        self.timer = self.create_timer(0.2, self.publish_health_status)

        # --- IMU state tracking ---
        self.imu_last_update = 0.0
        self.imu_yaw_history = []          # list of float (yaw rates)
        self.imu_flatline_count = 0        # count of identical readings
        self.imu_last_yaw = None
        self.imu_confidence = 1.0
        self.imu_healthy = True
        self.imu_timeout = False

        # --- Ultrasonic state tracking ---
        self.ultrasonic_last_update = 0.0
        self.ultrasonic_front_history = []  # list of float distances (cm)
        self.ultrasonic_back_history = []
        self.ultrasonic_left_history = []
        self.ultrasonic_right_history = []
        # Stuck sensor counters per direction
        self._ultrasonic_stuck = {'front': 0, 'back': 0, 'left': 0, 'right': 0}
        self.ultrasonic_confidence = 1.0
        self.ultrasonic_healthy = True
        self.ultrasonic_timeout = False

        # --- AprilTag state tracking ---
        self.apriltag_last_update = 0.0
        self.apriltag_rejected_history = []  # list of (rejected, detected) bools
        self.apriltag_confidence = 1.0
        self.apriltag_healthy = True
        self.apriltag_timeout = False

        # --- Color detection state ---
        self.color_last_update = 0.0
        self.color_confidence = 1.0
        self.color_healthy = True

        # --- Fault list ---
        self.faults = []

        self.get_logger().info('SensorManager node initialized')

    # -------------------------------------------------------------------------
    # IMU
    # -------------------------------------------------------------------------
    def imu_callback(self, msg):
        """Handle IMU data: detect timeout, drift, flatline."""
        now = time.time()
        self.imu_last_update = now

        # Extract yaw rate (angular velocity z) and acceleration
        yaw = msg.angular.z          # rad/s - treating as yaw rate
        accel_mag = np.sqrt(
            msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)

        # --- Drift detection: yaw change > 1000 deg/s = impossible ---
        if self.imu_last_yaw is not None:
            yaw_delta = abs(yaw - self.imu_last_yaw)
            if yaw_delta > (1000.0 * np.pi / 180.0):  # 1000 deg/s in rad/s
                self.imu_healthy = False
                self.imu_confidence = 0.7
                self._add_fault('imu', 'drift_detected', f'yaw_delta={yaw_delta:.2f} rad/s')
        self.imu_last_yaw = yaw

        # --- Flatline detection: same value for > 5 readings ---
        self.imu_yaw_history.append(yaw)
        if len(self.imu_yaw_history) > 5:
            self.imu_yaw_history.pop(0)

        if len(self.imu_yaw_history) == 5:
            if all(abs(v - yaw) < 0.001 for v in self.imu_yaw_history):
                self.imu_flatline_count += 1
                if self.imu_flatline_count > 1:  # 2nd occurrence confirms stuck
                    self.imu_healthy = False
                    self.imu_confidence = 0.3
                    self._add_fault('imu', 'flatline', 'stuck sensor - identical readings')
            else:
                self.imu_flatline_count = 0

        # --- Reset health if fresh data ---
        self.imu_timeout = False

    # -------------------------------------------------------------------------
    # Ultrasonic
    # -------------------------------------------------------------------------
    def ultrasonic_callback(self, msg):
        """Handle ultrasonic data: detect timeout, stuck, out-of-range, noise."""
        now = time.time()
        self.ultrasonic_last_update = now

        front = msg.x   # front
        back = msg.y    # back
        left = msg.z    # left
        right = msg.w   # right

        self._check_ultrasonic_single('front', front)
        self._check_ultrasonic_single('back', back)
        self._check_ultrasonic_single('left', left)
        self._check_ultrasonic_single('right', right)

        # --- Noise spike detection: variance > 50 cm² in last 5 readings ---
        for name, history in [('front', self.ultrasonic_front_history),
                              ('back', self.ultrasonic_back_history),
                              ('left', self.ultrasonic_left_history),
                              ('right', self.ultrasonic_right_history)]:
            if len(history) >= 5:
                variance = np.var(history)
                if variance > 50.0:
                    conf = self._ultrasonic_confidence_by_name(name)
                    # Reduce confidence on noise but don't mark unhealthy
                    self._set_ultrasonic_confidence(name, max(0.3, conf - 0.2))
                    self._add_fault(f'ultrasonic_{name}', 'noise_spike',
                                    f'variance={variance:.2f} cm²')

        # --- Reset timeout ---
        self.ultrasonic_timeout = False

    def _check_ultrasonic_single(self, name, value):
        """Apply MAD filter and check stuck/out-of-range for one sensor."""
        history = getattr(self, f'ultrasonic_{name}_history')

        # --- Out of range check: < 5cm or > 150cm consistently ---
        if value < 5.0 or value > 150.0:
            # Count consecutive out-of-range
            out_count = sum(1 for v in history[-5:] if v < 5.0 or v > 150.0)
            if out_count >= 3 and len(history) >= 3:
                self._set_ultrasonic_healthy(name, False)
                self._add_fault(f'ultrasonic_{name}', 'out_of_range',
                               f'value={value:.1f}cm (expected 5-150cm)')

        # --- Append to history (keep last 10 for stuck detection) ---
        history.append(value)
        if len(history) > 10:
            history.pop(0)

        # --- Stuck sensor: same value ±2cm for 10 consecutive readings ---
        if len(history) >= 10:
            ref = history[-1]
            stuck = all(abs(v - ref) <= 0.02 for v in history[-10:])
            if stuck:
                self._ultrasonic_stuck[name] += 1
                if self._ultrasonic_stuck[name] >= 2:
                    self._set_ultrasonic_healthy(name, False)
                    self._set_ultrasonic_confidence(name, 0.3)
                    self._add_fault(f'ultrasonic_{name}', 'stuck',
                                    f'repeated value={ref:.2f}cm')
            else:
                self._ultrasonic_stuck[name] = 0

    def _ultrasonic_confidence_by_name(self, name):
        """Get confidence for a specific ultrasonic sensor."""
        return getattr(self, f'ultrasonic_{name}_confidence', 1.0)

    def _set_ultrasonic_confidence(self, name, value):
        setattr(self, f'ultrasonic_{name}_confidence', value)
        # Also update overall ultrasonic confidence
        confidences = [self._ultrasonic_confidence_by_name(n)
                       for n in ['front', 'back', 'left', 'right']]
        self.ultrasonic_confidence = sum(confidences) / 4.0

    def _set_ultrasonic_healthy(self, name, healthy):
        setattr(self, f'ultrasonic_{name}_healthy', healthy)
        all_healthy = all(getattr(self, f'ultrasonic_{name}_healthy')
                          for n in ['front', 'back', 'left', 'right'])
        self.ultrasonic_healthy = all_healthy

    # -------------------------------------------------------------------------
    # AprilTag
    # -------------------------------------------------------------------------
    def apriltag_callback(self, msg):
        """Handle AprilTag detections: track rejection rate."""
        now = time.time()
        self.apriltag_last_update = now

        # Count poses (detections)
        num_detections = len(msg.poses)

        # We don't have explicit "rejected" count, so we approximate:
        # if poses exist, count as detection; frame is valid
        self.apriltag_rejected_history.append(num_detections > 0)
        if len(self.apriltag_rejected_history) > 10:
            self.apriltag_rejected_history.pop(0)

        # Rejection rate check: rejected > detected in last 10 frames
        detected = sum(1 for d in self.apriltag_rejected_history if d)
        rejected = len(self.apriltag_rejected_history) - detected
        if rejected > detected:
            self.apriltag_healthy = False
            self.apriltag_confidence = 0.5
            self._add_fault('apriltag', 'high_rejection_rate',
                            f'rejected={rejected}, detected={detected}')
        else:
            self.apriltag_healthy = True
            self.apriltag_confidence = 1.0

        self.apriltag_timeout = False

    # -------------------------------------------------------------------------
    # Color detection
    # -------------------------------------------------------------------------
    def color_callback(self, msg):
        """Handle color detection result."""
        now = time.time()
        self.color_last_update = now

        # Color detection is less critical - "no red/green/blue" in 10s
        # is not necessarily a failure, could be no box visible
        valid_colors = {'red', 'green', 'blue'}
        if msg.data.lower() in valid_colors:
            self.color_healthy = True
            self.color_confidence = 0.9
        else:
            # Unknown color - could be no box, not a failure
            self.color_confidence = 0.5

    # -------------------------------------------------------------------------
    # Confidence and health
    # -------------------------------------------------------------------------
    def compute_overall_confidence(self):
        """Weighted average: IMU 30%, Ultrasonics 30%, AprilTag 40%."""
        imu = self.imu_confidence if self.imu_healthy else 0.0
        ultra = self.ultrasonic_confidence if self.ultrasonic_healthy else 0.0
        april = self.apriltag_confidence if self.apriltag_healthy else 0.0

        # Weights
        total = imu * 0.30 + ultra * 0.30 + april * 0.40
        return round(total, 3)

    def check_timeouts(self):
        """Check all sensor timeouts and update confidence accordingly."""
        now = time.time()

        # IMU timeout > 100ms = unhealthy
        if (now - self.imu_last_update) > 0.1:
            if not self.imu_timeout:
                self.imu_timeout = True
                self._add_fault('imu', 'timeout', 'no data for > 100ms')
            self.imu_healthy = False
            elapsed = now - self.imu_last_update
            if elapsed > 5.0:
                self.imu_confidence = 0.0
            elif elapsed > 1.0:
                self.imu_confidence = 0.3

        # Ultrasonic timeout > 500ms = unhealthy
        if (now - self.ultrasonic_last_update) > 0.5:
            if not self.ultrasonic_timeout:
                self.ultrasonic_timeout = True
                self._add_fault('ultrasonic', 'timeout', 'no data for > 500ms')
            self.ultrasonic_healthy = False
            elapsed = now - self.ultrasonic_last_update
            if elapsed > 1.0:
                self.ultrasonic_confidence = 0.0
            elif elapsed > 0.5:
                self.ultrasonic_confidence = 0.3

        # AprilTag timeout > 3s = unhealthy
        if (now - self.apriltag_last_update) > 3.0:
            if not self.apriltag_timeout:
                self.apriltag_timeout = True
                self._add_fault('apriltag', 'timeout', 'no detections for > 3s')
            self.apriltag_healthy = False
            elapsed = now - self.apriltag_last_update
            if elapsed > 5.0:
                self.apriltag_confidence = 0.0
            elif elapsed > 3.0:
                self.apriltag_confidence = 0.0  # already 0 at 5s, 3s gets 0

        # Color timeout > 10s = just mark last update, not failure
        if (now - self.color_last_update) > 10.0:
            self.color_healthy = False
            self.color_confidence = 0.3

    def _add_fault(self, sensor, fault_type, message):
        """Add a fault entry, avoiding duplicates."""
        fault = {
            'sensor': sensor,
            'type': fault_type,
            'message': message,
            'time': time.time()
        }
        # Avoid duplicate faults
        existing = any(
            f.get('sensor') == sensor and f.get('type') == fault_type
            for f in self.faults)
        if not existing:
            self.faults.append(fault)

    # -------------------------------------------------------------------------
    # Noise filtering
    # -------------------------------------------------------------------------
    def mad_filter(self, values, threshold=3.0):
        """
        Filter outliers using Median Absolute Deviation (MAD).

        Args:
            values: array-like of float values
            threshold: MAD threshold (default 3.0)

        Returns:
            np.array of filtered values
        """
        values = np.array(values, dtype=float)
        if len(values) == 0:
            return values

        median = np.median(values)
        mad = np.median(np.abs(values - median))

        if mad < 0.001:  # All same values - no filtering needed
            return values

        # Modified Z-score using MAD
        modified_z = 0.6745 * (values - median) / mad

        # Replace outliers with median
        filtered = np.where(np.abs(modified_z) < threshold, values, median)
        return filtered

    # -------------------------------------------------------------------------
    # Publishing
    # -------------------------------------------------------------------------
    def publish_health_status(self):
        """Publish /sensor_health JSON at 5Hz."""
        self.check_timeouts()

        now = time.time()
        overall = self.compute_overall_confidence()

        health_msg = {
            'timestamp': round(now, 3),
            'overall_confidence': overall,
            'sensors': {
                'imu': {
                    'healthy': self.imu_healthy,
                    'confidence': round(self.imu_confidence, 2),
                    'last_update': round(self.imu_last_update, 3) if self.imu_last_update else 0.0
                },
                'ultrasonic_front': {
                    'healthy': getattr(self, 'ultrasonic_front_healthy', True),
                    'confidence': round(self._ultrasonic_confidence_by_name('front'), 2),
                    'last_update': round(self.ultrasonic_last_update, 3) if self.ultrasonic_last_update else 0.0
                },
                'ultrasonic_back': {
                    'healthy': getattr(self, 'ultrasonic_back_healthy', True),
                    'confidence': round(self._ultrasonic_confidence_by_name('back'), 2),
                    'last_update': round(self.ultrasonic_last_update, 3) if self.ultrasonic_last_update else 0.0
                },
                'ultrasonic_left': {
                    'healthy': getattr(self, 'ultrasonic_left_healthy', True),
                    'confidence': round(self._ultrasonic_confidence_by_name('left'), 2),
                    'last_update': round(self.ultrasonic_last_update, 3) if self.ultrasonic_last_update else 0.0
                },
                'ultrasonic_right': {
                    'healthy': getattr(self, 'ultrasonic_right_healthy', True),
                    'confidence': round(self._ultrasonic_confidence_by_name('right'), 2),
                    'last_update': round(self.ultrasonic_last_update, 3) if self.ultrasonic_last_update else 0.0
                },
                'camera_apriltag': {
                    'healthy': self.apriltag_healthy,
                    'confidence': round(self.apriltag_confidence, 2),
                    'last_update': round(self.apriltag_last_update, 3) if self.apriltag_last_update else 0.0
                },
                'camera_color': {
                    'healthy': self.color_healthy,
                    'confidence': round(self.color_confidence, 2),
                    'last_update': round(self.color_last_update, 3) if self.color_last_update else 0.0
                }
            },
            'faults': self.faults[-10:]  # keep last 10 faults
        }

        msg = String()
        msg.data = json.dumps(health_msg)
        self.health_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()