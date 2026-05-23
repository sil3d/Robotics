#!/usr/bin/env python3
"""
RobotState - Central state management for the robot.

Maintains belief about:
- Robot pose with confidence
- Sensor health and last known good states
- Gripper state
- Fault state
"""

import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Any, List


@dataclass
class SensorReading:
    """Single sensor reading with metadata"""
    value: Optional[Any] = None
    confidence: float = 1.0
    timestamp: float = 0.0
    is_healthy: bool = True


@dataclass
class GripperState:
    """Gripper state tracking"""
    position: float = 0.0  # 0 = open, 90 = closed
    has_box: bool = False
    last_verified_time: float = 0.0
    confidence: float = 1.0


@dataclass
class FaultState:
    """Fault state tracking"""
    in_fault: bool = False
    fault_type: Optional[str] = None
    fault_message: Optional[str] = None
    recovery_attempts: int = 0
    max_recovery_attempts: int = 3


class RobotState:
    """
    Central state management class for the robot.

    Thread-safe state container that maintains belief about robot pose,
    sensor health, gripper state, fault state, and mission progress.

    Used by task_manager, localization, navigation, and other nodes.
    """

    # Sensor names
    SENSOR_IMU = 'imu'
    SENSOR_ULTRASONIC_FRONT = 'ultrasonic_front'
    SENSOR_ULTRASONIC_BACK = 'ultrasonic_back'
    SENSOR_ULTRASONIC_LEFT = 'ultrasonic_left'
    SENSOR_ULTRASONIC_RIGHT = 'ultrasonic_right'
    SENSOR_CAMERA_APRILTAG = 'camera_apriltag'
    SENSOR_CAMERA_COLOR = 'camera_color'
    SENSOR_GRIPPER = 'gripper'

    ALL_SENSORS = [
        SENSOR_IMU,
        SENSOR_ULTRASONIC_FRONT,
        SENSOR_ULTRASONIC_BACK,
        SENSOR_ULTRASONIC_LEFT,
        SENSOR_ULTRASONIC_RIGHT,
        SENSOR_CAMERA_APRILTAG,
        SENSOR_CAMERA_COLOR,
        SENSOR_GRIPPER,
    ]

    def __init__(self):
        """Initialize robot state with default values."""
        # Mutex for thread-safe access
        self._lock = threading.RLock()

        # Pose state
        self.pose_x: float = 0.0
        self.pose_y: float = 0.0
        self.pose_theta: float = 0.0
        self.pose_confidence: float = 0.0
        self._pose_lkgs_x: float = 0.0
        self._pose_lkgs_y: float = 0.0
        self._pose_lkgs_theta: float = 0.0
        self._pose_lkgs_timestamp: float = 0.0

        # Sensor readings - dict of {sensor_name: {'lkgs': SensorReading, 'current': SensorReading}}
        self.sensors: Dict[str, Dict[str, SensorReading]] = {}
        for sensor_name in self.ALL_SENSORS:
            self.sensors[sensor_name] = {
                'lkgs': SensorReading(),
                'current': SensorReading(),
            }

        # Gripper state
        self.gripper_state = GripperState()

        # Fault state
        self.fault_state = FaultState()

        # Mission state
        self.mission_state: str = "IDLE"

        # Confidence thresholds
        self._apriltag_fresh_seconds: float = 2.0
        self._imu_fresh_seconds: float = 0.1
        self._pose_uncertain_seconds: float = 30.0
        self._confidence_decay_rate: float = 0.1  # per second

        # Tracking timestamps for confidence calculation
        self._last_apriltag_time: float = 0.0
        self._last_imu_time: float = 0.0

    def update_pose(self, x: float, y: float, theta: float, confidence: float) -> None:
        """
        Update robot pose with new estimate.

        Args:
            x: X position in meters
            y: Y position in meters
            theta: Orientation in radians
            confidence: Confidence score (0.0 to 1.0)
        """
        with self._lock:
            self.pose_x = x
            self.pose_y = y
            self.pose_theta = theta
            self.pose_confidence = confidence

            # Update LKGS if confidence is good
            if confidence > 0.5:
                self._pose_lkgs_x = x
                self._pose_lkgs_y = y
                self._pose_lkgs_theta = theta
                self._pose_lkgs_timestamp = time.time()

    def update_sensor_health(self, sensor_name: str, is_healthy: bool,
                             confidence: float, value: Any = None) -> None:
        """
        Update sensor health status.

        When sensor fails, the current reading becomes the LKGS.

        Args:
            sensor_name: Name of sensor (from ALL_SENSORS)
            is_healthy: Whether sensor is currently healthy
            confidence: Confidence score (0.0 to 1.0)
            value: Optional sensor value to store
        """
        with self._lock:
            if sensor_name not in self.sensors:
                return

            current = self.sensors[sensor_name]['current']
            lkgs = self.sensors[sensor_name]['lkgs']

            # Store old reading as LKGS if sensor transitioning from healthy to unhealthy
            if current.is_healthy and not is_healthy:
                lkgs.value = current.value
                lkgs.confidence = current.confidence
                lkgs.timestamp = current.timestamp
                lkgs.is_healthy = True

            # Update current reading
            current.is_healthy = is_healthy
            current.confidence = confidence
            current.timestamp = time.time()
            if value is not None:
                current.value = value

            # Track IMU and AprilTag timestamps for confidence calculation
            if sensor_name == self.SENSOR_IMU:
                self._last_imu_time = time.time()
            elif sensor_name == self.SENSOR_CAMERA_APRILTAG and is_healthy:
                self._last_apriltag_time = time.time()

    def update_apriltag_time(self, timestamp: float) -> None:
        """
        Update the last AprilTag detection timestamp.

        Args:
            timestamp: Unix timestamp of AprilTag detection
        """
        with self._lock:
            self._last_apriltag_time = timestamp

    def get_last_known_good_pose(self) -> tuple:
        """
        Get the last known good pose (LKGS).

        Returns:
            Tuple of (x, y, theta)
        """
        with self._lock:
            return (self._pose_lkgs_x, self._pose_lkgs_y, self._pose_lkgs_theta)

    def update_gripper_state(self, position: float, has_box: bool) -> None:
        """
        Update gripper state.

        Args:
            position: Gripper position (0 = open, 90 = closed)
            has_box: Whether gripper has detected a box
        """
        with self._lock:
            self.gripper_state.position = position
            self.gripper_state.has_box = has_box
            self.gripper_state.last_verified_time = time.time()
            # Confidence degrades if not verified recently
            self.gripper_state.confidence = min(1.0, self.gripper_state.confidence)

    def verify_gripper(self) -> bool:
        """
        Verify gripper state is reliable.

        Returns:
            True if gripper state is trustworthy
        """
        with self._lock:
            # Gripper is reliable if verified within last 2 seconds
            time_since_verify = time.time() - self.gripper_state.last_verified_time
            return (time_since_verify < 2.0 and
                    self.gripper_state.confidence > 0.5 and
                    self.sensors[self.SENSOR_GRIPPER]['current'].is_healthy)

    def get_gripper_confidence(self) -> float:
        """
        Get current gripper confidence score.

        Returns:
            Confidence value between 0.0 and 1.0
        """
        with self._lock:
            return self.gripper_state.confidence

    def enter_fault(self, fault_type: str, fault_message: Optional[str] = None) -> None:
        """
        Enter fault state.

        Args:
            fault_type: Type of fault (e.g., 'sensor_timeout', 'collision')
            fault_message: Optional human-readable description
        """
        with self._lock:
            self.fault_state.in_fault = True
            self.fault_state.fault_type = fault_type
            self.fault_state.fault_message = fault_message
            self.fault_state.recovery_attempts += 1

    def exit_fault(self) -> None:
        """Exit fault state and reset recovery attempts."""
        with self._lock:
            self.fault_state.in_fault = False
            self.fault_state.fault_type = None
            self.fault_state.fault_message = None
            self.fault_state.recovery_attempts = 0

    def get_overall_confidence(self) -> float:
        """
        Calculate weighted confidence from all sensors.

        Confidence factors:
        - AprilTag detection fresh (< 2s): +0.4
        - IMU fresh (< 100ms): +0.3
        - Ultrasonics agreeing (variance < threshold): +0.3
        - Sensor timeout penalties: reduce confidence over time

        Returns:
            Overall confidence score (0.0 to 1.0)
        """
        with self._lock:
            confidence = 0.0
            current_time = time.time()

            # AprilTag contribution (+0.4 if fresh)
            time_since_apriltag = current_time - self._last_apriltag_time
            if time_since_apriltag < self._apriltag_fresh_seconds:
                apriltag_conf = self.sensors[self.SENSOR_CAMERA_APRILTAG]['current'].confidence
                confidence += 0.4 * apriltag_conf

            # IMU contribution (+0.3 if fresh)
            time_since_imu = current_time - self._last_imu_time
            if time_since_imu < self._imu_fresh_seconds:
                imu_conf = self.sensors[self.SENSOR_IMU]['current'].confidence
                confidence += 0.3 * imu_conf

            # Ultrasonics agreement (+0.3 if agreeing)
            ultrasonic_variance = self._calc_ultrasonic_variance()
            if ultrasonic_variance < 0.1:  # Low variance means sensors agree
                confidence += 0.3

            # Pose confidence contribution
            confidence += 0.0 * self.pose_confidence

            # Decay if pose uncertain (no AprilTag for > 30s)
            if self._last_apriltag_time > 0 and time_since_apriltag > self._pose_uncertain_seconds:
                decay = (time_since_apriltag - self._pose_uncertain_seconds) * self._confidence_decay_rate
                confidence = max(0.0, confidence - decay)

            # Normalize to [0, 1]
            return max(0.0, min(1.0, confidence))

    def _calc_ultrasonic_variance(self) -> float:
        """
        Calculate variance across ultrasonic sensors.

        Returns:
            Variance of ultrasonic readings (lower = more agreement)
        """
        with self._lock:
            readings = []
            for sensor_name in [self.SENSOR_ULTRASONIC_FRONT, self.SENSOR_ULTRASONIC_BACK,
                                self.SENSOR_ULTRASONIC_LEFT, self.SENSOR_ULTRASONIC_RIGHT]:
                current = self.sensors[sensor_name]['current']
                if current.is_healthy and current.value is not None:
                    readings.append(current.value)

            if len(readings) < 2:
                return 1.0  # High variance (no agreement possible)

            mean = sum(readings) / len(readings)
            variance = sum((r - mean) ** 2 for r in readings) / len(readings)
            return variance

    def reset(self) -> None:
        """Reset to initial state."""
        with self._lock:
            # Reset pose
            self.pose_x = 0.0
            self.pose_y = 0.0
            self.pose_theta = 0.0
            self.pose_confidence = 0.0
            self._pose_lkgs_x = 0.0
            self._pose_lkgs_y = 0.0
            self._pose_lkgs_theta = 0.0
            self._pose_lkgs_timestamp = 0.0

            # Reset sensors
            for sensor_name in self.sensors:
                self.sensors[sensor_name] = {
                    'lkgs': SensorReading(),
                    'current': SensorReading(),
                }

            # Reset gripper
            self.gripper_state = GripperState()

            # Reset fault
            self.fault_state = FaultState()

            # Reset mission
            self.mission_state = "IDLE"

            # Reset timestamps
            self._last_apriltag_time = 0.0
            self._last_imu_time = 0.0

    def get_sensor_health_status(self) -> Dict[str, Dict[str, Any]]:
        """
        Get health status for all sensors.

        Returns:
            Dict of sensor status with healthy, confidence, last_update info
        """
        with self._lock:
            status = {}
            current_time = time.time()
            for sensor_name, readings in self.sensors.items():
                current = readings['current']
                lkgs = readings['lkgs']
                status[sensor_name] = {
                    'healthy': current.is_healthy,
                    'confidence': current.confidence,
                    'last_update': current.timestamp,
                    'has_lkgs': lkgs.value is not None,
                    'time_since_update': current_time - current.timestamp if current.timestamp > 0 else float('inf'),
                }
            return status

    def get_fault_status(self) -> Dict[str, Any]:
        """
        Get current fault status.

        Returns:
            Dict with fault state information
        """
        with self._lock:
            return {
                'in_fault': self.fault_state.in_fault,
                'fault_type': self.fault_state.fault_type,
                'fault_message': self.fault_state.fault_message,
                'recovery_attempts': self.fault_state.recovery_attempts,
                'max_recovery_attempts': self.fault_state.max_recovery_attempts,
            }

    def get_mission_state(self) -> str:
        """
        Get current mission state.

        Returns:
            Current mission state string
        """
        with self._lock:
            return self.mission_state

    def set_mission_state(self, state: str) -> None:
        """
        Set mission state.

        Args:
            state: New mission state
        """
        with self._lock:
            self.mission_state = state

    def get_state_summary(self) -> Dict[str, Any]:
        """
        Get complete state summary for debugging.

        Returns:
            Dict with all state information
        """
        with self._lock:
            return {
                'pose': {
                    'x': self.pose_x,
                    'y': self.pose_y,
                    'theta': self.pose_theta,
                    'confidence': self.pose_confidence,
                },
                'gripper': {
                    'position': self.gripper_state.position,
                    'has_box': self.gripper_state.has_box,
                    'confidence': self.gripper_state.confidence,
                },
                'fault': self.get_fault_status(),
                'mission_state': self.mission_state,
                'overall_confidence': self.get_overall_confidence(),
            }

    def __repr__(self) -> str:
        """String representation for debugging."""
        with self._lock:
            return (f"RobotState(pose=({self.pose_x:.2f}, {self.pose_y:.2f}, "
                    f"{self.pose_theta:.2f}, conf={self.pose_confidence:.2f}), "
                    f"mission={self.mission_state}, fault={self.fault_state.in_fault})")


# Alias for backwards compatibility
SensorReading = SensorReading


if __name__ == '__main__':
    # Quick test when run directly
    rs = RobotState()
    rs.update_pose(1.0, 2.0, 0.5, 0.9)
    print(f"Pose: x={rs.pose_x}, y={rs.pose_y}, theta={rs.pose_theta}, conf={rs.pose_confidence}")
    rs.update_sensor_health('imu', True, 0.95)
    print(f"IMU healthy: {rs.sensors['imu']['current'].is_healthy}")
    print(f"IMU confidence: {rs.sensors['imu']['current'].confidence}")
    print('RobotState OK')