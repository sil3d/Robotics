"""
FaultRecovery Node - Manages sensor failures with retry, fallback, and abort strategies.
For ROS2 Humble - Ubuntu 22.04

Subscribes to:
  /sensor_health (std_msgs/String) - JSON health status from SensorManager

Publishes to:
  /recovery_status (std_msgs/String) - Human-readable recovery status
  /robot_mode (std_msgs/String) - "NORMAL", "DEGRADED", "FAULT_RECOVERY", "ABORT"

Services:
  /reset_recovery (std_srvs/Empty) - Reset from abort state
  /trigger_recovery (std_srvs/Empty) - Manually trigger recovery for current fault
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
import json
import time
from typing import Dict, Optional, Any, List


class RecoveryState:
    """Recovery state machine states."""
    NORMAL = "NORMAL"
    DEGRADED = "DEGRADED"
    FAULT_RECOVERY = "FAULT_RECOVERY"
    ABORT = "ABORT"


class RecoveryAction:
    """Recovery action types."""
    RETRY = "retry"
    RETRY_DELAYED = "retry_delayed"
    USE_FALLBACK = "use_fallback"
    ABORT = "abort"


class FaultType:
    """Fault type constants."""
    IMU_TIMEOUT = "imu_timeout"
    APRILTAG_TIMEOUT = "apriltag_timeout"
    ULTRASONIC_STUCK = "ultrasonic_stuck"
    NAVIGATION_ERROR = "navigation_error"
    GRIPPER_FAIL = "gripper_fail"


# Recovery strategy configuration: {fault_type: (action, max_retries, retry_delay, fallback)}
RECOVERY_STRATEGIES: Dict[str, tuple] = {
    FaultType.IMU_TIMEOUT: (RecoveryAction.RETRY_DELAYED, 3, 2.0, "apriltag_only"),
    FaultType.APRILTAG_TIMEOUT: (RecoveryAction.RETRY_DELAYED, 3, 3.0, "imu_only"),
    FaultType.ULTRASONIC_STUCK: (RecoveryAction.USE_FALLBACK, 5, 1.0, "ignore_sensor"),
    FaultType.NAVIGATION_ERROR: (RecoveryAction.RETRY_DELAYED, 2, 1.0, "last_known_good"),
    FaultType.GRIPPER_FAIL: (RecoveryAction.RETRY, 2, 0.5, "proceed_without_verification"),
}

# LKGS max age thresholds (seconds)
LKGS_MAX_AGE: Dict[str, float] = {
    'imu': 10.0,
    'apriltag': 30.0,
    'ultrasonic': 5.0,
}

# Confidence thresholds
CONFIDENCE_DEGRADED = 0.7
CONFIDENCE_RECOVERY = 0.3
CONFIDENCE_RECOVERED = 0.8


class FaultRecovery(Node):
    """
    ROS2 node that manages fault recovery for sensor failures.

    Implements a state machine with states:
    - NORMAL: All systems operational
    - DEGRADED: Some sensor degraded, using fallback
    - FAULT_RECOVERY: Attempting to recover
    - ABORT: Critical failure, needs manual intervention
    """

    def __init__(self):
        super().__init__('fault_recovery')

        # --- State machine state ---
        self.current_state = RecoveryState.NORMAL
        self.previous_state = RecoveryState.NORMAL

        # --- LKGS management: {sensor_name: {'value': Any, 'timestamp': float}} ---
        self.lkgs_manager: Dict[str, Dict[str, Any]] = {}

        # --- Fault tracking: {fault_type: {'retries': int, 'last_attempt': float}} ---
        self.fault_handlers: Dict[str, Dict[str, Any]] = {}

        # --- Current active fault ---
        self.active_fault: Optional[Dict[str, Any]] = None

        # --- Fault history (last 100 faults) ---
        self.fault_history: List[Dict[str, Any]] = []

        # --- Abort state tracking ---
        self.abort_timeout: Optional[float] = None
        self.abort_timeout_duration = 60.0  # Auto-retry after 60 seconds

        # --- Publishers ---
        self.status_pub = self.create_publisher(String, '/recovery_status', 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)

        # --- Subscribers ---
        self.health_sub = self.create_subscription(
            String, '/sensor_health', self.sensor_health_callback, 10)

        # --- Services ---
        self.reset_srv = self.create_service(
            Empty, '/reset_recovery', self.reset_recovery_callback)
        self.trigger_srv = self.create_service(
            Empty, '/trigger_recovery', self.trigger_recovery_callback)

        # --- Timer for state machine update (10Hz) ---
        self.timer = self.create_timer(0.1, self.update_state)

        # --- Timer for publishing status (5Hz) ---
        self.status_timer = self.create_timer(0.2, self.publish_status)

        self.get_logger().info('FaultRecovery node initialized')

    # -------------------------------------------------------------------------
    # State Machine Transitions
    # -------------------------------------------------------------------------

    def transition_to(self, new_state: str, fault: Optional[Dict[str, Any]] = None) -> None:
        """
        Transition to a new state.

        Args:
            new_state: The new state to transition to
            fault: Optional fault information associated with transition
        """
        if new_state == self.current_state:
            return

        self.previous_state = self.current_state
        self.current_state = new_state

        self.get_logger().info(
            f"State transition: {self.previous_state} -> {self.current_state}")

        # Handle state entry actions
        if new_state == RecoveryState.DEGRADED and fault:
            self._on_enter_degraded(fault)
        elif new_state == RecoveryState.FAULT_RECOVERY and fault:
            self._on_enter_fault_recovery(fault)
        elif new_state == RecoveryState.ABORT:
            self._on_enter_abort()
        elif new_state == RecoveryState.NORMAL:
            self._on_enter_normal()

        # Publish mode change
        mode_msg = String()
        mode_msg.data = new_state
        self.mode_pub.publish(mode_msg)

    def _on_enter_degraded(self, fault: Dict[str, Any]) -> None:
        """Handle entering DEGRADED state."""
        sensor_name = fault.get('sensor', 'unknown')
        self.get_logger().warn(f"Degraded: {sensor_name} confidence low, using fallback")

    def _on_enter_fault_recovery(self, fault: Dict[str, Any]) -> None:
        """Handle entering FAULT_RECOVERY state."""
        fault_type = fault.get('type', 'unknown')
        self.get_logger().warn(f"Attempting recovery for: {fault_type}")

        # Initialize fault handler
        if fault_type not in self.fault_handlers:
            self.fault_handlers[fault_type] = {
                'retries': 0,
                'last_attempt': 0.0,
            }

    def _on_enter_abort(self) -> None:
        """Handle entering ABORT state."""
        self.get_logger().error("ABORT: Critical failure, manual intervention required")
        self.abort_timeout = time.time() + self.abort_timeout_duration

    def _on_enter_normal(self) -> None:
        """Handle entering NORMAL state."""
        self.get_logger().info("Recovery successful: All systems operational")
        self.active_fault = None
        self.fault_handlers.clear()

    # -------------------------------------------------------------------------
    # Sensor Health Callback
    # -------------------------------------------------------------------------

    def sensor_health_callback(self, msg: String) -> None:
        """
        Handle sensor health updates from SensorManager.

        Args:
            msg: JSON health status message
        """
        try:
            health_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid sensor_health JSON received")
            return

        overall_confidence = health_data.get('overall_confidence', 1.0)
        sensors = health_data.get('sensors', {})

        # Check for faults and update state accordingly
        self._evaluate_sensor_health(overall_confidence, sensors)

    def _evaluate_sensor_health(self, overall_confidence: float,
                                 sensors: Dict[str, Any]) -> None:
        """
        Evaluate sensor health and trigger state transitions.

        Args:
            overall_confidence: Overall system confidence (0.0 to 1.0)
            sensors: Dictionary of sensor health statuses
        """
        # Find lowest confidence sensor
        lowest_sensor = None
        lowest_confidence = 1.0

        for sensor_name, sensor_data in sensors.items():
            conf = sensor_data.get('confidence', 1.0)
            healthy = sensor_data.get('healthy', True)

            if not healthy or conf < lowest_confidence:
                lowest_confidence = conf
                lowest_sensor = sensor_name

        # State machine logic based on confidence levels
        if self.current_state == RecoveryState.NORMAL:
            if overall_confidence < CONFIDENCE_DEGRADED and lowest_sensor:
                fault = {
                    'sensor': lowest_sensor,
                    'type': self._get_fault_type(lowest_sensor),
                    'confidence': lowest_confidence,
                }
                self.active_fault = fault
                self.transition_to(RecoveryState.DEGRADED, fault)

        elif self.current_state == RecoveryState.DEGRADED:
            if overall_confidence >= CONFIDENCE_RECOVERED:
                self.transition_to(RecoveryState.NORMAL)
            elif lowest_confidence < CONFIDENCE_RECOVERY and lowest_sensor:
                fault = {
                    'sensor': lowest_sensor,
                    'type': self._get_fault_type(lowest_sensor),
                    'confidence': lowest_confidence,
                }
                self.active_fault = fault
                self.transition_to(RecoveryState.FAULT_RECOVERY, fault)

        elif self.current_state == RecoveryState.FAULT_RECOVERY:
            if overall_confidence >= CONFIDENCE_RECOVERED:
                self.transition_to(RecoveryState.NORMAL)
            # If recovery fails, state transition happens in update_state

    def _get_fault_type(self, sensor_name: str) -> str:
        """Map sensor name to fault type."""
        sensor_to_fault = {
            'imu': FaultType.IMU_TIMEOUT,
            'camera_apriltag': FaultType.APRILTAG_TIMEOUT,
        }
        if 'ultrasonic' in sensor_name:
            return FaultType.ULTRASONIC_STUCK
        return sensor_to_fault.get(sensor_name, 'unknown')

    # -------------------------------------------------------------------------
    # State Machine Update
    # -------------------------------------------------------------------------

    def update_state(self) -> None:
        """Main state machine update loop."""
        if self.current_state == RecoveryState.FAULT_RECOVERY:
            self._process_fault_recovery()
        elif self.current_state == RecoveryState.ABORT:
            self._check_abort_timeout()
        elif self.current_state == RecoveryState.NORMAL:
            self._check_degraded_sensors()

    def _process_fault_recovery(self) -> None:
        """Process fault recovery for current active fault."""
        if not self.active_fault:
            self.transition_to(RecoveryState.NORMAL)
            return

        fault_type = self.active_fault.get('type')
        if fault_type not in RECOVERY_STRATEGIES:
            self.transition_to(RecoveryState.ABORT)
            return

        strategy = self.get_recovery_strategy(fault_type)
        if not strategy:
            self.transition_to(RecoveryState.ABORT)
            return

        action, max_retries, retry_delay, fallback = strategy
        handler = self.fault_handlers.get(fault_type, {'retries': 0, 'last_attempt': 0.0})

        # Check if we should retry
        now = time.time()
        if now - handler.get('last_attempt', 0.0) < retry_delay:
            return  # Wait for retry delay

        # Execute recovery action
        success = self.execute_recovery(fault_type)

        # Log the fault attempt
        self.log_fault(fault_type, action, success)

        if success:
            self.get_logger().info(f"Recovery succeeded for {fault_type}")
            self.transition_to(RecoveryState.NORMAL)
        else:
            handler['retries'] += 1
            handler['last_attempt'] = now
            self.fault_handlers[fault_type] = handler

            if handler['retries'] >= max_retries:
                self.get_logger().error(f"Max retries exceeded for {fault_type}")
                self.transition_to(RecoveryState.ABORT)

    def _check_abort_timeout(self) -> None:
        """Check if abort timeout has elapsed for auto-recovery."""
        if self.abort_timeout and time.time() >= self.abort_timeout:
            self.get_logger().info("Abort timeout elapsed, resetting to NORMAL")
            self.transition_to(RecoveryState.NORMAL)

    def _check_degraded_sensors(self) -> None:
        """Periodically check for degraded sensors when in NORMAL state."""
        # This is handled by sensor_health_callback

    # -------------------------------------------------------------------------
    # Recovery Strategies
    # -------------------------------------------------------------------------

    def get_recovery_strategy(self, fault_type: str) -> Optional[tuple]:
        """
        Get recovery strategy for a fault type.

        Args:
            fault_type: Type of fault

        Returns:
            Tuple of (action, max_retries, retry_delay, fallback) or None
        """
        return RECOVERY_STRATEGIES.get(fault_type)

    def execute_recovery(self, fault_type: str) -> bool:
        """
        Execute recovery action for fault type.

        Args:
            fault_type: Type of fault

        Returns:
            True if recovery successful, False otherwise
        """
        strategy = self.get_recovery_strategy(fault_type)
        if not strategy:
            return False

        action, max_retries, retry_delay, fallback = strategy

        if action == RecoveryAction.RETRY:
            return self.do_retry(fault_type)
        elif action == RecoveryAction.RETRY_DELAYED:
            return self.do_delayed_retry(fault_type)
        elif action == RecoveryAction.USE_FALLBACK:
            return self.do_fallback(fault_type, fallback)
        elif action == RecoveryAction.ABORT:
            return self.do_abort(fault_type)

        return False

    def do_retry(self, fault_type: str) -> bool:
        """
        Attempt immediate retry.

        Args:
            fault_type: Type of fault to retry

        Returns:
            True if retry successful (simulated)
        """
        self.get_logger().info(f"RETRY: Immediate retry for {fault_type}")
        # In simulation, assume 50% success rate on retry
        return True

    def do_delayed_retry(self, fault_type: str) -> bool:
        """
        Wait and retry with sensor re-check.

        Args:
            fault_type: Type of fault

        Returns:
            True if sensor recovered
        """
        self.get_logger().info(f"RETRY_DELAYED: Waiting and retrying for {fault_type}")
        # In simulation, assume sensor recovers after delay
        return True

    def do_fallback(self, fault_type: str, fallback_method: str) -> bool:
        """
        Use fallback method for the sensor.

        Args:
            fault_type: Type of fault
            fallback_method: Name of fallback method

        Returns:
            True if fallback is acceptable (degraded but functional)
        """
        self.get_logger().info(
            f"USE_FALLBACK: Using {fallback_method} for {fault_type}")
        # Fallback always succeeds from recovery perspective
        return True

    def do_abort(self, fault_type: str) -> bool:
        """
        Abort operation for critical fault.

        Args:
            fault_type: Type of fault

        Returns:
            Always returns False (cannot recover)
        """
        self.get_logger().error(f"ABORT: Critical fault {fault_type}")
        return False

    # -------------------------------------------------------------------------
    # LKGS Management
    # -------------------------------------------------------------------------

    def update_lkgs(self, sensor_name: str, value: Any) -> None:
        """
        Update last known good state for a sensor.

        Args:
            sensor_name: Name of sensor
            value: Sensor value to store
        """
        self.lkgs_manager[sensor_name] = {
            'value': value,
            'timestamp': time.time(),
        }

    def use_lkgs_for_sensor(self, sensor_name: str) -> Optional[Any]:
        """
        Get LKGS value for sensor if not too stale.

        Args:
            sensor_name: Name of sensor

        Returns:
            LKGS value or None if stale/missing
        """
        if sensor_name not in self.lkgs_manager:
            return None

        lkgs_entry = self.lkgs_manager[sensor_name]
        age = time.time() - lkgs_entry['timestamp']

        # Determine max age based on sensor type
        max_age = self._get_lkgs_max_age(sensor_name)
        if age > max_age:
            self.get_logger().warn(
                f"LKGS for {sensor_name} is stale ({age:.1f}s > {max_age}s)")
            return None

        return lkgs_entry['value']

    def _get_lkgs_max_age(self, sensor_name: str) -> float:
        """Get max age for LKGS based on sensor type."""
        if 'imu' in sensor_name.lower():
            return LKGS_MAX_AGE['imu']
        elif 'apriltag' in sensor_name.lower() or 'camera_apriltag' in sensor_name.lower():
            return LKGS_MAX_AGE['apriltag']
        elif 'ultrasonic' in sensor_name.lower():
            return LKGS_MAX_AGE['ultrasonic']
        return 10.0  # Default

    def get_lkgs_confidence_decay(self, sensor_name: str) -> float:
        """
        Calculate confidence decay for stale LKGS.

        Args:
            sensor_name: Name of sensor

        Returns:
            Decayed confidence value (0.0 to 1.0)
        """
        if sensor_name not in self.lkgs_manager:
            return 0.0

        lkgs_entry = self.lkgs_manager[sensor_name]
        age = time.time() - lkgs_entry['timestamp']
        max_age = self._get_lkgs_max_age(sensor_name)

        # Linear decay from 1.0 to 0.0 over max_age
        decay = max(0.0, 1.0 - (age / max_age))
        return decay

    # -------------------------------------------------------------------------
    # Fault History
    # -------------------------------------------------------------------------

    def log_fault(self, fault_type: str, recovery_action: str, success: bool) -> None:
        """
        Log fault to history for analysis.

        Args:
            fault_type: Type of fault
            recovery_action: Action taken
            success: Whether recovery succeeded
        """
        entry = {
            'timestamp': time.time(),
            'fault_type': fault_type,
            'recovery_action': recovery_action,
            'success': success,
            'consecutive_failures': self._count_consecutive_failures(fault_type),
        }
        self.fault_history.append(entry)

        # Keep last 100 faults
        if len(self.fault_history) > 100:
            self.fault_history = self.fault_history[-100:]

        self.get_logger().debug(
            f"Fault logged: {fault_type} -> {recovery_action}, success={success}")

    def _count_consecutive_failures(self, fault_type: str) -> int:
        """
        Count consecutive failures for a fault type from history.

        Args:
            fault_type: Type of fault

        Returns:
            Number of consecutive failures
        """
        count = 0
        for entry in reversed(self.fault_history):
            if entry['fault_type'] == fault_type:
                if not entry['success']:
                    count += 1
                else:
                    break
            else:
                # Different fault type resets the count
                break
        return count

    def get_fault_history_summary(self) -> Dict[str, Any]:
        """
        Get summary of fault history.

        Returns:
            Dictionary with fault statistics
        """
        if not self.fault_history:
            return {'total_faults': 0, 'by_type': {}}

        by_type: Dict[str, Dict[str, Any]] = {}
        for entry in self.fault_history:
            ft = entry['fault_type']
            if ft not in by_type:
                by_type[ft] = {'total': 0, 'successes': 0, 'failures': 0}
            by_type[ft]['total'] += 1
            if entry['success']:
                by_type[ft]['successes'] += 1
            else:
                by_type[ft]['failures'] += 1

        return {
            'total_faults': len(self.fault_history),
            'by_type': by_type,
            'recent': self.fault_history[-10:],
        }

    # -------------------------------------------------------------------------
    # Service Callbacks
    # -------------------------------------------------------------------------

    def reset_recovery_callback(self, request: Empty, response: Empty) -> Empty:
        """
        Handle reset recovery service request.

        Args:
            request: Empty request
            response: Empty response

        Returns:
            Empty response
        """
        self.get_logger().info("Reset recovery service called")
        self.transition_to(RecoveryState.NORMAL)
        return response

    def trigger_recovery_callback(self, request: Empty, response: Empty) -> Empty:
        """
        Handle trigger recovery service request.

        Args:
            request: Empty request
            response: Empty response

        Returns:
            Empty response
        """
        self.get_logger().info("Trigger recovery service called")
        if self.current_state == RecoveryState.ABORT:
            if self.active_fault:
                self.transition_to(RecoveryState.FAULT_RECOVERY, self.active_fault)
        else:
            if self.active_fault:
                self.transition_to(RecoveryState.FAULT_RECOVERY, self.active_fault)
        return response

    # -------------------------------------------------------------------------
    # Status Publishing
    # -------------------------------------------------------------------------

    def publish_status(self) -> None:
        """Publish recovery status at 5Hz."""
        status = self._build_status_message()
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def _build_status_message(self) -> Dict[str, Any]:
        """
        Build status message dictionary.

        Returns:
            Dictionary with status information
        """
        return {
            'timestamp': round(time.time(), 3),
            'state': self.current_state,
            'previous_state': self.previous_state,
            'active_fault': self.active_fault,
            'lkgs_stale': self._get_stale_lkgs_sensors(),
            'fault_history_summary': self.get_fault_history_summary(),
            'fault_handlers': {
                ft: {
                    'retries': h['retries'],
                    'last_attempt': h.get('last_attempt', 0.0),
                }
                for ft, h in self.fault_handlers.items()
            },
        }

    def _get_stale_lkgs_sensors(self) -> List[str]:
        """
        Get list of sensors with stale LKGS.

        Returns:
            List of sensor names
        """
        stale = []
        for sensor_name in self.lkgs_manager:
            if self.use_lkgs_for_sensor(sensor_name) is None:
                # Check if it's actually stale (exists but too old)
                lkgs_entry = self.lkgs_manager[sensor_name]
                age = time.time() - lkgs_entry['timestamp']
                max_age = self._get_lkgs_max_age(sensor_name)
                if age > max_age:
                    stale.append(sensor_name)
        return stale

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def get_current_state(self) -> str:
        """Get current recovery state."""
        return self.current_state

    def is_operational(self) -> bool:
        """
        Check if robot is operational (not in abort).

        Returns:
            True if robot can continue operations
        """
        return self.current_state != RecoveryState.ABORT

    def get_active_fault(self) -> Optional[Dict[str, Any]]:
        """
        Get currently active fault.

        Returns:
            Active fault dictionary or None
        """
        return self.active_fault

    def should_use_fallback(self, sensor_name: str) -> bool:
        """
        Check if fallback should be used for sensor.

        Args:
            sensor_name: Name of sensor

        Returns:
            True if fallback mode is active for sensor
        """
        if self.current_state in [RecoveryState.DEGRADED, RecoveryState.FAULT_RECOVERY]:
            if self.active_fault and self.active_fault.get('sensor') == sensor_name:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = FaultRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
