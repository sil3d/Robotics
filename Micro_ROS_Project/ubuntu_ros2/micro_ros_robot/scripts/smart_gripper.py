#!/usr/bin/env python3
"""
===========================================================================
 SMART GRIPPER CONTROLLER - Intelligent Gripper with Sensor Verification
===========================================================================

 ROS2 Node that:
   - Manages gripper calibration (min/max open distance)
   - Verifies box presence before/after picking using ultrasonic
   - Detects if gripper actually grabbed the box
   - Reports gripper state (empty, has_box, failed)

 Subscribes to:
   - /ultrasonic_data (geometry_msgs/Point) - front, back, left, right distances
   - /gripper_feedback (std_msgs/Float32) - actual gripper position (0-90 degrees)

 Publishes to:
   - /gripper_cmd (std_msgs/String) - commands to ESP32: "open", "close"
   - /gripper_status (std_msgs/String) - status: {state, has_box, confidence, calibration}

 Services:
   - /calibrate_gripper (std_srvs/Empty) - trigger calibration
   - /gripper_pick (std_srvs/Empty) - perform verified pick sequence
   - /gripper_release (std_srvs/Empty) - release box

 Usage:
   ros2 run micro_ros_robot smart_gripper

===========================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float32
from std_srvs.srv import Empty
import time
import json


class CalibrationState:
    """Gripper calibration states"""
    UNCALIBRATED = "uncallibrated"
    CALIBRATING = "calibrating"
    CALIBRATED = "calibrated"
    FAILED = "failed"


class GripperState:
    """Gripper mechanical states"""
    OPEN = "open"
    CLOSING = "closing"
    CLOSED = "closed"
    OPENING = "opening"
    ERROR = "error"


class GrabResult:
    """Grab detection results"""
    SUCCESS = "success"
    FAILED = "failed"
    NO_PRE_PICK = "no_pre_pick"
    NO_GRIP = "no_grip"
    SENSOR_ERROR = "sensor_error"


class ServiceResponse:
    """Service call response"""
    def __init__(self, success: bool, message: str, has_box: bool = False):
        self.success = success
        self.message = message
        self.has_box = has_box


class SmartGripperController(Node):
    """
    Intelligent gripper controller with sensor verification.

    Features:
    - Calibration mode to learn max open distance
    - Pre-pick verification using ultrasonic
    - Post-pick verification to confirm grab
    - Retry logic for failed grabs
    - State machine for gripper control
    """

    def __init__(self):
        super().__init__('smart_gripper_controller')

        # --- Calibration State ---
        self.calibration_state = CalibrationState.UNCALIBRATED
        self.max_open_distance = 0.0  # cm, set during calibration

        # --- Gripper State ---
        self.gripper_state = GripperState.OPEN
        self.gripper_position = 180.0  # 180 = open, 20 = closed
        self.has_box = False
        self.confidence = 1.0

        # --- Ultrasonic Data ---
        self.ultrasonic_front = 0.0
        self.ultrasonic_back = 0.0
        self.ultrasonic_left = 0.0
        self.ultrasonic_right = 0.0
        self._ultrasonic_last_update = 0.0

        # --- Pre-pick tracking ---
        self.pre_pick_distance = None

        # --- Publishers ---
        self.gripper_cmd_pub = self.create_publisher(String, '/gripper_cmd', 10)
        self.gripper_status_pub = self.create_publisher(String, '/gripper_status', 10)

        # --- Subscribers ---
        self.ultrasonic_sub = self.create_subscription(
            Point, '/ultrasonic_data', self._ultrasonic_callback, 10)
        self.gripper_feedback_sub = self.create_subscription(
            Float32, '/gripper_feedback', self._gripper_feedback_callback, 10)

        # --- Services ---
        self.calibrate_srv = self.create_service(
            Empty, '/calibrate_gripper', self._calibrate_gripper_callback)
        self.pick_srv = self.create_service(
            Empty, '/gripper_pick', self._gripper_pick_callback)
        self.release_srv = self.create_service(
            Empty, '/gripper_release', self._gripper_release_callback)

        # --- Timer for status publishing ---
        self.timer = self.create_timer(0.5, self._publish_status)

        self.get_logger().info('SmartGripperController initialized')

    # =========================================================================
    # Ultrasonic & Feedback Callbacks
    # =========================================================================

    def _ultrasonic_callback(self, msg: Point):
        """Handle ultrasonic data from ESP32.

        Point.x = front distance (cm)
        Point.y = back distance (cm)
        Point.z = left distance (cm)
        Point.w = right distance (cm)
        -1 or negative = no reading / error
        """
        self.ultrasonic_front = msg.x
        self.ultrasonic_back = msg.y
        self.ultrasonic_left = msg.z
        self.ultrasonic_right = msg.w
        self._ultrasonic_last_update = time.time()

    def _gripper_feedback_callback(self, msg: Float32):
        """Handle gripper position feedback from ESP32.

        Position in degrees: 180 = fully open, 20 = fully closed
        """
        self.gripper_position = msg.data

        # Update state based on position feedback
        if self.gripper_state == GripperState.CLOSING:
            if msg.data < 40:  # Closed position threshold (near 20°)
                self.gripper_state = GripperState.CLOSED

        elif self.gripper_state == GripperState.OPENING:
            if msg.data > 150:  # Open position threshold (near 180°)
                self.gripper_state = GripperState.OPEN

    # =========================================================================
    # Helper Methods
    # =========================================================================

    def get_ultrasonic_distance(self, direction: str) -> float:
        """
        Get ultrasonic distance for a specific direction.

        Args:
            direction: 'front', 'back', 'left', or 'right'

        Returns:
            Distance in cm, or -1 if invalid/no reading
        """
        distances = {
            'front': self.ultrasonic_front,
            'back': self.ultrasonic_back,
            'left': self.ultrasonic_left,
            'right': self.ultrasonic_right,
        }
        dist = distances.get(direction, -1)

        # Validate: -1 or negative means no reading
        if dist < 0:
            return -1

        return dist

    def send_command(self, command: str):
        """
        Send command to gripper via ESP32.

        Args:
            command: "open" or "close"
        """
        msg = String()
        msg.data = command
        self.gripper_cmd_pub.publish(msg)
        self.get_logger().debug(f'Sent gripper command: {command}')

    def set_state(self, state: GripperState):
        """Set gripper mechanical state."""
        self.gripper_state = state
        self.get_logger().debug(f'Gripper state -> {state}')

    def wait_for_state(self, target_state: GripperState, timeout: float = 2.0) -> bool:
        """
        Wait for gripper to reach target state.

        Args:
            target_state: The state to wait for
            timeout: Maximum time to wait in seconds

        Returns:
            True if state reached, False if timeout
        """
        start_time = time.time()
        while rclpy.ok():
            if self.gripper_state == target_state:
                return True
            if time.time() - start_time > timeout:
                return False
            time.sleep(0.05)

    # =========================================================================
    # Calibration
    # =========================================================================

    def start_calibration(self):
        """Start calibration sequence.

        User should place box against closed gripper jaws before calling.
        """
        self.calibration_state = CalibrationState.CALIBRATING
        self.get_logger().info('Calibration started - place box against gripper')
        return ServiceResponse(
            success=True,
            message='Calibration started - place box against gripper jaws, then call complete'
        )

    def complete_calibration(self) -> ServiceResponse:
        """
        Complete calibration using ultrasonic reading.

        Reads front ultrasonic distance and sets it as max_open_distance
        if the reading is valid (positive and less than 50cm).

        Returns:
            ServiceResponse with success status and message
        """
        dist = self.get_ultrasonic_distance('front')

        if dist > 0 and dist < 50:  # Valid reading
            self.max_open_distance = dist
            self.calibration_state = CalibrationState.CALIBRATED
            self.confidence = 1.0
            self.get_logger().info(f'Calibration complete: max_open={dist:.1f}cm')
            return ServiceResponse(
                success=True,
                message=f'Calibration complete: max_open={dist:.1f}cm',
                has_box=False
            )
        else:
            self.calibration_state = CalibrationState.FAILED
            self.confidence = 0.0
            self.get_logger().error(f'Calibration failed - invalid distance: {dist}cm')
            return ServiceResponse(
                success=False,
                message=f'Calibration failed - invalid distance: {dist}cm',
                has_box=False
            )

    def is_calibrated(self) -> bool:
        """Check if gripper is calibrated."""
        return self.calibration_state == CalibrationState.CALIBRATED

    # =========================================================================
    # Pre-Pick and Post-Pick Verification
    # =========================================================================

    def verify_pre_pick(self) -> dict:
        """
        Verify box is in position for picking (pre-pick check).

        Returns:
            dict with 'ok' (bool), 'reason' (str), 'distance' (float)
        """
        dist = self.get_ultrasonic_distance('front')

        # Check for sensor errors
        if dist < 0:
            return {'ok': False, 'reason': 'SENSOR_ERROR', 'distance': dist}

        # Box should be close (5-30cm from ultrasonic sensor)
        if dist < 5:
            return {'ok': False, 'reason': 'BOX_TOO_CLOSE', 'distance': dist}
        if dist > 30:
            return {'ok': False, 'reason': 'BOX_TOO_FAR', 'distance': dist}

        # Store pre-pick distance for later comparison
        self.pre_pick_distance = dist
        return {'ok': True, 'distance': dist}

    def verify_pickup(self) -> dict:
        """
        Verify gripper actually grabbed the box (post-pick check).

        Compares ultrasonic reading before and after closing gripper.
        If box was there before and still there after, grab was successful.

        Returns:
            dict with 'ok' (bool), 'reason' (str), 'has_box' (bool), 'distance' (float)
        """
        dist_after = self.get_ultrasonic_distance('front')

        if self.pre_pick_distance is None:
            return {'ok': False, 'reason': 'NO_PRE_PICK_DATA', 'has_box': False, 'distance': dist_after}

        if dist_after < 0:  # Sensor error
            return {'ok': False, 'reason': 'SENSOR_ERROR', 'has_box': False, 'distance': dist_after}

        # Check if box is still detected in roughly same position
        distance_change = abs(dist_after - self.pre_pick_distance)

        if distance_change < 5:  # Box still roughly there
            # Additional check: did gripper actually close?
            if self.gripper_position > 45:  # Gripper is closed
                return {'ok': True, 'has_box': True, 'distance': dist_after}
            else:
                return {'ok': False, 'reason': 'GRIPPER_NOT_CLOSED', 'has_box': False, 'distance': dist_after}
        else:
            return {'ok': False, 'reason': 'BOX_MOVED', 'has_box': False, 'distance': dist_after}

    # =========================================================================
    # Grab Detection
    # =========================================================================

    def detect_grab(self) -> str:
        """
        Detect if gripper successfully grabbed box.

        Logic:
        1. Before closing: ultrasonic shows object at distance D1
        2. After closing:
           - If gripper is at 90° (closed) AND
           - Ultrasonic still shows something at distance D2 (box between jaws) AND
           - |D2 - D1| < threshold (box didn't move away)
           → Grab successful

        3. If gripper closed but ultrasonic shows far distance:
           → Box was pushed away or not there → Grab failed

        Returns:
            GrabResult string: SUCCESS, FAILED, NO_PRE_PICK, NO_GRIP, SENSOR_ERROR
        """
        current_dist = self.get_ultrasonic_distance('front')
        gripper_closed = self.gripper_position > 45

        if not gripper_closed:
            return GrabResult.NO_GRIP

        if current_dist < 0:  # Sensor error
            return GrabResult.SENSOR_ERROR

        if self.pre_pick_distance is None:
            return GrabResult.NO_PRE_PICK

        dist_diff = abs(current_dist - self.pre_pick_distance)

        if dist_diff < 5 and current_dist < 30:  # Box still detected nearby
            return GrabResult.SUCCESS
        else:
            return GrabResult.FAILED

    # =========================================================================
    # Pick Sequence
    # =========================================================================

    def pick_sequence(self) -> ServiceResponse:
        """
        Complete verified picking sequence with retry logic.

        Steps:
        1. Verify pre-pick (box in position)
        2. Send close command
        3. Wait for gripper to close
        4. Verify pickup (box still between jaws)
        5. If failed: retry once, then abort

        Returns:
            ServiceResponse with success, message, and has_box flag
        """
        # Step 1: Pre-pick verification
        self.get_logger().info('Performing pre-pick verification...')
        pre_check = self.verify_pre_pick()
        if not pre_check['ok']:
            self.get_logger().warn(f'Pre-pick failed: {pre_check["reason"]}')
            return ServiceResponse(
                success=False,
                message=f'Pre-pick failed: {pre_check["reason"]}',
                has_box=False
            )
        self.get_logger().info(f'Pre-pick OK: box at {pre_check["distance"]:.1f}cm')

        # Step 2: Close gripper
        self.get_logger().info('Closing gripper...')
        self.send_command('close')
        self.set_state(GripperState.CLOSING)

        # Step 3: Wait for gripper to reach position (2 seconds max)
        if not self.wait_for_state(GripperState.CLOSED, timeout=2.0):
            # Retry once
            self.get_logger().warn('Gripper close timeout, retrying...')
            self.send_command('open')
            time.sleep(0.5)
            self.send_command('close')

            if not self.wait_for_state(GripperState.CLOSED, timeout=2.0):
                self.get_logger().error('Gripper close failed after retry')
                self.set_state(GripperState.ERROR)
                return ServiceResponse(
                    success=False,
                    message='Gripper close failed after retry',
                    has_box=False
                )

        self.get_logger().info('Gripper closed')

        # Step 4: Verify pickup
        self.get_logger().info('Verifying pickup...')
        result = self.verify_pickup()

        if result['ok']:
            self.gripper_state = GripperState.CLOSED
            self.has_box = True
            self.confidence = 0.9
            self.get_logger().info('Pickup successful - box acquired')
            return ServiceResponse(
                success=True,
                message='Pickup successful',
                has_box=True
            )

        # Retry once
        self.get_logger().warn(f'Pickup verification failed: {result["reason"]}, retrying...')
        self.send_command('open')
        time.sleep(0.5)

        # Re-verify pre-pick after opening
        pre_check = self.verify_pre_pick()
        if not pre_check['ok']:
            self.set_state(GripperState.ERROR)
            return ServiceResponse(
                success=False,
                message=f'Pickup retry failed: {pre_check["reason"]}',
                has_box=False
            )

        self.send_command('close')
        result = self.verify_pickup()

        if result['ok']:
            self.gripper_state = GripperState.CLOSED
            self.has_box = True
            self.confidence = 0.8  # Lower confidence for retry success
            self.get_logger().info('Pickup successful on retry')
            return ServiceResponse(
                success=True,
                message='Pickup successful on retry',
                has_box=True
            )

        self.set_state(GripperState.ERROR)
        self.has_box = False
        self.confidence = 0.0
        self.get_logger().error(f'Pickup failed: {result["reason"]}')
        return ServiceResponse(
            success=False,
            message=f'Pickup failed: {result["reason"]}',
            has_box=False
        )

    # =========================================================================
    # Release Sequence
    # =========================================================================

    def release_sequence(self) -> ServiceResponse:
        """
        Release the box by opening gripper.

        Returns:
            ServiceResponse with success status and message
        """
        self.get_logger().info('Releasing box...')
        self.send_command('open')
        self.set_state(GripperState.OPENING)

        if not self.wait_for_state(GripperState.OPEN, timeout=2.0):
            self.get_logger().warn('Gripper open timeout, continuing anyway')

        self.has_box = False
        self.gripper_state = GripperState.OPEN
        self.pre_pick_distance = None  # Reset for next pick
        self.get_logger().info('Box released')
        return ServiceResponse(
            success=True,
            message='Box released',
            has_box=False
        )

    # =========================================================================
    # Service Callbacks
    # =========================================================================

    def _calibrate_gripper_callback(self, request, response):
        """Handle /calibrate_gripper service call.

        Starts calibration sequence. User must place box against gripper
        before calling complete. Results published via /gripper_status topic.
        """
        self.get_logger().info('Calibrate gripper service called')

        if self.calibration_state == CalibrationState.CALIBRATING:
            result = self.complete_calibration()
            if result.success:
                self.get_logger().info(f'Calibration succeeded: {result.message}')
            else:
                self.get_logger().error(f'Calibration failed: {result.message}')
        else:
            result = self.start_calibration()
            self.get_logger().info(f'Calibration started: {result.message}')

        return response

    def _gripper_pick_callback(self, request, response):
        """Handle /gripper_pick service call.

        Executes verified pick sequence. Results published via /gripper_status topic.
        """
        self.get_logger().info('Gripper pick service called')

        if not self.is_calibrated():
            self.get_logger().warn('Pick attempted but gripper not calibrated')
            self.confidence = 0.0
            return response

        result = self.pick_sequence()
        if result.success:
            self.get_logger().info(f'Pick succeeded: {result.message}')
        else:
            self.get_logger().error(f'Pick failed: {result.message}')

        return response

    def _gripper_release_callback(self, request, response):
        """Handle /gripper_release service call.

        Releases box by opening gripper. Results published via /gripper_status topic.
        """
        self.get_logger().info('Gripper release service called')

        result = self.release_sequence()
        if result.success:
            self.get_logger().info(f'Release succeeded: {result.message}')
        else:
            self.get_logger().warn(f'Release completed with issues: {result.message}')

        return response

    # =========================================================================
    # Status Publishing
    # =========================================================================

    def _publish_status(self):
        """Publish gripper status at 2Hz."""
        status = {
            'state': self.gripper_state.value if hasattr(self.gripper_state, 'value') else str(self.gripper_state),
            'calibration': self.calibration_state.value if hasattr(self.calibration_state, 'value') else str(self.calibration_state),
            'has_box': self.has_box,
            'confidence': round(self.confidence, 2),
            'gripper_position': round(self.gripper_position, 1),
            'ultrasonic_front': round(self.ultrasonic_front, 1),
            'max_open_distance': round(self.max_open_distance, 1),
        }

        msg = String()
        msg.data = json.dumps(status)
        self.gripper_status_pub.publish(msg)

    # =========================================================================
    # Error Recovery
    # =========================================================================

    def reset_error(self) -> ServiceResponse:
        """Reset error state and return to open state."""
        if self.gripper_state == GripperState.ERROR:
            self.get_logger().info('Resetting from error state')
            self.send_command('open')
            time.sleep(0.5)
            self.set_state(GripperState.OPEN)
            self.has_box = False
            self.confidence = 0.5
            return ServiceResponse(
                success=True,
                message='Error reset - gripper open',
                has_box=False
            )
        return ServiceResponse(
            success=True,
            message='Not in error state',
            has_box=self.has_box
        )


class SmartGripperServiceResponse:
    """Service response wrapper for proper ROS2 service handling."""
    def __init__(self):
        self.success = True
        self.message = ""
        self.has_box = False


def main(args=None):
    """Main entry point for the smart gripper controller node."""
    rclpy.init(args=args)

    node = SmartGripperController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down SmartGripperController')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()