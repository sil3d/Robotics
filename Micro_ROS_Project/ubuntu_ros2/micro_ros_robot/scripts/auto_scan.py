#!/usr/bin/env python3
"""
 ===========================================================================
    AUTO SCAN MODE - Autonomous AprilTag Marker Discovery and Mapping
 ===========================================================================

 Scans the environment to autonomously discover AprilTag markers and build
 a marker map for localization.

 ROS2 Node that:
  - Rotates robot slowly while scanning for AprilTags
  - Accumulates marker detections with multiple views
  - Builds marker map with position estimates
  - Reports when all expected markers are found

 Triggered when:
  - User starts auto mode with no prior marker map
  - Robot approaches a marker but doesn't see it (wrong position assumed)
  - User provides marker count and robot should verify/find all markers

 PUBLISHERS:
  - /cmd_vel (geometry_msgs/msg/Twist) - velocity commands for scanning
  - /scan_status (std_msgs/String) - scan progress status
  - /marker_map (std_msgs/String) - discovered marker map as JSON

 SUBSCRIBERS:
  - /robot_pose (geometry_msgs/msg/Pose) - current robot position
  - /aruco_detections (geometry_msgs/msg/PoseArray) - AprilTag detections

 SERVICES:
  - /start_scan (std_srvs/Empty) - start scanning with expected count param

 ===========================================================================
 """

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseArray, Twist
from std_msgs.msg import String
from std_srvs.srv import Empty
import json
import os
import math
import time


class ScanState:
    """Scan state machine states"""
    IDLE = "idle"
    ROTATING = "rotating"
    MARKER_FOUND = "marker_found"
    SCAN_COMPLETE = "scan_complete"
    SCAN_FAILED = "scan_failed"


class AutoScanMode(Node):
    """
    Autonomous room scanning to detect and map AprilTag markers.

    Scans environment by slowly rotating the robot while accumulating
    AprilTag detections from multiple viewpoints, then triangulates
    marker positions to build a world map.
    """

    def __init__(self):
        super().__init__('auto_scan_mode')

        # --- Scan Parameters ---
        self.declare_parameter('expected_marker_count', 4)
        self.declare_parameter('scan_rotation_speed', 0.3)  # rad/s - slow rotation
        self.declare_parameter('scan_timeout', 30.0)  # seconds before giving up
        self.declare_parameter('confidence_threshold', 3)  # N detections to confirm marker

        self.expected_marker_count = self.get_parameter('expected_marker_count').value
        self.scan_rotation_speed = self.get_parameter('scan_rotation_speed').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        # --- Scan State ---
        self.scan_state = ScanState.IDLE
        self.detected_markers = {}  # marker_id -> {positions: [], count: int}
        self.scan_start_time = 0.0
        self.total_rotation = 0.0

        # --- Robot State ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # --- Marker Map (output) ---
        self.marker_map = {}

        # --- Publishers ---
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/scan_status', 10)
        self.map_pub = self.create_publisher(String, '/marker_map', 10)

        # --- Subscribers ---
        self.pose_sub = self.create_subscription(
            Pose, '/robot_pose', self._robot_pose_callback, 10)
        self.aruco_sub = self.create_subscription(
            PoseArray, '/aruco_detections', self._apriltag_callback, 10)

        # --- Services ---
        self.start_scan_srv = self.create_service(
            Empty, '/start_scan', self._start_scan_callback)

        # --- Timer for scan loop ---
        self.scan_timer = self.create_timer(0.1, self._scan_loop)

        self.get_logger().info('AutoScanMode node initialized')
        self.get_logger().info(f'  Expected markers: {self.expected_marker_count}')
        self.get_logger().info(f'  Rotation speed: {self.scan_rotation_speed} rad/s')
        self.get_logger().info(f'  Timeout: {self.scan_timeout}s')
        self.get_logger().info(f'  Confidence threshold: {self.confidence_threshold} detections')

    def _robot_pose_callback(self, msg: Pose):
        """Update robot pose from localization"""
        self.robot_x = msg.position.x
        self.robot_y = msg.position.y

        # Extract yaw from quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Yaw from quaternion: atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _apriltag_callback(self, msg: PoseArray):
        """Accumulate AprilTag detections from multiple views"""
        if self.scan_state != ScanState.ROTATING:
            return

        if len(msg.poses) == 0:
            return

        # Extract marker IDs from header.frame_id if available
        # Format: "marker_<id>" or just the id number
        marker_ids = []
        if msg.header.frame_id:
            frame_ids = msg.header.frame_id.split(',')
            for fid in frame_ids:
                fid = fid.strip()
                if fid.isdigit():
                    marker_ids.append(int(fid))
                elif fid.startswith('marker_'):
                    try:
                        marker_ids.append(int(fid.split('_')[1]))
                    except (IndexError, ValueError):
                        pass

        # If no valid IDs in header, use sequential indices
        if not marker_ids:
            marker_ids = list(range(len(msg.poses)))

        for i, tag_pose in enumerate(msg.poses):
            if i >= len(marker_ids):
                marker_id = i
            else:
                marker_id = marker_ids[i]

            # Extract position from camera frame
            tx = tag_pose.position.x
            ty = tag_pose.position.y
            tz = tag_pose.position.z

            # Skip if too close (likely noise)
            distance = math.sqrt(tx*tx + ty*ty + tz*tz)
            if distance < 0.05:
                continue

            # Initialize marker entry if new
            if marker_id not in self.detected_markers:
                self.detected_markers[marker_id] = {'positions': [], 'count': 0}

            # Record this detection with robot pose at detection time
            self.detected_markers[marker_id]['positions'].append({
                'robot_x': self.robot_x,
                'robot_y': self.robot_y,
                'robot_yaw': self.robot_yaw,
                'tag_x': tx,
                'tag_y': ty,
                'tag_z': tz,
                'distance': distance
            })
            self.detected_markers[marker_id]['count'] += 1

            self.get_logger().debug(
                f"Marker {marker_id} detected (count: {self.detected_markers[marker_id]['count']}, "
                f"dist: {distance:.2f}m)")

        # Check if we found enough markers with sufficient confidence
        self._check_scan_progress()

    def _check_scan_progress(self):
        """Check if scan has found all expected markers with sufficient confidence"""
        if len(self.detected_markers) < self.expected_marker_count:
            return

        # Verify each detected marker has sufficient detection count
        all_verified = all(
            m['count'] >= self.confidence_threshold
            for m in self.detected_markers.values()
        )

        if all_verified:
            self._complete_scan()

    def _scan_loop(self):
        """Main scan loop - rotate slowly while looking for markers"""
        if self.scan_state != ScanState.ROTATING:
            return

        # Check timeout
        elapsed = self.get_clock().now().seconds_nanosec() / 1e9 - self.scan_start_time
        if elapsed > self.scan_timeout:
            self.scan_state = ScanState.SCAN_FAILED
            self.get_logger().error(f"Scan timeout after {elapsed:.1f}s")
            self._publish_vel(0.0, 0.0)  # Stop
            self._publish_status(f"timeout after {elapsed:.1f}s - found {len(self.detected_markers)} markers")
            return

        # Rotate slowly
        self._publish_vel(0.0, self.scan_rotation_speed)

        # Track rotation for 360 check
        self.total_rotation += abs(self.scan_rotation_speed * 0.1)  # 10Hz update

        if int(self.total_rotation / (2 * math.pi)) > 0:
            # Completed full rotation without finding expected markers
            if len(self.detected_markers) < self.expected_marker_count:
                self.get_logger().warn(
                    f"Only found {len(self.detected_markers)} markers after full rotation")

        # Periodic status update
        if int(elapsed) % 5 == 0 and int(elapsed * 10) % 50 == 0:
            self._publish_status(
                f"scanning... found {len(self.detected_markers)}/{self.expected_marker_count} "
                f"markers ({elapsed:.1f}s elapsed)")

    def _start_scan_callback(self, request, response):
        """Handle start_scan service request"""
        self.start_scan()
        return response

    def start_scan(self, expected_count=None):
        """
        Start autonomous scan.

        Args:
            expected_count: Optional override for expected marker count
        """
        if expected_count is not None:
            self.expected_marker_count = expected_count

        self.scan_state = ScanState.ROTATING
        self.detected_markers = {}
        self.scan_start_time = self.get_clock().now().seconds_nanosec() / 1e9
        self.total_rotation = 0.0

        self.get_logger().info(
            f"Starting scan for {self.expected_marker_count} markers...")
        self._publish_status(f"starting scan for {self.expected_marker_count} markers")

    def _complete_scan(self):
        """Build marker map from accumulated detections"""
        self.scan_state = ScanState.SCAN_COMPLETE
        self._publish_vel(0.0, 0.0)  # Stop rotation

        marker_map = {}

        for marker_id, detections in self.detected_markers.items():
            # Collect world positions from all detection viewpoints
            world_x_coords = []
            world_y_coords = []

            for det in detections['positions']:
                # Robot pose at detection time
                rx = det['robot_x']
                ry = det['robot_y']
                r_yaw = det['robot_yaw']

                # Tag position relative to camera (in camera frame)
                dx = det['tag_x']
                dy = det['tag_y']
                dz = det['tag_z']

                # Transform from robot/camera frame to world frame
                # Camera is at robot position, looking forward (+X in camera frame)
                # World X-axis is forward, Y-axis is left
                # Robot yaw 0 = facing world +X
                # Camera +X (forward) -> world: cos(yaw)*dx - sin(yaw)*dy
                # Camera +Y (left) -> world: sin(yaw)*dx + cos(yaw)*dy

                # World position of tag
                world_dx = math.cos(r_yaw) * dx - math.sin(r_yaw) * dy
                world_dy = math.sin(r_yaw) * dx + math.cos(r_yaw) * dy

                wx = rx + world_dx
                wy = ry + world_dy

                world_x_coords.append(wx)
                world_y_coords.append(wy)

            # Average world position
            if world_x_coords and world_y_coords:
                avg_x = sum(world_x_coords) / len(world_x_coords)
                avg_y = sum(world_y_coords) / len(world_y_coords)

                marker_map[str(marker_id)] = [avg_x, avg_y, 0.0]  # z=0 plane

        self.marker_map = marker_map

        self.get_logger().info(f"Scan complete. Found {len(marker_map)} markers:")
        for mid, pos in marker_map.items():
            self.get_logger().info(f"  Marker {mid}: ({pos[0]:.2f}, {pos[1]:.2f})")

        # Save to file
        self._save_marker_map()

        # Publish for use by localization
        self._publish_marker_map()

        self._publish_status(f"scan complete - found {len(marker_map)} markers")

    def _save_marker_map(self):
        """Save discovered marker map to JSON"""
        map_file = os.path.join(
            os.path.dirname(__file__),
            '..', '..', '..', '..', 'data', 'marker_map_auto', 'marker_map_auto.json'
        )

        # Ensure directory exists
        os.makedirs(os.path.dirname(map_file), exist_ok=True)

        with open(map_file, 'w') as f:
            json.dump(self.marker_map, f, indent=2)

        self.get_logger().info(f"Marker map saved to {map_file}")

    def load_marker_map(self):
        """Load marker map from file"""
        map_file = os.path.join(
            os.path.dirname(__file__),
            '..', '..', '..', '..', 'data', 'marker_map_auto', 'marker_map_auto.json'
        )

        if os.path.exists(map_file):
            with open(map_file, 'r') as f:
                self.marker_map = json.load(f)
            self.get_logger().info(
                f"Loaded {len(self.marker_map)} markers from {map_file}")
            return True
        return False

    def _publish_marker_map(self):
        """Publish marker map for use by localization"""
        msg = String()
        msg.data = json.dumps(self.marker_map)
        self.map_pub.publish(msg)

    def _publish_status(self, status: str):
        """Publish scan status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _publish_vel(self, linear: float, angular: float):
        """Publish velocity command"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.vel_pub.publish(twist)

    def get_scan_state(self) -> str:
        """Get current scan state"""
        return self.scan_state

    def get_marker_map(self) -> dict:
        """Get discovered marker map"""
        return self.marker_map


def main(args=None):
    rclpy.init(args=args)
    node = AutoScanMode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
