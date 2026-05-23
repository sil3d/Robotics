#!/usr/bin/env python3
"""
 ===========================================================================
   LOCALIZATION NODE - Robot Pose Estimation from AprilTag + IMU
 ===========================================================================

 Estimates robot pose (x, y, theta) using:
 1. AprilTag detections - known marker positions in map
 2. IMU yaw - orientation from gyroscope

 Robot pose is published to /robot_pose (geometry_msgs/msg/Pose)

 AprilTag Map (to be configured):
   Marker ID -> (x, y, z) global position in meters
   Example:
     marker_0  -> (0.0, 0.0, 0.0)   Home
     marker_1  -> (1.5, 0.0, 0.0)   Manufacturing
     marker_2  -> (0.0, 1.5, 0.0)   Storage A
     marker_3  -> (1.5, 1.5, 0.0)   Storage B

  PUBLISHERS:
   - /robot_pose (geometry_msgs/msg/Pose) - estimated robot pose
   - /localization_confidence (std_msgs/msg/Float32) - pose confidence 0.0-1.0

  SUBSCRIBERS:
   - /aruco_detections (geometry_msgs/msg/PoseArray) - detected tag poses
   - /imu_data (geometry_msgs/msg/Accel) - yaw angle

  CONFIDENCE TRACKING:
   - pose_confidence: 0.0 to 1.0 metric
   - Decays at 0.1/s after 2s of no AprilTag detection
   - LKGS (Last Known Good State) used when confidence < 0.3
   - IMU-only mode activates after 30s of no AprilTag

 PARAMETERS:
  - marker_map: dict of marker_id -> (x, y, z) positions
  - robot_frame: frame_id for robot pose (default: "map")

 ===========================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Accel, TransformStamped, Transform
from std_msgs.msg import String, Float32
import numpy as np
import math


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')

        # Declare parameters
        self.declare_parameter('marker_map', {
            '0': [0.0, 0.0, 0.0],
            '1': [1.5, 0.0, 0.0],
            '2': [0.0, 1.5, 0.0],
            '3': [1.5, 1.5, 0.0],
        })
        self.declare_parameter('robot_frame', 'map')

        # Load marker map from parameters
        marker_map_param = self.get_parameter('marker_map').value
        self.marker_map = {}
        for key, val in marker_map_param.items():
            self.marker_map[int(key)] = np.array(val, dtype=np.float32)

        self.get_logger().info(f'Localization Node started with {len(self.marker_map)} markers')

        # Robot state
        self.robot_x = 0.0  # meters
        self.robot_y = 0.0  # meters
        self.robot_yaw = 0.0  # radians
        self.last_yaw = 0.0
        self.yaw_bias = 0.0
        self.has_initial_pose = False

        # Last detection time for timeout
        self.last_detection_time = 0
        self.pose_timeout = 5.0  # seconds

        # Confidence tracking
        self.pose_confidence = 1.0  # 0.0 to 1.0
        self.last_apriltag_time = 0
        self.last_imu_time = 0
        self.pose_timeout_conf = 5.0  # seconds before confidence decay
        self.confidence_decay_rate = 0.1  # per second when no AprilTag

        # Last Known Good State (LKGS)
        self.lkgs_x = 0.0
        self.lkgs_y = 0.0
        self.lkgs_yaw = 0.0
        self.lkgs_timestamp = 0

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.confidence_pub = self.create_publisher(Float32, '/localization_confidence', 10)

        # Subscribers
        self.aruco_sub = self.create_subscription(
            PoseArray, '/aruco_detections', self.aruco_callback, 10)
        self.imu_sub = self.create_subscription(
            Accel, '/imu_data', self.imu_callback, 10)
        self.cmd_result_sub = self.create_subscription(
            String, '/cmd_result', self.cmd_result_callback, 10)

        # Timer for publishing pose at 20Hz
        self.timer = self.create_timer(0.05, self.publish_pose)

    def imu_callback(self, msg):
        """Update yaw from IMU with confidence tracking"""
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_imu_time = now

        self.robot_yaw = math.radians(msg.linear.x)  # yaw in degrees -> radians
        self.last_yaw = self.robot_yaw

    def cmd_result_callback(self, msg):
        """Log command results"""
        self.get_logger().debug(f'CMD: {msg.data}')

    def aruco_callback(self, msg: PoseArray):
        """Update robot pose from AprilTag detections"""
        if len(msg.poses) == 0:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        self.last_apriltag_time = now
        self.last_detection_time = now

        # For each detected marker, compute robot position
        # Robot position = marker_position - rotation_matrix * tag_translation
        # where tag_translation is the detected distance from camera to tag

        detected_poses = []
        for i, tag_pose in enumerate(msg.poses):
            # Get marker ID from header if available, otherwise use index
            # In practice, camera_node should include marker IDs in header or separate topic
            # For now, we use detection index and assume markers are detected in order of proximity

            # Extract tag position relative to camera
            tx = tag_pose.position.x
            ty = tag_pose.position.y
            tz = tag_pose.position.z

            # Tag rotation (quaternion to euler)
            qx, qy, qz, qw = tag_pose.orientation.x, tag_pose.orientation.y, \
                             tag_pose.orientation.z, tag_pose.orientation.w

            # Convert quaternion to rotation matrix
            rot_matrix = self._quaternion_to_rotation_matrix(qx, qy, qz, qw)

            # Camera to tag vector in camera frame
            cam_to_tag = np.array([tx, ty, tz])

            # Transform to world frame using tag orientation
            # The tag's rotation tells us how to rotate the camera-to-tag vector
            world_to_tag = rot_matrix @ cam_to_tag

            # Robot position relative to marker
            # If we know marker position in world, robot is at:
            # robot_pos = marker_pos - (marker_to_camera_vector)
            # The camera is at robot position + camera offset
            # We approximate: robot_pos ≈ marker_pos - world_to_tag direction * |cam_to_tag|

            tag_dist = np.linalg.norm(cam_to_tag)
            if tag_dist < 0.01:
                continue

            # Find closest marker position (simplified - in production would use marker IDs)
            # This assumes the closest detected tag is the one with minimal distance
            detected_poses.append({
                'dist': tag_dist,
                'pose': tag_pose,
                'world_vec': world_to_tag
            })

        if not detected_poses:
            return

        # Sort by distance, use closest tag for localization
        detected_poses.sort(key=lambda x: x['dist'])
        best = detected_poses[0]

        # Use the translation vector to estimate robot position
        # tag_detected_pos is the position of the tag relative to camera
        # We need to flip the vector: camera is in front of tag, so
        # robot_pos = marker_pos + (camera_to_tag direction normalized * tag_dist)

        cam_to_tag_dir = best['world_vec'] / np.linalg.norm(best['world_vec'])

        # Assume camera is mounted at front of robot, looking forward
        # The tag appears in front of camera, so robot is behind the tag
        # robot direction from tag = -cam_to_tag_dir
        robot_from_tag = -cam_to_tag_dir * best['dist']

        # Find which marker the robot is currently closest to (by robot pose)
        closest_marker_id = min(self.marker_map.keys(),
                                key=lambda m: np.linalg.norm(
                                    self.marker_map[m][:2] - np.array([self.robot_x, self.robot_y])))

        marker_world = self.marker_map[closest_marker_id]

        # Update robot position
        self.robot_x = marker_world[0] + robot_from_tag[0]
        self.robot_y = marker_world[1] + robot_from_tag[1]

        # Also extract yaw from tag orientation if available
        # The tag's rotation tells us the camera's orientation relative to tag
        # which we can use to estimate robot heading

        self.has_initial_pose = True

        # High confidence when AprilTag is fresh
        self.pose_confidence = min(1.0, self.pose_confidence + 0.2)

        # Update LKGS when we have good pose
        if self.pose_confidence > 0.7:
            self.lkgs_x = self.robot_x
            self.lkgs_y = self.robot_y
            self.lkgs_yaw = self.robot_yaw
            self.lkgs_timestamp = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().debug(
            f'Pose update: x={self.robot_x:.2f}, y={self.robot_y:.2f}, '
            f'yaw={math.degrees(self.robot_yaw):.1f}deg, conf={self.pose_confidence:.2f}')

    def _quaternion_to_rotation_matrix(self, x, y, z, w):
        """Convert quaternion to rotation matrix"""
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm

        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ], dtype=np.float32)

    def publish_pose(self):
        """Publish robot pose at 20Hz with confidence metric"""
        now = self.get_clock().now().nanoseconds / 1e9

        # Apply confidence decay when no AprilTag
        if self.last_apriltag_time > 0:
            time_since_tag = now - self.last_apriltag_time
            if time_since_tag > 2.0:  # After 2 seconds of no detection
                decay = self.confidence_decay_rate * (time_since_tag - 2.0)
                self.pose_confidence = max(0.0, self.pose_confidence - decay)

        # If no AprilTag at all for 30s, use IMU-only dead reckoning
        if self.last_apriltag_time == 0 or (now - self.last_apriltag_time) > 30.0:
            # IMU-only mode - confidence is lower
            self.pose_confidence = 0.3
            self.get_logger().warn("Using IMU-only mode - no AprilTag detections")

        # Check if we should use LKGS
        self.use_lkgs_if_needed()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.robot_x
        pose_msg.pose.position.y = self.robot_y
        pose_msg.pose.position.z = 0.0

        # Convert yaw to quaternion
        q = self._yaw_to_quaternion(self.robot_yaw)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.pose_pub.publish(pose_msg)

        # Also publish confidence separately for navigation_node
        confidence_msg = Float32()
        confidence_msg.data = self.pose_confidence
        self.confidence_pub.publish(confidence_msg)

    def _yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion [x, y, z, w]"""
        half = yaw / 2.0
        return [0.0, 0.0, math.sin(half), math.cos(half)]

    def _quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle"""
        # Roll and pitch from quaternion (assuming planar robot, only yaw matters)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def use_lkgs_if_needed(self):
        """Return LKGS if current confidence is too low"""
        if self.pose_confidence > 0.3:
            return False  # Current pose is OK

        if self.lkgs_timestamp == 0:
            return False  # No LKGS available

        # Check if LKGS is stale (older than 10 seconds)
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.lkgs_timestamp > 10.0:
            self.get_logger().warn("LKGS too stale to use!")
            return False

        self.get_logger().info("Using Last Known Good State")
        self.robot_x = self.lkgs_x
        self.robot_y = self.lkgs_y
        self.robot_yaw = self.lkgs_yaw
        return True

    def compute_fused_pose(self, tag_pose, imu_yaw):
        """
        Fuse AprilTag and IMU for better pose estimate.

        Weight based on confidence:
        - AprilTag provides good position, poor rotation (from tag orientation)
        - IMU provides good rotation, no position
        """
        # Position from AprilTag (already computed)
        pos_x, pos_y = self.robot_x, self.robot_y

        # Rotation primarily from IMU (more reliable)
        # But could be refined by tag orientation when tag is close and reliable
        fused_yaw = imu_yaw

        # If tag is very close (< 0.3m), trust tag rotation more
        tag_dist = math.sqrt(
            tag_pose.position.x**2 +
            tag_pose.position.y**2 +
            tag_pose.position.z**2
        )
        if tag_dist < 0.3 and tag_dist > 0:
            # Blend IMU and tag rotation
            tag_yaw = self._quaternion_to_yaw(
                tag_pose.orientation.x,
                tag_pose.orientation.y,
                tag_pose.orientation.z,
                tag_pose.orientation.w
            )
            # Weight based on distance
            tag_weight = max(0, 1 - tag_dist / 0.3) * 0.3  # Up to 30% tag contribution
            fused_yaw = imu_yaw * (1 - tag_weight) + tag_yaw * tag_weight

        return pos_x, pos_y, fused_yaw

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()